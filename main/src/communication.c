#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include "communication.h"
#include "../inc/config.h"
#include "../inc/pid.h"
#include "cJSON.h"

static pid_t pid;  

void process_json_command(const char* json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON parse error");
        return;
    }
    
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(cmd)) {
        if (strcmp(cmd->valuestring, "GET_TEMP_IN") == 0) {
            send_json_response("temp_in", sensor_data.temp_in);
        }
        else if (strcmp(cmd->valuestring, "GET_TEMP_OUT") == 0) {
            send_json_response("temp_out", sensor_data.temp_out);
        }
        else if (strcmp(cmd->valuestring, "SET_SETPOINT") == 0) {
            cJSON *value = cJSON_GetObjectItem(root, "value");
            if (cJSON_IsNumber(value)) {
                pid_set_setpoint(value->valuedouble);
                send_json_response("setpoint", value->valuedouble);
            }
        }
    }
    
    cJSON_Delete(root);
}

void send_json_response(const char* type, float value) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", type);
    cJSON_AddNumberToObject(root, "value", value);
    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000); // ms
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str != NULL) {
        uart_write_bytes(UART_NUM, json_str, strlen(json_str));
        uart_write_bytes(UART_NUM, "\n", 1);
        free(json_str);  // Viktigt!
    }
    
    cJSON_Delete(root);
}


static void uart_task(void *arg)
{
    static uint8_t buf[BUF_SIZE]; 
    static pid_t pid;
    
    pid_init(&pid, 1.0, 0.1, 0.01, -100.0, 100.0);

    size_t buf_pos = 0;
    uint8_t rx_byte;
    TickType_t last_tick = 0;
    bool first_update = true;

    while (1) {
        int len = uart_read_bytes(UART_PORT, &rx_byte, 1, pdMS_TO_TICKS(RX_TIMEOUT_MS));
        if (len > 0) {
            if (rx_byte == '\r') continue;
            
            if (buf_pos >= BUF_SIZE - 1) {
                ESP_LOGW(TAG, "Buffer full, resetting");
                buf_pos = 0;
            }
            
            buf[buf_pos++] = rx_byte;

            if (rx_byte == '\n') {
                buf[buf_pos] = '\0';
                ESP_LOGD(TAG, "Received: %s", (char*)buf);

                cJSON *root = cJSON_Parse((char*)buf);
                if (root) {
                    cJSON *j_set = cJSON_GetObjectItem(root, "setpoint");
                    cJSON *j_meas = cJSON_GetObjectItem(root, "measurement");
                    
                    if (cJSON_IsNumber(j_set) && cJSON_IsNumber(j_meas)) {
                        double setpoint = j_set->valuedouble;
                        double measurement = j_meas->valuedouble;

                        // dt-beräkning
                        TickType_t now = xTaskGetTickCount();
                        double dt;
                        if (first_update) {
                            dt = 0.1;
                            first_update = false;
                        } else {
                            dt = (now - last_tick) / (double)configTICK_RATE_HZ;
                            if (dt <= 0 || dt > 10.0) dt = 0.1; // Säkerhetsgräns
                        }
                        last_tick = now;

                        double pid_out = pid_update(&pid, setpoint, measurement, dt);

                        // Skicka svar
                        cJSON *resp = cJSON_CreateObject();
                        cJSON_AddNumberToObject(resp, "pid", pid_out);
                        cJSON_AddNumberToObject(resp, "dt", dt); // Debug
                        char *resp_str = cJSON_PrintUnformatted(resp);
                        if (resp_str) {
                            uart_write_bytes(UART_PORT, resp_str, strlen(resp_str));
                            uart_write_bytes(UART_PORT, "\n", 1);
                            free(resp_str);
                        }
                        cJSON_Delete(resp);
                    } else {
                        const char *err = "{\"error\":\"missing_fields\"}\n";
                        uart_write_bytes(UART_PORT, err, strlen(err));
                    }
                    cJSON_Delete(root);
                } else {
                    ESP_LOGW(TAG, "Invalid JSON");
                    const char *err = "{\"error\":\"invalid_json\"}\n";
                    uart_write_bytes(UART_PORT, err, strlen(err));
                }

                buf_pos = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}