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

                cJSON *root = cJSON_Parse((char*)buf);
                if (!root) { buf_pos = 0; continue; }

                double indoor  = cJSON_GetObjectItem(root, "indoor_temp")->valuedouble;
                double outdoor = cJSON_GetObjectItem(root, "outdoor_temp")->valuedouble;
                double airflow = cJSON_GetObjectItem(root, "airflow_rate")->valuedouble;
                double solar   = cJSON_GetObjectItem(root, "solar_intensity")->valuedouble;

                cJSON_Delete(root);
                
                // 1. Energi beräkning
                energy_calc_t energy = calculate_energy_need(indoor, outdoor, airflow, solar);

                // 2. PID
                double setpoint = 21.0;     // du kan senare skicka detta från Python
                double dt = 0.1;
                double pid_out = pid_update(&pid, setpoint, indoor, dt);

                double required_power = energy.net_power + pid_out;

                // 3. Bygg PIDResponse JSON
                cJSON *resp = cJSON_CreateObject();
                cJSON_AddNumberToObject(resp, "heating_power", required_power);
                cJSON_AddNumberToObject(resp, "pid_p", pid.kp * (setpoint - indoor));
                cJSON_AddNumberToObject(resp, "pid_i", pid.ki * pid.integrator);
                cJSON_AddNumberToObject(resp, "pid_d", pid.kd * pid.last_error);
                cJSON_AddNumberToObject(resp, "error", setpoint - indoor);
                cJSON_AddNumberToObject(resp, "timestamp", esp_timer_get_time() / 1000);

                char *resp_str = cJSON_PrintUnformatted(resp);
                uart_write_bytes(UART_PORT, resp_str, strlen(resp_str));
                uart_write_bytes(UART_PORT, "\n", 1);

                free(resp_str);
                cJSON_Delete(resp);

                buf_pos = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}