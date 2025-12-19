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
#include "../inc/calc.h"
#include "cJSON.h"

// Skapa variabeln (typen finns i communication.h)
sensor_data_t sensor_data = {0};

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
        uart_write_bytes(UART_PORT, json_str, strlen(json_str));
        uart_write_bytes(UART_PORT, "\n", 1);
        free(json_str);
    }
    
    cJSON_Delete(root);
}

void uart_task(void *arg)
{
    static uint8_t buf[BUF_SIZE]; 
    pid_t pid;
    pid_init(&pid,
            20.0,  // kp: Proportional gain
            0.005, // ki: Integral gain
            0.0,   // kd: Derivative gain
            0.0,   // out_min: Minimum clamp
            100.0  // out_max: Maximum clamp
    );

    // VIKTIGT: Inaktivera ESP_LOG för UART på samma port
    esp_log_level_set("uart", ESP_LOG_NONE);
    
    size_t buf_pos = 0;
    uint8_t rx_byte;
    
    ESP_LOGI(TAG, "UART task ready - waiting for data");

    while (1) {
        int len = uart_read_bytes(UART_PORT, &rx_byte, 1, pdMS_TO_TICKS(RX_TIMEOUT_MS));
        if (len > 0) {
            // Ignorera carriage return
            if (rx_byte == '\r') continue;
            
            // Buffer overflow protection
            if (buf_pos >= BUF_SIZE - 1) {
                ESP_LOGW(TAG, "Buffer full, resetting");
                buf_pos = 0;
            }
            
            buf[buf_pos++] = rx_byte;

            // När vi får newline, bearbeta meddelandet
            if (rx_byte == '\n') {
                buf[buf_pos] = '\0';  // Null-terminate
                
                // DEBUG: Skriv ut vad vi fick (ta bort detta senare)
                // ESP_LOGI(TAG, "RX: %s", (char*)buf);

                cJSON *root = cJSON_Parse((char*)buf);
                if (!root) { 
                    ESP_LOGE(TAG, "JSON parse failed: %s", (char*)buf);
                    buf_pos = 0; 
                    continue; 
                }

                // Säkrare JSON parsing med nullcheck
                cJSON *indoor_json = cJSON_GetObjectItem(root, "indoor_temp");
                cJSON *outdoor_json = cJSON_GetObjectItem(root, "outdoor_temp");
                cJSON *airflow_json = cJSON_GetObjectItem(root, "airflow_rate");
                cJSON *solar_json = cJSON_GetObjectItem(root, "solar_intensity");
                cJSON *setpoint_json = cJSON_GetObjectItem(root, "setpoint");

                if (!indoor_json || !outdoor_json || !airflow_json || !solar_json) {
                    ESP_LOGE(TAG, "Missing JSON fields");
                    cJSON_Delete(root);
                    buf_pos = 0;
                    continue;
                }

                double indoor  = indoor_json->valuedouble;
                double outdoor = outdoor_json->valuedouble;
                double airflow = airflow_json->valuedouble;
                double solar   = solar_json->valuedouble;
                
                // Använd setpoint från JSON, eller fallback till 21.0
                double setpoint = 21.0;  // Default
                if (setpoint_json && cJSON_IsNumber(setpoint_json)) {
                    setpoint = setpoint_json->valuedouble;
                }

                cJSON_Delete(root);
                
                // Uppdatera sensor_data
                sensor_data.temp_in = (float)indoor;
                sensor_data.temp_out = (float)outdoor;

                // PID med dynamiskt setpoint
                static int64_t last_update_time = 0;
                int64_t now = esp_timer_get_time();
                double dt = (now - last_update_time) / 1e6;  // µs → sekunder
                if (last_update_time == 0) dt = 0.1;  // First run
                last_update_time = now;

                double pid_pct = pid_update(&pid, setpoint, indoor, dt);

                // Clamping
                if (pid_pct < 0.0) pid_pct = 0.0;
                if (pid_pct > 100.0) pid_pct = 100.0;

                // Bygg PIDResponse JSON
                cJSON *resp = cJSON_CreateObject();
                if (resp == NULL) {
                    ESP_LOGE(TAG, "Failed to create JSON response");
                    buf_pos = 0;
                    continue;
                }
                
                cJSON_AddNumberToObject(resp, "heating_power_pct", pid_pct);
                cJSON_AddNumberToObject(resp, "setpoint", setpoint);
                cJSON_AddNumberToObject(resp, "error", setpoint - indoor);
                cJSON_AddNumberToObject(resp, "pid_p", pid.kp * (setpoint - indoor));
                cJSON_AddNumberToObject(resp, "pid_i", pid.ki * pid.integrator);
                cJSON_AddNumberToObject(resp, "pid_d", pid.kd * (pid.last_error));
                cJSON_AddNumberToObject(resp, "timestamp", (int)(esp_timer_get_time() / 1000));

                char *resp_str = cJSON_PrintUnformatted(resp);
                if (resp_str != NULL) {
                    // Skicka ENDAST JSON, inget extra
                    uart_write_bytes(UART_PORT, resp_str, strlen(resp_str));
                    uart_write_bytes(UART_PORT, "\n", 1);
                    
                    // Vänta tills data är sänt
                    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
                    
                    free(resp_str);
                } else {
                    ESP_LOGE(TAG, "Failed to print JSON");
                }
                
                cJSON_Delete(resp);
                buf_pos = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}