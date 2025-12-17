#ifndef COMMUNICATION_H
#define COMMUNICATION_H


typedef struct {
    float temp_in;
    float temp_out;
} sensor_data_t;

void process_json_command(const char* json_str);
void send_json_response(const char* type, float value);
void uart_task(void *arg);


#endif