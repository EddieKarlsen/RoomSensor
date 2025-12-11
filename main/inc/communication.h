#ifndef COMMUNICATION_H
#define COMMUNICATION_H

void process_json_command(const char* json_str);
void send_json_response(const char* type, float value);
void uart_task(void *arg);


#endif