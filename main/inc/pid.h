#ifndef PID_H
#define PID_H

#include "config.h"


typedef struct {
    double kp;
    double ki;
    double kd;
    double integrator;
    double last_error;
    double out_min;
    double out_max;
} pid_t;


void pid_init(pid_t *p, double kp, double ki, double kd, double out_min, double out_max) ;
double pid_update(pid_t *p, double setpoint, double indoor_temp, double dt);
void pid_set_setpoint(double setpoint);



#endif