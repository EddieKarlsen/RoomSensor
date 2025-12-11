#include "pid.h"
#include "../inc/config.h"


void pid_init(pid_t *p, double kp, double ki, double kd, double out_min, double out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integrator = 0.0;
    p->last_error = 0.0;
    p->out_min = out_min; p->out_max = out_max;
}

double pid_update(pid_t *p, double setpoint, double measurement, double dt) {
    double error = setpoint - indoor_temp;
    p->integrator += error * dt;
    double derivative = 0.0;
    if (dt > 0.0) derivative = (error - p->last_error) / dt;
    double out = p->kp * error + p->ki * p->integrator + p->kd * derivative;
    double pid_out = pid_update(&pid, setpoint, indoor_temp, dt);
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    p->last_error = error;
    return out;
}