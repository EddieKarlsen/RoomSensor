#include "pid.h"
#include "../inc/config.h"


void pid_init(pid_t *p, double kp, double ki, double kd, double out_min, double out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integrator = 0.0;
    p->last_error = 0.0;
    p->out_min = out_min; p->out_max = out_max;
}

double pid_update(pid_t *p, double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    // Proportionell
    p->p_term = p->kp * error;

    // Integrator (anti-windup)
    if (!((p->p_term >= p->out_max && error > 0) ||
          (p->p_term <= p->out_min && error < 0))) {
        p->integrator += error * dt;
    }
    p->i_term = p->ki * p->integrator;

    // Derivata
    p->d_term = 0.0;
    if (dt > 0.0) {
        p->d_term = p->kd * (error - p->last_error) / dt;
    }
    p->last_error = error;

    double out = p->p_term + p->i_term + p->d_term;

    // Klipp
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    return out;
}