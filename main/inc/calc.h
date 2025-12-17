#ifndef CALC_H
#define CALC_H



typedef struct {
    double conduction_loss;   // W
    double ventilation_loss;  // W
    double solar_gain;        // W
    double net_power;         // W (positiv = behöver värme)
} energy_calc_t;

energy_calc_t calculate_energy_need(double indoor, double outdoor,
                                    double airflow, double solar);
#endif