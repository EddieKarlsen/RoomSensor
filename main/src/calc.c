
#include "../inc/calc.h"



energy_calc_t calculate_energy_need(double indoor, double outdoor,
                                    double airflow, double solar) 
{
    energy_calc_t e = {0};

    // 1. Konduktion (placeholder värden)
    double U_value = 0.3;     // W/(m²K)
    double Area = 50.0;       // m² (väggar)
    e.conduction_loss = U_value * Area * (indoor - outdoor);

    // 2. Ventilation
    double air_density = 1.225;   // kg/m³
    double cp = 1005;             // J/kgK
    double m_dot = airflow * air_density;  // kg/s (om flödet är m³/s)
    e.ventilation_loss = m_dot * cp * (indoor - outdoor);

    // 3. Solinstrålning (placeholder)
    double window_area = 2.0;     // m²
    double SHGC = 0.6;
    e.solar_gain = solar * window_area * SHGC;

    // 4. Nettoenergi
    e.net_power = e.conduction_loss + e.ventilation_loss - e.solar_gain;

    return e;
}