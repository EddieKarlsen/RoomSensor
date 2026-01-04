# RoomSensor

Detta projekt är en PID-baserad temperaturregulator implementerad för ESP32. Systemet tar emot sensordata i JSON-format via UART, beräknar en PID-signal och returnerar svar som JSON.
## License
![MIT License](https://img.shields.io/badge/License-MIT-green.svg)

## Funktioner

* **Realtids PID-reglering**: Beräknar `heating_power_pct` (0–100%) baserat på börvärde och aktuell inomhustemperatur.
* **JSON-gränssnitt**: All kommunikation sker via strukturerad JSON för enkel integration med externa system.
* **Energiberäkning**: Innehåller moduler för att beräkna konduktionsförluster, ventilationsförluster och solinstrålning.
* **Anti-Windup**: PID-kontrollern har inbyggt skydd mot mättnad av integratorn för att undvika overshoot.
* **Robust UART-hantering**: Implementerad som en FreeRTOS-task med bufferhantering och skydd mot spill.

## Filstruktur

* **`main.c`**: Entry point som initierar UART-drivrutinen och startar `uart_task`.
* **`pid.c / .h`**: Implementering av PID-logiken (Proportionell, Integral, Derivata).
* **`communication.c / .h`**: Hanterar JSON-parsing med cJSON och UART-kommunikation.
* **`calc.c / .h`**: Innehåller termodynamiska formler för att beräkna byggnadens energibehov.
* **`config.h`**: Globala inställningar som UART-port (UART_NUM_0) och bufferstorlek.
