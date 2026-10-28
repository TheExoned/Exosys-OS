#ifndef POWER_ENGINE_H
#define POWER_ENGINE_H

#include <Arduino.h>

// Variables volátiles para lectura inmediata en hilos concurrentes
extern volatile int real_batt_pct;
extern volatile bool is_charging;

void init_pmu();
void pmu_loop_handler();

#endif