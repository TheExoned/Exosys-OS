#ifndef TIME_ENGINE_H
#define TIME_ENGINE_H

#include <Arduino.h>

// Variables de lectura asíncrona para LVGL
extern volatile uint8_t sys_hour;
extern volatile uint8_t sys_min;
extern volatile uint8_t sys_sec;
extern volatile bool time_is_gnss;

void init_time_engine();
void time_loop_handler();

// --- SISTEMA DE ALARMA HOSTIL ---
void init_time_engine();
void time_loop_handler();
void trigger_alarm_test();
void abort_alarm();
bool is_alarm_active();
void process_alarm_state_machine();

#endif