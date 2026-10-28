#ifndef KIN_ENGINE_H
#define KIN_ENGINE_H

#include <Arduino.h>

extern volatile float kin_actual_bpm;
extern volatile float kin_target_walk;
extern volatile float kin_target_run;
extern volatile int16_t kin_raw_gx;
extern volatile int16_t kin_raw_gy;
extern volatile int16_t kin_raw_gz;

extern volatile uint32_t current_steps;
extern volatile uint8_t current_activity;
extern volatile bool is_moving;
extern volatile uint8_t kin_event_latch; // <-- Interrupción visual

void init_kin_engine();
void kin_loop_handler();

#endif