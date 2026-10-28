#pragma once
#include <Arduino.h>
#include <lvgl.h>

extern lv_obj_t * scr_settings_main;
extern lv_obj_t * scr_settings_display;
extern lv_obj_t * scr_settings_wallpaper;
extern lv_obj_t * scr_settings_clock;
extern lv_obj_t * scr_settings_alarms;
extern lv_obj_t * scr_settings_wireless;
extern lv_obj_t * scr_settings_wifi_ph;
extern lv_obj_t * scr_settings_hotspot;
extern lv_obj_t * scr_settings_bt;

void init_settings_screens();
bool settings_back_router();
int check_alarm_trigger(uint8_t current_h, uint8_t current_m);