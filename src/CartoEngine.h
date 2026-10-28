#ifndef CARTO_ENGINE_H
#define CARTO_ENGINE_H

#include <Arduino.h>
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <SdFat.h>
#include <TinyGPS++.h>

// Herencia de hardware desde el Kernel
extern SemaphoreHandle_t lvgl_mutex;
extern SdFs sd;
extern HardwareSerial GPSSerial;
extern TinyGPSPlus gps;

// Rutinas expuestas al Kernel
extern void init_carto_engine();
void build_map_ui(lv_obj_t * parent_screen);
void carto_loop_handler();
bool hardware_back_router();
void actualizar_marcador_gps();

#endif