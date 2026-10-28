#ifndef BACKGROUND_ENGINE_H
#define BACKGROUND_ENGINE_H

#include <lvgl.h>
#include <Arduino.h>

void init_background_engine();
void apagar_motor_exnd();
void cycle_next_wallpaper();

// --- NÚCLEO ASÍNCRONO ---
void background_transfer_loop();
void iniciar_transferencia_async(uint8_t index);
bool is_transferring_bg();
void scan_animated_wallpapers();

extern lv_obj_t * scr_vault_menu;
void build_wallpaper_list();

// --- TELEMETRÍA (PARA SETTINGS) ---
uint8_t get_total_aw_count();
uint8_t get_current_aw_index();
void set_current_aw_index(uint8_t idx);
void arrancar_motor_exnd(uint8_t index);
bool get_aw_info_from_sd(uint8_t index, uint16_t *w, uint16_t *h, uint32_t *frames, uint8_t *fps, uint32_t *size_bytes);
bool read_aw_preview_from_sd(uint8_t index, uint16_t* buffer, uint32_t bytes_to_read);
bool reload_custom_placeholder();

#endif