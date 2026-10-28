#pragma once
#include <Arduino.h>
#include <lvgl.h>

extern lv_obj_t * net_screen_ptr;
extern lv_obj_t * scr_settings_hotspot;
extern lv_obj_t * scr_settings_bt;

// Variables globales de Fase 6 (GNSS pasivo recibido desde el telefono)
extern volatile float remote_gnss_lat;
extern volatile float remote_gnss_lon;
extern volatile float remote_gnss_alt;
extern volatile float remote_gnss_spd;
extern volatile uint32_t remote_gnss_rx_count;
extern volatile uint32_t remote_gnss_last_ms;

// Estados exportados para la matriz tactica del Main Menu
extern volatile bool wifi_scanning;
extern volatile bool wifi_connecting;
extern volatile bool hotspot_bg_connecting;
extern volatile bool wifi_connection_failed;

void build_network_ui();
void network_loop_handler();
bool network_back_router();
void refresh_wireless_status_ui(lv_timer_t * t);

// Funciones puente para Main Menu y Ajustes -> Conexion Inalambrica
void net_open_hotspot_scanner_from_settings();
void net_open_hotspot_scanner_from_hub();
void net_open_sub_from_hub(lv_obj_t * target_scr);
bool net_connect_saved_hotspot();
void net_disconnect_hotspot();
bool net_has_saved_hotspot(char* out_ssid, size_t max_len);
bool net_hotspot_is_app_linked();
bool net_wifi_is_connected();

void net_ble_set_enabled(bool enable);
bool net_ble_is_enabled();
bool net_ble_is_linked();
int  net_ble_get_rssi();

void net_lora_set_enabled(bool enable);
bool net_lora_is_enabled();