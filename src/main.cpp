// =====================================================================
// KERNEL MAESTRO - GESTOR DE RECURSOS Y HUB TACTICO CYBERPUNK
// ITERACION DE VUELO: ARQUITECTURA LIMPIA - CERO REDECLARACIONES
// =====================================================================
#include <Arduino.h>
#include "PowerEngine.h"
#include "TimeEngine.h"
#include "KinEngine.h"
#include "BackgroundEngine.h"
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <SPI.h>
#include <SdFat.h>
#include "esp_heap_caps.h"
#include <TinyGPS++.h>
#include "NetworkEngine.h"
#include <math.h>
#include <Preferences.h>

extern volatile uint32_t current_steps;
extern volatile uint8_t current_activity;

// --- DECLARACIONES EXTERNAS PARA SETTINGS Y TTW ---
extern void init_settings_screens();
extern bool settings_back_router();
extern lv_obj_t * scr_settings_main;
extern lv_obj_t * scr_settings_clock;
extern lv_obj_t * scr_screensaver;
extern lv_obj_t * scr_vision_menu;
extern volatile bool kin_tilt_detected;
extern volatile uint8_t ttw_mode;

// PROPIEDAD ABSOLUTA DEL HARDWARE
SemaphoreHandle_t lvgl_mutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL; // <--- AGREGA ESTA LÍNEA
SdFs sd;
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// PUNTEROS GRAFICOS
lv_obj_t * map_screen_ptr = NULL;
lv_obj_t * scr_selector = NULL;

// RELOJ Y SUBTITULO EN RACK DERECHO SUPERIOR
static lv_obj_t * lbl_hub_clock = NULL;
static lv_obj_t * lbl_hub_sub = NULL;

// MATRIZ DE RADIOS (0=BLE, 1=WIFI, 2=LORA, 3=AP)
static lv_obj_t * btn_rad[4] = {NULL, NULL, NULL, NULL};
static const char * rad_names[4] = {"BLE", "WIFI", "LORA", "AP"};
static const uint32_t rad_colors[4] = {0x00E5FF, 0x00FF41, 0xFF005C, 0xFFB300};
static bool ap_blink_phase = false;

// SLIDERS TACTICOS
static lv_obj_t * slider_vbus = NULL;
static lv_obj_t * slider_batt = NULL;

// ---------------------------------------------------------------------
// RACK DERECHO: PUNTEROS Y VARIABLES
// ---------------------------------------------------------------------
// Motor SPM (Orbital Tridimensional + Nucleo Neuronal)
static lv_obj_t * w_kin = NULL;
static lv_obj_t * kin_bg_circle = NULL; 
static lv_obj_t * kin_arc = NULL; 
static lv_obj_t * kin_tri_inner[3]; 
static lv_obj_t * kin_tri_median = NULL; 
static lv_point_precise_t tri_pts_0[2]; 
static lv_point_precise_t tri_pts_1[2];
static lv_point_precise_t tri_pts_2[2];
static lv_point_precise_t tri_median_pts[2]; 
static lv_obj_t * kin_dot = NULL;
static lv_obj_t * lbl_kin_data = NULL;

// Linterna Termica (Silueta Fisica)
static lv_obj_t * w_torch = NULL;
static lv_obj_t * torch_bar = NULL;
static lv_obj_t * torch_lens = NULL;
static lv_obj_t * lbl_torch_data = NULL;
static float current_soc_temp = 35.0f;
static bool thermal_throttle_active = false;
static lv_obj_t * torch_canvas = NULL; 
static lv_obj_t * torch_slider = NULL;

// PMU Fuel Cell Compacta
static lv_obj_t * w_pmu = NULL;
static lv_obj_t * box_pct = NULL;
static lv_obj_t * lbl_pmu_pct = NULL;
static lv_obj_t * line_dc_pos = NULL;
static lv_obj_t * line_dc_neg = NULL;
static lv_obj_t * line_wire = NULL;
static lv_obj_t * obj_pulse = NULL;
static lv_point_precise_t pts_pos[2];
static lv_point_precise_t pts_neg[2];
static lv_point_precise_t pts_wire[6]; 
static float pulse_pos = 0.0;

extern void init_carto_engine();
extern void build_ui();
extern void carto_loop_handler();
extern bool hardware_back_router(); 
extern void init_kin_engine(); 
extern void actualizar_marcador_gps();
volatile bool req_haptic_flash = false;

struct ModuleData {
    const char* title;
    const char* description;
    void (*exec_cb)();
    int id; 
};

void run_carto_module() { lv_scr_load(map_screen_ptr); }
void run_net_hub_module() { 
    if(scr_settings_main == NULL) {
        init_settings_screens(); 
    }
    lv_scr_load(net_screen_ptr); 
}

ModuleData mod_carto = {"CARTOGRAFIA", "", run_carto_module, 1};
ModuleData mod_net_hub = {"REDES RF", "", run_net_hub_module, 2};

// =====================================================================
// LOGICA DE EXCLUSION ESTRICTA DE RADIOS + LORA INDEPENDIENTE + BLINK AP
// =====================================================================
static void style_radio_btn(int idx, bool active, uint32_t color_hex, const char* custom_txt) {
    if (btn_rad[idx] == NULL) return;
    lv_obj_t * lbl = lv_obj_get_child(btn_rad[idx], 0);
    if (lbl && custom_txt) lv_label_set_text(lbl, custom_txt);

    if (active) {
        lv_obj_set_style_bg_color(btn_rad[idx], lv_color_hex(color_hex), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn_rad[idx], 235, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn_rad[idx], lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_shadow_color(btn_rad[idx], lv_color_hex(color_hex), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(btn_rad[idx], 10, LV_PART_MAIN);
        if (lbl) lv_obj_set_style_text_color(lbl, lv_color_black(), LV_PART_MAIN);
    } else {
        lv_obj_set_style_bg_color(btn_rad[idx], lv_color_hex(0x0B1317), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn_rad[idx], 200, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn_rad[idx], lv_color_hex(color_hex), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(btn_rad[idx], 0, LV_PART_MAIN);
        if (lbl) lv_obj_set_style_text_color(lbl, lv_color_hex(color_hex), LV_PART_MAIN);
    }
}

static void update_radio_matrix_ui() {
    // 0: BLE (Rastreador de Beacon)
    bool ble_on = net_ble_is_enabled();
    bool ble_lk = net_ble_is_linked();
    style_radio_btn(0, ble_on, ble_lk ? 0x00FF41 : rad_colors[0], ble_lk ? "BLE OK" : "BLE");

    // 1: WIFI (Escaner / Red Directa)
    bool wifi_on = net_wifi_is_connected();
    style_radio_btn(1, wifi_on, rad_colors[1], "WIFI");

    // 2: LORA (Independiente en chip SX1262)
    bool lora_on = net_lora_is_enabled();
    style_radio_btn(2, lora_on, rad_colors[2], lora_on ? "LORA ON" : "LORA");

    // 3: AP (Hotspot Auto-Enlace con parpadeo de negociacion y parpadeo de fallo SCAN!)
    if (wifi_on) {
        bool app_lk = net_hotspot_is_app_linked();
        style_radio_btn(3, true, app_lk ? 0x00FF41 : rad_colors[3], app_lk ? "AP LINK" : "AP ON");
    } else if (hotspot_bg_connecting || wifi_connecting) {
        style_radio_btn(3, ap_blink_phase, rad_colors[3], "LINK..");
    } else if (wifi_connection_failed) {
        // Parpadea entre Rojo Hostil y Ambar indicando que al tocarlo abre el escaner de redes
        uint32_t err_col = ap_blink_phase ? 0xFF003C : 0xFFB300;
        style_radio_btn(3, ap_blink_phase, err_col, "SCAN!");
    } else {
        style_radio_btn(3, false, rad_colors[3], "AP");
    }
}

static void radio_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);

    if (code == LV_EVENT_SHORT_CLICKED) {
        req_haptic_flash = true;

        if (idx == 0) { // BLE: Enciende el buscador de la beacon o lo apaga por completo
            if (net_ble_is_enabled()) {
                net_ble_set_enabled(false);
            } else {
                net_ble_set_enabled(true); // Apaga WiFi/AP internamente y arranca BLE
            }
        }
        else if (idx == 1) { // WIFI: Si esta conectado lo apaga (y vuelve a BLE); si no, abre escaner
            if (net_wifi_is_connected() || wifi_connecting || hotspot_bg_connecting) {
                net_disconnect_hotspot();
            } else {
                net_open_hotspot_scanner_from_hub();
            }
        }
        else if (idx == 2) { // LORA: Conmutacion independiente (coexiste con BLE/WIFI/AP)
            net_lora_set_enabled(!net_lora_is_enabled());
        }
        else if (idx == 3) { // AP (Hotspot): Sustituto de Autoenlazar + Redireccion a Escaner
            if (wifi_connection_failed) {
                // Estaba parpadeando SCAN! -> Lleva directo a la deteccion de redes
                net_open_hotspot_scanner_from_hub();
            } else if (net_wifi_is_connected() || wifi_connecting || hotspot_bg_connecting) {
                // Apaga Hotspot y activa automaticamente BLE
                net_disconnect_hotspot();
            } else {
                char ss[64];
                if (net_has_saved_hotspot(ss, sizeof(ss))) {
                    net_connect_saved_hotspot();
                } else {
                    // Sin red guardada -> Redirige automaticamente a la deteccion de redes
                    net_open_hotspot_scanner_from_hub();
                }
            }
        }
        update_radio_matrix_ui();
    }
    else if (code == LV_EVENT_LONG_PRESSED) {
        req_haptic_flash = true;
        if (scr_settings_main == NULL) init_settings_screens();

        if (idx == 0) {
            net_open_sub_from_hub(scr_settings_bt);
        } else if (idx == 1) {
            net_open_hotspot_scanner_from_hub();
        } else if (idx == 2) {
            lv_scr_load(net_screen_ptr);
        } else if (idx == 3) {
            net_open_sub_from_hub(scr_settings_hotspot);
        }
    }
}

static void btn_shadow_anim_cb(void * var, int32_t v) {
    lv_obj_set_style_shadow_width((lv_obj_t *)var, v, LV_PART_MAIN);
}

void hub_module_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e); 
    lv_obj_t * bg = lv_obj_get_child(btn, 0); 
    ModuleData * mod = (ModuleData *)lv_event_get_user_data(e);

    if (code == LV_EVENT_PRESSED) {
        lv_obj_set_style_bg_opa(bg, 255, LV_PART_MAIN); 
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        lv_obj_set_style_bg_opa(bg, 0, LV_PART_MAIN); 
    }
    else if (code == LV_EVENT_LONG_PRESSED) {
        req_haptic_flash = true; 
    }
    else if (code == LV_EVENT_SHORT_CLICKED) {
        if (mod->exec_cb != NULL) {
            mod->exec_cb();
        }
    }
}

lv_obj_t * create_exoned_btn(lv_obj_t * parent, const char * id_str, const char * text, ModuleData * mod_data, uint32_t color_hex) {
    lv_obj_t * btn = lv_obj_create(parent);
    lv_obj_set_size(btn, 155, 62); 
    lv_obj_set_style_bg_opa(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * bg = lv_obj_create(btn);
    lv_obj_set_size(bg, 155, 62);
    lv_obj_center(bg);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bg, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(bg, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bg, 0, LV_PART_MAIN);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * br_tl = lv_obj_create(btn);
    lv_obj_set_size(br_tl, 15, 15);
    lv_obj_align(br_tl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_opa(br_tl, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(br_tl, lv_color_hex(color_hex), LV_PART_MAIN);
    lv_obj_set_style_border_width(br_tl, 3, LV_PART_MAIN); 
    lv_obj_set_style_border_side(br_tl, (lv_border_side_t)(LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_LEFT), LV_PART_MAIN);
    lv_obj_set_style_radius(br_tl, 0, LV_PART_MAIN);
    lv_obj_clear_flag(br_tl, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * br_br = lv_obj_create(btn);
    lv_obj_set_size(br_br, 15, 15);
    lv_obj_align(br_br, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_bg_opa(br_br, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(br_br, lv_color_hex(color_hex), LV_PART_MAIN);
    lv_obj_set_style_border_width(br_br, 3, LV_PART_MAIN);
    lv_obj_set_style_border_side(br_br, (lv_border_side_t)(LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT), LV_PART_MAIN);
    lv_obj_set_style_radius(br_br, 0, LV_PART_MAIN);
    lv_obj_clear_flag(br_br, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t * lbl = lv_label_create(btn);
    char fmt[64];
    sprintf(fmt, "[%s] %s", id_str, text);
    lv_label_set_text(lbl, fmt);
    lv_obj_set_style_text_color(lbl, lv_color_hex(color_hex), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, LV_PART_MAIN); 
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_shadow_color(lbl, lv_color_hex(color_hex), LV_PART_MAIN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, lbl);
    lv_anim_set_values(&a, 0, 15); 
    lv_anim_set_time(&a, 600 + random(0, 400)); 
    lv_anim_set_playback_time(&a, 600 + random(0, 400));
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&a, btn_shadow_anim_cb);
    lv_anim_start(&a);

    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(btn, 10); 
    lv_obj_add_event_cb(btn, hub_module_event_cb, LV_EVENT_ALL, mod_data);
    
    return btn;
}

lv_obj_t * create_neon_slider(lv_obj_t * parent, const char * label_str, int y_pos, uint32_t color) {
    lv_obj_t * lbl = lv_label_create(parent);
    lv_label_set_text(lbl, label_str);
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_pos(lbl, 10, y_pos - 1); 

    lv_obj_t * slider = lv_slider_create(parent);
    lv_obj_set_size(slider, 290, 14); 
    lv_obj_set_pos(slider, 65, y_pos - 3); 
    
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x061A1E), LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);

    lv_obj_set_style_bg_color(slider, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_shadow_color(slider, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_shadow_width(slider, 8, LV_PART_INDICATOR); 

    lv_obj_set_style_bg_color(slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_hex(color), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 6, LV_PART_KNOB); 
    lv_obj_set_style_radius(slider, 3, LV_PART_KNOB); 

    lv_obj_set_ext_click_area(slider, 15); 
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    
    return slider;
}

void global_back_router() {
    extern bool is_alarm_active();
    extern void abort_alarm();
    if (is_alarm_active()) {
        abort_alarm();
        return;
    }

    if (torch_canvas != NULL) {
        lv_obj_del(torch_canvas);
        torch_canvas = NULL;
        instance.setBrightness((int)lv_slider_get_value(slider_batt));
        return; 
    }

    lv_obj_t * current_scr = lv_scr_act();
    
    if (current_scr == scr_screensaver) return; 
    
    if (current_scr == scr_selector) {
        arrancar_motor_exnd(get_current_aw_index());
        lv_scr_load(scr_screensaver);
    } else if (scr_vision_menu != NULL && current_scr == scr_vision_menu) {
        lv_scr_load(scr_selector);
    } else {
        extern bool vision_back_router();
        if (vision_back_router()) return;
        if (hardware_back_router()) return;
        if (network_back_router()) return;
        if (settings_back_router()) return;
        lv_scr_load(scr_selector);
    }
}

// =====================================================================
// RUTINAS DE LA LINTERNA TERMICA (TOP LAYER)
// =====================================================================
static void torch_slider_cb(lv_event_t * e) {
    lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
    int brt = (int)lv_slider_get_value(slider);
    instance.setBrightness(brt);
}

static void torch_gesture_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_GESTURE) {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (dir == LV_DIR_BOTTOM) {
            if (torch_canvas != NULL) {
                lv_obj_del(torch_canvas);
                torch_canvas = NULL;
                instance.setBrightness((int)lv_slider_get_value(slider_batt));
            }
        }
    }
}

static void torch_click_cb(lv_event_t * e) {
    if (torch_canvas != NULL) return; 

    torch_canvas = lv_obj_create(lv_layer_top());
    lv_obj_set_size(torch_canvas, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(torch_canvas, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(torch_canvas, 255, LV_PART_MAIN);
    lv_obj_set_style_border_width(torch_canvas, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(torch_canvas, 0, LV_PART_MAIN);
    lv_obj_clear_flag(torch_canvas, LV_OBJ_FLAG_SCROLLABLE);

    int start_brt = 255;
    if (thermal_throttle_active) {
        start_brt = 128; 
        lv_obj_t * warn_lbl = lv_label_create(torch_canvas);
        char warn_buf[32];
        sprintf(warn_buf, "THERMAL THROTTLE\n%.1f C", current_soc_temp);
        lv_label_set_text(warn_lbl, warn_buf);
        lv_obj_set_style_text_color(warn_lbl, lv_color_hex(0x999999), LV_PART_MAIN); 
        lv_obj_set_style_text_font(warn_lbl, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_align(warn_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_align(warn_lbl, LV_ALIGN_CENTER, 0, -40);
    }
    
    instance.setBrightness(start_brt);

    torch_slider = lv_slider_create(torch_canvas);
    lv_obj_set_size(torch_slider, 300, 30);
    lv_obj_align(torch_slider, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_slider_set_range(torch_slider, 10, 255);
    lv_slider_set_value(torch_slider, start_brt, LV_ANIM_OFF);
    
    lv_obj_set_style_bg_color(torch_slider, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_obj_set_style_bg_color(torch_slider, lv_color_hex(0x333333), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(torch_slider, lv_color_hex(0x000000), LV_PART_KNOB);
    lv_obj_set_style_pad_all(torch_slider, 8, LV_PART_KNOB);

    lv_obj_add_event_cb(torch_slider, torch_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(torch_canvas, torch_gesture_cb, LV_EVENT_GESTURE, NULL);
}

// ---------------------------------------------------------------------
// KERNEL SETUP
// ---------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(0, INPUT_PULLUP);

    // --- HABILITACION DE HARDWARE DMA ---
    instance.setDisplayParams(true, false);

    if (!instance.begin()) { while (1) delay(100); }
    instance.setBrightness(255);
    delay(500); 

    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(38400, SERIAL_8N1, 44, 43);

    // -----------------------------------------------------------------
    // LVGL ARRANCARA CON LOS 3.6 MB RESTANTES
    // -----------------------------------------------------------------
    beginLvglHelper(instance);
    lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    i2c_mutex = xSemaphoreCreateMutex(); // <--- AGREGA ESTA LÍNEA

    init_pmu(); 
    init_time_engine();
    init_kin_engine(); 

    if (!sd.begin(21, SD_SCK_MHZ(40))) Serial.println("[!] SD Fallo en Overclock a 40MHz.");

    init_carto_engine(); 
    
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        build_ui(); 
        map_screen_ptr = lv_scr_act(); 
        build_network_ui(); 

        scr_selector = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr_selector, lv_color_hex(0x02080A), LV_PART_MAIN);
        lv_obj_clear_flag(scr_selector, LV_OBJ_FLAG_SCROLLABLE);

        for(int x = 10; x < 400; x += 40) {
            for(int y = 15; y < 415; y += 40) {
                lv_obj_t * dot = lv_obj_create(scr_selector);
                lv_obj_set_size(dot, 2, 2);
                lv_obj_set_pos(dot, x, y);
                lv_obj_set_style_bg_color(dot, lv_color_hex(0x00E5FF), LV_PART_MAIN);
                lv_obj_set_style_bg_opa(dot, 80, LV_PART_MAIN); 
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
                lv_obj_set_style_border_width(dot, 0, LV_PART_MAIN);
            }
        }

        // 1. COLUMNA IZQUIERDA (X=8, Y=14, H=396 -> 5 botones de 62px con 18px de aire vertical)
        lv_obj_t * col_left = lv_obj_create(scr_selector);
        lv_obj_set_size(col_left, 160, 396); 
        lv_obj_set_pos(col_left, 8, 14);
        lv_obj_set_style_bg_opa(col_left, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(col_left, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(col_left, 0, LV_PART_MAIN);
        lv_obj_set_flex_flow(col_left, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(col_left, 18, LV_PART_MAIN);
        lv_obj_clear_flag(col_left, LV_OBJ_FLAG_SCROLLABLE);

        create_exoned_btn(col_left, "A01", "GNSS LINK", &mod_carto, 0x00E5FF);
        create_exoned_btn(col_left, "A02", "REDES RF", &mod_net_hub, 0x00E5FF);

        // --- BOTON V01: GESTOR DE VISION ---
        ModuleData* mod_vision = new ModuleData{"VISION", "", [](){ 
            extern void build_vision_menu();
            build_vision_menu();
            lv_scr_load(scr_vision_menu); 
        }, 3};
        create_exoned_btn(col_left, "V01", "VISION", mod_vision, 0x00FF41);

        // --- BOTON T01: RELOJ Y TIEMPO ---
        ModuleData* mod_clock = new ModuleData{"RELOJ", "", [](){ 
            if(scr_settings_clock == NULL) init_settings_screens();
            lv_scr_load(scr_settings_clock); 
        }, 4};
        create_exoned_btn(col_left, "T01", "RELOJ", mod_clock, 0xFF005C);

        // --- BOTON S01: AJUSTES ---
        ModuleData* mod_settings = new ModuleData{"AJUSTES", "", [](){ 
            if(scr_settings_main == NULL) init_settings_screens();
            lv_scr_load(scr_settings_main); 
        }, 5};
        create_exoned_btn(col_left, "S01", "AJUSTES", mod_settings, 0xFF8C00);

        // 2. RACK DERECHO (X=174, Y=14, W=228, H=396 -> 26px de aire vertical entre bloques)
        lv_obj_t * rack_right = lv_obj_create(scr_selector);
        lv_obj_set_size(rack_right, 228, 396); 
        lv_obj_set_pos(rack_right, 174, 14);
        lv_obj_set_style_bg_opa(rack_right, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(rack_right, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(rack_right, 0, LV_PART_MAIN);
        lv_obj_clear_flag(rack_right, LV_OBJ_FLAG_SCROLLABLE);

        // --- BLOQUE 1 (Y=0..64): RELOJ DIGITAL TACTICO + TELEMETRIA ---
        lv_obj_t * box_clk = lv_obj_create(rack_right);
        lv_obj_set_size(box_clk, 224, 64);
        lv_obj_set_pos(box_clk, 0, 0);
        lv_obj_set_style_bg_color(box_clk, lv_color_hex(0x051418), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(box_clk, 200, LV_PART_MAIN);
        lv_obj_set_style_border_color(box_clk, lv_color_hex(0x00E5FF), LV_PART_MAIN);
        lv_obj_set_style_border_width(box_clk, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(box_clk, 4, LV_PART_MAIN);
        lv_obj_set_style_pad_all(box_clk, 0, LV_PART_MAIN);
        lv_obj_clear_flag(box_clk, LV_OBJ_FLAG_SCROLLABLE);

        lbl_hub_clock = lv_label_create(box_clk);
        lv_label_set_text(lbl_hub_clock, "12:00:00");
        lv_obj_set_style_text_color(lbl_hub_clock, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_hub_clock, &lv_font_montserrat_28, LV_PART_MAIN);
        lv_obj_align(lbl_hub_clock, LV_ALIGN_TOP_MID, 0, 5);

        lbl_hub_sub = lv_label_create(box_clk);
        lv_label_set_text(lbl_hub_sub, "RTC | SATS:00 | BLE");
        lv_obj_set_style_text_color(lbl_hub_sub, lv_color_hex(0x00FF41), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_hub_sub, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align(lbl_hub_sub, LV_ALIGN_BOTTOM_MID, 0, -5);

        // --- BLOQUE 2 (Y=90..194): MATRIZ 2x2 DE RADIOS (26px debajo del reloj) ---
        for (int i = 0; i < 4; i++) {
            int col = i % 2;
            int row = i / 2;
            btn_rad[i] = lv_obj_create(rack_right);
            lv_obj_set_size(btn_rad[i], 105, 44);
            lv_obj_set_pos(btn_rad[i], col * 119, 90 + (row * 60));
            lv_obj_set_style_border_width(btn_rad[i], 2, LV_PART_MAIN);
            lv_obj_set_style_radius(btn_rad[i], 4, LV_PART_MAIN);
            lv_obj_set_style_pad_all(btn_rad[i], 0, LV_PART_MAIN);
            lv_obj_clear_flag(btn_rad[i], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(btn_rad[i], LV_OBJ_FLAG_CLICKABLE);

            lv_obj_t * lbl_r = lv_label_create(btn_rad[i]);
            lv_label_set_text(lbl_r, rad_names[i]);
            lv_obj_set_style_text_font(lbl_r, &lv_font_montserrat_16, LV_PART_MAIN);
            lv_obj_center(lbl_r);

            lv_obj_add_event_cb(btn_rad[i], radio_btn_event_cb, LV_EVENT_ALL, (void*)(uintptr_t)i);
        }
        update_radio_matrix_ui();

        // --- BLOQUE 3 (Y=220..330): RADAR ORBITAL SPM + PMU COMPACTA (26px debajo de radios) ---
        w_kin = lv_obj_create(rack_right);
        lv_obj_set_size(w_kin, 110, 110); 
        lv_obj_set_pos(w_kin, 0, 220); 
        lv_obj_set_style_bg_opa(w_kin, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(w_kin, 0, LV_PART_MAIN); 
        lv_obj_set_style_pad_all(w_kin, 0, LV_PART_MAIN); 
        lv_obj_clear_flag(w_kin, LV_OBJ_FLAG_SCROLLABLE);

        kin_bg_circle = lv_arc_create(w_kin);
        lv_obj_set_size(kin_bg_circle, 110, 110);
        lv_obj_set_pos(kin_bg_circle, 0, 0); 
        lv_obj_set_style_pad_all(kin_bg_circle, 0, LV_PART_MAIN); 
        lv_obj_set_style_border_width(kin_bg_circle, 0, LV_PART_MAIN); 
        lv_obj_set_style_arc_width(kin_bg_circle, 4, LV_PART_MAIN);
        lv_obj_set_style_arc_color(kin_bg_circle, lv_color_hex(0x333333), LV_PART_MAIN);
        lv_arc_set_bg_angles(kin_bg_circle, 0, 360);
        lv_obj_remove_style(kin_bg_circle, NULL, LV_PART_INDICATOR);
        lv_obj_remove_style(kin_bg_circle, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(kin_bg_circle, LV_OBJ_FLAG_CLICKABLE);

        kin_arc = lv_arc_create(w_kin);
        lv_obj_set_size(kin_arc, 110, 110);
        lv_obj_set_pos(kin_arc, 0, 0); 
        lv_obj_set_style_pad_all(kin_arc, 0, LV_PART_MAIN); 
        lv_obj_set_style_border_width(kin_arc, 0, LV_PART_MAIN);
        lv_obj_set_style_arc_width(kin_arc, 4, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(kin_arc, 0, LV_PART_INDICATOR); 
        lv_obj_remove_style(kin_arc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(kin_arc, LV_OBJ_FLAG_CLICKABLE);

        // Nucleo Neuronal
        tri_pts_0[0] = {33, 68}; tri_pts_0[1] = {55, 29}; 
        tri_pts_1[0] = {55, 29}; tri_pts_1[1] = {77, 68}; 
        tri_pts_2[0] = {77, 68}; tri_pts_2[1] = {33, 68}; 

        kin_tri_inner[0] = lv_line_create(w_kin); lv_line_set_points(kin_tri_inner[0], tri_pts_0, 2);
        kin_tri_inner[1] = lv_line_create(w_kin); lv_line_set_points(kin_tri_inner[1], tri_pts_1, 2);
        kin_tri_inner[2] = lv_line_create(w_kin); lv_line_set_points(kin_tri_inner[2], tri_pts_2, 2);

        for(int i = 0; i < 3; i++) {
            lv_obj_set_style_line_width(kin_tri_inner[i], 4, LV_PART_MAIN);
            lv_obj_set_style_line_color(kin_tri_inner[i], lv_color_hex(0x333333), LV_PART_MAIN);
        }

        // Vector de impacto visual (Bisectriz)
        tri_median_pts[0] = {55, 29}; 
        tri_median_pts[1] = {55, 68}; 
        kin_tri_median = lv_line_create(w_kin);
        lv_line_set_points(kin_tri_median, tri_median_pts, 2);
        lv_obj_set_style_line_width(kin_tri_median, 4, LV_PART_MAIN);
        lv_obj_set_style_line_color(kin_tri_median, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_add_flag(kin_tri_median, LV_OBJ_FLAG_HIDDEN); 

        kin_dot = lv_obj_create(w_kin);
        lv_obj_set_size(kin_dot, 12, 12);
        lv_obj_set_style_bg_color(kin_dot, lv_color_hex(0x555555), LV_PART_MAIN); 
        lv_obj_set_style_radius(kin_dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_border_width(kin_dot, 0, LV_PART_MAIN);

        // Texto Inercial Orbital
        lbl_kin_data = lv_label_create(w_kin);
        lv_obj_set_size(lbl_kin_data, 110, 20);
        lv_obj_set_pos(lbl_kin_data, 0, 78); 
        lv_obj_set_style_text_align(lbl_kin_data, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_text(lbl_kin_data, "QUIETO");
        lv_obj_set_style_text_color(lbl_kin_data, lv_color_hex(0x555555), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_kin_data, &lv_font_montserrat_12, LV_PART_MAIN);

        // --- PMU FUEL CELL A LA DERECHA DEL RADAR (X=122, Y=245 -> 12px de aire horizontal) ---
        w_pmu = lv_obj_create(rack_right);
        lv_obj_set_size(w_pmu, 102, 65); 
        lv_obj_set_pos(w_pmu, 122, 245); 
        lv_obj_set_style_bg_opa(w_pmu, 0, LV_PART_MAIN); 
        lv_obj_set_style_border_width(w_pmu, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(w_pmu, 0, LV_PART_MAIN);
        lv_obj_clear_flag(w_pmu, LV_OBJ_FLAG_SCROLLABLE);

        line_dc_pos = lv_line_create(w_pmu);
        pts_pos[0] = {2, 25}; pts_pos[1] = {14, 25}; 
        lv_line_set_points(line_dc_pos, pts_pos, 2);

        line_dc_neg = lv_line_create(w_pmu);
        pts_neg[0] = {4, 45}; pts_neg[1] = {12, 45}; 
        lv_line_set_points(line_dc_neg, pts_neg, 2);

        line_wire = lv_line_create(w_pmu);
        pts_wire[0] = {8, 25}; 
        pts_wire[1] = {8, 5};  
        pts_wire[2] = {98, 5};  
        pts_wire[3] = {98, 55}; 
        pts_wire[4] = {8, 55};  
        pts_wire[5] = {8, 45};  
        lv_line_set_points(line_wire, pts_wire, 6);

        box_pct = lv_obj_create(w_pmu);
        lv_obj_set_size(box_pct, 72, 26);
        lv_obj_set_pos(box_pct, 18, 17); 
        lv_obj_set_style_bg_color(box_pct, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(box_pct, 200, LV_PART_MAIN);
        lv_obj_set_style_border_width(box_pct, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(box_pct, 0, LV_PART_MAIN);
        lv_obj_clear_flag(box_pct, LV_OBJ_FLAG_SCROLLABLE);

        lbl_pmu_pct = lv_label_create(box_pct);
        lv_label_set_text(lbl_pmu_pct, "[82%]");
        lv_obj_set_style_text_color(lbl_pmu_pct, lv_color_hex(0x00FF41), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_pmu_pct, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_center(lbl_pmu_pct); 

        obj_pulse = lv_obj_create(w_pmu);
        lv_obj_set_style_radius(obj_pulse, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(obj_pulse, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(obj_pulse, 8, LV_PART_MAIN);

        // --- BLOQUE 4 (Y=356..396): LINTERNA TACTICA FISICA (26px debajo del radar) ---
        w_torch = lv_obj_create(rack_right);
        lv_obj_set_size(w_torch, 224, 40); 
        lv_obj_set_pos(w_torch, 0, 356); 
        lv_obj_set_style_bg_opa(w_torch, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(w_torch, 0, LV_PART_MAIN); 
        lv_obj_set_style_pad_all(w_torch, 0, LV_PART_MAIN);
        lv_obj_clear_flag(w_torch, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(w_torch, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(w_torch, 10); 
        lv_obj_add_event_cb(w_torch, torch_click_cb, LV_EVENT_CLICKED, NULL);

        // 1. Tapa trasera (Tailcap)
        lv_obj_t * t_tail = lv_obj_create(w_torch);
        lv_obj_set_size(t_tail, 8, 16);
        lv_obj_set_pos(t_tail, 0, 12);
        lv_obj_set_style_bg_color(t_tail, lv_color_hex(0x2B2B2B), LV_PART_MAIN);
        lv_obj_set_style_border_color(t_tail, lv_color_hex(0x666666), LV_PART_MAIN);
        lv_obj_set_style_border_width(t_tail, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(t_tail, 2, LV_PART_MAIN);
        lv_obj_clear_flag(t_tail, LV_OBJ_FLAG_CLICKABLE);

        // 2. Cuerpo cilindrico (Barrel) con interruptor tactico
        lv_obj_t * t_body = lv_obj_create(w_torch);
        lv_obj_set_size(t_body, 38, 20);
        lv_obj_set_pos(t_body, 7, 10);
        lv_obj_set_style_bg_color(t_body, lv_color_hex(0x444444), LV_PART_MAIN);
        lv_obj_set_style_border_color(t_body, lv_color_hex(0x888888), LV_PART_MAIN);
        lv_obj_set_style_border_width(t_body, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(t_body, 2, LV_PART_MAIN);
        lv_obj_clear_flag(t_body, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t * t_sw = lv_obj_create(t_body);
        lv_obj_set_size(t_sw, 10, 6);
        lv_obj_center(t_sw);
        lv_obj_set_style_bg_color(t_sw, lv_color_hex(0x00E5FF), LV_PART_MAIN);
        lv_obj_set_style_border_width(t_sw, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(t_sw, 2, LV_PART_MAIN);
        lv_obj_clear_flag(t_sw, LV_OBJ_FLAG_CLICKABLE);

        // 3. Cabezal reflector ensanchado (Bezel) + Lente LED
        lv_obj_t * t_head = lv_obj_create(w_torch);
        lv_obj_set_size(t_head, 16, 32);
        lv_obj_set_pos(t_head, 44, 4);
        lv_obj_set_style_bg_color(t_head, lv_color_hex(0x777777), LV_PART_MAIN);
        lv_obj_set_style_border_color(t_head, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
        lv_obj_set_style_border_width(t_head, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(t_head, 3, LV_PART_MAIN);
        lv_obj_clear_flag(t_head, LV_OBJ_FLAG_CLICKABLE);

        torch_lens = lv_obj_create(w_torch);
        lv_obj_set_size(torch_lens, 4, 28);
        lv_obj_set_pos(torch_lens, 60, 6);
        lv_obj_set_style_bg_color(torch_lens, lv_color_hex(0xFFEE00), LV_PART_MAIN);
        lv_obj_set_style_shadow_color(torch_lens, lv_color_hex(0xFFEE00), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(torch_lens, 10, LV_PART_MAIN);
        lv_obj_set_style_border_width(torch_lens, 0, LV_PART_MAIN);
        lv_obj_clear_flag(torch_lens, LV_OBJ_FLAG_CLICKABLE);

        // 4. Haz de Luz (Barra Termica que emerge del lente)
        torch_bar = lv_bar_create(w_torch);
        lv_obj_set_size(torch_bar, 156, 24);
        lv_obj_set_pos(torch_bar, 66, 8);
        lv_obj_set_style_bg_color(torch_bar, lv_color_hex(0x111A1E), LV_PART_MAIN);
        lv_obj_set_style_border_color(torch_bar, lv_color_hex(0x333333), LV_PART_MAIN);
        lv_obj_set_style_border_width(torch_bar, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(torch_bar, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(torch_bar, 2, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(torch_bar, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
        lv_bar_set_range(torch_bar, 25, 65); 
        lv_bar_set_value(torch_bar, 35, LV_ANIM_OFF);
        lv_obj_clear_flag(torch_bar, LV_OBJ_FLAG_CLICKABLE);

        lbl_torch_data = lv_label_create(w_torch);
        lv_label_set_text(lbl_torch_data, "LINTERNA | 35.0 C");
        lv_obj_set_style_text_color(lbl_torch_data, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_torch_data, &lv_font_montserrat_12, LV_PART_MAIN); 
        lv_obj_set_pos(lbl_torch_data, 74, 13);
        lv_obj_clear_flag(lbl_torch_data, LV_OBJ_FLAG_CLICKABLE);

        // 3. ZONA DE SLIDERS TACTICOS (Y=422..496 -> 26px debajo de la linterna)
        lv_obj_t * panel_sliders = lv_obj_create(scr_selector);
        lv_obj_set_size(panel_sliders, 390, 74);
        lv_obj_align(panel_sliders, LV_ALIGN_BOTTOM_MID, 0, -6); 
        lv_obj_set_style_bg_opa(panel_sliders, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(panel_sliders, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(panel_sliders, 0, LV_PART_MAIN); 
        lv_obj_clear_flag(panel_sliders, LV_OBJ_FLAG_SCROLLABLE);

        slider_vbus = create_neon_slider(panel_sliders, "VAR", 8, 0x00E5FF); 
        lv_slider_set_range(slider_vbus, 0, 100);
        lv_slider_set_value(slider_vbus, 100, LV_ANIM_OFF);

        slider_batt = create_neon_slider(panel_sliders, "BRT", 46, 0xFFB300); 
        lv_slider_set_range(slider_batt, 10, 255); 
        lv_slider_set_value(slider_batt, 255, LV_ANIM_OFF);

        // =============================================================
        // MOTOR DE ESTADOS ASINCRONOS (FPU HARDWARE ENABLED)
        // =============================================================
        lv_timer_create([](lv_timer_t * t) {
            if (lv_scr_act() != scr_selector) return;

            static uint32_t last_time = millis();
            uint32_t current_time = millis();
            float dt = (current_time - last_time) / 1000.0f;
            last_time = current_time;

            if (torch_canvas == NULL) { 
                int brt_val = lv_slider_get_value(slider_batt);
                static int last_brt = -1;
                if (brt_val != last_brt) {
                    instance.setBrightness(brt_val);
                    last_brt = brt_val;
                }
            }

            // --- MOTOR SPM (ORBITAL RELATIVO Y NUCLEO NEURONAL) ---
            float actual_bpm = kin_actual_bpm;
            float target_bpm = 110.0f;
            uint32_t core_color = 0x333333;
            bool valid_activity = false;

            if (current_activity == 1) { 
                target_bpm = kin_target_walk; 
                core_color = 0x00E5FF; 
                valid_activity = true;
            } else if (current_activity == 2) { 
                target_bpm = kin_target_run; 
                core_color = 0xFF00EA; 
                valid_activity = true;
            } else {
                valid_activity = false;
            }

            // --- LATCH DE POSTURA (BISECTRIZ VISUAL) ---
            static uint32_t flash_ms = 0;
            extern volatile uint8_t kin_event_latch;
            if (kin_event_latch > 0) {
                flash_ms = millis();
                kin_event_latch = 0;
            }
            bool is_flashing = (millis() - flash_ms < 300); 
            
            if (is_flashing && valid_activity) {
                lv_obj_clear_flag(kin_tri_median, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(kin_tri_median, LV_OBJ_FLAG_HIDDEN);
            }

            // Actualizacion del Triangulo Interior
            static uint32_t last_core_color = 0;
            if (core_color != last_core_color) {
                for(int i = 0; i < 3; i++) {
                    lv_obj_set_style_line_color(kin_tri_inner[i], lv_color_hex(core_color), LV_PART_MAIN);
                }
                last_core_color = core_color;
            }
            
            static bool is_active = false;
            static uint32_t last_motion_ms = 0;

            if (actual_bpm >= 10.0f && valid_activity) {
                last_motion_ms = millis();
                is_active = true;
            } else {
                if (is_active && (millis() - last_motion_ms > 15000)) is_active = false;
            }

            int tri_state = -1; 
            if (!is_active) tri_state = -1; 
            else {
                if (actual_bpm < target_bpm - 5.0f) tri_state = 0; 
                else if (actual_bpm > target_bpm + 5.0f) tri_state = 2; 
                else tri_state = 1; 
            }

            // Sincronizador Maestro de Renderizado OSD
            static int last_tri_state = -99;
            if (tri_state != last_tri_state) { 
                if (tri_state == -1) {
                    lv_obj_add_flag(kin_dot, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(kin_arc, LV_OBJ_FLAG_HIDDEN); 
                    lv_label_set_text(lbl_kin_data, "QUIETO");
                    lv_obj_set_style_text_color(lbl_kin_data, lv_color_hex(0x555555), LV_PART_MAIN);
                } 
                else {
                    lv_obj_clear_flag(kin_dot, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(kin_arc, LV_OBJ_FLAG_HIDDEN);
                    
                    uint32_t ui_color;
                    const char* state_str;
                    
                    if (tri_state == 0) { 
                        ui_color = 0xFFB300; 
                        state_str = "FATIGA"; 
                    } else if (tri_state == 1) { 
                        ui_color = 0x00FF41; 
                        state_str = "ESTABLE"; 
                    } else { 
                        ui_color = 0xFF005C; 
                        state_str = "DESGASTE"; 
                    }
                    
                    lv_obj_set_style_arc_color(kin_arc, lv_color_hex(ui_color), LV_PART_MAIN);
                    lv_obj_set_style_bg_color(kin_dot, lv_color_hex(ui_color), LV_PART_MAIN);
                    
                    lv_label_set_text(lbl_kin_data, state_str);
                    lv_obj_set_style_text_color(lbl_kin_data, lv_color_hex(ui_color), LV_PART_MAIN);
                }
                last_tri_state = tri_state;
            }

            // Fisica Orbital
            if (tri_state != -1) {
                static float arc_angle = 0.0f;
                static float current_dot_offset = 0.0f;
                
                float spd; 
                float target_offset; 

                if (tri_state == 0) { 
                    spd = 90.0f; 
                    target_offset = -45.0f; 
                } 
                else if (tri_state == 1) { 
                    spd = 180.0f; 
                    target_offset = 0.0f; 
                } 
                else { 
                    spd = 270.0f; 
                    target_offset = 45.0f; 
                } 

                arc_angle += spd * dt;
                if (arc_angle >= 360.0f) arc_angle -= 360.0f;

                current_dot_offset += (target_offset - current_dot_offset) * 4.0f * dt;

                uint16_t a_start = (uint16_t)fmodf(arc_angle - 30.0f + 360.0f, 360.0f);
                uint16_t a_end   = (uint16_t)fmodf(arc_angle + 30.0f + 360.0f, 360.0f);
                lv_arc_set_bg_angles(kin_arc, a_start, a_end);

                float dot_angle = arc_angle + current_dot_offset;
                float rad = dot_angle * M_PI / 180.0f;
                
                int dx = 55 + (int)(53.0f * cosf(rad));
                int dy = 55 + (int)(53.0f * sinf(rad));
                lv_obj_set_pos(kin_dot, dx - 6, dy - 6); 
            }

            // --- AUDITORIA TERMICA Y LINTERNA ---
            static uint32_t thermal_update = 0;
            if (millis() - thermal_update > 1000) {
                current_soc_temp = temperatureRead(); 

                lv_bar_set_value(torch_bar, (int)current_soc_temp, LV_ANIM_OFF);

                char t_buf[32];
                if (current_soc_temp < 46.0f) {
                    thermal_throttle_active = false;
                    lv_obj_set_style_bg_color(torch_bar, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
                    sprintf(t_buf, "LINTERNA | %.1f C", current_soc_temp);
                } else if (current_soc_temp < 55.0f) {
                    thermal_throttle_active = false;
                    lv_obj_set_style_bg_color(torch_bar, lv_color_hex(0xFFB300), LV_PART_INDICATOR);
                    sprintf(t_buf, "CAUTION | %.1f C", current_soc_temp);
                } else {
                    thermal_throttle_active = true;
                    lv_obj_set_style_bg_color(torch_bar, lv_color_hex(0xFF005C), LV_PART_INDICATOR);
                    sprintf(t_buf, "WARN | %.1f C", current_soc_temp);
                }
                lv_label_set_text(lbl_torch_data, t_buf);
                
                thermal_update = millis();
            }

            // --- TELEMETRIA PMU ---
            extern volatile int real_batt_pct;
            extern volatile bool is_charging;
            int batt_pct = real_batt_pct; 
            uint32_t wire_color = 0x00FF41; 
            uint32_t pulse_color = 0xFFFFFF;
            float pulse_speed = 6.0f; 
            int pulse_length = 14;
            int wire_thickness = 4; 

            if (is_charging) {
                wire_color = 0x00E5FF; 
                pulse_speed = 10.0f;   
            } else if (batt_pct <= 20) {
                wire_color = 0xFF005C; 
                pulse_speed = 3.0f;    
            } else if (batt_pct <= 50) {
                wire_color = 0xFFB300; 
            }

            static uint32_t last_wire_color = 0;
            if (wire_color != last_wire_color) { 
                lv_obj_set_style_border_color(box_pct, lv_color_hex(wire_color), LV_PART_MAIN);
                lv_obj_set_style_text_color(lbl_pmu_pct, lv_color_hex(wire_color), LV_PART_MAIN);
                lv_obj_set_style_line_color(line_wire, lv_color_hex(wire_color), LV_PART_MAIN);
                lv_obj_set_style_line_color(line_dc_pos, lv_color_hex(wire_color), LV_PART_MAIN);
                lv_obj_set_style_line_color(line_dc_neg, lv_color_hex(wire_color), LV_PART_MAIN);
                
                lv_obj_set_style_line_width(line_wire, wire_thickness, LV_PART_MAIN);
                lv_obj_set_style_line_width(line_dc_pos, 3, LV_PART_MAIN);
                lv_obj_set_style_line_width(line_dc_neg, 4, LV_PART_MAIN);
                last_wire_color = wire_color;
            }

            static uint32_t pmu_update = 0;
            if(millis() - pmu_update > 200) { 
                char pct_buf[16];
                sprintf(pct_buf, "[%02d%%]", batt_pct);
                lv_label_set_text(lbl_pmu_pct, pct_buf);
                pmu_update = millis();
            }

            static uint32_t last_pulse_color = 0;
            if (pulse_color != last_pulse_color) { 
                lv_obj_set_style_bg_color(obj_pulse, lv_color_hex(pulse_color), LV_PART_MAIN);
                lv_obj_set_style_shadow_color(obj_pulse, lv_color_hex(pulse_color), LV_PART_MAIN);
                last_pulse_color = pulse_color;
            }

            static int pulse_pause = 0;
            if (pulse_pause > 0) {
                pulse_pause--; 
            } else {
                if (is_charging) {
                    pulse_pos -= pulse_speed; 
                    if (pulse_pos < 0) {
                        pulse_pos = 260.0f;
                        pulse_pause = 8; 
                    }
                } else {
                    pulse_pos += pulse_speed; 
                    if (pulse_pos > 260.0f) {
                        pulse_pos = 0.0f;
                        pulse_pause = 8; 
                    }
                }
            }

            int px = 0, py = 0, pw = 0, ph = 0;
            int p = (int)pulse_pos;
            int thick = wire_thickness + 2; 

            if (p < 20) { 
                px = 8 - (thick / 2);
                py = 25 - p;
                if (py < 5) py = 5;
                pw = thick;
                ph = 10; 
            } else if (p < 110) { 
                px = 8 + (p - 20);
                if (px + pulse_length > 98) px = 98 - pulse_length;
                py = 5 - (thick / 2);
                pw = pulse_length;
                ph = thick; 
            } else if (p < 160) { 
                px = 98 - (thick / 2);
                py = 5 + (p - 110);
                if (py + 12 > 55) py = 43;
                pw = thick;
                ph = 12; 
            } else if (p < 250) { 
                px = 98 - (p - 160) - pulse_length;
                if (px < 8) px = 8;
                py = 55 - (thick / 2);
                pw = pulse_length;
                ph = thick; 
            } else { 
                px = 8 - (thick / 2);
                py = 45;
                pw = thick;
                ph = 10; 
            }

            static int last_px = -1, last_py = -1, last_pw = -1, last_ph = -1;
            if (px != last_px || py != last_py || pw != last_pw || ph != last_ph) { 
                lv_obj_set_pos(obj_pulse, px, py);
                lv_obj_set_size(obj_pulse, pw, ph);
                last_px = px; last_py = py;
                last_pw = pw; last_ph = ph;
            }

            // --- SINCRONIZADOR DE RADIOS Y RELOJ SUPERIOR DEL HUB (250ms) ---
            static uint32_t last_osd_update = 0;
            if (millis() - last_osd_update > 250) {
                ap_blink_phase = !ap_blink_phase;
                update_radio_matrix_ui();

                char clk_buf[16];
                char sub_buf[48];
                extern volatile uint8_t sys_hour, sys_min, sys_sec;
                extern volatile bool time_is_gnss;
                int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

                const char* r24_str = "OFF";
                if (net_wifi_is_connected()) r24_str = "AP";
                else if (hotspot_bg_connecting || wifi_connecting) r24_str = "LINK";
                else if (wifi_connection_failed) r24_str = "SCAN!";
                else if (net_ble_is_enabled()) r24_str = "BLE";

                sprintf(clk_buf, "%02d:%02d:%02d", sys_hour, sys_min, sys_sec);
                sprintf(sub_buf, "%s | SATS:%02d | %s%s", 
                        time_is_gnss ? "GNSS" : "RTC", 
                        sats, 
                        r24_str,
                        net_lora_is_enabled() ? "+LR" : "");
                
                lv_label_set_text(lbl_hub_clock, clk_buf);
                lv_label_set_text(lbl_hub_sub, sub_buf);
                last_osd_update = millis();
            }

        }, 20, NULL);

        // --- AUDITORIA DE HARDWARE ABSOLUTA ---
        Serial.println("\n[!] ========================================");
        Serial.println("[!] REPORTE DE MEMORIA DEL KERNEL");
        Serial.printf("[!] SRAM Interna Total: %d bytes\n", ESP.getHeapSize());
        Serial.printf("[!] SRAM Interna Libre: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("[!] PSRAM Externa Total: %d bytes\n", ESP.getPsramSize());
        Serial.printf("[!] PSRAM Externa Libre: %d bytes\n", ESP.getFreePsram());
        Serial.println("[!] ========================================\n");

        // ---------------------------------------------------------
        // IGNICION DEL MOTOR DE PROTECTOR DE PANTALLA
        // ---------------------------------------------------------
        init_background_engine();
        
        // RESCATE NVS: Extraer fondo configurado desde la Flash
        Preferences prefs;
        prefs.begin("exoned", true); 
        uint32_t saved_aw = prefs.getUInt("aw_idx", 1); 
        prefs.end();
        
        Serial.printf("[KERNEL] Ignicion OSD. Fondo montado desde NVS: Indice %d\n", saved_aw);
        set_current_aw_index(saved_aw);
        arrancar_motor_exnd(saved_aw);
        lv_scr_load(scr_screensaver); 

        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

void loop() {
    static bool last_boot_state = HIGH;
    bool current_boot_state = digitalRead(0);
    
    if (last_boot_state == HIGH && current_boot_state == LOW) {
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            global_back_router();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    }
    last_boot_state = current_boot_state;

    static uint32_t backend_timer = 0;
    if (millis() - backend_timer > 40) { 
        carto_loop_handler();
        network_loop_handler(); 
        pmu_loop_handler(); 
        time_loop_handler(); 
        kin_loop_handler(); 
        
        // --- LOOP DE TRANSFERENCIA ASINCRONA ---
        background_transfer_loop(); 

        // ==============================================================
        // MOTOR TILT-TO-WAKE HIBRIDO + GESTION ENERGETICA CONTEXTUAL
        // ==============================================================
        static uint32_t ttw_wake_timer = millis();
        static bool screen_is_awake = true;
        static bool ttw_armed = false;
        static uint8_t pose_confirm_ticks = 0;
        static uint8_t out_of_pose_ticks = 0;

        static int16_t hist_gy = 0, hist_gz = 0;
        static uint32_t hist_timer = 0;
        static uint32_t last_bosch_tilt_ms = 0;
        static bool was_in_protector = false;

        if (slider_batt != NULL) {
            int master_brt = (int)lv_slider_get_value(slider_batt);
            bool force_tactical = (ttw_mode == 2 && master_brt <= 10);
            uint8_t effective_mode = force_tactical ? 1 : ttw_mode;

            extern lv_obj_t * scr_screensaver;
            extern bool is_transferring_bg();
            bool in_protector = (scr_screensaver != NULL && lv_scr_act() == scr_screensaver);

            // Reseteo balistico: Si acabamos de entrar al protector, reinicia el timer a 0
            if (in_protector && !was_in_protector) {
                ttw_wake_timer = millis();
            }
            was_in_protector = in_protector;

            // 1. Mantener despierto por toque fisico, linterna o transferencia VRAM
            if (lv_disp_get_inactive_time(NULL) < 200 || torch_canvas != NULL || is_transferring_bg()) {
                ttw_wake_timer = millis();
                if (!screen_is_awake) {
                    screen_is_awake = true;
                    instance.setBrightness(master_brt);
                }
            }

            extern volatile int16_t kin_raw_gx, kin_raw_gy, kin_raw_gz;

            if (kin_tilt_detected) {
                kin_tilt_detected = false;
                last_bosch_tilt_ms = millis();
            }

            // 2. CAJAS ESPACIALES Y DERIVADA CINEMATICA
            bool wrist_x_ok  = (abs(kin_raw_gx) < 2600);
            bool pose_normal = (kin_raw_gz > 2600) && (kin_raw_gy < -1200 && kin_raw_gy > -3600);
            bool pose_bed    = (kin_raw_gz < -700 && kin_raw_gz > -2600) && (kin_raw_gy < -2400);
            bool in_reading_zone = wrist_x_ok && (pose_normal || pose_bed);

            int32_t delta_motion = abs(kin_raw_gy - hist_gy) + abs(kin_raw_gz - hist_gz);
            bool bosch_confirmed = (millis() - last_bosch_tilt_ms < 500);
            bool intentional_flick = bosch_confirmed || (delta_motion > 900);

            if (millis() - hist_timer > 250 && pose_confirm_ticks == 0) {
                hist_gy = kin_raw_gy;
                hist_gz = kin_raw_gz;
                hist_timer = millis();
            }

            // 3. MAQUINA DE ESTADOS TTW
            if (!in_reading_zone) {
                out_of_pose_ticks++;
                if (out_of_pose_ticks > 4) {
                    ttw_armed = true;
                    pose_confirm_ticks = 0;
                }
            } else {
                out_of_pose_ticks = 0;
                if (ttw_armed && intentional_flick) {
                    pose_confirm_ticks++;
                    uint8_t req_ticks = bosch_confirmed ? 1 : 2;
                    if (pose_confirm_ticks >= req_ticks) {
                        ttw_armed = false;
                        pose_confirm_ticks = 0;
                        ttw_wake_timer = millis();
                        if (!screen_is_awake) {
                            screen_is_awake = true;
                            instance.setBrightness(master_brt);
                        }
                    }
                }
            }

            // 4. TIMEOUT CONTEXTUAL
            uint32_t active_timeout_ms = in_protector ? 4000UL : 300000UL;

            if (screen_is_awake && (millis() - ttw_wake_timer > active_timeout_ms)) {
                screen_is_awake = false;
                
                int dimmed_brt = (master_brt * 50) / 100;
                if (dimmed_brt < 20) {
                    dimmed_brt = 20; 
                }

                if (in_protector) {
                    if (effective_mode == 1) {
                        instance.setBrightness(0);
                    } else {
                        instance.setBrightness(dimmed_brt);
                    }
                } else {
                    instance.setBrightness(dimmed_brt);
                }
            }
        }

        // LOG DE ALTA FRECUENCIA (100ms) CON CLASIFICADOR DE POLARIDAD EN VIVO
        static uint32_t neural_debug_timer = 0;
        if (millis() - neural_debug_timer > 100) {
            const char* pose_tag = "[TRANSICION]";
            if (kin_raw_gy < -800 && kin_raw_gz > 2000) {
                pose_tag = "[POSE Y-NEG (ACTIVA)]";
            } else if (kin_raw_gy > 800 && kin_raw_gz > 2000) {
                pose_tag = "[POSE Y-POS (OPUESTA)]";
            } else if (kin_raw_gz < 1000) {
                pose_tag = "[PANTALLA VERTICAL/ABAJO]";
            }

            Serial.printf("GRAV -> X:%6d | Y:%6d | Z:%6d  %s\n", 
                          kin_raw_gx, kin_raw_gy, kin_raw_gz, pose_tag);
            neural_debug_timer = millis();
        }

        if (req_haptic_flash) {
            Wire.beginTransmission(0x5A); Wire.write(0x04); Wire.write(1); Wire.endTransmission();
            Wire.beginTransmission(0x5A); Wire.write(0x05); Wire.write(0); Wire.endTransmission();
            Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(1); Wire.endTransmission();
            req_haptic_flash = false;
        }

        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            actualizar_marcador_gps();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
        backend_timer = millis();
    }

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        lv_timer_handler();
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    
    delay(5); 
}