#include "Settings.h"
#include "NetworkEngine.h"
#include "BackgroundEngine.h"
#include "esp_heap_caps.h"
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <Preferences.h>

lv_obj_t * scr_settings_main = NULL;
lv_obj_t * scr_settings_clock = NULL;
lv_obj_t * scr_settings_alarms = NULL;

extern lv_obj_t * scr_selector; 
extern lv_obj_t * net_screen_ptr;
extern SemaphoreHandle_t lvgl_mutex;

// --- ESTRUCTURAS DE ALARMA ---
#define MAX_ALARMS 5
typedef struct {
    uint8_t hour;
    uint8_t min;
    uint8_t snooze;
    bool active;
    bool used;
} sys_alarm_t;

static sys_alarm_t sys_alarms[MAX_ALARMS];
static bool alarms_initialized = false;

static lv_obj_t * modal_alarm = NULL;
static uint8_t editing_alarm_idx = 0;
static lv_obj_t * roller_h = NULL;
static lv_obj_t * roller_m = NULL;
static lv_obj_t * roller_s = NULL;

static void build_alarms_list(lv_obj_t * parent);

// =========================================================
// MÓDULO DE PERSISTENCIA EN MEMORIA NVS
// =========================================================
static void save_alarms_to_nvs() {
    Preferences prefs;
    prefs.begin("exoned_alarms", false);
    prefs.putBytes("alarm_data", sys_alarms, sizeof(sys_alarms));
    prefs.end();
    Serial.println("[NVS] Estado de alarmas sincronizado en Flash.");
}

static void load_alarms_from_nvs_if_needed() {
    if (!alarms_initialized) {
        Preferences prefs;
        prefs.begin("exoned_alarms", true);
        size_t len = prefs.getBytesLength("alarm_data");
        if (len == sizeof(sys_alarms)) {
            prefs.getBytes("alarm_data", sys_alarms, sizeof(sys_alarms));
            Serial.println("[NVS] Matriz de alarmas restaurada.");
        } else {
            for(int i=0; i<MAX_ALARMS; i++) { sys_alarms[i].used = false; sys_alarms[i].active = false; }
            Serial.println("[NVS] Inicializando matriz de alarmas en blanco.");
        }
        prefs.end();
        alarms_initialized = true;
    }
}

// =========================================================
// INICIALIZACIÓN DE AJUSTES GENERALES
// =========================================================
// =========================================================
// INICIALIZACIÓN DE AJUSTES GENERALES
// =========================================================
void init_settings_screens() {
    load_alarms_from_nvs_if_needed();

    // [!] CANDADO: Evita recrear las pantallas en RAM si ya existen
    if (scr_settings_main != NULL && scr_settings_clock != NULL) return;

    // --- MENÚ PRINCIPAL ---
    scr_settings_main = lv_obj_create(NULL);
    // ... el resto sigue igual
    lv_obj_set_style_bg_color(scr_settings_main, lv_color_black(), 0);
    lv_obj_t * list_main = lv_list_create(scr_settings_main);
    lv_obj_set_size(list_main, 390, 482);
    lv_obj_center(list_main);
    lv_obj_set_style_bg_color(list_main, lv_color_black(), 0);
    lv_obj_set_style_border_width(list_main, 0, 0);

    lv_list_add_text(list_main, "AJUSTES GENERALES");
    
    // [X] Botón de Reloj extirpado de Ajustes Generales
    lv_obj_t * btn_back_main = lv_list_add_btn(list_main, LV_SYMBOL_LEFT, "Volver al Menu");
    lv_obj_add_event_cb(btn_back_main, [](lv_event_t * e) { lv_scr_load(scr_selector); }, LV_EVENT_CLICKED, NULL);

    // --- MENÚ RELOJ (Ahora App Independiente) ---
    scr_settings_clock = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_settings_clock, lv_color_black(), 0);
    lv_obj_t * list_clock = lv_list_create(scr_settings_clock);
    lv_obj_set_size(list_clock, 390, 482);
    lv_obj_center(list_clock);
    lv_obj_set_style_bg_color(list_clock, lv_color_black(), 0);
    lv_obj_set_style_border_width(list_clock, 0, 0);

    lv_list_add_text(list_clock, "RELOJ Y TIEMPO");
    lv_obj_t * btn_alarmas = lv_list_add_btn(list_clock, LV_SYMBOL_WARNING, "Alarmas Hostiles");
    lv_obj_add_event_cb(btn_alarmas, [](lv_event_t * e) {
        if(scr_settings_alarms != NULL) lv_obj_del(scr_settings_alarms);
        scr_settings_alarms = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr_settings_alarms, lv_color_black(), 0);
        build_alarms_list(scr_settings_alarms);
        lv_scr_load(scr_settings_alarms);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t * btn_back_clk = lv_list_add_btn(list_clock, LV_SYMBOL_LEFT, "Volver al Menu");
    lv_obj_add_event_cb(btn_back_clk, [](lv_event_t * e) { lv_scr_load(scr_selector); }, LV_EVENT_CLICKED, NULL);
}

// =========================================================
// MÓDULO DE ALARMAS HOSTILES
// =========================================================
static void close_alarm_modal_cb(lv_event_t * e) {
    if(modal_alarm) {
        lv_obj_add_flag(modal_alarm, LV_OBJ_FLAG_HIDDEN);
        lv_obj_del_async(modal_alarm);
        modal_alarm = NULL;
    }
}

// [!] INYECCIÓN: Tarea suicida para limpiar la RAM antes de redibujar
static void rebuild_alarms_task(lv_timer_t * t) {
    if (scr_settings_alarms != NULL) {
        lv_obj_clean(scr_settings_alarms);
        build_alarms_list(scr_settings_alarms);
    }
    lv_timer_del(t); 
}

static void save_alarm_cb(lv_event_t * e) {
    sys_alarms[editing_alarm_idx].hour = lv_roller_get_selected(roller_h);
    sys_alarms[editing_alarm_idx].min = lv_roller_get_selected(roller_m);
    
    uint16_t s_idx = lv_roller_get_selected(roller_s);
    uint8_t s_vals[] = {5, 10, 15, 20, 25, 30};
    sys_alarms[editing_alarm_idx].snooze = s_vals[s_idx];
    
    sys_alarms[editing_alarm_idx].used = true;
    sys_alarms[editing_alarm_idx].active = true;

    save_alarms_to_nvs(); 
    close_alarm_modal_cb(NULL);
    
    // [!] BLINDAJE GME: Delegamos la reconstrucción 50ms al futuro
    lv_timer_create(rebuild_alarms_task, 50, NULL);
}

static void delete_alarm_cb(lv_event_t * e) {
    sys_alarms[editing_alarm_idx].used = false;
    sys_alarms[editing_alarm_idx].active = false;
    
    save_alarms_to_nvs(); 
    close_alarm_modal_cb(NULL);
    
    // [!] BLINDAJE GME: Delegamos la reconstrucción 50ms al futuro
    lv_timer_create(rebuild_alarms_task, 50, NULL);
}

static void build_alarm_modal(uint8_t idx) {
    if (modal_alarm != NULL) return;
    editing_alarm_idx = idx;
    bool is_new = !sys_alarms[editing_alarm_idx].used;

    modal_alarm = lv_obj_create(lv_layer_top());
    lv_obj_set_size(modal_alarm, 390, 482);
    lv_obj_set_style_bg_color(modal_alarm, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal_alarm, 230, 0);
    lv_obj_set_style_border_width(modal_alarm, 0, 0);
    lv_obj_clear_flag(modal_alarm, LV_OBJ_FLAG_SCROLLABLE); // [!] ANIQUILACIÓN DEL EFECTO ELÁSTICO
    lv_obj_center(modal_alarm);

    lv_obj_t * popup = lv_obj_create(modal_alarm);
    lv_obj_set_size(popup, 360, 420);
    lv_obj_center(popup);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(popup, lv_color_hex(0xFF005C), 0);
    lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * title = lv_label_create(popup);
    lv_label_set_text(title, is_new ? "NUEVA ALARMA" : "EDITAR ALARMA");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    char opts_h[96];
    int offset_h = 0;
    for(int i = 0; i < 24; i++) {
        offset_h += snprintf(opts_h + offset_h, sizeof(opts_h) - offset_h, "%02d\n", i);
    }
    opts_h[offset_h - 1] = '\0';

    char opts_m[192];
    int offset_m = 0;
    for(int i = 0; i < 60; i++) {
        offset_m += snprintf(opts_m + offset_m, sizeof(opts_m) - offset_m, "%02d\n", i);
    }
    opts_m[offset_m - 1] = '\0';

    roller_h = lv_roller_create(popup);
    lv_roller_set_options(roller_h, opts_h, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_h, 3);
    lv_obj_align(roller_h, LV_ALIGN_CENTER, -80, -30);
    if(!is_new) lv_roller_set_selected(roller_h, sys_alarms[editing_alarm_idx].hour, LV_ANIM_OFF);

    roller_m = lv_roller_create(popup);
    lv_roller_set_options(roller_m, opts_m, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller_m, 3);
    lv_obj_align(roller_m, LV_ALIGN_CENTER, 0, -30);
    if(!is_new) lv_roller_set_selected(roller_m, sys_alarms[editing_alarm_idx].min, LV_ANIM_OFF);

    roller_s = lv_roller_create(popup);
    lv_roller_set_options(roller_s, "05m\n10m\n15m\n20m\n25m\n30m", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(roller_s, 3);
    lv_obj_align(roller_s, LV_ALIGN_CENTER, 80, -30);
    
    if(!is_new) {
        uint8_t snz = sys_alarms[editing_alarm_idx].snooze;
        uint16_t s_idx = (snz >= 5 && snz <= 30 && snz % 5 == 0) ? (snz / 5) - 1 : 1;
        lv_roller_set_selected(roller_s, s_idx, LV_ANIM_OFF);
    } else {
        lv_roller_set_selected(roller_s, 1, LV_ANIM_OFF);
    }

    lv_obj_t * btn_save = lv_btn_create(popup);
    lv_obj_set_size(btn_save, 140, 50);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btn_save, lv_color_hex(0x00FF41), 0);
    lv_obj_t * lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "GUARDAR");
    lv_obj_set_style_text_color(lbl_save, lv_color_black(), 0);
    lv_obj_center(lbl_save);
    lv_obj_add_event_cb(btn_save, save_alarm_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_close = lv_btn_create(popup);
    lv_obj_set_size(btn_close, 140, 50);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x333333), 0);
    lv_obj_t * lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, "CANCELAR");
    lv_obj_center(lbl_close);
    lv_obj_add_event_cb(btn_close, close_alarm_modal_cb, LV_EVENT_CLICKED, NULL);

    if(!is_new) {
        lv_obj_t * btn_del = lv_btn_create(popup);
        lv_obj_set_size(btn_del, 300, 50);
        lv_obj_align(btn_del, LV_ALIGN_BOTTOM_MID, 0, -70);
        lv_obj_set_style_bg_color(btn_del, lv_color_hex(0xFF005C), 0);
        lv_obj_t * lbl_del = lv_label_create(btn_del);
        lv_label_set_text(lbl_del, "ELIMINAR ALARMA");
        lv_obj_center(lbl_del);
        lv_obj_add_event_cb(btn_del, delete_alarm_cb, LV_EVENT_CLICKED, NULL);
    }
}

static void open_alarm_modal_cb(lv_event_t * e) {
    build_alarm_modal((uint8_t)(uintptr_t)lv_event_get_user_data(e));
}

static void build_alarms_list(lv_obj_t * parent) {
    lv_obj_t * title = lv_label_create(parent);
    lv_label_set_text(title, "ALARMAS HOSTILES");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF005C), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * list = lv_list_create(parent);
    lv_obj_set_size(list, 390, 360);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -60);
    lv_obj_set_style_bg_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);

    uint8_t used_count = 0;

    for(int i = 0; i < MAX_ALARMS; i++) {
        if(!sys_alarms[i].used) continue;
        used_count++;

        lv_obj_t * item_container = lv_obj_create(list);
        lv_obj_set_size(item_container, 350, 80);
        lv_obj_set_style_bg_color(item_container, lv_color_hex(0x111111), 0);
        lv_obj_set_style_border_color(item_container, lv_color_hex(0x333333), 0);
        lv_obj_clear_flag(item_container, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * lbl_time = lv_label_create(item_container);
        lv_label_set_text_fmt(lbl_time, "%02d:%02d", sys_alarms[i].hour, sys_alarms[i].min);
        lv_obj_set_style_text_color(lbl_time, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_20, 0);
        lv_obj_align(lbl_time, LV_ALIGN_LEFT_MID, 10, -10);

        lv_obj_t * lbl_snz = lv_label_create(item_container);
        lv_label_set_text_fmt(lbl_snz, "Snooze: %dm", sys_alarms[i].snooze);
        lv_obj_set_style_text_color(lbl_snz, lv_color_hex(0xAAAAAA), 0);
        lv_obj_align(lbl_snz, LV_ALIGN_LEFT_MID, 10, 15);

        lv_obj_t * sw = lv_switch_create(item_container);
        lv_obj_align(sw, LV_ALIGN_RIGHT_MID, -70, 0);
        if(sys_alarms[i].active) lv_obj_add_state(sw, LV_STATE_CHECKED);
        lv_obj_add_event_cb(sw, [](lv_event_t * e) {
            uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
            lv_obj_t * target = (lv_obj_t *)lv_event_get_target(e);
            sys_alarms[idx].active = lv_obj_has_state(target, LV_STATE_CHECKED);
            save_alarms_to_nvs(); // Guardar el encendido/apagado en Flash
        }, LV_EVENT_VALUE_CHANGED, (void*)(uintptr_t)i);

        lv_obj_t * btn_edit = lv_btn_create(item_container);
        lv_obj_set_size(btn_edit, 50, 50);
        lv_obj_align(btn_edit, LV_ALIGN_RIGHT_MID, -5, 0);
        lv_obj_set_style_bg_color(btn_edit, lv_color_hex(0x333333), 0);
        lv_obj_t * lbl_edit = lv_label_create(btn_edit);
        lv_label_set_text(lbl_edit, LV_SYMBOL_EDIT);
        lv_obj_center(lbl_edit);
        lv_obj_add_event_cb(btn_edit, open_alarm_modal_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }

    if (used_count < MAX_ALARMS) {
        lv_obj_t * btn_add = lv_list_add_btn(list, LV_SYMBOL_PLUS, "NUEVA ALARMA");
        lv_obj_set_style_bg_color(btn_add, lv_color_hex(0x111111), 0);
        lv_obj_set_style_text_color(btn_add, lv_color_hex(0x00E5FF), 0);
        lv_obj_add_event_cb(btn_add, [](lv_event_t * e) {
            for(int i=0; i<MAX_ALARMS; i++) {
                if(!sys_alarms[i].used) {
                    build_alarm_modal(i);
                    return;
                }
            }
        }, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t * btn_back = lv_btn_create(parent);
    lv_obj_set_size(btn_back, 370, 50);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "VOLVER");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) { lv_scr_load(scr_settings_clock); }, LV_EVENT_CLICKED, NULL);
}

int check_alarm_trigger(uint8_t current_h, uint8_t current_m) {
    load_alarms_from_nvs_if_needed(); // <-- AGREGAR ESTA LÍNEA AQUÍ

    for(int i = 0; i < MAX_ALARMS; i++) {
        if(sys_alarms[i].used && sys_alarms[i].active) {
            if(sys_alarms[i].hour == current_h && sys_alarms[i].min == current_m) {
                sys_alarms[i].active = false; 
                save_alarms_to_nvs(); // La alarma se apaga a sí misma tras detonar
                return sys_alarms[i].snooze; 
            }
        }
    }
    return -1;
}

bool settings_back_router() {
    lv_obj_t * cur = lv_scr_act();
    if (cur == scr_settings_alarms) {
        if (modal_alarm != NULL) { close_alarm_modal_cb(NULL); return true; }
        lv_scr_load(scr_settings_clock);
        return true;
    }
    else if (cur == scr_settings_clock) {
        lv_scr_load(scr_selector);
        return true;
    }
    else if (cur == scr_settings_main) {
        lv_scr_load(scr_selector);
        return true;
    }
    return false;
}