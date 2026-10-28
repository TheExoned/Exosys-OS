#include "BackgroundEngine.h"
#include "TimeEngine.h"  
#include "PowerEngine.h" 
#include <SdFat.h>           
#include "esp_heap_caps.h"
#include <FFat.h> 
#include <Preferences.h>
#include <Wire.h>
#include <WiFi.h>

lv_obj_t * scr_screensaver = NULL;
lv_obj_t * scr_vision_menu = NULL;
lv_obj_t * scr_vault_menu = NULL; 

extern SdFs sd;             
static fs::File internal_exnd; 

static lv_img_dsc_t exnd_img_dsc; 
static lv_obj_t* img_screensaver = NULL; 
static uint8_t* local_decoder_buf = NULL; 
static lv_timer_t* exnd_timer = NULL;

static lv_obj_t * lbl_bg_time_shadow = NULL;
static lv_obj_t * lbl_bg_time = NULL;
static lv_obj_t * lbl_bg_batt = NULL;
static lv_obj_t * bar_bg_progress = NULL;
static lv_obj_t * lbl_bg_progress = NULL;

static uint32_t exnd_total_frames = 0;
static uint32_t exnd_current_frame = 0;
static uint8_t exnd_target_fps = 0;
static uint16_t exnd_target_w = 0, exnd_target_h = 0;

static uint8_t total_aw_count = 0;
static uint8_t current_aw_index = 1;

extern lv_obj_t * scr_selector;
extern uint8_t * vault_arena;       
extern void invalidate_map_cache(); 
extern volatile bool pausa_lectura_sd; 
extern SemaphoreHandle_t lvgl_mutex;
extern volatile bool ghost_miner_parked;
extern TaskHandle_t ghost_miner_handle;

static bool async_transfer_active = false;
static uint8_t async_target_index = 0;
static FsFile async_sd_source;
static fs::File async_flash_dest;
static uint32_t async_total_bytes = 0;
static uint32_t async_copied_bytes = 0;
static uint8_t* async_transfer_buf = NULL;

static volatile uint8_t pending_fast_reload_idx = 0;

static char custom_aw_names[100][32];
static bool names_initialized = false;
static lv_obj_t * modal_bg = NULL;
static uint16_t* preview_psram_buf = NULL;
static lv_obj_t * kb_rename = NULL;
static lv_obj_t * ta_rename = NULL;
static uint8_t editing_index = 0;
static lv_obj_t * active_label_ptr = NULL;

volatile uint8_t ttw_mode = 1; 
static lv_obj_t * btn_ttw_giro = NULL;
static lv_obj_t * btn_ttw_pasivo = NULL;

static bool clock_edit_mode = false;
static uint32_t clock_press_start = 0;
static bool is_exiting_bg = false;

static lv_obj_t * hud_clock_edit = NULL;
static lv_obj_t * slider_color = NULL; 
static lv_obj_t * slider_scale = NULL;

static int16_t cur_clk_x = 145;
static int16_t cur_clk_y = 20;
static uint8_t cur_clk_color_idx = 0;
static uint8_t cur_clk_size_idx = 2;
static bool force_time_label_refresh = true;

static const lv_font_t * clk_sizes[8] = {
    &lv_font_montserrat_16, &lv_font_montserrat_20, &lv_font_montserrat_24, &lv_font_montserrat_28,
    &lv_font_montserrat_32, &lv_font_montserrat_36, &lv_font_montserrat_40, &lv_font_montserrat_48  
};

static const uint32_t clk_palette[37] = {
    0xFFFFFF, 0xFF0000, 0xFF2A00, 0xFF5500, 0xFF8000, 0xFFAA00, 0xFFD500,
    0xFFFF00, 0xD4FF00, 0xAAFF00, 0x80FF00, 0x55FF00, 0x2AFF00, 0x00FF00, 
    0x00FF2A, 0x00FF55, 0x00FF80, 0x00FFAA, 0x00FFD4, 0x00FFFF, 0x00D4FF, 
    0x00AAFF, 0x0080FF, 0x0055FF, 0x002AFF, 0x0000FF, 0x2A00FF, 0x5500FF, 
    0x8000FF, 0xAA00FF, 0xD400FF, 0xFF00FF, 0xFF00D4, 0xFF00AA, 0xFF0080, 
    0xFF0055, 0xFF002A
};

static void park_sd_bus() {
    pausa_lectura_sd = true;
    if (ghost_miner_handle != NULL) xTaskNotifyGive(ghost_miner_handle);
    uint32_t t0 = millis();
    while (!ghost_miner_parked && (millis() - t0 < 1500)) delay(2);
}

static void unpark_sd_bus() {
    pausa_lectura_sd = false;
    if (ghost_miner_handle != NULL) xTaskNotifyGive(ghost_miner_handle);
}

void secure_spi_lock() { park_sd_bus(); }
void release_spi_lock() { unpark_sd_bus(); }

void apply_clock_profile(uint8_t aw_index) {
    if (lbl_bg_time == NULL || lbl_bg_time_shadow == NULL) return;

    Preferences p_clk;
    p_clk.begin("exoned_clk", true);
    char key_pos[16], key_sty[16], key_scl[16];
    sprintf(key_pos, "clk_p_%d", aw_index);
    sprintf(key_sty, "clk_s_%d", aw_index);
    sprintf(key_scl, "clk_c_%d", aw_index);

    if (p_clk.isKey(key_pos)) {
        uint32_t pos = p_clk.getUInt(key_pos, (145UL << 16) | 20UL);
        cur_clk_x = (int16_t)(pos >> 16);
        cur_clk_y = (int16_t)(pos & 0xFFFF);
    } else {
        cur_clk_x = 145;
        cur_clk_y = 20;
    }

    if (p_clk.isKey(key_sty)) {
        uint32_t hex = p_clk.getUInt(key_sty, 0xFFFFFF);
        cur_clk_color_idx = 0;
        for (uint8_t i = 0; i < 37; i++) {
            if (clk_palette[i] == hex) {
                cur_clk_color_idx = i;
                break;
            }
        }
        lv_obj_set_style_text_color(lbl_bg_time, lv_color_hex(hex), 0);
    } else {
        cur_clk_color_idx = 0;
        lv_obj_set_style_text_color(lbl_bg_time, lv_color_hex(clk_palette[0]), 0);
    }

    cur_clk_size_idx = (uint8_t)p_clk.getUInt(key_scl, 2);
    if (cur_clk_size_idx > 7) cur_clk_size_idx = 2;
    p_clk.end();

    lv_obj_set_align(lbl_bg_time_shadow, LV_ALIGN_TOP_LEFT);
    lv_obj_set_align(lbl_bg_time, LV_ALIGN_TOP_LEFT);
    lv_obj_set_pos(lbl_bg_time_shadow, cur_clk_x + 2, cur_clk_y + 2);
    lv_obj_set_pos(lbl_bg_time, cur_clk_x, cur_clk_y);
    lv_obj_set_style_transform_scale(lbl_bg_time_shadow, 256, 0);
    lv_obj_set_style_transform_scale(lbl_bg_time, 256, 0);
    lv_obj_set_style_text_font(lbl_bg_time_shadow, clk_sizes[cur_clk_size_idx], 0);
    lv_obj_set_style_text_font(lbl_bg_time, clk_sizes[cur_clk_size_idx], 0);

    force_time_label_refresh = true;
}

static void save_current_clock_profile() {
    Preferences p_clk;
    p_clk.begin("exoned_clk", false);
    char key_pos[16], key_sty[16], key_scl[16];
    sprintf(key_pos, "clk_p_%d", current_aw_index);
    sprintf(key_sty, "clk_s_%d", current_aw_index);
    sprintf(key_scl, "clk_c_%d", current_aw_index);

    uint32_t pos = ((uint16_t)cur_clk_x << 16) | ((uint16_t)cur_clk_y & 0xFFFF);
    uint32_t hex = clk_palette[cur_clk_color_idx];

    p_clk.putUInt(key_pos, pos);
    p_clk.putUInt(key_sty, hex);
    p_clk.putUInt(key_scl, cur_clk_size_idx); 
    p_clk.end();
}

static void build_clock_edit_hud() {
    if (hud_clock_edit != NULL) return;
    if (lbl_bg_batt) lv_obj_add_flag(lbl_bg_batt, LV_OBJ_FLAG_HIDDEN);

    slider_color = lv_slider_create(scr_screensaver);
    lv_obj_set_size(slider_color, 280, 25);
    lv_obj_align(slider_color, LV_ALIGN_BOTTOM_MID, 0, -80);
    lv_slider_set_range(slider_color, 0, 36); 
    lv_slider_set_value(slider_color, cur_clk_color_idx, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider_color, lv_color_hex(clk_palette[cur_clk_color_idx]), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider_color, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_add_flag(slider_color, LV_OBJ_FLAG_HIDDEN);
    
    lv_obj_add_event_cb(slider_color, [](lv_event_t * e) {
        lv_obj_t * sl = (lv_obj_t *)lv_event_get_target(e);
        cur_clk_color_idx = lv_slider_get_value(sl);
        uint32_t hex = clk_palette[cur_clk_color_idx];
        lv_obj_set_style_bg_color(sl, lv_color_hex(hex), LV_PART_INDICATOR);
        lv_obj_set_style_text_color(lbl_bg_time, lv_color_hex(hex), 0);
    }, LV_EVENT_VALUE_CHANGED, NULL);

    slider_scale = lv_slider_create(scr_screensaver);
    lv_obj_set_size(slider_scale, 280, 25);
    lv_obj_align(slider_scale, LV_ALIGN_BOTTOM_MID, 0, -80);
    lv_slider_set_range(slider_scale, 0, 7); 
    lv_slider_set_value(slider_scale, cur_clk_size_idx, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider_scale, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider_scale, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_add_flag(slider_scale, LV_OBJ_FLAG_HIDDEN);
    
    lv_obj_add_event_cb(slider_scale, [](lv_event_t * e) {
        lv_obj_t * sl = (lv_obj_t *)lv_event_get_target(e);
        cur_clk_size_idx = (uint8_t)lv_slider_get_value(sl);
        lv_obj_set_style_text_font(lbl_bg_time_shadow, clk_sizes[cur_clk_size_idx], 0);
        lv_obj_set_style_text_font(lbl_bg_time, clk_sizes[cur_clk_size_idx], 0);
    }, LV_EVENT_VALUE_CHANGED, NULL);

    hud_clock_edit = lv_obj_create(scr_screensaver);
    lv_obj_set_size(hud_clock_edit, 390, 64);
    lv_obj_align(hud_clock_edit, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_color(hud_clock_edit, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_bg_opa(hud_clock_edit, 240, 0);
    lv_obj_set_style_border_color(hud_clock_edit, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_border_width(hud_clock_edit, 2, 0);
    lv_obj_set_style_pad_all(hud_clock_edit, 4, 0);
    lv_obj_clear_flag(hud_clock_edit, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * btn_size = lv_btn_create(hud_clock_edit);
    lv_obj_set_size(btn_size, 120, 48);
    lv_obj_align(btn_size, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(btn_size, lv_color_hex(0x222222), 0);
    lv_obj_t * lbl_s = lv_label_create(btn_size);
    lv_label_set_text(lbl_s, "ESCALA");
    lv_obj_center(lbl_s);
    lv_obj_add_event_cb(btn_size, [](lv_event_t * e) {
        lv_obj_add_flag(slider_color, LV_OBJ_FLAG_HIDDEN); 
        if(lv_obj_has_flag(slider_scale, LV_OBJ_FLAG_HIDDEN)) lv_obj_clear_flag(slider_scale, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(slider_scale, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_col = lv_btn_create(hud_clock_edit);
    lv_obj_set_size(btn_col, 120, 48);
    lv_obj_align(btn_col, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(btn_col, lv_color_hex(0x222222), 0);
    lv_obj_t * lbl_c = lv_label_create(btn_col);
    lv_label_set_text(lbl_c, "COLOR");
    lv_obj_center(lbl_c);
    lv_obj_add_event_cb(btn_col, [](lv_event_t * e) {
        lv_obj_add_flag(slider_scale, LV_OBJ_FLAG_HIDDEN); 
        if(lv_obj_has_flag(slider_color, LV_OBJ_FLAG_HIDDEN)) lv_obj_clear_flag(slider_color, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(slider_color, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_ok = lv_btn_create(hud_clock_edit);
    lv_obj_set_size(btn_ok, 120, 48);
    lv_obj_align(btn_ok, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(btn_ok, lv_color_hex(0x00FF41), 0);
    lv_obj_t * lbl_ok = lv_label_create(btn_ok);
    lv_label_set_text(lbl_ok, "OK");
    lv_obj_set_style_text_color(lbl_ok, lv_color_black(), 0);
    lv_obj_center(lbl_ok);
    lv_obj_add_event_cb(btn_ok, [](lv_event_t * e) {
        save_current_clock_profile();
        if (slider_scale) { lv_obj_del(slider_scale); slider_scale = NULL; }
        if (slider_color) { lv_obj_del(slider_color); slider_color = NULL; }
        if (hud_clock_edit) { lv_obj_del(hud_clock_edit); hud_clock_edit = NULL; }
        
        if (lbl_bg_batt) lv_obj_clear_flag(lbl_bg_batt, LV_OBJ_FLAG_HIDDEN);
        clock_edit_mode = false;
        if (exnd_timer) lv_timer_resume(exnd_timer);
    }, LV_EVENT_CLICKED, NULL);
}

static void exit_to_map_task(lv_timer_t * t) {
    apagar_motor_exnd();
    invalidate_map_cache();
    lv_scr_load(scr_selector);
    lv_timer_del(t);
}

static void exit_to_alarms_task(lv_timer_t * t) {
    extern lv_obj_t * scr_settings_clock;
    extern void init_settings_screens();
    
    if (scr_settings_clock == NULL) {
        init_settings_screens();
    }

    apagar_motor_exnd();
    invalidate_map_cache();
    lv_scr_load(scr_settings_clock);
    lv_timer_del(t);
}

static void clock_touch_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);

    if (code == LV_EVENT_PRESSED) {
        clock_press_start = millis();
    }
    else if (code == LV_EVENT_PRESSING) {
        if (!clock_edit_mode && clock_press_start > 0 && (millis() - clock_press_start >= 3000)) {
            clock_edit_mode = true;
            clock_press_start = 0;
            if (exnd_timer) lv_timer_pause(exnd_timer);

            Wire.beginTransmission(0x5A); Wire.write(0x04); Wire.write(47); Wire.endTransmission();
            Wire.beginTransmission(0x5A); Wire.write(0x05); Wire.write(0); Wire.endTransmission();
            Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(1); Wire.endTransmission();

            build_clock_edit_hud();
        }

        if (clock_edit_mode) {
            lv_indev_t * indev = lv_indev_get_act();
            if (indev == NULL) return;

            lv_point_t vect;
            lv_indev_get_vect(indev, &vect);

            if (vect.x != 0 || vect.y != 0) {
                cur_clk_x += vect.x;
                cur_clk_y += vect.y;

                if (cur_clk_x < -120) cur_clk_x = -120;
                if (cur_clk_x > 360) cur_clk_x = 360;
                if (cur_clk_y < -40) cur_clk_y = -40;
                if (cur_clk_y > 450) cur_clk_y = 450;

                lv_obj_set_pos(obj, cur_clk_x, cur_clk_y);
                lv_obj_set_pos(lbl_bg_time_shadow, cur_clk_x + 2, cur_clk_y + 2);
            }
        }
    }
    else if (code == LV_EVENT_SHORT_CLICKED) {
        if (!clock_edit_mode) {
            if (is_exiting_bg) return;
            is_exiting_bg = true;
            lv_timer_create(exit_to_alarms_task, 50, NULL);
        }
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        clock_press_start = 0;
    }
}

static void screensaver_touch_cb(lv_event_t * e) {
    if (e != NULL) {
        lv_obj_t * target = (lv_obj_t *)lv_event_get_target(e);
        if (target != scr_screensaver) return; 
    }

    static uint32_t last_touch = 0;
    if (millis() - last_touch < 500) return;
    last_touch = millis();

    if (async_transfer_active || clock_edit_mode) return;
    
    if (is_exiting_bg) return; 
    is_exiting_bg = true;

    lv_timer_create(exit_to_map_task, 50, NULL);
}

void scan_animated_wallpapers() {
    secure_spi_lock(); 
    total_aw_count = 0;
    char path[64];
    for (int i = 1; i <= 99; i++) {
        sprintf(path, "/WALLPAPERS/ANIMATED/AW%d.exnd", i);
        if (sd.exists(path)) total_aw_count++; 
        else break; 
    }
    release_spi_lock(); 
}

static void close_modal_cb(lv_event_t * e) {
    if (modal_bg != NULL) {
        lv_obj_del(modal_bg);
        modal_bg = NULL;
    }
    if (preview_psram_buf != NULL) {
        heap_caps_free(preview_psram_buf);
        preview_psram_buf = NULL;
    }
}

static void apply_wallpaper_cb(lv_event_t * e) {
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    close_modal_cb(NULL);

    Preferences prefs;
    prefs.begin("exoned", false);
    prefs.putUInt("aw_idx", idx);
    prefs.end();

    lv_scr_load(scr_screensaver);
    iniciar_transferencia_async(idx);
}

static void wp_item_click_cb(lv_event_t * e) {
    if (modal_bg != NULL) return;
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    uint16_t w = 0, h = 0;
    uint32_t frames = 0, size_bytes = 0;
    uint8_t fps = 0;
    
    park_sd_bus();
    bool info_ok = get_aw_info_from_sd(idx, &w, &h, &frames, &fps, &size_bytes);
    if (!info_ok) {
        unpark_sd_bus();
        return;
    }

    modal_bg = lv_obj_create(scr_vault_menu);
    lv_obj_set_size(modal_bg, 390, 482);
    lv_obj_set_style_bg_color(modal_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal_bg, 230, 0);
    lv_obj_set_style_border_width(modal_bg, 0, 0);
    lv_obj_clear_flag(modal_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(modal_bg);

    lv_obj_t * popup = lv_obj_create(modal_bg);
    lv_obj_set_size(popup, 340, 420);
    lv_obj_center(popup);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(popup, lv_color_hex(0x00E5FF), 0);
    lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * title = lv_label_create(popup);
    lv_label_set_text(title, custom_aw_names[idx]);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    if (preview_psram_buf != NULL) {
        heap_caps_free(preview_psram_buf);
        preview_psram_buf = NULL;
    }
    preview_psram_buf = (uint16_t*)heap_caps_malloc(w * h * 2, MALLOC_CAP_SPIRAM);
    if (preview_psram_buf) {
        if (read_aw_preview_from_sd(idx, preview_psram_buf, w * h * 2)) {
            lv_obj_t * canvas = lv_canvas_create(popup);
            lv_canvas_set_buffer(canvas, preview_psram_buf, w, h, LV_COLOR_FORMAT_RGB565);
            lv_img_set_zoom(canvas, 102); 
            lv_obj_align(canvas, LV_ALIGN_TOP_MID, 0, 40);
        }
    }
    unpark_sd_bus();

    lv_obj_t * info = lv_label_create(popup);
    char infobuf[128];
    sprintf(infobuf, "Geometria: %dx%d\nTasa: %d FPS\nLongitud: %u cuadros\nPeso: %.2f MB", w, h, fps, frames, (float)size_bytes / (1024*1024));
    lv_label_set_text(info, infobuf);
    lv_obj_set_style_text_color(info, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(info, LV_ALIGN_BOTTOM_LEFT, 10, -80);

    lv_obj_t * btn_apply = lv_btn_create(popup);
    lv_obj_set_size(btn_apply, 140, 50);
    lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btn_apply, lv_color_hex(0x00E5FF), 0);
    lv_obj_t * lbl_apply = lv_label_create(btn_apply);
    lv_label_set_text(lbl_apply, "APLICAR");
    lv_obj_set_style_text_color(lbl_apply, lv_color_black(), 0);
    lv_obj_center(lbl_apply);
    lv_obj_add_event_cb(btn_apply, apply_wallpaper_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)idx);

    lv_obj_t * btn_close = lv_btn_create(popup);
    lv_obj_set_size(btn_close, 140, 50);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, "CANCELAR");
    lv_obj_center(lbl_close);
    lv_obj_add_event_cb(btn_close, close_modal_cb, LV_EVENT_CLICKED, NULL);
}

static void kb_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if(code == LV_EVENT_READY && ta_rename != NULL && active_label_ptr != NULL) {
            strncpy(custom_aw_names[editing_index], lv_textarea_get_text(ta_rename), 31);
            custom_aw_names[editing_index][31] = '\0';
            lv_label_set_text(active_label_ptr, custom_aw_names[editing_index]);

            Preferences prefs_bg;
            prefs_bg.begin("exoned", false);
            char key[16];
            sprintf(key, "aw_name_%d", editing_index);
            prefs_bg.putString(key, custom_aw_names[editing_index]);
            prefs_bg.end();
        }
        if (kb_rename) {
            lv_obj_add_flag(kb_rename, LV_OBJ_FLAG_HIDDEN);
            lv_obj_del_async(kb_rename);
            kb_rename = NULL;
        }
        if (ta_rename) {
            lv_obj_add_flag(ta_rename, LV_OBJ_FLAG_HIDDEN);
            lv_obj_del_async(ta_rename);
            ta_rename = NULL;
        }
    }
}

static void edit_name_cb(lv_event_t * e) {
    if (kb_rename != NULL || ta_rename != NULL) return;
    editing_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
    active_label_ptr = lv_obj_get_child(lv_obj_get_parent(btn), 0); 

    ta_rename = lv_textarea_create(scr_vault_menu);
    lv_obj_set_size(ta_rename, 370, 50);
    lv_obj_align(ta_rename, LV_ALIGN_TOP_MID, 0, 10);
    lv_textarea_set_text(ta_rename, custom_aw_names[editing_index]);

    kb_rename = lv_keyboard_create(scr_vault_menu);
    lv_keyboard_set_textarea(kb_rename, ta_rename);
    lv_obj_add_event_cb(kb_rename, kb_event_cb, LV_EVENT_ALL, NULL);
}

void build_wallpaper_list() {
    lv_obj_t * title = lv_label_create(scr_vault_menu);
    lv_label_set_text(title, "BOVEDA DE FONDOS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t * list = lv_list_create(scr_vault_menu);
    lv_obj_set_size(list, 390, 360);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -60);
    lv_obj_set_style_bg_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);

    for(int i = 1; i <= total_aw_count; i++) {
        lv_obj_t * item_container = lv_obj_create(list);
        lv_obj_set_size(item_container, 350, 70);
        lv_obj_set_style_bg_color(item_container, lv_color_hex(0x111111), 0);
        lv_obj_set_style_border_color(item_container, lv_color_hex(0x333333), 0);
        lv_obj_clear_flag(item_container, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t * lbl_name = lv_label_create(item_container);
        lv_label_set_text(lbl_name, custom_aw_names[i]);
        lv_obj_set_style_text_color(lbl_name, lv_color_white(), 0);
        lv_obj_align(lbl_name, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_add_flag(lbl_name, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(lbl_name, 20);
        lv_obj_add_event_cb(lbl_name, wp_item_click_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);

        lv_obj_t * btn_edit = lv_btn_create(item_container);
        lv_obj_set_size(btn_edit, 50, 50);
        lv_obj_align(btn_edit, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btn_edit, lv_color_hex(0x333333), 0);
        lv_obj_t * lbl_edit = lv_label_create(btn_edit);
        lv_label_set_text(lbl_edit, LV_SYMBOL_EDIT);
        lv_obj_center(lbl_edit);
        lv_obj_add_event_cb(btn_edit, edit_name_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }

    lv_obj_t * btn_back = lv_btn_create(scr_vault_menu);
    lv_obj_set_size(btn_back, 370, 50);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "VOLVER");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) { lv_scr_load(scr_vision_menu); }, LV_EVENT_CLICKED, NULL);
}

static void update_ttw_ui() {
    if (!btn_ttw_giro || !btn_ttw_pasivo) return;
    if (ttw_mode == 1) {
        lv_obj_set_style_bg_color(btn_ttw_giro, lv_color_hex(0x00E5FF), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn_ttw_giro, 0), lv_color_black(), 0);
        lv_obj_set_style_bg_color(btn_ttw_pasivo, lv_color_hex(0x111111), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn_ttw_pasivo, 0), lv_color_hex(0xFFFFFF), 0);
    } else {
        lv_obj_set_style_bg_color(btn_ttw_pasivo, lv_color_hex(0x00FF41), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn_ttw_pasivo, 0), lv_color_black(), 0);
        lv_obj_set_style_bg_color(btn_ttw_giro, lv_color_hex(0x111111), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn_ttw_giro, 0), lv_color_hex(0xFFFFFF), 0);
    }
}

static void set_ttw_mode(uint8_t mode) {
    ttw_mode = mode;
    Preferences p; 
    p.begin("exoned", false); 
    p.putUInt("ttw_mode", mode); 
    p.end();
    update_ttw_ui();
}

void build_vision_menu() {
    if (scr_vision_menu != NULL) return;
    
    scr_vision_menu = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_vision_menu, lv_color_hex(0x05000A), 0);
    lv_obj_clear_flag(scr_vision_menu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * title = lv_label_create(scr_vision_menu);
    lv_label_set_text(title, ">> GESTOR DE VISION <<");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t * btn_wp = lv_btn_create(scr_vision_menu);
    lv_obj_set_size(btn_wp, 340, 70);
    lv_obj_align(btn_wp, LV_ALIGN_TOP_MID, 0, 90);
    lv_obj_set_style_bg_color(btn_wp, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(btn_wp, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_border_width(btn_wp, 2, 0);
    lv_obj_t * lbl_wp = lv_label_create(btn_wp);
    lv_label_set_text(lbl_wp, "BOVEDA DE FONDOS");
    lv_obj_set_style_text_font(lbl_wp, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_wp);
    
    lv_obj_add_event_cb(btn_wp, [](lv_event_t * e) {
        park_sd_bus(); scan_animated_wallpapers(); unpark_sd_bus();
        if(scr_vault_menu != NULL) lv_obj_del(scr_vault_menu);
        scr_vault_menu = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr_vault_menu, lv_color_black(), 0);
        build_wallpaper_list();
        lv_scr_load(scr_vault_menu);
    }, LV_EVENT_CLICKED, NULL);

    btn_ttw_giro = lv_btn_create(scr_vision_menu);
    lv_obj_set_size(btn_ttw_giro, 340, 70);
    lv_obj_align(btn_ttw_giro, LV_ALIGN_TOP_MID, 0, 180);
    lv_obj_set_style_border_color(btn_ttw_giro, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_border_width(btn_ttw_giro, 2, 0);
    lv_obj_t * lbl_giro = lv_label_create(btn_ttw_giro);
    lv_label_set_text(lbl_giro, "GIRO TACTICO\n(Pantalla apagada)");
    lv_obj_set_style_text_font(lbl_giro, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(lbl_giro, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_giro);
    lv_obj_add_event_cb(btn_ttw_giro, [](lv_event_t* e){ set_ttw_mode(1); }, LV_EVENT_CLICKED, NULL);

    btn_ttw_pasivo = lv_btn_create(scr_vision_menu);
    lv_obj_set_size(btn_ttw_pasivo, 340, 70);
    lv_obj_align(btn_ttw_pasivo, LV_ALIGN_TOP_MID, 0, 270);
    lv_obj_set_style_border_color(btn_ttw_pasivo, lv_color_hex(0x00FF41), 0);
    lv_obj_set_style_border_width(btn_ttw_pasivo, 2, 0);
    lv_obj_t * lbl_pasivo = lv_label_create(btn_ttw_pasivo);
    lv_label_set_text(lbl_pasivo, "VISION PASIVA\n(Atenuado Always-On)");
    lv_obj_set_style_text_font(lbl_pasivo, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(lbl_pasivo, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_pasivo);
    lv_obj_add_event_cb(btn_ttw_pasivo, [](lv_event_t* e){ set_ttw_mode(2); }, LV_EVENT_CLICKED, NULL);

    update_ttw_ui();

    lv_obj_t * btn_back = lv_btn_create(scr_vision_menu);
    lv_obj_set_size(btn_back, 340, 50);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "VOLVER");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) { 
        lv_scr_load(scr_selector); 
    }, LV_EVENT_CLICKED, NULL);
}

bool vision_back_router() {
    lv_obj_t * cur = lv_scr_act();
    if (cur == scr_vault_menu) {
        if (modal_bg != NULL) { close_modal_cb(NULL); return true; }
        if (kb_rename != NULL) {
            lv_obj_del(kb_rename); lv_obj_del(ta_rename);
            kb_rename = NULL; ta_rename = NULL;
            return true;
        }
        lv_scr_load(scr_vision_menu);
        return true;
    }
    else if (cur == scr_vision_menu) {
        lv_scr_load(scr_selector);
        return true;
    }
    return false;
}

// =====================================================================
// FUNCIONES DE TRANSFERENCIA ASÍNCRONA
// =====================================================================
void iniciar_transferencia_async(uint8_t index) {
    if (async_transfer_active || total_aw_count == 0) return;

    apagar_motor_exnd();
    secure_spi_lock(); 

    char sd_path[64];
    sprintf(sd_path, "/WALLPAPERS/ANIMATED/AW%d.exnd", index);
    char flash_path[32];
    sprintf(flash_path, "/AW_%d.exnd", index);

    async_sd_source = sd.open(sd_path, O_READ);
    if (!async_sd_source) {
        release_spi_lock();
        return;
    }

    async_total_bytes = async_sd_source.size();
    
    if (FFat.exists(flash_path)) {
        fs::File check_file = FFat.open(flash_path, FILE_READ);
        if (check_file && check_file.size() == async_total_bytes) { 
            check_file.close();
            async_sd_source.close();
            release_spi_lock();
            set_current_aw_index(index);
            pending_fast_reload_idx = index;
            return;
        }
        if (check_file) check_file.close();
    }

    fs::File root = FFat.open("/");
    if(root){
        fs::File file = root.openNextFile();
        while(file) {
            if(strncmp(file.name(), "AW_", 3) == 0) {
                String fullPath = String("/") + file.name();
                file.close();
                FFat.remove(fullPath);
            } else {
                file.close();
            }
            file = root.openNextFile();
        }
        root.close();
    }

    async_flash_dest = FFat.open(flash_path, FILE_WRITE);
    if (!async_flash_dest) {
        async_sd_source.close();
        release_spi_lock();
        return;
    }

    async_transfer_buf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!async_transfer_buf) {
        async_transfer_buf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM);
    }
    if (!async_transfer_buf) {
        async_flash_dest.close();
        async_sd_source.close();
        release_spi_lock();
        return;
    }

    async_target_index = index;
    async_copied_bytes = 0;
    async_transfer_active = true;

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        lv_obj_clear_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(bar_bg_progress, 0, LV_ANIM_OFF);
        
        lv_obj_set_align(lbl_bg_time_shadow, LV_ALIGN_CENTER);
        lv_obj_set_align(lbl_bg_time, LV_ALIGN_CENTER);
        lv_obj_set_pos(lbl_bg_time_shadow, 2, 2);
        lv_obj_set_pos(lbl_bg_time, 0, 0);
        lv_obj_set_style_text_color(lbl_bg_time, lv_color_hex(0xFFFFFF), 0);

        lv_label_set_text(lbl_bg_progress, "INICIALIZANDO MOTOR...");
        lv_label_set_text(lbl_bg_time, "CAMBIANDO FONDO");
        lv_label_set_text(lbl_bg_time_shadow, "CAMBIANDO FONDO");
        lv_label_set_text(lbl_bg_batt, "NO APAGAR EL RELOJ");
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

// =====================================================================
// SERVIDOR TCP PUERTO 8892 (BLINDADO CONTRA COLISIONES DEL MAPA)
// =====================================================================
static WiFiServer exnd_tcp_server(8892);
static WiFiClient exnd_tcp_client;
static bool tcp_server_started = false;
static bool tcp_rx_active = false;
static FsFile tcp_sd_dest;
static uint32_t tcp_total_bytes = 0;
static uint32_t tcp_copied_bytes = 0;
static uint8_t tcp_req_slot = 0;
static uint8_t tcp_flag_activate = 1;
static char tcp_custom_name[32] = {0};
static uint32_t tcp_last_rx_ms = 0;

static void delete_sd_aw_cascade(uint8_t del_idx) {
    park_sd_bus();
    total_aw_count = 0;
    char p1[64], p2[64];
    for (int i = 1; i <= 99; i++) {
        sprintf(p1, "/WALLPAPERS/ANIMATED/AW%d.exnd", i);
        if (sd.exists(p1)) total_aw_count++;
        else break;
    }

    if (del_idx >= 1 && del_idx <= total_aw_count) {
        sprintf(p1, "/WALLPAPERS/ANIMATED/AW%d.exnd", del_idx);
        sd.remove(p1);

        Preferences prefs_bg;
        prefs_bg.begin("exoned", false);
        Preferences prefs_clk;
        prefs_clk.begin("exoned_clk", false);

        for (int k = del_idx + 1; k <= total_aw_count; k++) {
            sprintf(p1, "/WALLPAPERS/ANIMATED/AW%d.exnd", k);
            sprintf(p2, "/WALLPAPERS/ANIMATED/AW%d.exnd", k - 1);
            sd.rename(p1, p2);

            strncpy(custom_aw_names[k - 1], custom_aw_names[k], 31);
            custom_aw_names[k - 1][31] = '\0';

            char kn_prev[16], kn_cur[16];
            sprintf(kn_prev, "aw_name_%d", k - 1);
            sprintf(kn_cur, "aw_name_%d", k);
            prefs_bg.putString(kn_prev, custom_aw_names[k - 1]);
            prefs_bg.remove(kn_cur);

            char kp_prev[16], kp_cur[16], ks_prev[16], ks_cur[16], kc_prev[16], kc_cur[16];
            sprintf(kp_prev, "clk_p_%d", k - 1); sprintf(kp_cur, "clk_p_%d", k);
            sprintf(ks_prev, "clk_s_%d", k - 1); sprintf(ks_cur, "clk_s_%d", k);
            sprintf(kc_prev, "clk_c_%d", k - 1); sprintf(kc_cur, "clk_c_%d", k);

            if (prefs_clk.isKey(kp_cur)) prefs_clk.putUInt(kp_prev, prefs_clk.getUInt(kp_cur, 0));
            if (prefs_clk.isKey(ks_cur)) prefs_clk.putUInt(ks_prev, prefs_clk.getUInt(ks_cur, 0));
            if (prefs_clk.isKey(kc_cur)) prefs_clk.putUInt(kc_prev, prefs_clk.getUInt(kc_cur, 2));
        }

        prefs_clk.end();
        prefs_bg.end();
        total_aw_count--;
    }
    unpark_sd_bus();
}

static void poll_exnd_tcp_server() {
    if (WiFi.status() != WL_CONNECTED) {
        if (tcp_rx_active) {
            if (tcp_sd_dest) tcp_sd_dest.close();
            sd.remove("/WALLPAPERS/ANIMATED/AW_TMP.exnd");
            if (async_transfer_buf) { heap_caps_free(async_transfer_buf); async_transfer_buf = NULL; }
            tcp_rx_active = false;
            release_spi_lock();
        }
        if (tcp_server_started) {
            exnd_tcp_server.end();
            tcp_server_started = false;
        }
        return;
    }

    if (!tcp_server_started) {
        exnd_tcp_server.begin(8892);
        exnd_tcp_server.setNoDelay(true);
        tcp_server_started = true;
    }

    if (!tcp_rx_active && !async_transfer_active) {
        WiFiClient client = exnd_tcp_server.accept();
        if (!client) return;

        uint32_t t0 = millis();
        while (client.connected() && client.available() < 4 && (millis() - t0 < 400)) delay(2);
        if (client.available() < 4) { client.stop(); return; }

        char magic[5] = {0};
        client.readBytes(magic, 4);

        if (strncmp(magic, "EXLS", 4) == 0) {
            scan_animated_wallpapers();
            for (uint8_t i = 1; i <= total_aw_count; i++) {
                uint16_t w = 0, h = 0; uint32_t frames = 0, sz = 0; uint8_t fps = 0;
                if (get_aw_info_from_sd(i, &w, &h, &frames, &fps, &sz)) {
                    char line[128];
                    snprintf(line, sizeof(line), "%u|%s|%u|%u|%u|%u|%u\n", i, custom_aw_names[i], w, h, frames, fps, sz);
                    client.print(line);
                }
            }
            client.print("END\n");
            client.stop();
            return;
        }
        else if (strncmp(magic, "EXRM", 4) == 0) {
            while (client.connected() && client.available() < 1 && (millis() - t0 < 400)) delay(2);
            if (client.available() >= 1) {
                uint8_t del_idx = client.read();
                apagar_motor_exnd();
                delete_sd_aw_cascade(del_idx);
                if (current_aw_index > total_aw_count && total_aw_count > 0) current_aw_index = total_aw_count;
                if (lv_scr_act() == scr_screensaver && total_aw_count > 0) arrancar_motor_exnd(current_aw_index);
                client.print("OK\n");
            }
            client.stop();
            return;
        }
        else if (strncmp(magic, "EXUP", 4) == 0) {
            while (client.connected() && client.available() < 40 && (millis() - t0 < 800)) delay(2);
            if (client.available() < 40) { client.stop(); return; }

            uint8_t hdr[40];
            client.readBytes(hdr, 40);
            tcp_req_slot = hdr[0];
            tcp_flag_activate = hdr[1];
            tcp_total_bytes = hdr[4] | (hdr[5] << 8) | (hdr[6] << 16) | (hdr[7] << 24);
            memcpy(tcp_custom_name, &hdr[8], 31);
            tcp_custom_name[31] = '\0';

            if (tcp_total_bytes < 1000 || tcp_total_bytes > 8000000) {
                client.print("ERR_SIZE\n");
                client.stop();
                return;
            }

            if (tcp_req_slot == 250 || tcp_req_slot == 251) {
                secure_spi_lock();
                if (!sd.exists("/UI")) sd.mkdir("/UI");
                const char* ph_path = (tcp_req_slot == 250) ? "/UI/placeholder.qoi" : "/UI/placeholder.bin";
                const char* tmp_path = "/UI/placeholder.tmp";

                FsFile pf = sd.open(tmp_path, O_WRITE | O_CREAT | O_TRUNC);
                uint32_t rem = tcp_total_bytes;
                uint8_t tcp_buf[2048];

                while (rem > 0 && client.connected()) {
                    int avail = client.available();
                    if (avail > 0) {
                        int to_read = (avail > (int)sizeof(tcp_buf)) ? (int)sizeof(tcp_buf) : avail;
                        if ((uint32_t)to_read > rem) to_read = (int)rem;
                        int r = client.read(tcp_buf, to_read);
                        if (r > 0) {
                            pf.write(tcp_buf, r);
                            rem -= r;
                        }
                    } else {
                        delay(1);
                    }
                }
                pf.close();

                if (rem == 0) {
                    if (sd.exists(ph_path)) sd.remove(ph_path);
                    sd.rename(tmp_path, ph_path);
                    
                    if (reload_custom_placeholder()) {
                        client.println("OK_PLACEHOLDER_256");
                    } else {
                        client.println("ERR_DECODE");
                    }
                } else {
                    sd.remove(tmp_path);
                    client.println("ERR_INCOMPLETE");
                }
                client.stop();
                release_spi_lock();
                return;
            }

            apagar_motor_exnd();
            secure_spi_lock();

            if (!sd.exists("/WALLPAPERS")) sd.mkdir("/WALLPAPERS");
            if (!sd.exists("/WALLPAPERS/ANIMATED")) sd.mkdir("/WALLPAPERS/ANIMATED");
            if (sd.exists("/WALLPAPERS/ANIMATED/AW_TMP.exnd")) sd.remove("/WALLPAPERS/ANIMATED/AW_TMP.exnd");

            tcp_sd_dest = sd.open("/WALLPAPERS/ANIMATED/AW_TMP.exnd", O_WRITE | O_CREAT | O_TRUNC);
            if (!tcp_sd_dest) {
                release_spi_lock();
                client.print("ERR_SD\n");
                client.stop();
                return;
            }

            async_transfer_buf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!async_transfer_buf) async_transfer_buf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM);
            if (!async_transfer_buf) {
                tcp_sd_dest.close();
                release_spi_lock();
                client.print("ERR_MEM\n");
                client.stop();
                return;
            }

            exnd_tcp_client = client;
            tcp_copied_bytes = 0;
            tcp_last_rx_ms = millis();
            tcp_rx_active = true;

            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                if (lv_scr_act() != scr_screensaver) lv_scr_load(scr_screensaver);
                lv_obj_clear_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);
                lv_bar_set_value(bar_bg_progress, 0, LV_ANIM_OFF);

                lv_obj_set_align(lbl_bg_time_shadow, LV_ALIGN_CENTER);
                lv_obj_set_align(lbl_bg_time, LV_ALIGN_CENTER);
                lv_obj_set_pos(lbl_bg_time_shadow, 2, 2);
                lv_obj_set_pos(lbl_bg_time, 0, 0);
                lv_obj_set_style_text_color(lbl_bg_time, lv_color_hex(0x00FF41), 0);

                lv_label_set_text(lbl_bg_progress, "RECIBIENDO DESDE APP [0%]");
                lv_label_set_text(lbl_bg_time, "INYECTANDO SD");
                lv_label_set_text(lbl_bg_time_shadow, "INYECTANDO SD");
                lv_label_set_text(lbl_bg_batt, tcp_custom_name);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        } else {
            client.stop();
        }
    }

    if (tcp_rx_active) {
        uint32_t slice_t0 = millis();
        while (millis() - slice_t0 < 25 && tcp_copied_bytes < tcp_total_bytes) {
            int avail = exnd_tcp_client.available();
            if (avail > 0) {
                int to_read = (avail > 8192) ? 8192 : avail;
                if (tcp_copied_bytes + to_read > tcp_total_bytes) {
                    to_read = tcp_total_bytes - tcp_copied_bytes;
                }
                int r = exnd_tcp_client.read(async_transfer_buf, to_read);
                if (r > 0) {
                    tcp_sd_dest.write(async_transfer_buf, r);
                    tcp_copied_bytes += r;
                    tcp_last_rx_ms = millis();
                }
            } else {
                if (!exnd_tcp_client.connected()) break;
                delay(1);
            }
        }

        uint8_t pct = (uint8_t)(((float)tcp_copied_bytes / (float)tcp_total_bytes) * 100.0f);
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            lv_bar_set_value(bar_bg_progress, pct, LV_ANIM_OFF);
            char buf[40];
            sprintf(buf, "GUARDANDO EN SD [%d%%]", pct);
            lv_label_set_text(lbl_bg_progress, buf);
            xSemaphoreGiveRecursive(lvgl_mutex);
        }

        if (tcp_copied_bytes >= tcp_total_bytes) {
            tcp_sd_dest.flush();
            tcp_sd_dest.close();
            heap_caps_free(async_transfer_buf);
            async_transfer_buf = NULL;

            total_aw_count = 0;
            char path[64];
            for (int i = 1; i <= 99; i++) {
                sprintf(path, "/WALLPAPERS/ANIMATED/AW%d.exnd", i);
                if (sd.exists(path)) total_aw_count++;
                else break;
            }

            uint8_t final_idx = (tcp_req_slot >= 1 && tcp_req_slot <= total_aw_count + 1)
                                ? tcp_req_slot
                                : (total_aw_count + 1);
            if (final_idx > 99) final_idx = 99;

            sprintf(path, "/WALLPAPERS/ANIMATED/AW%d.exnd", final_idx);
            if (sd.exists(path)) sd.remove(path);
            sd.rename("/WALLPAPERS/ANIMATED/AW_TMP.exnd", path);

            if (final_idx > total_aw_count) total_aw_count = final_idx;

            if (tcp_custom_name[0] != '\0') {
                strncpy(custom_aw_names[final_idx], tcp_custom_name, 31);
                custom_aw_names[final_idx][31] = '\0';
                Preferences prefs_bg;
                prefs_bg.begin("exoned", false);
                char key[16];
                sprintf(key, "aw_name_%d", final_idx);
                prefs_bg.putString(key, custom_aw_names[final_idx]);
                prefs_bg.end();
            }

            char ack[24];
            sprintf(ack, "OK:AW%d\n", final_idx);
            exnd_tcp_client.print(ack);
            exnd_tcp_client.stop();

            tcp_rx_active = false;
            release_spi_lock();

            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                lv_obj_add_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }

            if (tcp_flag_activate) {
                Preferences prefs;
                prefs.begin("exoned", false);
                prefs.putUInt("aw_idx", final_idx);
                prefs.end();
                iniciar_transferencia_async(final_idx);
            } else {
                force_time_label_refresh = true;
                arrancar_motor_exnd(current_aw_index);
            }
        }
        else if (millis() - tcp_last_rx_ms > 10000UL || !exnd_tcp_client.connected()) {
            tcp_sd_dest.close();
            sd.remove("/WALLPAPERS/ANIMATED/AW_TMP.exnd");
            heap_caps_free(async_transfer_buf);
            async_transfer_buf = NULL;
            exnd_tcp_client.stop();
            tcp_rx_active = false;
            release_spi_lock();

            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                lv_obj_add_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
            force_time_label_refresh = true;
            arrancar_motor_exnd(current_aw_index);
        }
    }
}

void background_transfer_loop() {
    poll_exnd_tcp_server();
    if (tcp_rx_active) return;

    if (pending_fast_reload_idx != 0) {
        uint8_t idx = pending_fast_reload_idx;
        pending_fast_reload_idx = 0;
        arrancar_motor_exnd(idx);
        return;
    }

    if (!async_transfer_active) return;

    if (async_sd_source.available()) {
        int bytes_read = async_sd_source.read(async_transfer_buf, 8192);
        if (bytes_read > 0) {
            async_flash_dest.write(async_transfer_buf, bytes_read);
            async_copied_bytes += bytes_read;
            
            uint8_t pct = (uint8_t)(((float)async_copied_bytes / async_total_bytes) * 100.0f);
            
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                lv_bar_set_value(bar_bg_progress, pct, LV_ANIM_OFF);
                char buf[32];
                sprintf(buf, "TRANSFIRIENDO [%d%%]", pct);
                lv_label_set_text(lbl_bg_progress, buf);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        }
    } else {
        heap_caps_free(async_transfer_buf);
        async_transfer_buf = NULL;
        async_flash_dest.close();
        async_sd_source.close();
        async_transfer_active = false;
        
        release_spi_lock();
        
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            lv_obj_add_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
        
        set_current_aw_index(async_target_index);
        arrancar_motor_exnd(async_target_index);
    }
}

bool is_transferring_bg() {
    return async_transfer_active || tcp_rx_active || clock_edit_mode;
}

// =====================================================================
// INICIALIZACIÓN CON PURGA INTELIGENTE
// =====================================================================
void init_background_engine() {
    if (local_decoder_buf == NULL) {
        local_decoder_buf = (uint8_t*)heap_caps_malloc(120000, MALLOC_CAP_SPIRAM);
    }

    if (!names_initialized) {
        Preferences prefs_bg;
        prefs_bg.begin("exoned", true); 
        for(int i = 1; i <= 99; i++) {
            char key[16];
            sprintf(key, "aw_name_%d", i);
            if (prefs_bg.isKey(key)) {
                String saved_name = prefs_bg.getString(key, "");
                strncpy(custom_aw_names[i], saved_name.c_str(), 31);
                custom_aw_names[i][31] = '\0';
            } else {
                sprintf(custom_aw_names[i], "AW por defecto %d", i);
            }
        }
        prefs_bg.end();
        names_initialized = true;
    }

    if (!FFat.begin()) {
        if(!FFat.begin(true)) Serial.println("[MOTOR] FATAL: No se pudo montar FFat.");
    } else {
        fs::File root = FFat.open("/");
        if (root) {
            fs::File file = root.openNextFile();
            while (file) {
                String fname = String(file.name());
                if (fname.startsWith("AW_") && fname.endsWith(".exnd")) {
                    uint8_t magic[4] = {0};
                    file.read(magic, 4);
                    file.close();
                    
                    if (strncmp((char*)magic, "EXND", 4) != 0) {
                        FFat.remove("/" + fname);
                        Serial.printf("[PURGA] Basura eliminada de FFat: %s\n", fname.c_str());
                        
                        if (current_aw_index == fname.substring(3, fname.indexOf('.')).toInt()) {
                            Preferences prefs_clk;
                            prefs_clk.begin("exoned", false);
                            prefs_clk.putUInt("aw_idx", 1);
                            prefs_clk.end();
                            current_aw_index = 1;
                        }
                    }
                } else {
                    file.close();
                }
                file = root.openNextFile();
            }
            root.close();
        }
    }

    Preferences prefs_bg;
    prefs_bg.begin("exoned", true);
    if (prefs_bg.isKey("ttw_mode")) {
        ttw_mode = prefs_bg.getUInt("ttw_mode", 1);
    }
    prefs_bg.end();

    scr_screensaver = lv_obj_create(NULL);
    lv_obj_clear_flag(scr_screensaver, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr_screensaver, lv_color_hex(0x000000), LV_PART_MAIN);

    lv_obj_add_flag(scr_screensaver, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr_screensaver, screensaver_touch_cb, LV_EVENT_PRESSED, NULL);

    lbl_bg_time_shadow = lv_label_create(scr_screensaver);
    lv_obj_set_style_text_color(lbl_bg_time_shadow, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_opa(lbl_bg_time_shadow, 255, LV_PART_MAIN); 

    lbl_bg_time = lv_label_create(scr_screensaver);
    lv_obj_add_flag(lbl_bg_time, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(lbl_bg_time, 25);
    lv_obj_add_event_cb(lbl_bg_time, clock_touch_cb, LV_EVENT_ALL, NULL);
    
    apply_clock_profile(current_aw_index);

    lbl_bg_batt = lv_label_create(scr_screensaver);
    lv_obj_set_style_text_color(lbl_bg_batt, lv_color_hex(0x00E5FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_bg_batt, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_bg_batt, LV_ALIGN_BOTTOM_MID, 0, -20);

    bar_bg_progress = lv_bar_create(scr_screensaver);
    lv_obj_set_size(bar_bg_progress, 300, 15);
    lv_obj_align(bar_bg_progress, LV_ALIGN_BOTTOM_MID, 0, -60);
    lv_obj_set_style_bg_color(bar_bg_progress, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_bg_progress, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
    lv_obj_set_style_shadow_color(bar_bg_progress, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
    lv_obj_set_style_shadow_width(bar_bg_progress, 10, LV_PART_INDICATOR);
    lv_bar_set_range(bar_bg_progress, 0, 100);
    lv_obj_add_flag(bar_bg_progress, LV_OBJ_FLAG_HIDDEN);

    lbl_bg_progress = lv_label_create(scr_screensaver);
    lv_obj_set_style_text_color(lbl_bg_progress, lv_color_hex(0x00E5FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_bg_progress, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_bg_progress, LV_ALIGN_BOTTOM_MID, 0, -85);
    lv_obj_add_flag(lbl_bg_progress, LV_OBJ_FLAG_HIDDEN);

    lv_timer_create([](lv_timer_t * t) {
        if (lbl_bg_time != NULL && lbl_bg_time_shadow != NULL && lbl_bg_batt != NULL && !async_transfer_active) {
            extern volatile uint8_t sys_hour, sys_min;
            extern volatile int real_batt_pct;
            static uint8_t last_h = 255, last_m = 255;
            static int last_b = -999;

            if (force_time_label_refresh || sys_hour != last_h || sys_min != last_m) {
                char buf[16];
                sprintf(buf, "%02d:%02d", sys_hour, sys_min);
                lv_label_set_text(lbl_bg_time_shadow, buf);
                lv_label_set_text(lbl_bg_time, buf);
                last_h = sys_hour;
                last_m = sys_min;
            }
            if (force_time_label_refresh || real_batt_pct != last_b) {
                char buf[24];
                sprintf(buf, "BATT [%d%%]", real_batt_pct);
                lv_label_set_text(lbl_bg_batt, buf);
                last_b = real_batt_pct;
            }
            force_time_label_refresh = false;
        }
    }, 1000, NULL);
}

// =====================================================================
// DECODIFICADOR PRINCIPAL
// =====================================================================
static void exnd_decoder_cb(lv_timer_t * timer) {
    if (!internal_exnd || img_screensaver == NULL || async_transfer_active || !local_decoder_buf) return;

    uint16_t native_w = exnd_target_w * 2; 
    uint16_t* p_vram = (uint16_t*)exnd_img_dsc.data;

    if (exnd_current_frame >= exnd_total_frames) {
        internal_exnd.seek(16);
        uint32_t logical_bytes = exnd_target_w * exnd_target_h * 2; 
        
        if (internal_exnd.read(local_decoder_buf, logical_bytes) != logical_bytes) return;

        uint8_t* raw_src = local_decoder_buf;
        uint16_t lx = 0, ly = 0;
        for(uint32_t i = 0; i < logical_bytes; i += 2) {
            uint16_t color = (raw_src[i] << 8) | raw_src[i+1]; 
            uint32_t p_row1 = (ly * 2) * native_w + (lx * 2);
            uint32_t p_row2 = p_row1 + native_w;
            p_vram[p_row1] = color;
            p_vram[p_row1 + 1] = color;
            p_vram[p_row2] = color;
            p_vram[p_row2 + 1] = color;
            lx++;
            if (lx >= exnd_target_w) { lx = 0; ly++; }
        }
        
        lv_obj_invalidate(img_screensaver);
        exnd_current_frame = 1;
        return; 
    }

    uint8_t size_buf[4];
    if (internal_exnd.read(size_buf, 4) != 4) return;
    uint32_t frame_total_size = size_buf[0] | (size_buf[1] << 8) | (size_buf[2] << 16) | (size_buf[3] << 24);

    if (frame_total_size == 4) {
        internal_exnd.seek(internal_exnd.position() + 4);
    } 
    else if (frame_total_size > 4) {
        if (frame_total_size <= 120000) { 
            if (internal_exnd.read(local_decoder_buf, frame_total_size) != frame_total_size) return; 
            
            uint32_t ptr = 8;
            uint32_t cursor = 0;
            uint32_t max_pixels = exnd_target_w * exnd_target_h; 
            uint16_t min_ly = exnd_target_h;
            uint16_t max_ly = 0;

            while (ptr + 3 < frame_total_size) {
                uint16_t skip = local_decoder_buf[ptr] | (local_decoder_buf[ptr+1] << 8);
                uint16_t write_len = local_decoder_buf[ptr+2] | (local_decoder_buf[ptr+3] << 8);
                ptr += 4;
                
                if (skip == 0xFFFF && write_len == 0) break;
                cursor += skip;
                
                if (cursor + write_len > max_pixels) break; 
                if (ptr + (write_len * 2) > frame_total_size) break;

                if (write_len > 0) {
                    uint16_t lx = cursor % exnd_target_w;
                    uint16_t ly = cursor / exnd_target_w;
                    if (ly < min_ly) min_ly = ly;

                    for(uint16_t i = 0; i < write_len; i++) {
                        uint16_t color = (local_decoder_buf[ptr + (i * 2)] << 8) | local_decoder_buf[ptr + (i * 2) + 1];
                        uint32_t p_row1 = (ly * 2) * native_w + (lx * 2);
                        uint32_t p_row2 = p_row1 + native_w;
                        p_vram[p_row1] = color;
                        p_vram[p_row1 + 1] = color;
                        p_vram[p_row2] = color;
                        p_vram[p_row2 + 1] = color;
                        lx++;
                        if (lx >= exnd_target_w) { lx = 0; ly++; }
                    }
                    if (ly > max_ly) max_ly = ly;
                    ptr += (write_len * 2);
                    cursor += write_len;
                }
            }
            if (min_ly <= max_ly) {
                lv_area_t dirty_area;
                dirty_area.x1 = 0;
                dirty_area.x2 = native_w - 1;
                dirty_area.y1 = min_ly * 2;
                dirty_area.y2 = (max_ly * 2) + 1;
                if (dirty_area.y2 >= exnd_target_h * 2) dirty_area.y2 = (exnd_target_h * 2) - 1;
                lv_obj_invalidate_area(img_screensaver, &dirty_area);
            }
        } else {
            size_t current_pos = internal_exnd.position();
            size_t file_size = internal_exnd.size();
            if (current_pos + frame_total_size <= file_size) {
                internal_exnd.seek(current_pos + frame_total_size);
            } else {
                internal_exnd.seek(file_size);
            }
        }
    }
    exnd_current_frame++;
}

void arrancar_motor_exnd(uint8_t index) {
    is_exiting_bg = false;
    current_aw_index = index;

    secure_spi_lock();
    invalidate_map_cache();

    if (exnd_timer != NULL) lv_timer_pause(exnd_timer);
    if (internal_exnd) internal_exnd.close();

    char flash_path[32];
    sprintf(flash_path, "/AW_%d.exnd", index);

    internal_exnd = FFat.open(flash_path, FILE_READ); 
    if (!internal_exnd) {
        fs::File root = FFat.open("/");
        if (root) {
            fs::File f = root.openNextFile();
            while (f) {
                if (strncmp(f.name(), "AW_", 3) == 0) {
                    int found_idx = atoi(f.name() + 3);
                    snprintf(flash_path, sizeof(flash_path), "/%s", f.name());
                    f.close();
                    if (found_idx > 0) current_aw_index = (uint8_t)found_idx;
                    internal_exnd = FFat.open(flash_path, FILE_READ);
                    break;
                }
                f.close();
                f = root.openNextFile();
            }
            root.close();
        }
        if (!internal_exnd) {
            release_spi_lock();
            return;
        }
    }

    uint8_t header[16];
    if(internal_exnd.read(header, 16) != 16) {
        internal_exnd.close();
        release_spi_lock();
        return;
    }

    exnd_target_w = header[4] | (header[5] << 8);
    exnd_target_h = header[6] | (header[7] << 8);
    exnd_total_frames = header[8] | (header[9] << 8) | (header[10] << 16) | (header[11] << 24);
    exnd_target_fps = header[12];
    if (exnd_target_fps == 0) exnd_target_fps = 15;

    uint16_t native_w = exnd_target_w * 2;
    uint16_t native_h = exnd_target_h * 2;

    if (local_decoder_buf == NULL) {
        Serial.println("[MOTOR] FALLO: Buffer de decodificacion no asignado.");
        internal_exnd.close();
        release_spi_lock();
        return;
    }

    uint32_t logical_bytes = exnd_target_w * exnd_target_h * 2;
    uint16_t* p_vram = (uint16_t*)vault_arena;
    internal_exnd.seek(16); 
    
    if (logical_bytes <= 120000 && internal_exnd.read(local_decoder_buf, logical_bytes) == logical_bytes) {
        uint8_t* raw_src = local_decoder_buf;
        uint16_t lx = 0, ly = 0;
        for(uint32_t i = 0; i < logical_bytes; i += 2) {
            uint16_t color = (raw_src[i] << 8) | raw_src[i+1]; 
            uint32_t p_row1 = (ly * 2) * native_w + (lx * 2);
            uint32_t p_row2 = p_row1 + native_w;
            p_vram[p_row1] = color;
            p_vram[p_row1 + 1] = color;
            p_vram[p_row2] = color;
            p_vram[p_row2 + 1] = color;
            lx++;
            if (lx >= exnd_target_w) { lx = 0; ly++; }
        }
    }

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (img_screensaver == NULL) {
            img_screensaver = lv_img_create(scr_screensaver);
            lv_obj_align(img_screensaver, LV_ALIGN_CENTER, 0, 0);
            lv_obj_move_background(img_screensaver);
        }

        exnd_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        exnd_img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        exnd_img_dsc.header.flags = 0;
        exnd_img_dsc.header.w = native_w;
        exnd_img_dsc.header.h = native_h;
        exnd_img_dsc.header.stride = native_w * 2; 
        exnd_img_dsc.data_size = native_w * native_h * 2;
        exnd_img_dsc.data = vault_arena; 
        
        lv_img_set_src(img_screensaver, &exnd_img_dsc);
        lv_obj_invalidate(img_screensaver);

        if (exnd_timer == NULL) {
            exnd_timer = lv_timer_create(exnd_decoder_cb, 1000 / exnd_target_fps, NULL);
        } else {
            lv_timer_set_period(exnd_timer, 1000 / exnd_target_fps);
            lv_timer_resume(exnd_timer);
        }

        apply_clock_profile(current_aw_index);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }

    exnd_current_frame = 1;
}

void apagar_motor_exnd() {
    if (exnd_timer != NULL) lv_timer_pause(exnd_timer);
    if (internal_exnd) internal_exnd.close();
    
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (slider_scale) { lv_obj_del(slider_scale); slider_scale = NULL; }
        if (slider_color) { lv_obj_del(slider_color); slider_color = NULL; }
        if (hud_clock_edit != NULL) {
            lv_obj_del(hud_clock_edit);
            hud_clock_edit = NULL;
        }
        if (lbl_bg_batt) lv_obj_clear_flag(lbl_bg_batt, LV_OBJ_FLAG_HIDDEN);
        clock_edit_mode = false;

        if (img_screensaver != NULL) {
            lv_img_set_src(img_screensaver, NULL);
        }
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    
    apply_clock_profile(current_aw_index); 
    release_spi_lock();
}

void cycle_next_wallpaper() {
    if (total_aw_count == 0) return;
    current_aw_index++;
    if (current_aw_index > total_aw_count) current_aw_index = 1;
    iniciar_transferencia_async(current_aw_index);
}

uint8_t get_total_aw_count() { return total_aw_count; }
uint8_t get_current_aw_index() { return current_aw_index; }
void set_current_aw_index(uint8_t idx) { current_aw_index = idx; }

bool get_aw_info_from_sd(uint8_t index, uint16_t *w, uint16_t *h, uint32_t *frames, uint8_t *fps, uint32_t *size_bytes) {
    char sd_path[64];
    sprintf(sd_path, "/WALLPAPERS/ANIMATED/AW%d.exnd", index);
    
    secure_spi_lock(); 

    FsFile file = sd.open(sd_path, O_READ);
    if(!file) { release_spi_lock(); return false; }
    
    *size_bytes = file.size();
    uint8_t header[16];
    file.read(header, 16);
    *w = header[4] | (header[5] << 8);
    *h = header[6] | (header[7] << 8);
    *frames = header[8] | (header[9] << 8) | (header[10] << 16) | (header[11] << 24);
    *fps = header[12];
    
    file.close();
    release_spi_lock(); 
    return true;
}

bool read_aw_preview_from_sd(uint8_t index, uint16_t* buffer, uint32_t bytes_to_read) {
    char sd_path[64];
    sprintf(sd_path, "/WALLPAPERS/ANIMATED/AW%d.exnd", index);
    
    secure_spi_lock(); 

    FsFile file = sd.open(sd_path, O_READ);
    if(!file) { release_spi_lock(); return false; }
    
    file.seek(16); 
    file.read((uint8_t*)buffer, bytes_to_read);
    file.close();

    release_spi_lock();
    
    uint8_t* raw = (uint8_t*)buffer;
    for(uint32_t i = 0; i < bytes_to_read; i += 2) {
        uint8_t temp = raw[i];
        raw[i] = raw[i+1];
        raw[i+1] = temp;
    }
    return true;
}