#include "TimeEngine.h"
#include "BackgroundEngine.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <LilyGoLib.h>

extern LilyGoUltra& instance;

#define RTC_ADDR 0x51
#define TIMEZONE_OFFSET -6 // CST - Ajuste balistico

extern TinyGPSPlus gps;
extern volatile bool gps_locked;

volatile uint8_t sys_hour = 0;
volatile uint8_t sys_min = 0;
volatile uint8_t sys_sec = 0;
volatile bool time_is_gnss = false;

// JERARQUIA DE ESTADO GLOBAL
enum AlarmState { STANDBY, ALARM_BASE, SNOOZE, VIGILANCE, ASSAULT };
volatile AlarmState current_alarm_state = STANDBY;
static uint32_t state_timer = 0;
uint32_t active_snooze_ms = 600000; // 10 minutos por defecto

extern int check_alarm_trigger(uint8_t current_h, uint8_t current_m);
extern void build_alarm_ui(bool is_assault);

static inline uint8_t bcd_to_dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
static inline uint8_t dec_to_bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

void init_time_engine() {
    Wire.beginTransmission(RTC_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[!] FATAL: RTC PCF85063A ciego en bus I2C (0x51).");
        return;
    }

    Wire.beginTransmission(RTC_ADDR);
    Wire.write(0x00); // Control_1
    Wire.write(0x00); // Normal mode, RTC run
    Wire.endTransmission();

    Serial.println("[*] Motor Temporal Inicializado. PCF85063A en linea.");
}

void time_loop_handler() {
    static uint32_t last_time_read = 0;
    
    if (millis() - last_time_read > 1000) {
        if (gps_locked && gps.time.isValid() && gps.time.age() < 2000) {
            time_is_gnss = true;
            
            int raw_hour = gps.time.hour() + TIMEZONE_OFFSET;
            if (raw_hour < 0) raw_hour += 24;
            else if (raw_hour >= 24) raw_hour -= 24;
            
            uint8_t g_hour = (uint8_t)raw_hour;
            uint8_t g_min = gps.time.minute();
            uint8_t g_sec = gps.time.second();
            
            sys_hour = g_hour;
            sys_min = g_min;
            sys_sec = g_sec;
            
            Wire.beginTransmission(RTC_ADDR);
            Wire.write(0x04);
            Wire.endTransmission();
            Wire.requestFrom(RTC_ADDR, 3);
            
            if (Wire.available() >= 3) {
                uint8_t rtc_sec = bcd_to_dec(Wire.read() & 0x7F);
                uint8_t rtc_min = bcd_to_dec(Wire.read() & 0x7F);
                uint8_t rtc_hour = bcd_to_dec(Wire.read() & 0x3F);
                
                if (rtc_sec != g_sec || rtc_min != g_min || rtc_hour != g_hour) {
                    Wire.beginTransmission(RTC_ADDR);
                    Wire.write(0x04);
                    Wire.write(dec_to_bcd(g_sec));
                    Wire.write(dec_to_bcd(g_min));
                    Wire.write(dec_to_bcd(g_hour));
                    Wire.endTransmission();
                }
            }
        } 
        else {
            time_is_gnss = false;
            
            Wire.beginTransmission(RTC_ADDR);
            Wire.write(0x04);
            Wire.endTransmission();
            Wire.requestFrom(RTC_ADDR, 3);
            
            if (Wire.available() >= 3) {
                sys_sec = bcd_to_dec(Wire.read() & 0x7F);
                sys_min = bcd_to_dec(Wire.read() & 0x7F);
                sys_hour = bcd_to_dec(Wire.read() & 0x3F);
            }
        }
        
        last_time_read = millis();
    }

    static uint8_t last_eval_min = 60;
    if (sys_min != last_eval_min) {
        last_eval_min = sys_min; 

        if (current_alarm_state == STANDBY) {
            int snz_minutes = check_alarm_trigger(sys_hour, sys_min);
            if (snz_minutes != -1) {
                active_snooze_ms = snz_minutes * 60000UL;
                Serial.printf("\n[!] ALARMA DETONADA. Hora: %02d:%02d | Snooze config: %d min\n", sys_hour, sys_min, snz_minutes);
                
                current_alarm_state = ALARM_BASE;
                state_timer = millis();
                build_alarm_ui(false);
            }
        }
    }

    process_alarm_state_machine();
}

// =====================================================================
// MAQUINA DE ESTADOS: ALARMA HOSTIL DE ASALTO
// =====================================================================
extern volatile uint32_t current_steps;
extern volatile float kin_actual_bpm;
extern lv_obj_t * scr_screensaver;
extern SemaphoreHandle_t lvgl_mutex;

static uint32_t steps_quota = 0;
static int math_solution = 0;
static bool math_solved = false;

static char math_input_buf[8] = "";
static char equation_str[32] = "";

static lv_obj_t * scr_alarm = NULL;
static lv_obj_t * lbl_equation = NULL;
static lv_obj_t * lbl_math_input = NULL;
static lv_obj_t * lbl_status = NULL;
static lv_obj_t * numpad = NULL;

bool is_alarm_active() { return current_alarm_state != STANDBY; }

// --- ARMAS ACUSTICAS (Buffer estatico en BSS para proteger el Stack) ---
static bool siren_active = false;
static uint32_t audio_debug_timer = 0;
static int16_t siren_sample_buffer[2048]; 

static void stop_alarm_hardware_outputs() {
    Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(0); Wire.endTransmission();

    if (siren_active) {
        instance.player.end();
        instance.powerControl(POWER_SPEAK, false);
        siren_active = false;
    }
}

void trigger_pcm_siren(bool max_volume) {
    if (!siren_active) {
        Serial.println("\n[!] ========================================");
        Serial.println("[AUDIO] INICIANDO ASALTO V6 (BANDA CERRADA Y MICRO-CONMUTACION)");
        instance.powerControl(POWER_SPEAK, true);
        delay(50); 
        
        instance.player.end(); 
        instance.player.begin(I2S_MODE_STD, 32000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        siren_active = true;
    }

    static float phase = 0.0f;
    static float current_freq = 4500.0f;
    static float duty_cycle = 0.15f; 
    static int state_ticks = 0;
    static bool is_muted = false;

    if (state_ticks <= 0) {
        if (random(0, 100) > 15) { 
            current_freq = (float)random(3800, 6200); 
            duty_cycle = (float)random(10, 25) / 100.0f; 
            state_ticks = random(3, 8); 
            is_muted = false;
        } else {
            state_ticks = random(2, 4); 
            is_muted = !is_muted;
        }
    }

    state_ticks--;

    float max_amp = max_volume ? 32767.0f : 15000.0f; 
    if (is_muted) max_amp = 0.0f;

    for (int i = 0; i < 2048; i += 2) {
        phase += (current_freq / 32000.0f); 
        if (phase >= 1.0f) phase -= 1.0f;
        
        float wave = (phase < duty_cycle) ? 1.0f : -1.0f; 
        int16_t val = (int16_t)(wave * max_amp);
        
        siren_sample_buffer[i] = val;     // L
        siren_sample_buffer[i+1] = val;   // R
    }
    
    instance.player.write((uint8_t*)siren_sample_buffer, sizeof(siren_sample_buffer));

    if (millis() - audio_debug_timer > 500) {
        Serial.printf("[AUDIO] Psycho-V6 | Freq: %.0f Hz | Duty: %.2f | Amp: %.0f\n", 
                      current_freq, duty_cycle, max_amp);
        audio_debug_timer = millis();
    }
}

static void restore_screensaver_from_alarm() {
    stop_alarm_hardware_outputs();
    arrancar_motor_exnd(get_current_aw_index());

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        lv_scr_load(scr_screensaver);
        if (scr_alarm != NULL) {
            lv_obj_del_async(scr_alarm);
            scr_alarm = NULL;
            lbl_equation = NULL;
            lbl_math_input = NULL;
            lbl_status = NULL;
            numpad = NULL;
        }
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

void abort_alarm() {
    Serial.println("\n[X] ABORTO DE ALARMA INICIADO");
    current_alarm_state = STANDBY;
    math_solved = false;
    restore_screensaver_from_alarm();
    Serial.println("[X] ABORTO COMPLETADO. Sistema en Standby.");
}

void trigger_alarm_test() {
    if (is_alarm_active()) return;
    Serial.println("\n[!] DETONADOR TACTIL ACTIVADO");
    current_alarm_state = ALARM_BASE;
    state_timer = millis();
    build_alarm_ui(false);
}

void trigger_drv2605_chaos() {
    uint8_t effects[] = {1, 12, 14, 15, 27, 47, 64, 118};
    uint8_t ef = effects[random(0, 8)];
    Wire.beginTransmission(0x5A); Wire.write(0x04); Wire.write(ef); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x05); Wire.write(0); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(1); Wire.endTransmission();
}

void generate_math_equation() {
    int a = random(10, 99);
    int b = random(10, 99);
    if (random(0, 2) == 0) { 
        math_solution = a + b; 
        sprintf(equation_str, "%d + %d = ?", a, b); 
    } else { 
        if (a < b) { int temp = a; a = b; b = temp; } 
        math_solution = a - b; 
        sprintf(equation_str, "%d - %d = ?", a, b); 
    }
    if (lbl_equation) lv_label_set_text(lbl_equation, equation_str);
    math_input_buf[0] = '\0';
    if (lbl_math_input) lv_label_set_text(lbl_math_input, "...");
}

static void btn_snooze_cb(lv_event_t * e) {
    current_alarm_state = SNOOZE;
    state_timer = millis();
    restore_screensaver_from_alarm();
}

static void btn_wakeup_cb(lv_event_t * e) {
    current_alarm_state = VIGILANCE;
    state_timer = millis();
    restore_screensaver_from_alarm();
}

static const char * btnm_map[] = {"1", "2", "3", "\n",
                                  "4", "5", "6", "\n",
                                  "7", "8", "9", "\n",
                                  "CLR", "0", "OK", ""};

static void numpad_cb(lv_event_t * e) {
    lv_obj_t * btnm = (lv_obj_t *)lv_event_get_target(e);
    uint32_t id = lv_btnmatrix_get_selected_btn(btnm);
    const char * txt = lv_btnmatrix_get_btn_text(btnm, id);
    if (txt == NULL) return;

    if (strcmp(txt, "CLR") == 0) {
        math_input_buf[0] = '\0';
    } else if (strcmp(txt, "OK") == 0) {
        int val = atoi(math_input_buf);
        if (val == math_solution) {
            math_solved = true;
            lv_obj_add_flag(numpad, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl_equation, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lbl_math_input, LV_OBJ_FLAG_HIDDEN);
            
            char stat_buf[64];
            int faltan = steps_quota - current_steps;
            if (faltan < 0) faltan = 0;
            sprintf(stat_buf, "DESBLOQUEADO\nCAMINA %d PASOS", faltan);
            lv_label_set_text(lbl_status, stat_buf);
            lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x00FF41), LV_PART_MAIN);
        } else {
            steps_quota += 5;
            math_input_buf[0] = '\0';
            char err_buf[64];
            sprintf(err_buf, "FALLO. +5 PASOS\nOBJETIVO: %d", steps_quota);
            lv_label_set_text(lbl_status, err_buf);
            lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xFF005C), LV_PART_MAIN);
            generate_math_equation();
        }
    } else {
        if (strlen(math_input_buf) < 4) strcat(math_input_buf, txt);
    }
    
    if (!math_solved && strcmp(txt, "OK") != 0) {
        lv_label_set_text(lbl_math_input, math_input_buf);
    }
}

void build_alarm_ui(bool is_assault) {
    apagar_motor_exnd();

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (scr_alarm != NULL) {
            lv_obj_del(scr_alarm);
            scr_alarm = NULL;
        }
        
        scr_alarm = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr_alarm, lv_color_hex(0x061A1E), LV_PART_MAIN);
        lv_obj_clear_flag(scr_alarm, LV_OBJ_FLAG_SCROLLABLE);

        if (!is_assault) {
            lv_obj_t * title = lv_label_create(scr_alarm);
            lv_label_set_text(title, ">> DESPIERTA <<");
            lv_obj_set_style_text_color(title, lv_color_hex(0x00E5FF), LV_PART_MAIN);
            lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
            lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 80);

            lv_obj_t * btn_snz = lv_btn_create(scr_alarm);
            lv_obj_set_size(btn_snz, 220, 60);
            lv_obj_align(btn_snz, LV_ALIGN_CENTER, 0, -20);
            lv_obj_set_style_bg_color(btn_snz, lv_color_hex(0xFFB300), LV_PART_MAIN);
            lv_obj_t * lbl_snz = lv_label_create(btn_snz);
            lv_label_set_text(lbl_snz, "SNOOZE");
            lv_obj_set_style_text_color(lbl_snz, lv_color_black(), LV_PART_MAIN);
            lv_obj_center(lbl_snz);
            lv_obj_add_event_cb(btn_snz, btn_snooze_cb, LV_EVENT_CLICKED, NULL);

            lv_obj_t * btn_wake = lv_btn_create(scr_alarm);
            lv_obj_set_size(btn_wake, 220, 60);
            lv_obj_align(btn_wake, LV_ALIGN_CENTER, 0, 60);
            lv_obj_set_style_bg_color(btn_wake, lv_color_hex(0x00FF41), LV_PART_MAIN);
            lv_obj_t * lbl_wake = lv_label_create(btn_wake);
            lv_label_set_text(lbl_wake, "YA ESTOY DE PIE");
            lv_obj_set_style_text_color(lbl_wake, lv_color_black(), LV_PART_MAIN);
            lv_obj_center(lbl_wake);
            lv_obj_add_event_cb(btn_wake, btn_wakeup_cb, LV_EVENT_CLICKED, NULL);

        } else {
            lbl_status = lv_label_create(scr_alarm);
            lv_label_set_text(lbl_status, "SISTEMA BLOQUEADO");
            lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xFF005C), LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_20, LV_PART_MAIN);
            lv_obj_set_style_text_align(lbl_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
            lv_obj_align(lbl_status, LV_ALIGN_TOP_MID, 0, 40);

            lbl_equation = lv_label_create(scr_alarm);
            lv_obj_set_style_text_color(lbl_equation, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl_equation, &lv_font_montserrat_20, LV_PART_MAIN);
            lv_obj_align(lbl_equation, LV_ALIGN_TOP_MID, 0, 100);

            lbl_math_input = lv_label_create(scr_alarm);
            lv_label_set_text(lbl_math_input, "...");
            lv_obj_set_style_text_color(lbl_math_input, lv_color_hex(0x00E5FF), LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl_math_input, &lv_font_montserrat_20, LV_PART_MAIN);
            lv_obj_align(lbl_math_input, LV_ALIGN_TOP_MID, 0, 140);

            numpad = lv_btnmatrix_create(scr_alarm);
            lv_btnmatrix_set_map(numpad, btnm_map);
            lv_obj_set_size(numpad, 300, 240);
            lv_obj_align(numpad, LV_ALIGN_BOTTOM_MID, 0, -20);
            lv_obj_set_style_bg_color(numpad, lv_color_hex(0x111111), LV_PART_MAIN);
            lv_obj_set_style_border_width(numpad, 0, LV_PART_MAIN);
            lv_obj_clear_flag(numpad, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_event_cb(numpad, numpad_cb, LV_EVENT_VALUE_CHANGED, NULL);

            generate_math_equation();
        }
        
        lv_scr_load(scr_alarm);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

void transition_to_assault() {
    current_alarm_state = ASSAULT;
    math_solved = false;
    steps_quota = current_steps + 30;
    build_alarm_ui(true);
}

void process_alarm_state_machine() {
    if (current_alarm_state == STANDBY) return;

    uint32_t now = millis();
    uint32_t elapsed = now - state_timer;

    if (current_alarm_state == ALARM_BASE) {
        if (now % 1000 < 50) trigger_drv2605_chaos(); 
        trigger_pcm_siren(false); 
    }
    else if (current_alarm_state == SNOOZE) {
        if (elapsed > active_snooze_ms) { 
            current_alarm_state = VIGILANCE;
            state_timer = millis();
        }
    }
    else if (current_alarm_state == VIGILANCE) {
        if (elapsed > 5000) {
            if (kin_actual_bpm < 5.0f) { 
                transition_to_assault();
            }
        }
        if (elapsed > 15000) abort_alarm(); 
    }
    else if (current_alarm_state == ASSAULT) {
        if (now % 400 < 50) trigger_drv2605_chaos(); 
        trigger_pcm_siren(true); 
        
        if (math_solved) {
            int steps_left = steps_quota - current_steps;
            if (steps_left <= 0) {
                abort_alarm();
            } else {
                char buf[32];
                sprintf(buf, "FALTAN %d PASOS", steps_left);
                if (lbl_status) lv_label_set_text(lbl_status, buf);
            }
        }
    }
}