#include "KinEngine.h"
#include <LilyGoLib.h> 
#include <Preferences.h>

extern LilyGoUltra& instance; 
Preferences prefs;

// Mapa de Silicio Absoluto
#define FUSER_ID_STEP      52
#define FUSER_ID_ACTIVITY  63
#define FUSER_ID_STEP      52
#define FUSER_ID_ACTIVITY  63
#define FUSER_ID_GRAVITY   4  // NUEVO: Vector Gravitacional

// Telemetría Viva y Perfiles NVS
volatile float kin_actual_bpm = 0.0f; 
volatile float kin_target_walk = 110.0f;
volatile float kin_target_run = 150.0f;

volatile uint32_t current_steps = 0;
volatile uint8_t current_activity = 0;
volatile bool is_moving = false;

// Ruteo Inercial
static uint32_t last_step_time = 0;
static uint32_t last_step_count = 0;
volatile float kin_actual_shock = 1.0f; 

// Trackers de Calibración en la Sombra
static uint32_t walk_start_ms = 0;
static uint32_t walk_start_steps = 0;
static uint32_t run_start_ms = 0;
static uint32_t run_start_steps = 0;

void step_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data) {
    if (size >= 4) current_steps = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
}

volatile uint8_t kin_event_latch = 0;
volatile int16_t kin_raw_gx = 0;
volatile int16_t kin_raw_gy = 0;
volatile int16_t kin_raw_gz = 0;
volatile bool kin_tilt_detected = false; // NUEVO: Gatillo aislado para Tilt-to-Wake

void activity_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data) {
    if (size >= 2) {
        uint16_t mask = (uint16_t)(data[0] | (data[1] << 8));
        
        if (mask     & 0x0200) { current_activity = 1; is_moving = true; } 
        else if (mask & 0x0400) { current_activity = 2; is_moving = true; } 
        else if ((mask & 0x0002) || (mask & 0x0004) || (mask & 0x0100)) {
            current_activity = 0;
            is_moving = false;
        }

        // Fuego visual inmediato aislado
        if (mask & 0x0100) kin_event_latch = 1;         // Paro abrupto / Choque
        if (mask & 0x2000) {
            kin_tilt_detected = true;    // Gesto de mirar el reloj (Tilt-to-Wake)
            Serial.println("[KIN] GESTO TILT-TO-WAKE DETECTADO (Mask 0x2000)");
        }
    }
}

void gravity_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data) {
    if (size >= 6) {
        // Combinación de 16-bits (LSB primero)
        kin_raw_gx = (int16_t)(data[0] | (data[1] << 8));
        kin_raw_gy = (int16_t)(data[2] | (data[3] << 8));
        kin_raw_gz = (int16_t)(data[4] | (data[5] << 8));
    }
}

void init_kin_engine() {
    // Extracción de perfiles biomecánicos de la Flash
    prefs.begin("kin_data", false);
    kin_target_walk = prefs.getFloat("tgt_w", 110.0f);
    kin_target_run = prefs.getFloat("tgt_r", 150.0f);

    instance.sensor.onResultEvent((BoschVirtualSensor::BoschSensorID)FUSER_ID_STEP, step_callback, nullptr);
    instance.sensor.onResultEvent((BoschVirtualSensor::BoschSensorID)FUSER_ID_ACTIVITY, activity_callback, nullptr);
    instance.sensor.onResultEvent((BoschVirtualSensor::BoschSensorID)FUSER_ID_GRAVITY, gravity_callback, nullptr);
    instance.sensor.configure(FUSER_ID_GRAVITY, 50.0f, 0); // 50Hz: Cero lag de fase en el giro

    instance.sensor.configure(FUSER_ID_STEP, 1.0f, 0); 
    instance.sensor.configure(FUSER_ID_ACTIVITY, 1.0f, 0); 

    Serial.printf("[*] KinEngine: Shadow Calibration Activa. Walk: %.1f | Run: %.1f\n", kin_target_walk, kin_target_run);
}

void kin_loop_handler() {
    instance.sensor.update(); 
    uint32_t current_time = millis();

    // ========================================================
    // 1. CÁLCULO INSTANTÁNEO SPM
    // ========================================================
    if (current_steps > last_step_count) {
        if (last_step_time != 0) {
            uint32_t steps_diff = current_steps - last_step_count;
            uint32_t delta_ms = current_time - last_step_time;
            float time_per_step = (float)delta_ms / (float)steps_diff;
            
            if (time_per_step > 250.0f && time_per_step < 2000.0f) {
                float instant_spm = 60000.0f / time_per_step;
                kin_actual_bpm = (kin_actual_bpm == 0.0f) ? instant_spm : (kin_actual_bpm * 0.7f) + (instant_spm * 0.3f);
            }
        }
        last_step_time = current_time;
        last_step_count = current_steps;
    }

    if (current_time - last_step_time > 2500) kin_actual_bpm = 0.0f;

    // ========================================================
    // 2. SHADOW CALIBRATION (Filtro EMA en ventanas de 90s)
    // ========================================================
    if (current_activity == 1) { // Perfil Caminata
        run_start_ms = 0; 
        if (walk_start_ms == 0) {
            walk_start_ms = current_time;
            walk_start_steps = current_steps;
        } else if (current_time - walk_start_ms >= 90000) { 
            float avg_spm = (float)(current_steps - walk_start_steps) / 1.5f; 
            
            // Limitador de cordura biomecánica (Rechazo de anomalías)
            if (avg_spm >= 80.0f && avg_spm <= 140.0f) {
                kin_target_walk = (kin_target_walk * 0.95f) + (avg_spm * 0.05f);
                prefs.putFloat("tgt_w", kin_target_walk);
                Serial.printf("[+] NVS: Caminata recalibrada a %.1f SPM\n", kin_target_walk);
            }
            walk_start_ms = current_time;
            walk_start_steps = current_steps;
        }
    } 
    else if (current_activity == 2) { // Perfil Trote
        walk_start_ms = 0;
        if (run_start_ms == 0) {
            run_start_ms = current_time;
            run_start_steps = current_steps;
        } else if (current_time - run_start_ms >= 90000) {
            float avg_spm = (float)(current_steps - run_start_steps) / 1.5f;
            
            if (avg_spm >= 130.0f && avg_spm <= 220.0f) {
                kin_target_run = (kin_target_run * 0.95f) + (avg_spm * 0.05f);
                prefs.putFloat("tgt_r", kin_target_run);
                Serial.printf("[+] NVS: Trote recalibrado a %.1f SPM\n", kin_target_run);
            }
            run_start_ms = current_time;
            run_start_steps = current_steps;
        }
    } 
    else { // Interrupción absoluta del tracking (Semáforos, vehículos, reposo)
        walk_start_ms = 0;
        run_start_ms = 0;
    }
}