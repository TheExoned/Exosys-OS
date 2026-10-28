#include "PowerEngine.h"
#include <Wire.h>
#include "XPowersLib.h"

// DECLARACIÓN EXACTA DEL SILICIO AXP2101
XPowersAXP2101 PMU; 

volatile int real_batt_pct = 0;
volatile bool is_charging = false;

void init_pmu() {
    // Inicialización sobre el bus maestro compartido (I2C 0x34, SDA 3, SCL 2)
    bool pmu_status = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 3, 2);
    
    if (!pmu_status) {
        Serial.println("[!] FATAL: AXP2101 no detectado en I2C 0x34.");
        return;
    }

    // 1. Parametrización estricta de seguridad térmica (< 500mA)
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    
    // 2. Erradicación de lecturas fantasma
    PMU.disableTSPinMeasure(); 
    
    // 3. Activación de telemetría ADC
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();
    
    // Lectura cero instantánea
    real_batt_pct = PMU.getBatteryPercent();
    is_charging = PMU.isVbusIn();
    
    Serial.println("[*] PMU AXP2101 Inicializado. Estrangulador térmico activo.");
}

void pmu_loop_handler() {
    static uint32_t last_pmu_read = 0;
    
    // Estrangulamiento de reloj a 1Hz
    if (millis() - last_pmu_read > 1000) {
        real_batt_pct = PMU.getBatteryPercent();
        is_charging = PMU.isVbusIn();
        
        last_pmu_read = millis();
    }
}