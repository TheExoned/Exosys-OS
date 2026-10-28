#include "NetworkEngine.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <vector>
#include <string>
#include <algorithm>
#include <TJpg_Decoder.h>
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>

extern SemaphoreHandle_t lvgl_mutex;
extern volatile bool pausa_lectura_sd;
extern volatile bool ghost_miner_parked;
extern lv_obj_t * scr_selector; 

static lv_obj_t* create_tactical_rail(lv_obj_t* parent, int16_t x_ofs, int16_t y_ofs, uint32_t hex_color, int dir, lv_obj_t** out_fill);

// --- PUNTEROS DE INTERFAZ DE RED ---
lv_obj_t * net_screen_ptr = NULL;
lv_obj_t * scr_settings_hotspot = NULL;
lv_obj_t * scr_settings_bt = NULL;

static lv_obj_t * lbl_hs_state = NULL;
static lv_obj_t * btn_hs_action = NULL;
static lv_obj_t * lbl_hs_action = NULL;
static lv_obj_t * modal_qr_hs = NULL;
static uint16_t * qr_canvas_buf = NULL;

static lv_obj_t * lbl_bt_state = NULL;
static lv_obj_t * bar_bt_rssi = NULL;
static lv_obj_t * btn_bt_action = NULL;
static lv_obj_t * lbl_bt_action = NULL;
// Nodos del semáforo GNSS en UI
static lv_obj_t * lbl_bt_gnss_status = NULL;

static lv_timer_t * wireless_ui_timer = NULL;
static lv_obj_t * btn_hs_video = NULL;

// Ruteo inteligente si se abrio desde el Main Menu (Hub)
static bool nav_from_main_hub = false;

// --- VARIABLES GNSS, BLE Y LORA ---
volatile float remote_gnss_lat = 0.0f;
volatile float remote_gnss_lon = 0.0f;
volatile float remote_gnss_alt = 0.0f;
volatile float remote_gnss_spd = 0.0f;
volatile uint32_t remote_gnss_rx_count = 0;
volatile uint32_t remote_gnss_last_ms = 0;

static const NimBLEUUID EXOSYS_BLE_UUID("E20A39F4-73F5-4BC4-A12F-17D1AD07A961");
static bool nimble_stack_ready = false;
volatile bool ble_scanning = false;
static volatile bool ble_bg_link_enabled = false;
static volatile bool lora_radio_enabled = false;

volatile uint32_t ble_exosys_last_seen_ms = 0;
volatile int ble_exosys_rssi = -100;

static void ensure_nimble_init() {
    if (!nimble_stack_ready) {
        NimBLEDevice::init("");
        nimble_stack_ready = true;
    }
}

static void parse_pos_payload_str(const char* buf, int len) {
    if (len < 10 || strncmp(buf, "POS:", 4) != 0) return;
    float lat = 0.0f, lon = 0.0f, alt = 0.0f, spd = 0.0f;
    if (sscanf(buf + 4, "%f,%f,%f,%f", &lat, &lon, &alt, &spd) >= 2) {
        if (lat != 0.0f || lon != 0.0f) {
            remote_gnss_lat = lat;
            remote_gnss_lon = lon;
            remote_gnss_alt = alt;
            remote_gnss_spd = spd;
            remote_gnss_rx_count = remote_gnss_rx_count + 1UL;
            remote_gnss_last_ms = millis();
        }
    }
}

class RadarBleCallback: public NimBLEScanCallbacks {
public:
    void processDevice(const NimBLEAdvertisedDevice* device) {
        if (device == NULL) return;
        std::string mfg = device->getManufacturerData();
        bool has_exosys_mfg = (mfg.size() >= 14 && (uint8_t)mfg[0] == 0x0A && (uint8_t)mfg[1] == 0xE2);
        bool is_exosys_beacon = has_exosys_mfg || device->isAdvertisingService(EXOSYS_BLE_UUID);

        if (is_exosys_beacon) {
            ble_exosys_last_seen_ms = millis();
            ble_exosys_rssi = device->getRSSI();

            if (has_exosys_mfg) {
                float lat = 0.0f, lon = 0.0f;
                int16_t alt_m = 0, spd_tenths = 0;
                memcpy(&lat, &mfg[2], 4);
                memcpy(&lon, &mfg[6], 4);
                memcpy(&alt_m, &mfg[10], 2);
                memcpy(&spd_tenths, &mfg[12], 2);

                if (lat != 0.0f || lon != 0.0f) {
                    remote_gnss_lat = lat;
                    remote_gnss_lon = lon;
                    remote_gnss_alt = (float)alt_m;
                    remote_gnss_spd = ((float)spd_tenths) * 0.1f;
                    remote_gnss_rx_count = remote_gnss_rx_count + 1UL;
                    remote_gnss_last_ms = millis();
                }
            }
        }
    }
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) { processDevice(advertisedDevice); }
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) { processDevice(advertisedDevice); }
};

static RadarBleCallback ble_scan_cb_instance;

#define MAX_SCANNED_NETS 15
#define VISION_W 368
#define VISION_H 464
#define FULL_MODE_W 208
#define RAIL_W 36
#define RAIL_H 224
#define HOLD_TIME_MS 5000

struct ScannedNet { char ssid[64]; bool is_open; };
static ScannedNet scanned_nets[MAX_SCANNED_NETS];

static lv_obj_t * scr_wifi_scan = NULL;
static lv_obj_t * list_wifi = NULL;
static lv_obj_t * lbl_wifi_status = NULL;

static lv_obj_t * scr_wifi_pass = NULL;
static lv_obj_t * ta_wifi_pass = NULL;
static lv_obj_t * kb_wifi_pass = NULL;

static lv_obj_t * scr_vision = NULL;
static lv_obj_t * img_vision = NULL;
static lv_obj_t * lbl_vision_status = NULL;
static lv_img_dsc_t vision_img_dsc;

static lv_obj_t * rail_up_r = NULL;
static lv_obj_t * rail_down_r = NULL;
static lv_obj_t * fill_up_r = NULL;
static lv_obj_t * fill_down_r = NULL;
static lv_obj_t * border_full_l = NULL;
static lv_obj_t * border_full_r = NULL;

static uint32_t rail_press_start = 0;
static bool rail_hold_triggered = false;

static lv_obj_t * popup_link = NULL;
static lv_timer_t * popup_timer = NULL;

static uint8_t * vision_vram = NULL;     
static uint8_t * udp_jpeg_buffer = NULL; 

WiFiUDP udp_vision;
static IPAddress remote_phone_ip;
static volatile bool has_remote_ip = false;
static volatile uint32_t last_app_udp_rx_ms = 0;
static uint32_t last_hello_beacon_ms = 0;

static bool wifi_scan_from_settings = false;
volatile bool hotspot_bg_connecting = false;
static uint32_t hotspot_connect_start_ms = 0;
static uint32_t wifi_retry_at_ms = 0;

volatile int8_t pending_scroll_steps = 0;
volatile uint8_t pending_mode_cmd = 0;

struct PendingTouch { int16_t x1, y1, x2, y2; uint16_t dur; };
static volatile PendingTouch pending_touch = {0, 0, 0, 0, 0};
static volatile bool has_pending_touch = false;
static lv_obj_t * touch_reticle = NULL;
static lv_timer_t * reticle_timer = NULL;

volatile bool vision_active = false;
volatile bool wifi_scanning = false;
volatile bool wifi_connecting = false;
volatile bool wifi_connection_failed = false; 
volatile bool udp_started = false;
volatile bool vision_task_running = false;
volatile bool frame_ready_flag = false;
volatile bool trigger_ack_popup = false;
volatile int16_t active_x_offset = 0;
static bool link_confirmed = false;

char selected_ssid[64] = "";
char selected_pass[64] = "";

// =========================================================
// GENERADOR NATIVO DE CODIGO QR (ISO/IEC 18004 VERSION 2-L)
// =========================================================
static uint8_t gf256_mul(uint8_t a, uint8_t b) {
    uint8_t p = 0;
    for (int i = 0; i < 8; i++) {
        if (b & 1) p ^= a;
        bool hi = (a & 0x80) != 0;
        a <<= 1;
        if (hi) a ^= 0x1D;
        b >>= 1;
    }
    return p;
}

static void encode_qr_v2_matrix(const char* text, uint8_t matrix[25][25]) {
    uint8_t is_func[25][25];
    memset(matrix, 0, 25 * 25); memset(is_func, 0, 25 * 25);

    auto place_finder = [&](int r0, int c0) {
        for (int dr = -1; dr <= 7; dr++) {
            for (int dc = -1; dc <= 7; dc++) {
                int r = r0 + dr, c = c0 + dc;
                if (r < 0 || r >= 25 || c < 0 || c >= 25) continue;
                bool dark = (dr >= 0 && dr <= 6 && dc >= 0 && dc <= 6) &&
                            (dr == 0 || dr == 6 || dc == 0 || dc == 6 || (dr >= 2 && dr <= 4 && dc >= 2 && dc <= 4));
                matrix[r][c] = dark ? 1 : 0; is_func[r][c] = 1;
            }
        }
    };
    place_finder(0, 0); place_finder(0, 18); place_finder(18, 0);

    for (int dr = -2; dr <= 2; dr++) {
        for (int dc = -2; dc <= 2; dc++) {
            int r = 18 + dr, c = 18 + dc;
            bool dark = (dr == -2 || dr == 2 || dc == -2 || dc == 2 || (dr == 0 && dc == 0));
            matrix[r][c] = dark ? 1 : 0; is_func[r][c] = 1;
        }
    }
    for (int i = 8; i <= 16; i++) {
        matrix[6][i] = (i % 2 == 0) ? 1 : 0; is_func[6][i] = 1;
        matrix[i][6] = (i % 2 == 0) ? 1 : 0; is_func[i][6] = 1;
    }
    for (int i = 0; i <= 8; i++) { is_func[8][i] = 1; is_func[i][8] = 1; }
    for (int i = 17; i < 25; i++) { is_func[8][i] = 1; is_func[i][8] = 1; }
    matrix[17][8] = 1; is_func[17][8] = 1;

    size_t len = strlen(text);
    if (len > 32) len = 32;
    uint8_t cw[44]; memset(cw, 0, sizeof(cw));

    int bit_pos = 0;
    auto push_bits = [&](uint16_t val, int nbits) {
        for (int i = nbits - 1; i >= 0; i--) {
            if ((val >> i) & 1) cw[bit_pos >> 3] |= (1 << (7 - (bit_pos & 7)));
            bit_pos++;
        }
    };

    push_bits(0b0100, 4); push_bits((uint16_t)len, 8);
    for (size_t i = 0; i < len; i++) push_bits((uint8_t)text[i], 8);
    push_bits(0, 4);
    while ((bit_pos & 7) != 0) push_bits(0, 1);

    uint8_t pad_byte = 0xEC;
    while ((bit_pos >> 3) < 34) {
        push_bits(pad_byte, 8);
        pad_byte = (pad_byte == 0xEC) ? 0x11 : 0xEC;
    }

    uint8_t gen[10] = {216, 194, 159, 111, 199, 94, 95, 113, 157, 193};
    uint8_t ecc[10]; memset(ecc, 0, sizeof(ecc));

    for (int i = 0; i < 34; i++) {
        uint8_t factor = cw[i] ^ ecc[0];
        for (int j = 0; j < 9; j++) ecc[j] = ecc[j + 1] ^ gf256_mul(gen[j], factor);
        ecc[9] = gf256_mul(gen[9], factor);
    }
    for (int i = 0; i < 10; i++) cw[34 + i] = ecc[i];

    int bit_idx = 0; bool upward = true;
    for (int right = 24; right >= 1; right -= 2) {
        if (right == 6) right = 5;
        for (int vert = 0; vert < 25; vert++) {
            int r = upward ? (24 - vert) : vert;
            for (int j = 0; j < 2; j++) {
                int c = right - j;
                if (!is_func[r][c]) {
                    bool bit = false;
                    if (bit_idx < 352) { bit = ((cw[bit_idx >> 3] >> (7 - (bit_idx & 7))) & 1) != 0; bit_idx++; }
                    if (((r + c) & 1) == 0) bit = !bit;
                    matrix[r][c] = bit ? 1 : 0;
                }
            }
        }
        upward = !upward;
    }

    const uint16_t fmt = 0x77C4;
    const int r_tl[15] = {8,8,8,8,8,8,8,8,7,5,4,3,2,1,0};
    const int c_tl[15] = {0,1,2,3,4,5,7,8,8,8,8,8,8,8,8};
    const int r_br[15] = {24,23,22,21,20,19,18,8,8,8,8,8,8,8,8};
    const int c_br[15] = {8,8,8,8,8,8,8,17,18,19,20,21,22,23,24};

    for (int i = 0; i < 15; i++) {
        uint8_t b = ((fmt >> (14 - i)) & 1);
        matrix[r_tl[i]][c_tl[i]] = b; matrix[r_br[i]][c_br[i]] = b;
    }
}

static void close_qr_modal_cb(lv_event_t * e) {
    if (modal_qr_hs != NULL) {
        lv_obj_del(modal_qr_hs);
        modal_qr_hs = NULL;
    }
    if (qr_canvas_buf != NULL) {
        heap_caps_free(qr_canvas_buf);
        qr_canvas_buf = NULL;
    }
}

void open_qr_hotspot_modal_cb(lv_event_t * e) {
    if (modal_qr_hs != NULL) return;
    
    String ipStr = "0.0.0.0";
    if (WiFi.status() == WL_CONNECTED) ipStr = WiFi.localIP().toString();
    
    modal_qr_hs = lv_obj_create(scr_settings_hotspot);
    lv_obj_set_size(modal_qr_hs, 390, 482);
    lv_obj_center(modal_qr_hs);
    lv_obj_set_style_bg_color(modal_qr_hs, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal_qr_hs, 245, 0);
    lv_obj_set_style_border_width(modal_qr_hs, 0, 0);
    lv_obj_clear_flag(modal_qr_hs, LV_OBJ_FLAG_SCROLLABLE);

    if (ipStr == "0.0.0.0") {
        lv_obj_t * lbl_err = lv_label_create(modal_qr_hs);
        lv_label_set_text(lbl_err, ">> ERROR DE ENLACE <<\n\nEl reloj no tiene red activa.\nConecta el Hotspot primero\npara generar una IP valida.");
        lv_obj_set_style_text_color(lbl_err, lv_color_hex(0xFF003C), 0);
        lv_obj_set_style_text_align(lbl_err, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(lbl_err, &lv_font_montserrat_16, 0);
        lv_obj_center(lbl_err);
    } else {
        const int QR_PX = 232; 
        if (qr_canvas_buf != NULL) { heap_caps_free(qr_canvas_buf); qr_canvas_buf = NULL; }
        qr_canvas_buf = (uint16_t*)heap_caps_malloc(QR_PX * QR_PX * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        if (!qr_canvas_buf) return;

        char payload[40];
        snprintf(payload, sizeof(payload), "EXOSYS_IP:%s", ipStr.c_str());

        uint8_t mat[25][25];
        encode_qr_v2_matrix(payload, mat);

        for (int y = 0; y < QR_PX; y++) {
            int mr = (y / 8) - 2;
            for (int x = 0; x < QR_PX; x++) {
                int mc = (x / 8) - 2;
                bool dark = (mr >= 0 && mr < 25 && mc >= 0 && mc < 25) && (mat[mr][mc] != 0);
                qr_canvas_buf[y * QR_PX + x] = dark ? 0x0000 : 0xFFFF;
            }
        }

        lv_obj_t * lbl_t = lv_label_create(modal_qr_hs);
        lv_label_set_text(lbl_t, "ESCANEAR EN APP EXOSYS");
        lv_obj_set_style_text_color(lbl_t, lv_color_hex(0xFFB300), 0);
        lv_obj_set_style_text_font(lbl_t, &lv_font_montserrat_16, 0);
        lv_obj_align(lbl_t, LV_ALIGN_TOP_MID, 0, 22);

        lv_obj_t * canvas = lv_canvas_create(modal_qr_hs);
        lv_canvas_set_buffer(canvas, qr_canvas_buf, QR_PX, QR_PX, LV_COLOR_FORMAT_RGB565);
        lv_obj_align(canvas, LV_ALIGN_CENTER, 0, -15);

        lv_obj_t * lbl_ip = lv_label_create(modal_qr_hs);
        lv_label_set_text(lbl_ip, payload);
        lv_obj_set_style_text_color(lbl_ip, lv_color_hex(0x00FF41), 0);
        lv_obj_set_style_text_font(lbl_ip, &lv_font_montserrat_16, 0);
        lv_obj_align(lbl_ip, LV_ALIGN_BOTTOM_MID, 0, -82);
    }

    lv_obj_t * btn_c = lv_btn_create(modal_qr_hs);
    lv_obj_set_size(btn_c, 300, 48);
    lv_obj_align(btn_c, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(btn_c, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_c = lv_label_create(btn_c);
    lv_label_set_text(lbl_c, "CERRAR");
    lv_obj_center(lbl_c);
    lv_obj_add_event_cb(btn_c, close_qr_modal_cb, LV_EVENT_CLICKED, NULL);
}

// =========================================================
// MAQUINA DE ESTADOS: UI DE REDES TACTICAS
// =========================================================
void refresh_wireless_status_ui(lv_timer_t * t) {
    lv_obj_t * act = lv_scr_act();
    if (act != scr_settings_hotspot && act != scr_settings_bt && t != NULL) return;

    // 1. GESTOR DE HOTSPOT
    if (lbl_hs_state != NULL && btn_hs_action != NULL) {
        bool connected = (WiFi.status() == WL_CONNECTED);
        bool app_linked = connected && net_hotspot_is_app_linked();

        if (btn_hs_video != NULL) {
            lv_obj_clear_flag(btn_hs_video, LV_OBJ_FLAG_HIDDEN);
            if (connected) {
                lv_obj_set_style_bg_color(btn_hs_video, lv_color_hex(0x00E5FF), 0);
                lv_obj_set_style_text_color(lv_obj_get_child(btn_hs_video, 0), lv_color_black(), 0);
            } else {
                lv_obj_set_style_bg_color(btn_hs_video, lv_color_hex(0x333333), 0);
                lv_obj_set_style_text_color(lv_obj_get_child(btn_hs_video, 0), lv_color_hex(0x888888), 0);
            }
        }

        if (connected) {
            char st_buf[112];
            snprintf(st_buf, sizeof(st_buf), "%s\nRed: %s (%d dBm)\n[Toca aqui para ver QR]",
                     app_linked ? "ENLAZADO A LA APP" : "ENLAZADO (Esperando Video)", WiFi.SSID().c_str(), WiFi.RSSI());
            lv_label_set_text(lbl_hs_state, st_buf);
            lv_obj_set_style_text_color(lbl_hs_state, lv_color_hex(app_linked ? 0x00FF41 : 0x00E5FF), 0);
            
            lv_label_set_text(lbl_hs_action, "DESCONECTAR HOTSPOT");
            lv_obj_set_style_bg_color(btn_hs_action, lv_color_hex(0xFF003C), 0);
            lv_obj_set_style_text_color(lbl_hs_action, lv_color_white(), 0);
        } else if (wifi_connecting || hotspot_bg_connecting) {
            lv_label_set_text(lbl_hs_state, "NEGOCIANDO CONEXION...");
            lv_obj_set_style_text_color(lbl_hs_state, lv_color_hex(0xFFB300), 0);
            
            lv_label_set_text(lbl_hs_action, "CANCELAR ENLACE");
            lv_obj_set_style_bg_color(btn_hs_action, lv_color_hex(0xFF003C), 0);
            lv_obj_set_style_text_color(lbl_hs_action, lv_color_white(), 0);
        } else if (wifi_connection_failed) {
            lv_label_set_text(lbl_hs_state, "FALLO DE RED (O clave erronea).");
            lv_obj_set_style_text_color(lbl_hs_state, lv_color_hex(0xFF003C), 0);
            
            lv_label_set_text(lbl_hs_action, "RECONFIGURAR RED");
            lv_obj_set_style_bg_color(btn_hs_action, lv_color_hex(0xFFB300), 0);
            lv_obj_set_style_text_color(lbl_hs_action, lv_color_black(), 0);
        } else {
            lv_label_set_text(lbl_hs_state, "APAGADO");
            lv_obj_set_style_text_color(lbl_hs_state, lv_color_hex(0x666666), 0);
            
            lv_label_set_text(lbl_hs_action, "AUTO-ENLAZAR");
            lv_obj_set_style_bg_color(btn_hs_action, lv_color_hex(0x00E5FF), 0);
            lv_obj_set_style_text_color(lbl_hs_action, lv_color_black(), 0);
        }
    }

    // 2. GESTOR DEL RASTREADOR BLE Y SEMÁFORO GNSS
    if (lbl_bt_state != NULL && bar_bt_rssi != NULL && btn_bt_action != NULL) {
        bool ble_on = net_ble_is_enabled();
        
        // Bloque del semáforo GNSS en la parte inferior del menú BT
        if (lbl_bt_gnss_status != NULL) {
            uint32_t now_ms = millis();
            bool phone_valid = (remote_gnss_rx_count > 0) &&
                               (remote_gnss_last_ms > 0) &&
                               ((now_ms - remote_gnss_last_ms) <= 30000UL) &&
                               (remote_gnss_lat != 0.0f || remote_gnss_lon != 0.0f);
            
            if (phone_valid) {
                lv_label_set_text_fmt(lbl_bt_gnss_status, "GNSS ACTIVO (%u PULSOS)\nLAT: %.5f\nLON: %.5f", 
                                      (unsigned int)remote_gnss_rx_count, remote_gnss_lat, remote_gnss_lon);
                lv_obj_set_style_text_color(lbl_bt_gnss_status, lv_color_hex(0x00FF41), 0);
            } else if (remote_gnss_rx_count > 0) {
                lv_label_set_text(lbl_bt_gnss_status, "GNSS INACTIVO (SENAL PERDIDA)");
                lv_obj_set_style_text_color(lbl_bt_gnss_status, lv_color_hex(0xFF003C), 0);
            } else {
                lv_label_set_text(lbl_bt_gnss_status, "GNSS INACTIVO (ESPERANDO TELEFONO)");
                lv_obj_set_style_text_color(lbl_bt_gnss_status, lv_color_hex(0x666666), 0);
            }
        }

        if (!ble_on) {
            lv_label_set_text(lbl_bt_state, "BUSQUEDA APAGADA");
            lv_obj_set_style_text_color(lbl_bt_state, lv_color_hex(0x666666), 0);
            lv_bar_set_value(bar_bt_rssi, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(bar_bt_rssi, lv_color_hex(0x333333), LV_PART_INDICATOR);
            
            lv_label_set_text(lbl_bt_action, "INICIAR RASTREO");
            lv_obj_set_style_bg_color(btn_bt_action, lv_color_hex(0x00E5FF), 0);
            lv_obj_set_style_text_color(lbl_bt_action, lv_color_black(), 0);
        } else {
            lv_label_set_text(lbl_bt_action, "DETENER RASTREO");
            lv_obj_set_style_bg_color(btn_bt_action, lv_color_hex(0xFF003C), 0);
            lv_obj_set_style_text_color(lbl_bt_action, lv_color_white(), 0);

            if ((millis() - ble_exosys_last_seen_ms) < 3000UL && ble_exosys_last_seen_ms > 0) {
                char bs[32];
                snprintf(bs, sizeof(bs), "SENAL DETECTADA: %d dBm", ble_exosys_rssi);
                lv_label_set_text(lbl_bt_state, bs);
                lv_obj_set_style_text_color(lbl_bt_state, lv_color_hex(0x00E5FF), 0);
                
                int pct = map(ble_exosys_rssi, -100, -40, 5, 100);
                if (pct < 0) pct = 0; if (pct > 100) pct = 100;
                lv_bar_set_value(bar_bt_rssi, pct, LV_ANIM_ON);

                uint32_t c_bar = 0x00E5FF;
                if (pct >= 80) c_bar = 0x00FF41;
                else if (pct <= 30) c_bar = 0xFF003C;
                else if (pct <= 60) c_bar = 0xFFB300;
                
                lv_obj_set_style_bg_color(bar_bt_rssi, lv_color_hex(c_bar), LV_PART_INDICATOR);
                lv_obj_set_style_shadow_color(bar_bt_rssi, lv_color_hex(c_bar), LV_PART_INDICATOR);
            } else {
                lv_label_set_text(lbl_bt_state, "BUSCANDO BEACON...");
                lv_obj_set_style_text_color(lbl_bt_state, lv_color_hex(0xFFB300), 0);
                lv_bar_set_value(bar_bt_rssi, 0, LV_ANIM_ON);
                lv_obj_set_style_bg_color(bar_bt_rssi, lv_color_hex(0x333333), LV_PART_INDICATOR);
            }
        }
    }
}

static void send_hello_exosys_beacon() {
    if (!udp_started || WiFi.status() != WL_CONNECTED) return;

    IPAddress local = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    char msg[64];
    snprintf(msg, sizeof(msg), "HELLO_EXOSYS:%u.%u.%u.%u", local[0], local[1], local[2], local[3]);

    if (gw[0] != 0) {
        udp_vision.beginPacket(gw, 8889);
        udp_vision.print(msg);
        udp_vision.endPacket();
    }
    if (has_remote_ip && remote_phone_ip != gw) {
        udp_vision.beginPacket(remote_phone_ip, 8889);
        udp_vision.print(msg);
        udp_vision.endPacket();
    }
    IPAddress bcast(local[0], local[1], local[2], 255);
    udp_vision.beginPacket(bcast, 8889);
    udp_vision.print(msg);
    udp_vision.endPacket();
}

bool net_wifi_is_connected() {
    return (WiFi.status() == WL_CONNECTED);
}

bool net_hotspot_is_app_linked() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (last_app_udp_rx_ms == 0) return false;
    return (millis() - last_app_udp_rx_ms) <= 12000UL;
}

static void save_hotspot_credentials_nvs(const char* ssid, const char* pass) {
    if (!ssid || ssid[0] == '\0') return;
    Preferences prefs;
    prefs.begin("exoned_net", false);
    prefs.putString("hs_ssid", ssid);
    prefs.putString("hs_pass", pass ? pass : "");
    prefs.end();
}

bool net_has_saved_hotspot(char* out_ssid, size_t max_len) {
    Preferences prefs;
    prefs.begin("exoned_net", true);
    String s = prefs.getString("hs_ssid", "");
    prefs.end();
    if (s.length() == 0) return false;
    if (out_ssid && max_len > 0) {
        strncpy(out_ssid, s.c_str(), max_len - 1);
        out_ssid[max_len - 1] = '\0';
    }
    return true;
}

static void shutdown_wifi_phy_silent() {
    hotspot_bg_connecting = false;
    wifi_connecting = false;
    wifi_scanning = false;
    wifi_retry_at_ms = 0;
    has_remote_ip = false;
    last_app_udp_rx_ms = 0;
    vision_active = false;

    uint32_t t0 = millis();
    while (vision_task_running && (millis() - t0 < 800)) delay(5);

    if (udp_started) {
        udp_vision.stop();
        udp_started = false;
    }
    if (WiFi.getMode() != WIFI_OFF) {
        WiFi.scanDelete();
        WiFi.disconnect(true, true);
        delay(30);
        WiFi.mode(WIFI_OFF);
    }
}

void net_ble_set_enabled(bool enable) {
    if (enable) {
        wifi_connection_failed = false;
        shutdown_wifi_phy_silent();

        ensure_nimble_init();
        NimBLEScan* pScan = NimBLEDevice::getScan();
        pScan->stop();
        pScan->clearResults();
        pScan->setScanCallbacks(&ble_scan_cb_instance, true);
        pScan->setActiveScan(false);
        pScan->setMaxResults(0);
        pScan->setInterval(320); 
        pScan->setWindow(48);    
        pScan->start(0, false);
        ble_bg_link_enabled = true;
    } else {
        ble_bg_link_enabled = false;
        ble_scanning = false;
        ble_exosys_last_seen_ms = 0;
        if (nimble_stack_ready) {
            NimBLEDevice::getScan()->stop();
            NimBLEDevice::getScan()->clearResults();
            NimBLEDevice::deinit(true);
            nimble_stack_ready = false;
            delay(60);
        }
    }
}

bool net_ble_is_enabled() { return ble_bg_link_enabled || ble_scanning; }

bool net_ble_is_linked() {
    if (!net_ble_is_enabled() || ble_exosys_last_seen_ms == 0) return false;
    return (millis() - ble_exosys_last_seen_ms) <= 8000UL;
}

int net_ble_get_rssi() { return ble_exosys_rssi; }

void net_lora_set_enabled(bool enable) {
    lora_radio_enabled = enable;
}

bool net_lora_is_enabled() {
    return lora_radio_enabled;
}

static void trigger_clean_wifi_scan() {
    net_ble_set_enabled(false);

    hotspot_bg_connecting = false;
    wifi_connecting = false;
    wifi_scanning = false;
    wifi_retry_at_ms = 0;

    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);

    if (WiFi.getMode() != WIFI_STA) {
        WiFi.mode(WIFI_STA);
        uint32_t t0 = millis();
        while (WiFi.getMode() != WIFI_STA && (millis() - t0 < 500)) delay(10);
        delay(120);
    }

    WiFi.disconnect(false, true);
    delay(60);
    WiFi.scanDelete();

    int16_t res = WiFi.scanNetworks(true, true, false, 250);
    if (res == WIFI_SCAN_FAILED) {
        delay(120);
        res = WiFi.scanNetworks(true, true, false, 250);
    }

    if (res == WIFI_SCAN_FAILED) {
        wifi_retry_at_ms = millis() + 1000UL;
    } else {
        wifi_scanning = true;
    }
}

void start_hotspot_bg_connection(const char* ssid, const char* pass) {
    save_hotspot_credentials_nvs(ssid, pass);
    net_ble_set_enabled(false);
    
    WiFi.disconnect(true, true);
    delay(50);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, pass);
    hotspot_bg_connecting = true;
    wifi_connection_failed = false;
    hotspot_connect_start_ms = millis();
}

bool net_connect_saved_hotspot() {
    Preferences prefs;
    prefs.begin("exoned_net", true);
    String s = prefs.getString("hs_ssid", "");
    String p = prefs.getString("hs_pass", "");
    prefs.end();
    if (s.length() == 0) return false;

    strncpy(selected_ssid, s.c_str(), 63); selected_ssid[63] = '\0';
    strncpy(selected_pass, p.c_str(), 63); selected_pass[63] = '\0';

    start_hotspot_bg_connection(selected_ssid, selected_pass);
    return true;
}

void net_disconnect_hotspot() {
    wifi_connection_failed = false;
    shutdown_wifi_phy_silent();
    net_ble_set_enabled(true);
}

void net_open_hotspot_scanner_from_settings() {
    nav_from_main_hub = false;
    wifi_scan_from_settings = true;
    wifi_connection_failed = false;
    if (list_wifi) lv_obj_clean(list_wifi);
    lv_label_set_text(lbl_wifi_status, "BUSCANDO REDES 2.4 GHz...");
    lv_scr_load(scr_wifi_scan);
    trigger_clean_wifi_scan();
}

void net_open_hotspot_scanner_from_hub() {
    nav_from_main_hub = true;
    wifi_scan_from_settings = true;
    wifi_connection_failed = false;
    if (list_wifi) lv_obj_clean(list_wifi);
    lv_label_set_text(lbl_wifi_status, "BUSCANDO REDES 2.4 GHz...");
    lv_scr_load(scr_wifi_scan);
    trigger_clean_wifi_scan();
}

void net_open_sub_from_hub(lv_obj_t * target_scr) {
    if (target_scr == NULL) return;
    nav_from_main_hub = true;
    refresh_wireless_status_ui(NULL);
    lv_scr_load(target_scr);
}

static void queue_vision_cmd(uint8_t cmd_id) {
    if (cmd_id == 1) { if (pending_scroll_steps > -10) pending_scroll_steps = pending_scroll_steps - 1; } 
    else if (cmd_id == 2) { if (pending_scroll_steps < 10) pending_scroll_steps = pending_scroll_steps + 1; } 
    else { pending_mode_cmd = cmd_id; }
}

static void flush_pending_udp_cmds() {
    if (!udp_started) return;
    IPAddress target_ip = has_remote_ip ? remote_phone_ip : WiFi.gatewayIP();
    if (target_ip[0] == 0) return;

    if (pending_mode_cmd != 0) {
        uint8_t m = pending_mode_cmd;
        pending_mode_cmd = 0;
        udp_vision.beginPacket(target_ip, 8889);
        udp_vision.print((m == 3) ? "MODE_FULL" : "MODE_SEG");
        udp_vision.endPacket();
    }

    if (has_pending_touch) {
        int16_t tx1 = pending_touch.x1, ty1 = pending_touch.y1, tx2 = pending_touch.x2, ty2 = pending_touch.y2;
        uint16_t tdur = pending_touch.dur;
        has_pending_touch = false;
        char tbuf[64];
        snprintf(tbuf, sizeof(tbuf), "TOUCH:%d,%d,%d,%d,%u", tx1, ty1, tx2, ty2, tdur);
        udp_vision.beginPacket(target_ip, 8889);
        udp_vision.print(tbuf);
        udp_vision.endPacket();
    }

    while (pending_scroll_steps != 0) {
        const char* payload = (pending_scroll_steps < 0) ? "SCROLL_UP" : "SCROLL_DOWN";
        if (pending_scroll_steps < 0) pending_scroll_steps = pending_scroll_steps + 1; 
        else pending_scroll_steps = pending_scroll_steps - 1;
        udp_vision.beginPacket(target_ip, 8889);
        udp_vision.print(payload);
        udp_vision.endPacket();
    }
}

static void reset_rail_visuals(lv_obj_t* rail, lv_obj_t* fill_bar, uint32_t idle_hex) {
    if (rail) {
        lv_obj_set_style_bg_opa(rail, 0, LV_PART_MAIN);
        lv_obj_set_style_border_color(rail, lv_color_hex(idle_hex), LV_PART_MAIN);
    }
    if (fill_bar) lv_obj_set_height(fill_bar, 0);
}

static void set_ui_segmented_mode(bool segmented) {
    reset_rail_visuals(rail_up_r, fill_up_r, 0x00E5FF);
    reset_rail_visuals(rail_down_r, fill_down_r, 0xFFB300);

    if (segmented) {
        if (img_vision) lv_obj_align(img_vision, LV_ALIGN_CENTER, -8, 0);
        if (rail_up_r) { lv_obj_clear_flag(rail_up_r, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(rail_up_r); }
        if (rail_down_r) { lv_obj_clear_flag(rail_down_r, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(rail_down_r); }
        if (border_full_l) lv_obj_add_flag(border_full_l, LV_OBJ_FLAG_HIDDEN);
        if (border_full_r) lv_obj_add_flag(border_full_r, LV_OBJ_FLAG_HIDDEN);
        queue_vision_cmd(4);
    } else {
        if (img_vision) lv_obj_align(img_vision, LV_ALIGN_CENTER, 0, 0);
        if (rail_up_r) lv_obj_add_flag(rail_up_r, LV_OBJ_FLAG_HIDDEN);
        if (rail_down_r) lv_obj_add_flag(rail_down_r, LV_OBJ_FLAG_HIDDEN);
        if (border_full_l) { lv_obj_clear_flag(border_full_l, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(border_full_l); }
        if (border_full_r) { lv_obj_clear_flag(border_full_r, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(border_full_r); }
        queue_vision_cmd(3);
    }
}

static void rail_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * rail = lv_event_get_target_obj(e);
    int dir = (int)(uintptr_t)lv_event_get_user_data(e);
    lv_obj_t * fill_bar = (dir == 1) ? fill_up_r : fill_down_r;
    uint32_t idle_color = (dir == 1) ? 0x00E5FF : 0xFFB300;

    if (code == LV_EVENT_PRESSED) {
        rail_press_start = millis();
        rail_hold_triggered = false;
        lv_obj_set_style_bg_color(rail, lv_color_hex(0x00FF41), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(rail, 110, LV_PART_MAIN);
        lv_obj_set_style_border_color(rail, lv_color_hex(0x00FF41), LV_PART_MAIN);
        if (fill_bar) lv_obj_set_height(fill_bar, 0);
        if (dir == 1) queue_vision_cmd(1); else if (dir == 2) queue_vision_cmd(2);
    } 
    else if (code == LV_EVENT_PRESSING) {
        if (!rail_hold_triggered) {
            uint32_t elapsed = millis() - rail_press_start;
            if (elapsed >= HOLD_TIME_MS) {
                rail_hold_triggered = true;
                if (fill_bar) lv_obj_set_height(fill_bar, RAIL_H);
                set_ui_segmented_mode(false);
            } else {
                int16_t fill_h = (elapsed * RAIL_H) / HOLD_TIME_MS;
                if (fill_bar) lv_obj_set_height(fill_bar, fill_h);
            }
        }
    } 
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        reset_rail_visuals(rail, fill_bar, idle_color);
        rail_hold_triggered = false;
    }
}

static void full_border_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) set_ui_segmented_mode(true);
}

static lv_obj_t* create_tactical_rail(lv_obj_t* parent, int16_t x_ofs, int16_t y_ofs, uint32_t hex_color, int dir, lv_obj_t** out_fill) {
    lv_obj_t* rail = lv_obj_create(parent);
    lv_obj_set_size(rail, RAIL_W, RAIL_H);
    lv_obj_align(rail, LV_ALIGN_RIGHT_MID, x_ofs, y_ofs);
    lv_obj_set_style_bg_opa(rail, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(rail, lv_color_hex(hex_color), LV_PART_MAIN);
    lv_obj_set_style_border_width(rail, 4, LV_PART_MAIN);
    lv_obj_set_style_border_side(rail, LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_radius(rail, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(rail, 0, LV_PART_MAIN);
    lv_obj_clear_flag(rail, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(rail, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* fill = lv_obj_create(rail);
    lv_obj_set_size(fill, RAIL_W, 0);
    lv_obj_align(fill, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(fill, lv_color_hex(0xFF003C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(fill, 210, LV_PART_MAIN);
    lv_obj_set_style_border_width(fill, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(fill, 0, LV_PART_MAIN);
    lv_obj_clear_flag(fill, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(fill, LV_OBJ_FLAG_SCROLLABLE);
    *out_fill = fill;

    lv_obj_add_event_cb(rail, rail_event_cb, LV_EVENT_ALL, (void*)(uintptr_t)dir);
    return rail;
}

static void hide_reticle_cb(lv_timer_t * t) {
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (touch_reticle != NULL) lv_obj_add_flag(touch_reticle, LV_OBJ_FLAG_HIDDEN);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    lv_timer_del(t);
    reticle_timer = NULL;
}

static void vision_canvas_touch_cb(lv_event_t * e) {
    static int16_t start_lx = 0, start_ly = 0, last_lx = 0, last_ly = 0;
    static uint32_t press_ms = 0;
    static bool tracking = false;
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t * indev = lv_indev_get_act();

    if (code == LV_EVENT_PRESSED && indev != NULL && img_vision != NULL) {
        lv_point_t p; lv_indev_get_point(indev, &p);
        lv_area_t coords; lv_obj_get_coords(img_vision, &coords);
        start_lx = constrain(p.x - coords.x1, 0, VISION_W - 1);
        start_ly = constrain(p.y - coords.y1, 0, VISION_H - 1);
        last_lx = start_lx; last_ly = start_ly;
        press_ms = millis(); tracking = true;

        if (touch_reticle != NULL) {
            lv_obj_set_pos(touch_reticle, p.x - 7, p.y - 7);
            lv_obj_clear_flag(touch_reticle, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(touch_reticle);
        }
    }
    else if (code == LV_EVENT_PRESSING && tracking && indev != NULL && img_vision != NULL) {
        lv_point_t p; lv_indev_get_point(indev, &p);
        lv_area_t coords; lv_obj_get_coords(img_vision, &coords);
        last_lx = constrain(p.x - coords.x1, 0, VISION_W - 1);
        last_ly = constrain(p.y - coords.y1, 0, VISION_H - 1);
        if (touch_reticle != NULL) lv_obj_set_pos(touch_reticle, p.x - 7, p.y - 7);
    }
    else if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) && tracking) {
        tracking = false;
        uint32_t dur = millis() - press_ms;
        if (dur > 2000) dur = 2000;
        pending_touch.x1 = start_lx; pending_touch.y1 = start_ly;
        pending_touch.x2 = last_lx; pending_touch.y2 = last_ly;
        pending_touch.dur = (uint16_t)dur;
        has_pending_touch = true;
        if (reticle_timer != NULL) lv_timer_del(reticle_timer);
        reticle_timer = lv_timer_create(hide_reticle_cb, 180, NULL);
    }
}

bool IRAM_ATTR tft_output_vision(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    if (!vision_active || !vision_vram) return 0;
    uint16_t* __restrict p_vram = (uint16_t*)vision_vram;
    const uint8_t* __restrict src = (const uint8_t*)bitmap;
    int16_t base_x = active_x_offset + x;
    if (base_x >= VISION_W || base_x < 0) return 1;

    uint16_t valid_w = w;
    if (base_x + valid_w > VISION_W) valid_w = VISION_W - base_x;
    uint16_t line_buf[16];

    for (int16_t j = 0; j < h; j++) {
        int16_t py = y + j;
        if (py >= VISION_H) break;
        const uint8_t* __restrict src_ptr = &src[(j * w) * 3];
        for (uint16_t i = 0; i < valid_w; i++) {
            uint8_t b = src_ptr[0], g = src_ptr[1], r = src_ptr[2];
            line_buf[i] = ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
            src_ptr += 3;
        }
        memcpy(&p_vram[py * VISION_W + base_x], line_buf, valid_w * sizeof(uint16_t));
    }
    return 1;
}

static void hide_popup_cb(lv_timer_t * t) {
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (popup_link != NULL) { lv_obj_del(popup_link); popup_link = NULL; }
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    lv_timer_del(t);
    popup_timer = NULL;
}

static void show_success_popup() {
    if (popup_link != NULL) return;
    popup_link = lv_obj_create(scr_vision);
    lv_obj_set_size(popup_link, 220, 60);
    lv_obj_center(popup_link);
    lv_obj_set_style_bg_color(popup_link, lv_color_hex(0x00FF41), 0);
    lv_obj_set_style_border_width(popup_link, 0, 0);
    lv_obj_t * lbl = lv_label_create(popup_link);
    lv_label_set_text(lbl, "ENLACE CONFIRMADO");
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl);
    popup_timer = lv_timer_create(hide_popup_cb, 3000, NULL);
}

void vision_rx_task(void * pvParameters) {
    vision_task_running = true;
    uint8_t current_frame_id = 0, expected_chunks = 0;
    bool current_is_seg = true, prev_is_seg = true;
    uint32_t received_mask = 0, current_jpeg_len = 0, last_task_beacon_ms = 0;
    uint8_t chunk[1404];

    while (vision_active) {
        if (!udp_started) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

        if (millis() - last_task_beacon_ms > 1200UL) {
            last_task_beacon_ms = millis();
            send_hello_exosys_beacon();
        }
        flush_pending_udp_cmds();

        int packets_in_burst = 0;
        int packetSize = udp_vision.parsePacket();
        
        while (packetSize > 0 && vision_active && packets_in_burst < 40) { 
            packets_in_burst++;
            bool decoded_this_turn = false;

            if (packetSize > 4 && packetSize <= 1404) {
                udp_vision.read(chunk, packetSize);
                if (chunk[0] == 0xEA) {
                    remote_phone_ip = udp_vision.remoteIP();
                    has_remote_ip = true;
                    last_app_udp_rx_ms = millis();

                    uint8_t frame_id = chunk[1];
                    bool is_seg_frame = (chunk[2] & 0x80) != 0;
                    uint8_t total_chunks = chunk[2] & 0x7F;
                    uint8_t chunk_idx = chunk[3];
                    
                    if (total_chunks > 0 && total_chunks <= 24 && chunk_idx < total_chunks) {
                        if (frame_id != current_frame_id) {
                            current_frame_id = frame_id; expected_chunks = total_chunks;
                            current_is_seg = is_seg_frame; received_mask = 0; current_jpeg_len = 0;
                        }
                        uint32_t payload_len = packetSize - 4;
                        uint32_t offset = chunk_idx * 1400;
                        if (offset + payload_len <= 35000 && udp_jpeg_buffer != NULL) {
                            memcpy(udp_jpeg_buffer + offset, chunk + 4, payload_len);
                            received_mask |= (1UL << chunk_idx);
                            if (chunk_idx == total_chunks - 1) current_jpeg_len = offset + payload_len;
                        }
                        
                        uint32_t target_mask = (1UL << expected_chunks) - 1;
                        if (received_mask == target_mask && current_jpeg_len > 4 && vision_active) {
                            if (udp_jpeg_buffer[0] == 0xFF && udp_jpeg_buffer[1] == 0xD8 &&
                                udp_jpeg_buffer[current_jpeg_len - 2] == 0xFF && udp_jpeg_buffer[current_jpeg_len - 1] == 0xD9) {

                                if (!link_confirmed) {
                                    link_confirmed = true;
                                    udp_vision.beginPacket(remote_phone_ip, 8889);
                                    udp_vision.print("ACK_VISION");
                                    udp_vision.endPacket();
                                    trigger_ack_popup = true;
                                }

                                if (current_is_seg != prev_is_seg && vision_vram != NULL) {
                                    memset(vision_vram, 0, VISION_W * VISION_H * 2);
                                    prev_is_seg = current_is_seg;
                                }

                                active_x_offset = current_is_seg ? 0 : ((VISION_W - FULL_MODE_W) / 2);
                                flush_pending_udp_cmds();

                                JRESULT res = TJpgDec.drawJpg(0, 0, udp_jpeg_buffer, current_jpeg_len);
                                if (res == JDR_OK && vision_active) frame_ready_flag = true;
                                decoded_this_turn = true;
                            }
                            received_mask = 0; current_jpeg_len = 0;
                        }
                    }
                }
                else if (packetSize < 120) {
                    chunk[packetSize] = '\0';
                    remote_phone_ip = udp_vision.remoteIP();
                    has_remote_ip = true;
                    last_app_udp_rx_ms = millis();
                    if (memcmp(chunk, "POS:", 4) == 0) parse_pos_payload_str((const char*)chunk, packetSize);
                    else if (memcmp(chunk, "LINK_OK", 7) == 0 || memcmp(chunk, "PING_EXOSYS", 11) == 0) send_hello_exosys_beacon();
                }
            } else {
                udp_vision.flush();
            }
            if (decoded_this_turn || !vision_active) break;
            packetSize = udp_vision.parsePacket();
        }
        vTaskDelay(1);
    }
    vision_task_running = false;
    vTaskDelete(NULL);
}

static void cleanup_vision_session() {
    vision_active = false;
    wifi_connecting = false;
    pending_scroll_steps = 0;
    pending_mode_cmd = 0;
    has_pending_touch = false;
    frame_ready_flag = false;
    trigger_ack_popup = false;

    uint32_t t0 = millis();
    while (vision_task_running && (millis() - t0 < 800)) delay(5);

    if (udp_started) {
        udp_started = false;
        udp_vision.stop();
    }

    if (popup_timer) { lv_timer_del(popup_timer); popup_timer = NULL; }
    if (reticle_timer) { lv_timer_del(reticle_timer); reticle_timer = NULL; }

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (touch_reticle) lv_obj_add_flag(touch_reticle, LV_OBJ_FLAG_HIDDEN);
        if (popup_link) { lv_obj_del(popup_link); popup_link = NULL; }
        if (img_vision) { lv_img_set_src(img_vision, NULL); lv_obj_del(img_vision); img_vision = NULL; }
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    if (vision_vram) { heap_caps_free(vision_vram); vision_vram = NULL; }
    if (udp_jpeg_buffer) { heap_caps_free(udp_jpeg_buffer); udp_jpeg_buffer = NULL; }
}

void connect_to_target_wifi() {
    save_hotspot_credentials_nvs(selected_ssid, selected_pass);
    cleanup_vision_session();
    net_ble_set_enabled(false);

    extern void secure_spi_lock();
    extern void release_spi_lock();
    secure_spi_lock();

    vision_vram = (uint8_t*)heap_caps_malloc(VISION_W * VISION_H * 2, MALLOC_CAP_SPIRAM);
    if (vision_vram) memset(vision_vram, 0, VISION_W * VISION_H * 2);
    udp_jpeg_buffer = (uint8_t*)heap_caps_malloc(35000, MALLOC_CAP_SPIRAM);
    
    if (!vision_vram || !udp_jpeg_buffer) {
        Serial.println("[EXOSYS] PSRAM Agotada. Abortando enlace.");
        cleanup_vision_session();
        release_spi_lock();
        net_ble_set_enabled(true);
        return;
    }

    vision_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    vision_img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    vision_img_dsc.header.flags = 0;
    if (scr_vision == NULL) {
        scr_vision = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr_vision, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_clear_flag(scr_vision, LV_OBJ_FLAG_SCROLLABLE);

        rail_up_r = create_tactical_rail(scr_vision, -12, -116, 0x00E5FF, 1, &fill_up_r);
        rail_down_r = create_tactical_rail(scr_vision, -12, 116, 0xFFB300, 2, &fill_down_r);

        border_full_l = lv_obj_create(scr_vision);
        lv_obj_set_size(border_full_l, 80, 464);
        lv_obj_align(border_full_l, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_set_style_bg_opa(border_full_l, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(border_full_l, 0, LV_PART_MAIN);
        lv_obj_add_flag(border_full_l, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(border_full_l, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(border_full_l, full_border_event_cb, LV_EVENT_CLICKED, NULL);

        border_full_r = lv_obj_create(scr_vision);
        lv_obj_set_size(border_full_r, 80, 464);
        lv_obj_align(border_full_r, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_set_style_bg_opa(border_full_r, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(border_full_r, 0, LV_PART_MAIN);
        lv_obj_add_flag(border_full_r, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(border_full_r, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(border_full_r, full_border_event_cb, LV_EVENT_CLICKED, NULL);

        touch_reticle = lv_obj_create(scr_vision);
        lv_obj_set_size(touch_reticle, 14, 14);
        lv_obj_set_style_bg_color(touch_reticle, lv_color_hex(0x00FF41), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(touch_reticle, 180, LV_PART_MAIN);
        lv_obj_set_style_border_color(touch_reticle, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_border_width(touch_reticle, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(touch_reticle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_clear_flag(touch_reticle, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(touch_reticle, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(touch_reticle, LV_OBJ_FLAG_HIDDEN);

        lbl_vision_status = lv_label_create(scr_vision);
        lv_obj_set_style_text_color(lbl_vision_status, lv_color_hex(0x00FF41), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lbl_vision_status, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(lbl_vision_status, 150, LV_PART_MAIN);
        lv_obj_align(lbl_vision_status, LV_ALIGN_BOTTOM_MID, 0, -35);
    }
    vision_img_dsc.header.w = VISION_W;
    vision_img_dsc.header.h = VISION_H;
    vision_img_dsc.header.stride = VISION_W * 2;
    vision_img_dsc.data_size = VISION_W * VISION_H * 2;
    vision_img_dsc.data = vision_vram;

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        if (img_vision == NULL && scr_vision != NULL) {
            img_vision = lv_img_create(scr_vision);
            lv_obj_add_flag(img_vision, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(img_vision, vision_canvas_touch_cb, LV_EVENT_ALL, NULL);
            lv_obj_move_background(img_vision);
        }
        lv_img_set_src(img_vision, &vision_img_dsc);
        set_ui_segmented_mode(true);
        lv_label_set_text(lbl_vision_status, "NEGOCIANDO CONEXION...");
        lv_obj_clear_flag(lbl_vision_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(lbl_vision_status);
        lv_scr_load(scr_vision);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }

    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(false);
    TJpgDec.setCallback(tft_output_vision);

    if (WiFi.status() == WL_CONNECTED && strcmp(WiFi.SSID().c_str(), selected_ssid) == 0) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        
        if (!udp_started) { 
            udp_vision.begin(8888); 
            udp_vision.flush();
            udp_started = true; 
        }
        
        wifi_connecting = false;
        wifi_connection_failed = false;
        send_hello_exosys_beacon();
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            char buf[64];
            sprintf(buf, "Auto-Enlace IP: %s\nESPERANDO VIDEO...", WiFi.localIP().toString().c_str());
            lv_label_set_text(lbl_vision_status, buf);
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    } else {
        WiFi.disconnect(true, true);
        delay(40);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(selected_ssid, selected_pass);
        udp_started = false;
        wifi_connecting = true;
        wifi_connection_failed = false;
        hotspot_connect_start_ms = millis();
    }
    
    pending_scroll_steps = 0; pending_mode_cmd = 0; has_pending_touch = false;
    frame_ready_flag = false; trigger_ack_popup = false; link_confirmed = false;
    vision_active = true;

    release_spi_lock();
    xTaskCreatePinnedToCore(vision_rx_task, "VisionRX", 8192, NULL, 1, NULL, 0);
}

static void kb_pass_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (code == LV_EVENT_READY && ta_wifi_pass != NULL) {
            strncpy(selected_pass, lv_textarea_get_text(ta_wifi_pass), 63);
            selected_pass[63] = '\0';
            if (wifi_scan_from_settings) {
                start_hotspot_bg_connection(selected_ssid, selected_pass);
                if (nav_from_main_hub) {
                    nav_from_main_hub = false;
                    lv_scr_load(scr_selector);
                } else if (scr_settings_hotspot != NULL) {
                    lv_scr_load(scr_settings_hotspot);
                }
            } else {
                connect_to_target_wifi();
            }
        } else {
            lv_scr_load(scr_wifi_scan); 
        }
        if (kb_wifi_pass) { lv_obj_add_flag(kb_wifi_pass, LV_OBJ_FLAG_HIDDEN); lv_obj_del_async(kb_wifi_pass); kb_wifi_pass = NULL; }
        if (ta_wifi_pass) { lv_obj_add_flag(ta_wifi_pass, LV_OBJ_FLAG_HIDDEN); lv_obj_del_async(ta_wifi_pass); ta_wifi_pass = NULL; }
    }
}

bool network_back_router() {
    lv_obj_t * current_scr = lv_scr_act();
    if (current_scr == scr_wifi_scan) {
        wifi_scanning = false; wifi_retry_at_ms = 0; WiFi.scanDelete();
        if (WiFi.status() != WL_CONNECTED && !hotspot_bg_connecting) {
            WiFi.mode(WIFI_OFF);
            net_ble_set_enabled(true);
        }
        if (list_wifi) lv_obj_clean(list_wifi);
        if (nav_from_main_hub) {
            nav_from_main_hub = false;
            lv_scr_load(scr_selector);
        } else {
            lv_scr_load(net_screen_ptr);
        }
        return true;
    }
    else if (current_scr == scr_wifi_pass) {
        if (kb_wifi_pass) { lv_obj_del(kb_wifi_pass); kb_wifi_pass = NULL; }
        if (ta_wifi_pass) { lv_obj_del(ta_wifi_pass); ta_wifi_pass = NULL; }
        lv_scr_load(scr_wifi_scan);
        return true;
    }
    else if (current_scr == scr_vision) {
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            lv_scr_load(nav_from_main_hub ? scr_selector : net_screen_ptr);
            nav_from_main_hub = false;
            lv_refr_now(NULL);
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
        cleanup_vision_session();
        if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        else {
            WiFi.mode(WIFI_OFF);
            net_ble_set_enabled(true);
        }
        return true;
    }
    else if (current_scr == scr_settings_hotspot && modal_qr_hs != NULL) {
        close_qr_modal_cb(NULL);
        return true;
    }
    else if (current_scr == scr_settings_hotspot || current_scr == scr_settings_bt) {
        if (nav_from_main_hub) {
            nav_from_main_hub = false;
            lv_scr_load(scr_selector);
        } else {
            lv_scr_load(net_screen_ptr);
        }
        return true;
    }
    else if (current_scr == net_screen_ptr) {
        wifi_scanning = false; wifi_retry_at_ms = 0; WiFi.scanDelete();
        if (WiFi.status() != WL_CONNECTED && !hotspot_bg_connecting) WiFi.mode(WIFI_OFF);
        return false;
    }
    return false;
}

void build_network_ui() {
    net_screen_ptr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(net_screen_ptr, lv_color_hex(0x05000A), LV_PART_MAIN);
    lv_obj_clear_flag(net_screen_ptr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_header = lv_label_create(net_screen_ptr);
    lv_label_set_text(lbl_header, ">> ENLACES DE RADIO <<");
    lv_obj_set_style_text_color(lbl_header, lv_color_hex(0x00E5FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_header, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(lbl_header, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t * pnl = lv_obj_create(net_screen_ptr);
    lv_obj_set_size(pnl, 336, 400);
    lv_obj_align(pnl, LV_ALIGN_CENTER, 0, 15);
    lv_obj_set_style_bg_color(pnl, lv_color_hex(0x05000A), LV_PART_MAIN);
    lv_obj_set_style_border_width(pnl, 0, LV_PART_MAIN);
    lv_obj_clear_flag(pnl, LV_OBJ_FLAG_SCROLLABLE);

    const char* t[] = {"Wi-Fi", "Bluetooth", "Hotspot", "LoRa"};
    const int c[] = {0x00E5FF, 0x007AFF, 0xFF003C, 0x9D00FF};

    for(int i=0; i<4; i++) {
        lv_obj_t * btn = lv_btn_create(pnl);
        lv_obj_set_size(btn, 320, 80); 
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, i * 95);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x111111), LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, lv_color_hex(c[i]), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
        
        lv_obj_t * l = lv_label_create(btn);
        lv_label_set_text(l, t[i]);
        lv_obj_set_style_text_color(l, lv_color_hex(c[i]), LV_PART_MAIN);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_center(l);

        if (i == 0) lv_obj_add_event_cb(btn, [](lv_event_t * e){ net_open_hotspot_scanner_from_settings(); }, LV_EVENT_CLICKED, NULL);
        if (i == 1) lv_obj_add_event_cb(btn, [](lv_event_t * e){ nav_from_main_hub = false; refresh_wireless_status_ui(NULL); lv_scr_load(scr_settings_bt); }, LV_EVENT_CLICKED, NULL);
        if (i == 2) lv_obj_add_event_cb(btn, [](lv_event_t * e){ nav_from_main_hub = false; refresh_wireless_status_ui(NULL); lv_scr_load(scr_settings_hotspot); }, LV_EVENT_CLICKED, NULL);
        if (i == 3) lv_obj_add_event_cb(btn, [](lv_event_t * e){ 
            net_lora_set_enabled(!net_lora_is_enabled());
        }, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t * btn_b = lv_btn_create(net_screen_ptr);
    lv_obj_set_size(btn_b, 320, 46);
    lv_obj_align(btn_b, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(btn_b, lv_color_hex(0xFF005C), LV_PART_MAIN);
    lv_obj_t * lbl_b = lv_label_create(btn_b);
    lv_label_set_text(lbl_b, "CERRAR");
    lv_obj_center(lbl_b);
    lv_obj_add_event_cb(btn_b, [](lv_event_t * e) { lv_scr_load(scr_selector); }, LV_EVENT_CLICKED, NULL);

    scr_wifi_scan = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wifi_scan, lv_color_hex(0x05000A), LV_PART_MAIN);
    lbl_wifi_status = lv_label_create(scr_wifi_scan);
    lv_obj_set_style_text_color(lbl_wifi_status, lv_color_hex(0x00FF41), LV_PART_MAIN);
    lv_obj_align(lbl_wifi_status, LV_ALIGN_TOP_MID, 0, 52);
    
    list_wifi = lv_list_create(scr_wifi_scan);
    lv_obj_set_size(list_wifi, 336, 280);
    lv_obj_align(list_wifi, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(list_wifi, lv_color_hex(0x05000A), LV_PART_MAIN);
    lv_obj_set_style_border_color(list_wifi, lv_color_hex(0x00FF41), LV_PART_MAIN);
    lv_obj_set_style_border_width(list_wifi, 2, LV_PART_MAIN);

    scr_wifi_pass = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wifi_pass, lv_color_hex(0x000000), LV_PART_MAIN);

    scr_vision = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_vision, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_clear_flag(scr_vision, LV_OBJ_FLAG_SCROLLABLE);
    img_vision = NULL;

    rail_up_r = create_tactical_rail(scr_vision, -12, -116, 0x00E5FF, 1, &fill_up_r);
    rail_down_r = create_tactical_rail(scr_vision, -12, 116, 0xFFB300, 2, &fill_down_r);

    border_full_l = lv_obj_create(scr_vision);
    lv_obj_set_size(border_full_l, 80, 464);
    lv_obj_align(border_full_l, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(border_full_l, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(border_full_l, 0, LV_PART_MAIN);
    lv_obj_add_flag(border_full_l, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(border_full_l, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(border_full_l, full_border_event_cb, LV_EVENT_CLICKED, NULL);

    border_full_r = lv_obj_create(scr_vision);
    lv_obj_set_size(border_full_r, 80, 464);
    lv_obj_align(border_full_r, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_opa(border_full_r, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(border_full_r, 0, LV_PART_MAIN);
    lv_obj_add_flag(border_full_r, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(border_full_r, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(border_full_r, full_border_event_cb, LV_EVENT_CLICKED, NULL);

    touch_reticle = lv_obj_create(scr_vision);
    lv_obj_set_size(touch_reticle, 14, 14);
    lv_obj_set_style_bg_color(touch_reticle, lv_color_hex(0x00FF41), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(touch_reticle, 180, LV_PART_MAIN);
    lv_obj_set_style_border_color(touch_reticle, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(touch_reticle, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(touch_reticle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(touch_reticle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(touch_reticle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(touch_reticle, LV_OBJ_FLAG_HIDDEN);

    lbl_vision_status = lv_label_create(scr_vision);
    lv_obj_set_style_text_color(lbl_vision_status, lv_color_hex(0x00FF41), LV_PART_MAIN);
    lv_obj_set_style_bg_color(lbl_vision_status, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lbl_vision_status, 150, LV_PART_MAIN);
    lv_obj_align(lbl_vision_status, LV_ALIGN_BOTTOM_MID, 0, -35);

    scr_settings_hotspot = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_settings_hotspot, lv_color_hex(0x02080A), 0);
    lv_obj_t * t_hs = lv_label_create(scr_settings_hotspot);
    lv_label_set_text(t_hs, ">> HOTSPOT <<");
    lv_obj_set_style_text_color(t_hs, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_text_font(t_hs, &lv_font_montserrat_20, 0);
    lv_obj_align(t_hs, LV_ALIGN_TOP_MID, 0, 35);

    lbl_hs_state = lv_label_create(scr_settings_hotspot);
    lv_obj_set_style_text_font(lbl_hs_state, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(lbl_hs_state, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_hs_state, LV_ALIGN_CENTER, 0, -70);
    lv_obj_add_flag(lbl_hs_state, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_hs_state, open_qr_hotspot_modal_cb, LV_EVENT_CLICKED, NULL);

    btn_hs_action = lv_btn_create(scr_settings_hotspot);
    lv_obj_set_size(btn_hs_action, 340, 65);
    lv_obj_align(btn_hs_action, LV_ALIGN_CENTER, 0, 20);
    lbl_hs_action = lv_label_create(btn_hs_action);
    lv_obj_set_style_text_font(lbl_hs_action, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_hs_action);

    lv_obj_add_event_cb(btn_hs_action, [](lv_event_t * e) {
        if (WiFi.status() == WL_CONNECTED || wifi_connecting || hotspot_bg_connecting) {
            net_disconnect_hotspot();
        } else if (wifi_connection_failed) {
            net_open_hotspot_scanner_from_settings();
        } else {
            char ss[64];
            if (net_has_saved_hotspot(ss, sizeof(ss))) net_connect_saved_hotspot();
            else net_open_hotspot_scanner_from_settings();
        }
        refresh_wireless_status_ui(NULL);
    }, LV_EVENT_CLICKED, NULL);

    btn_hs_video = lv_btn_create(scr_settings_hotspot);
    lv_obj_set_size(btn_hs_video, 340, 60);
    lv_obj_align(btn_hs_video, LV_ALIGN_CENTER, 0, 95);
    lv_obj_set_style_bg_color(btn_hs_video, lv_color_hex(0xFFB300), 0);
    lv_obj_t * lbl_hs_video = lv_label_create(btn_hs_video);
    lv_label_set_text(lbl_hs_video, "CONTROL INALAMBRICO");
    lv_obj_set_style_text_color(lbl_hs_video, lv_color_black(), 0);
    lv_obj_center(lbl_hs_video);
    lv_obj_add_flag(btn_hs_video, LV_OBJ_FLAG_HIDDEN);
    
    lv_obj_add_event_cb(btn_hs_video, [](lv_event_t * e){ 
        if (WiFi.status() == WL_CONNECTED) {
            connect_to_target_wifi();
        }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_h_back = lv_btn_create(scr_settings_hotspot);
    lv_obj_set_size(btn_h_back, 340, 50);
    lv_obj_align(btn_h_back, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(btn_h_back, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_hb = lv_label_create(btn_h_back);
    lv_label_set_text(lbl_hb, "VOLVER");
    lv_obj_center(lbl_hb);
    lv_obj_add_event_cb(btn_h_back, [](lv_event_t * e) {
        if (nav_from_main_hub) { nav_from_main_hub = false; lv_scr_load(scr_selector); }
        else lv_scr_load(net_screen_ptr);
    }, LV_EVENT_CLICKED, NULL);

    // --- PANTALLA RASTREADOR BLE ---
    scr_settings_bt = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_settings_bt, lv_color_hex(0x02080A), 0);
    lv_obj_clear_flag(scr_settings_bt, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * t_bt = lv_label_create(scr_settings_bt);
    lv_label_set_text(t_bt, ">> RASTREADOR TACTICO <<");
    lv_obj_set_style_text_color(t_bt, lv_color_hex(0xFFB300), 0);
    lv_obj_set_style_text_font(t_bt, &lv_font_montserrat_20, 0);
    lv_obj_align(t_bt, LV_ALIGN_TOP_MID, 0, 35);

    lbl_bt_state = lv_label_create(scr_settings_bt);
    lv_obj_set_style_text_font(lbl_bt_state, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(lbl_bt_state, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_bt_state, LV_ALIGN_CENTER, 0, -110);

    bar_bt_rssi = lv_bar_create(scr_settings_bt);
    lv_obj_set_size(bar_bt_rssi, 340, 30);
    lv_obj_align(bar_bt_rssi, LV_ALIGN_CENTER, 0, -45);
    lv_obj_set_style_bg_color(bar_bt_rssi, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(bar_bt_rssi, 15, LV_PART_INDICATOR);
    lv_bar_set_range(bar_bt_rssi, 0, 100);
    lv_bar_set_value(bar_bt_rssi, 0, LV_ANIM_OFF);

    btn_bt_action = lv_btn_create(scr_settings_bt);
    lv_obj_set_size(btn_bt_action, 340, 60);
    lv_obj_align(btn_bt_action, LV_ALIGN_CENTER, 0, 25);
    lbl_bt_action = lv_label_create(btn_bt_action);
    lv_obj_set_style_text_font(lbl_bt_action, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_bt_action);

    lv_obj_add_event_cb(btn_bt_action, [](lv_event_t * e) {
        net_ble_set_enabled(!net_ble_is_enabled());
        refresh_wireless_status_ui(NULL);
    }, LV_EVENT_CLICKED, NULL);

    lbl_bt_gnss_status = lv_label_create(scr_settings_bt);
    lv_obj_set_style_text_font(lbl_bt_gnss_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(lbl_bt_gnss_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_bt_gnss_status, LV_ALIGN_CENTER, 0, 110);
    lv_label_set_text(lbl_bt_gnss_status, "GNSS INACTIVO (ESPERANDO TELEFONO)");
    lv_obj_set_style_text_color(lbl_bt_gnss_status, lv_color_hex(0x666666), 0);

    lv_obj_t * btn_bb_back = lv_btn_create(scr_settings_bt);
    lv_obj_set_size(btn_bb_back, 340, 50);
    lv_obj_align(btn_bb_back, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(btn_bb_back, lv_color_hex(0xFF005C), 0);
    lv_obj_t * lbl_bb = lv_label_create(btn_bb_back);
    lv_label_set_text(lbl_bb, "VOLVER");
    lv_obj_center(lbl_bb);
    lv_obj_add_event_cb(btn_bb_back, [](lv_event_t * e) {
        if (nav_from_main_hub) { nav_from_main_hub = false; lv_scr_load(scr_selector); }
        else lv_scr_load(net_screen_ptr);
    }, LV_EVENT_CLICKED, NULL);

    if (wireless_ui_timer == NULL) {
        wireless_ui_timer = lv_timer_create(refresh_wireless_status_ui, 500, NULL);
    }

    net_ble_set_enabled(true);
}

void network_loop_handler() {
    if (wifi_retry_at_ms != 0 && millis() >= wifi_retry_at_ms) {
        wifi_retry_at_ms = 0;
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            bool on_scan_scr = (lv_scr_act() == scr_wifi_scan);
            xSemaphoreGiveRecursive(lvgl_mutex);
            if (on_scan_scr) trigger_clean_wifi_scan();
        }
    }

    if (wifi_scanning) {
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_FAILED) {
            wifi_scanning = false;
            WiFi.scanDelete();
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                if (lv_scr_act() == scr_wifi_scan) {
                    lv_label_set_text(lbl_wifi_status, "REINTENTANDO ESCANEO 2.4GHz...");
                    wifi_retry_at_ms = millis() + 1200UL;
                }
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        }
        else if (n == 0) {
            wifi_scanning = false;
            WiFi.scanDelete();
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                if (lv_scr_act() == scr_wifi_scan) {
                    lv_label_set_text(lbl_wifi_status, "0 REDES (REESCANEO EN 1.5s)...");
                    wifi_retry_at_ms = millis() + 1500UL;
                }
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        }
        else if (n > 0) {
            wifi_scanning = false;
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                if (lv_scr_act() == scr_wifi_scan) {
                    if (list_wifi) lv_obj_clean(list_wifi);
                    char hdr[64];
                    snprintf(hdr, sizeof(hdr), "REDES 2.4GHz DETECTADAS (%d)", n);
                    lv_label_set_text(lbl_wifi_status, hdr);
                    
                    int unique_count = 0;
                    for (int i = 0; i < n && unique_count < MAX_SCANNED_NETS; ++i) {
                        String ssid_str = WiFi.SSID(i);
                        if (ssid_str.length() == 0) continue;
                        
                        bool is_duplicate = false;
                        for (int j = 0; j < unique_count; ++j) {
                            if (strcmp(scanned_nets[j].ssid, ssid_str.c_str()) == 0) {
                                is_duplicate = true; break;
                            }
                        }
                        if (is_duplicate) continue;

                        strncpy(scanned_nets[unique_count].ssid, ssid_str.c_str(), 63);
                        scanned_nets[unique_count].ssid[63] = '\0';
                        scanned_nets[unique_count].is_open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
                        
                        char buf[128];
                        snprintf(buf, sizeof(buf), "%s (%d dBm) %s", scanned_nets[unique_count].ssid, WiFi.RSSI(i), scanned_nets[unique_count].is_open ? "[ABIERTA]" : "[SEGURA]");
                        
                        lv_obj_t * btn = lv_list_add_btn(list_wifi, LV_SYMBOL_WIFI, buf);
                        lv_obj_set_style_bg_color(btn, lv_color_hex(0x111111), LV_PART_MAIN);
                        lv_obj_set_style_text_color(btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
                        
                        lv_obj_set_style_pad_top(btn, 18, LV_PART_MAIN);
                        lv_obj_set_style_pad_bottom(btn, 18, LV_PART_MAIN);
                        
                        lv_obj_add_event_cb(btn, [](lv_event_t * e){
                            int idx = (int)(uintptr_t)lv_event_get_user_data(e);
                            strncpy(selected_ssid, scanned_nets[idx].ssid, 63);
                            selected_ssid[63] = '\0';
                            
                            if (scanned_nets[idx].is_open) {
                                selected_pass[0] = '\0';
                                link_confirmed = false;
                                if (wifi_scan_from_settings) {
                                    start_hotspot_bg_connection(selected_ssid, selected_pass);
                                    if (nav_from_main_hub) {
                                        nav_from_main_hub = false;
                                        lv_scr_load(scr_selector);
                                    } else if (scr_settings_hotspot != NULL) {
                                        lv_scr_load(scr_settings_hotspot);
                                    }
                                } else {
                                    connect_to_target_wifi();
                                }
                            } else {
                                if (ta_wifi_pass) { lv_obj_del(ta_wifi_pass); ta_wifi_pass = NULL; }
                                if (kb_wifi_pass) { lv_obj_del(kb_wifi_pass); kb_wifi_pass = NULL; }

                                ta_wifi_pass = lv_textarea_create(scr_wifi_pass);
                                lv_obj_set_size(ta_wifi_pass, 330, 50);
                                lv_obj_align(ta_wifi_pass, LV_ALIGN_TOP_MID, 0, 35);
                                lv_textarea_set_placeholder_text(ta_wifi_pass, "Contrasena");
                                lv_textarea_set_password_mode(ta_wifi_pass, true);

                                kb_wifi_pass = lv_keyboard_create(scr_wifi_pass);
                                lv_obj_set_size(kb_wifi_pass, 350, 220);
                                lv_obj_align(kb_wifi_pass, LV_ALIGN_BOTTOM_MID, 0, -30);
                                lv_keyboard_set_textarea(kb_wifi_pass, ta_wifi_pass);
                                lv_obj_add_event_cb(kb_wifi_pass, kb_pass_event_cb, LV_EVENT_ALL, NULL);
                                lv_scr_load(scr_wifi_pass);
                            }
                        }, LV_EVENT_CLICKED, (void*)(uintptr_t)unique_count);
                        
                        unique_count++;
                    }
                }
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
            WiFi.scanDelete();
        }
    }

    if (hotspot_bg_connecting) {
        if (WiFi.status() == WL_CONNECTED) {
            hotspot_bg_connecting = false;
            wifi_connection_failed = false;
            if (!udp_started) {
                udp_vision.begin(8888);
                udp_started = true;
            }
            send_hello_exosys_beacon();
        } else if (millis() - hotspot_connect_start_ms > 16000UL) {
            hotspot_bg_connecting = false;
            shutdown_wifi_phy_silent();
            net_ble_set_enabled(true);
            wifi_connection_failed = true;
        }
    }

    if (!vision_active && WiFi.status() == WL_CONNECTED) {
        if (!udp_started) {
            udp_vision.begin(8888);
            udp_started = true;
        }

        if (millis() - last_hello_beacon_ms > 1200UL) {
            last_hello_beacon_ms = millis();
            if (!vision_active) {
                send_hello_exosys_beacon();
            }
        }

        int pSize = udp_vision.parsePacket();
        while (pSize > 0) {
            if (pSize < 120) {
                char pbuf[128];
                int r = udp_vision.read(pbuf, pSize);
                if (r > 0) {
                    pbuf[r] = '\0';
                    remote_phone_ip = udp_vision.remoteIP();
                    has_remote_ip = true;
                    last_app_udp_rx_ms = millis();

                    if (strncmp(pbuf, "POS:", 4) == 0) {
                        parse_pos_payload_str(pbuf, r);
                    } else if (strncmp(pbuf, "LINK_OK", 7) == 0 || strncmp(pbuf, "PING_EXOSYS", 11) == 0) {
                        // Silencio intencional
                    }
                }
            } else {
                udp_vision.flush();
            }
            pSize = udp_vision.parsePacket();
        }
    }

    if (!vision_active) return;

    if (wifi_connecting) {
        if (WiFi.status() == WL_CONNECTED) {
            wifi_connecting = false;
            wifi_connection_failed = false;
            udp_vision.begin(8888);
            link_confirmed = false;
            udp_started = true;
            send_hello_exosys_beacon();
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                char buf[64];
                sprintf(buf, "Auto-Enlace IP: %s", WiFi.localIP().toString().c_str());
                lv_label_set_text(lbl_vision_status, buf);
                lv_obj_clear_flag(lbl_vision_status, LV_OBJ_FLAG_HIDDEN);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        } else if (millis() - hotspot_connect_start_ms > 16000UL) {
            wifi_connecting = false;
            if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
                lv_label_set_text(lbl_vision_status, "FALLO WIFI (VERIFICA CLAVE)");
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
            shutdown_wifi_phy_silent();
            net_ble_set_enabled(true);
            wifi_connection_failed = true;
        }
        return; 
    }

    if (trigger_ack_popup) {
        trigger_ack_popup = false;
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            show_success_popup();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    }

    if (frame_ready_flag) {
        frame_ready_flag = false;
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            lv_obj_add_flag(lbl_vision_status, LV_OBJ_FLAG_HIDDEN);
            if (img_vision != NULL) lv_obj_invalidate(img_vision);
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    }
}