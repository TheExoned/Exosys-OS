#include "CartoEngine.h"
#include "NetworkEngine.h"
#include <Wire.h> 
#include <SPI.h> 
#include <math.h> 
#include <Preferences.h>
#include <vector>
#include <queue>
#include <algorithm>

// ---------------------------------------------------------------------
// 1. COORDENADAS BASE, LÍMITES DINÁMICOS DE ZOOM SD Y TELEMETRÍA GNSS
// ---------------------------------------------------------------------
uint8_t current_z = 16;
uint32_t current_x = 15112; 
uint32_t current_y = 29158;

uint8_t min_sd_z = 6;
uint8_t max_sd_z = 20;
uint32_t valid_z_mask = 0x1FFFC0UL;
volatile bool sd_zoom_bounds_scanned = false;
static volatile bool placeholder_sd_loaded = false;

volatile int32_t drag_pixel_x = 0;
volatile int32_t drag_pixel_y = 0;
volatile int8_t vector_x = 0; 
volatile int8_t vector_y = 0; 
volatile uint32_t last_pan_Input_ms = 0;
volatile bool touch_down_on_map = false;

void update_map_topology(int16_t dx, int16_t dy);
void mostrar_toast(const char * msg);
void ir_a_coordenada(double lat, double lng); 
void refrescar_linea_ruta();
void calcular_y_dibujar_ruta();
void anclar_a_gnss();
void scan_sd_zoom_bounds();
bool reload_custom_placeholder();

volatile bool modo_exploracion = false;
volatile bool osd_visible = false; 
volatile bool primer_enganche_logrado = false;
volatile bool seguimiento_gnss_activo = true;

// Watchdog estricto de 30 segundos para pérdida de señal GNSS
const uint32_t GNSS_TIMEOUT_MS = 30000UL;
static uint32_t last_watch_fix_ms = 0;
static uint32_t last_gsv_sentence_ms = 0;

// 0 = Sin señal, 1 = Solo Reloj (Azul), 2 = Solo Teléfono (Amarillo), 3 = Combinados (Magenta Neón)
volatile uint8_t gnss_active_source = 0;
static volatile bool gnss_pos_dirty = false;
static double fused_alt_m = 0.0;
static double fused_spd_kmh = 0.0;

extern TinyGPSPlus gps;
volatile double current_lat = 0.0;
volatile double current_lng = 0.0;
volatile uint32_t current_satellites = 0;
volatile bool gps_locked = false;
char active_constellation[15] = "RASTREANDO..."; 
char nmea_buffer[128];
uint8_t nmea_idx = 0;

// VelocityTracker Histórico + Motor Físico Exponencial Sub-Píxel (Google Maps Physics)
struct TouchHistorySample {
    int16_t x;
    int16_t y;
    uint32_t t_ms;
};
static TouchHistorySample touch_hist[8];
static uint8_t touch_hist_count = 0;

static bool glide_active = false;
static float glide_vx = 0.0f;
static float glide_vy = 0.0f;
static float glide_rem_x = 0.0f;
static float glide_rem_y = 0.0f;
static uint32_t glide_last_ms = 0;

static float drag_smooth_x = 0.0f;
static float drag_smooth_y = 0.0f;

static inline void stop_kinetic_glide() {
    glide_active = false;
    glide_vx = 0.0f;
    glide_vy = 0.0f;
    glide_rem_x = 0.0f;
    glide_rem_y = 0.0f;
}

// Variables de Multitouch Capacitivo (CST92xx 2-Finger Pinch-to-Zoom)
static volatile uint8_t g_mt_fingers = 0;
static volatile int16_t g_mt_x[2] = {0, 0};
static volatile int16_t g_mt_y[2] = {0, 0};
static bool pinch_active = false;
static float pinch_base_dist = 0.0f;
static uint32_t pinch_cooldown_until = 0;

// ESTRATEGIA MONOLÍTICA + ZERO-COPY POINTER SWAP (PRESUPUESTO PSRAM ESTRICTO)
const uint32_t GLOBAL_ARENA_BYTES = 16 * 131072; // 2,097,152 B
const uint32_t MAX_QOI_TILES = 20;               // 20 ranuras L2 de 40 KB (soporta teselas Pip-Boy de 37+ KB)
const uint32_t QOI_CHUNK_SIZE = 40960;           // 40 KB por ranura
const uint32_t VAULT_ARENA_BYTES = MAX_QOI_TILES * QOI_CHUNK_SIZE; // 819,200 B
const uint32_t COMPRESSED_BUF_BYTES = 65536;     // 64 KB para lectura QOI

uint8_t * global_arena = NULL;      
uint8_t * vault_arena = NULL;       
uint8_t * shadow_buffer = NULL;     
uint8_t * shadow_arena_base = NULL; 
uint8_t * compressed_buffer = NULL; 
uint8_t * static_placeholder = NULL; 

FsFile map_file;
const uint32_t CLUSTER_SIZE = 32;
TaskHandle_t ghost_miner_handle = NULL;
SemaphoreHandle_t sd_spi_mutex = NULL;
portMUX_TYPE tile_spinlock = portMUX_INITIALIZER_UNLOCKED;

float fast_distance(float lat1, float lon1, float lat2, float lon2);

// ---------------------------------------------------------------------
// 1.1 INYECCIÓN UBX BARE-METAL PARA RECEPTOR U-BLOX M10
// ---------------------------------------------------------------------
void enviar_ubx(uint8_t class_id, uint8_t msg_id, uint8_t *payload, uint16_t len) {
    GPSSerial.write(0xB5);
    GPSSerial.write(0x62);
    GPSSerial.write(class_id);
    GPSSerial.write(msg_id);
    GPSSerial.write(len & 0xFF);
    GPSSerial.write((len >> 8) & 0xFF);
    
    uint8_t cka = 0, ckb = 0;
    cka += class_id; ckb += cka;
    cka += msg_id;   ckb += cka;
    cka += (len & 0xFF); ckb += cka;
    cka += ((len >> 8) & 0xFF); ckb += cka;
    
    for(uint16_t i=0; i<len; i++) {
        GPSSerial.write(payload[i]);
        cka += payload[i];
        ckb += cka;
    }
    GPSSerial.write(cka);
    GPSSerial.write(ckb);
    GPSSerial.flush();
}

void configurar_gnss(uint8_t mode) {
    uint8_t bds_val = (mode == 2) ? 0x01 : 0x00;
    uint8_t glo_val = (mode == 1) ? 0x01 : 0x00;
    
    uint8_t payload[] = {
        0x00, 0x01, 0x00, 0x00, 
        0x22, 0x00, 0x31, 0x10, bds_val, 
        0x25, 0x00, 0x31, 0x10, glo_val  
    };
    enviar_ubx(0x06, 0x8A, payload, sizeof(payload));
    delay(50);
    
    uint8_t pms_payload[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    enviar_ubx(0x06, 0x86, pms_payload, sizeof(pms_payload));
    
    if(mode == 1) strcpy(active_constellation, "GPS+GAL+GLO");
    else strcpy(active_constellation, "GPS+GAL+BDS");
    
    mostrar_toast("CONSTELACION RECONFIGURADA");
}

// ---------------------------------------------------------------------
// 2. ARSENAL DE WAYPOINTS (PERSISTENCIA Y ESTRUCTURAS)
// ---------------------------------------------------------------------
Preferences wp_prefs;

struct Waypoint {
    char nombre[24];
    double lat;
    double lng;
    uint8_t origen; 
};

Waypoint wp_db[50];
uint8_t wp_count = 0;
int wp_selected_idx = -1;

void guardar_waypoint(double lat, double lng, uint8_t origen) {
    if (wp_count >= 50) return;
    sprintf(wp_db[wp_count].nombre, "Mi waypoint %d", wp_count + 1);
    wp_db[wp_count].lat = lat;
    wp_db[wp_count].lng = lng;
    wp_db[wp_count].origen = origen;
    
    char key_base[10];
    sprintf(key_base, "wp_%d", wp_count);
    wp_prefs.putBytes(key_base, &wp_db[wp_count], sizeof(Waypoint));
    
    wp_count++;
    wp_prefs.putUChar("wp_count", wp_count);
    Serial.printf("[WAYPOINT] Guardado en NVS. Lat: %f, Lng: %f\n", lat, lng);
}

void borrar_waypoint(int idx) {
    if (idx < 0 || idx >= wp_count) return;
    for(int i = idx; i < wp_count - 1; i++) wp_db[i] = wp_db[i+1];
    wp_count--;
    wp_prefs.putUChar("wp_count", wp_count);
    
    for(int i = 0; i < wp_count; i++) {
        char key_base[10];
        sprintf(key_base, "wp_%d", i);
        wp_prefs.putBytes(key_base, &wp_db[i], sizeof(Waypoint));
    }
    char last_key[10];
    sprintf(last_key, "wp_%d", wp_count);
    wp_prefs.remove(last_key);
    mostrar_toast("WAYPOINT ELIMINADO");
}

// ---------------------------------------------------------------------
// 3. PUNTEROS DE INTERFAZ GLOBAL Y ENRUTADOR HARDWARE
// ---------------------------------------------------------------------
static lv_obj_t * scr_mapa = NULL;
static lv_obj_t * tile_container = NULL;
static lv_obj_t * punto_rojo = NULL;
static lv_obj_t * punto_gps = NULL;
static lv_obj_t * btn_gps = NULL;
static lv_obj_t * panel_osd = NULL;
static lv_obj_t * lbl_osd = NULL;
static lv_obj_t * panel_osd_info = NULL;
static lv_obj_t * lbl_osd_info = NULL;

static lv_obj_t * btn_open_wp_menu = NULL;
static lv_obj_t * panel_wp_menu = NULL;
static lv_obj_t * lbl_wp_manual_inst = NULL;
static lv_anim_t anim_blinker;

static lv_obj_t * panel_toast = NULL;
static lv_obj_t * lbl_toast = NULL;

static lv_obj_t * scr_wp_coords = NULL;
static lv_obj_t * scr_wp_sd = NULL;
static lv_obj_t * scr_wp_ble = NULL;
static lv_obj_t * scr_wp_list = NULL;
static lv_obj_t * scr_wp_details = NULL;
static lv_obj_t * scr_wp_edit = NULL;

static lv_obj_t * ta_coords = NULL;
static lv_obj_t * ta_edit_name = NULL;
static lv_obj_t * lbl_wp_details = NULL;
static lv_obj_t * list_waypoints_obj = NULL;

volatile bool modo_inyeccion_tactil = false;
volatile bool modo_seleccion_ruta_tactil = false;
volatile bool seleccionando_ruta_db = false;
static lv_obj_t * btn_menu_ruta_ptr = NULL;
static lv_obj_t * lbl_menu_ruta_ptr = NULL;
static lv_obj_t * btn_menu_color_ptr = NULL;
static lv_obj_t * lbl_menu_color_ptr = NULL;
volatile double target_route_lat = 0.0;
volatile double target_route_lng = 0.0;
static lv_obj_t * panel_route_menu = NULL;

volatile bool modo_seleccion_origen_tactil = false;
volatile double start_route_lat = 0.0;
volatile double start_route_lng = 0.0;
volatile int max_snr = 0;
volatile int sats_in_view = 0; 
volatile bool flag_calc_route = false; 

static lv_obj_t * panel_route_origen = NULL;
static lv_obj_t * btn_origen_gnss = NULL;
static lv_obj_t * lbl_origen_gnss = NULL;

volatile bool pausa_lectura_sd = false;
volatile bool ghost_miner_parked = false;

struct RouteNode { float lat; float lon; };
struct ProjectedRouteNode { float norm_x; float norm_y; };

const uint32_t MAX_ROUTE_POINTS_POOL = 2048;
ProjectedRouteNode * route_norm_pool = NULL;
lv_point_precise_t * route_pixels_pool = NULL;
uint32_t active_route_len = 0;

lv_obj_t * route_line = NULL;
lv_anim_t anim_route_pulse;

// Paleta de colores tácticos para el punto de exploración y línea de ruta (contraste con Pip-Boy)
static uint8_t tactical_color_idx = 0; // 0 = Rosa Neón por defecto
static const uint32_t TACTICAL_COLORS_HEX[6] = {
    0xFF007F, // 0: ROSA NEON / MAGENTA
    0x00E5FF, // 1: CIAN CYBERPUNK
    0xFFB300, // 2: AMBAR / ORO
    0xFF003C, // 3: ROJO TACTICO
    0xFFFFFF, // 4: BLANCO PURO
    0x00FF00  // 5: VERDE MATRIZ
};
static const char* TACTICAL_COLOR_NAMES[6] = {
    "COLOR: ROSA NEON",
    "COLOR: CIAN",
    "COLOR: AMBAR",
    "COLOR: ROJO",
    "COLOR: BLANCO",
    "COLOR: VERDE"
};

static inline uint32_t get_active_tactical_color() {
    return TACTICAL_COLORS_HEX[tactical_color_idx % 6];
}

static void aplicar_color_tactico_ui() {
    uint32_t col = get_active_tactical_color();
    if (punto_rojo != NULL) {
        lv_obj_set_style_bg_color(punto_rojo, lv_color_hex(col), LV_PART_MAIN);
    }
    if (lbl_wp_manual_inst != NULL) {
        lv_obj_set_style_text_color(lbl_wp_manual_inst, lv_color_hex(col), LV_PART_MAIN);
    }
    if (route_line != NULL) {
        lv_obj_set_style_line_color(route_line, lv_color_hex(col), LV_PART_MAIN);
        lv_obj_invalidate(route_line);
    }
    if (btn_menu_color_ptr != NULL && lbl_menu_color_ptr != NULL) {
        lv_label_set_text(lbl_menu_color_ptr, TACTICAL_COLOR_NAMES[tactical_color_idx % 6]);
        lv_obj_set_style_bg_color(btn_menu_color_ptr, lv_color_hex(col), LV_PART_MAIN);
        uint32_t txt_col = (tactical_color_idx == 1 || tactical_color_idx == 2 || tactical_color_idx == 4 || tactical_color_idx == 5) ? 0x000000 : 0xFFFFFF;
        lv_obj_set_style_text_color(lbl_menu_color_ptr, lv_color_hex(txt_col), LV_PART_MAIN);
    }
}

static void liberar_buffers_ruta() {
    active_route_len = 0;
    if (route_norm_pool != NULL) {
        heap_caps_free(route_norm_pool);
        route_norm_pool = NULL;
    }
    if (route_pixels_pool != NULL) {
        heap_caps_free(route_pixels_pool);
        route_pixels_pool = NULL;
    }
}

bool hardware_back_router() {
    lv_obj_t * current = lv_scr_act();
    
    if (current == scr_wp_coords || current == scr_wp_sd || current == scr_wp_ble || current == scr_wp_list) {
        lv_scr_load(scr_mapa);
        lv_obj_clear_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN); 
        seleccionando_ruta_db = false; 
        return true;
    } 
    else if (current == scr_wp_details) { lv_scr_load(scr_wp_list); return true; }
    else if (current == scr_wp_edit) { lv_scr_load(scr_wp_details); return true; }
    else if (current == scr_mapa) {
        if (!lv_obj_has_flag(panel_osd_info, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(panel_osd_info, LV_OBJ_FLAG_HIDDEN);
            return true;
        } 
        else if (osd_visible) {
            osd_visible = false;
            lv_obj_add_flag(panel_osd, LV_OBJ_FLAG_HIDDEN);
            return true;
        } 
        else if (modo_inyeccion_tactil || modo_seleccion_ruta_tactil || modo_seleccion_origen_tactil) {
            modo_inyeccion_tactil = false; modo_seleccion_ruta_tactil = false; modo_seleccion_origen_tactil = false;
            lv_anim_del(punto_rojo, NULL);
            lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN); 
            lv_obj_add_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN); 
            return true;
        } 
        else if (!lv_obj_has_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN); 
            return true;
        } 
        else if (!lv_obj_has_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN); 
            return true;
        } 
        else if (!lv_obj_has_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN); 
            return true;
        } 
        else if (modo_exploracion) {
            modo_exploracion = false;
            seguimiento_gnss_activo = true;
            lv_obj_add_flag(btn_gps, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN); 
            lv_obj_add_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
            stop_kinetic_glide();
            touch_hist_count = 0; vector_x = 0; vector_y = 0;
            anclar_a_gnss();
            mostrar_toast("SEGUIMIENTO GNSS ACTIVO");
            return true;
        }
        else {
            stop_kinetic_glide();
            touch_hist_count = 0; vector_x = 0; vector_y = 0;
            return false;
        }
    }
    return false;
}

// ---------------------------------------------------------------------
// 4. BÓVEDA L2 LOCK-FREE DE 20 TESELAS (40 KB), 4 CABECERAS Y BLACKLIST
// ---------------------------------------------------------------------
struct MapTile {
    lv_obj_t * obj;
    lv_image_dsc_t dsc;
    uint8_t * buffer;
    int px, py;                
    uint32_t lx, ly;           
    volatile bool needs_load;
    volatile bool ready_to_refresh;
};
MapTile tiles[16];

struct QOIVault {
    volatile uint32_t lx, ly;
    volatile uint8_t z;
    uint8_t * buffer;
    volatile uint32_t length;
    volatile bool valid;
};
QOIVault l2_vault[MAX_QOI_TILES];
int l2_head = 0;

const int MAX_HEADER_CACHE = 4;
struct HeaderCacheSlot {
    uint32_t cx, cy;
    uint8_t z;
    uint32_t * data;
    bool valid;
};
HeaderCacheSlot hdr_cache[MAX_HEADER_CACHE];
uint8_t hdr_cache_head = 0;

uint64_t sd_blacklist[100];
uint8_t bl_head = 0;

static uint64_t empty_tile_ring[128] = {0};
static uint8_t empty_tile_head = 0;

static inline uint64_t make_cluster_key(uint8_t z, uint32_t cx, uint32_t cy) {
    return ((uint64_t)z << 48) | ((uint64_t)(cx & 0xFFFFFF) << 24) | (uint64_t)(cy & 0xFFFFFF);
}

static inline bool is_tile_known_empty(uint8_t z, uint32_t lx, uint32_t ly) {
    uint64_t key = make_cluster_key(z, lx, ly);
    for (int i = 0; i < 128; i++) {
        if (empty_tile_ring[i] == key) return true;
    }
    return false;
}

static inline void mark_tile_known_empty(uint8_t z, uint32_t lx, uint32_t ly) {
    uint64_t key = make_cluster_key(z, lx, ly);
    for (int i = 0; i < 128; i++) {
        if (empty_tile_ring[i] == key) return;
    }
    empty_tile_ring[empty_tile_head] = key;
    empty_tile_head = (empty_tile_head + 1) & 127;
}

bool is_blacklisted(uint8_t z, uint32_t cx, uint32_t cy) {
    uint64_t target = make_cluster_key(z, cx, cy);
    for(int i=0; i<100; i++) { if(sd_blacklist[i] == target) return true; }
    return false;
}

void blacklist_cluster(uint8_t z, uint32_t cx, uint32_t cy) {
    uint64_t target = make_cluster_key(z, cx, cy);
    for(int i=0; i<100; i++) { if(sd_blacklist[i] == target) return; }
    sd_blacklist[bl_head] = target;
    bl_head = (bl_head + 1) % 100;
}

IRAM_ATTR int search_vault_lockfree(uint8_t z, uint32_t lx, uint32_t ly) {
    for(int i = 0; i < MAX_QOI_TILES; i++) {
        if(l2_vault[i].valid && l2_vault[i].z == z && l2_vault[i].lx == lx && l2_vault[i].ly == ly) {
            return i;
        }
    }
    return -1;
}

int search_header_cache(uint8_t z, uint32_t cx, uint32_t cy) {
    for(int i = 0; i < MAX_HEADER_CACHE; i++) {
        if(hdr_cache[i].valid && hdr_cache[i].z == z && hdr_cache[i].cx == cx && hdr_cache[i].cy == cy) return i;
    }
    return -1;
}

// ---------------------------------------------------------------------
// 4.1 AUDITORÍA DE COBERTURA LOCAL POR ZONA (PROTEGIDA CON MUTEX SPI)
// ---------------------------------------------------------------------
static void check_zone_coverage_at_zoom(
    uint8_t test_z,
    uint32_t base_tile_x,
    uint32_t base_tile_y,
    int &out_center_valid,
    int &out_total_valid
) {
    out_center_valid = 0;
    out_total_valid = 0;
    bool miner_was_parked = false;

    for (int i = 0; i < 16; i++) {
        int px = (i % 4) - 1;
        int py = (i / 4) - 1;
        uint32_t tx = base_tile_x + px;
        uint32_t ty = base_tile_y + py;
        bool is_center = (px == 0 || px == 1) && (py == 0 || py == 1);

        if (search_vault_lockfree(test_z, tx, ty) != -1) {
            out_total_valid++;
            if (is_center) out_center_valid++;
            continue;
        }

        uint32_t cx = tx / CLUSTER_SIZE;
        uint32_t cy = ty / CLUSTER_SIZE;
        if (is_blacklisted(test_z, cx, cy)) continue;

        int slot = search_header_cache(test_z, cx, cy);
        if (slot == -1) {
            if (!miner_was_parked) {
                ghost_miner_parked = false;
                pausa_lectura_sd = true;
                if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
                uint32_t t0 = millis();
                while (!ghost_miner_parked && (millis() - t0 < 150)) {
                    delay(1);
                }
                if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
                if (map_file.isOpen()) map_file.close();
                miner_was_parked = true;
            }

            char sd_path[64];
            snprintf(sd_path, sizeof(sd_path), "/MAP/Z%d/C_%u_%u.QOI", test_z, cx, cy);
            if (!sd.exists(sd_path)) {
                blacklist_cluster(test_z, cx, cy);
                continue;
            }
            FsFile chk_file = sd.open(sd_path, O_READ);
            if (!chk_file) {
                continue;
            }

            slot = hdr_cache_head;
            hdr_cache_head = (hdr_cache_head + 1) % MAX_HEADER_CACHE;
            hdr_cache[slot].valid = false;
            if (chk_file.read(hdr_cache[slot].data, 8192) == 8192) {
                hdr_cache[slot].cx = cx;
                hdr_cache[slot].cy = cy;
                hdr_cache[slot].z = test_z;
                hdr_cache[slot].valid = true;
            } else {
                slot = -1;
            }
            chk_file.close();
        }

        if (slot >= 0 && hdr_cache[slot].valid) {
            uint32_t pair_idx = ((tx % CLUSTER_SIZE) * CLUSTER_SIZE + (ty % CLUSTER_SIZE));
            uint32_t off = hdr_cache[slot].data[pair_idx * 2];
            uint32_t len = hdr_cache[slot].data[pair_idx * 2 + 1];
            if (off >= 8192 && len > 20) {
                out_total_valid++;
                if (is_center) out_center_valid++;
            }
        }
    }

    if (miner_was_parked) {
        if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
        pausa_lectura_sd = false;
        if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
    }
}

// ---------------------------------------------------------------------
// 5. DESCOMPRESOR QOI-565 EN IRAM + GESTOR DINÁMICO DE PLACEHOLDER SD
// ---------------------------------------------------------------------
IRAM_ATTR void decode_qoi565_fast(const uint8_t * __restrict__ in_buffer, uint32_t comp_len, uint8_t * __restrict__ out_buffer) {
    uint16_t cache[64] = {0};
    uint16_t sram_strip[1024];
    uint8_t * __restrict__ psram_dst = out_buffer;
    uint32_t in_idx = 0;
    uint32_t strip_idx = 0;
    uint32_t total_written = 0;
    uint16_t prev_px = 0;

    while (in_idx < comp_len && total_written < 65536) {
        uint8_t op = in_buffer[in_idx++];
        if ((op & 0xC0) == 0x00) {
            uint32_t run = (op & 0x3F) + 1;
            if (total_written + run > 65536) run = 65536 - total_written;
            total_written += run;
            while (run > 0) {
                uint32_t space = 1024 - strip_idx;
                uint32_t chunk = (run < space) ? run : space;
                for (uint32_t k = 0; k < chunk; k++) {
                    sram_strip[strip_idx + k] = prev_px;
                }
                strip_idx += chunk;
                run -= chunk;
                if (strip_idx == 1024) {
                    memcpy(psram_dst, sram_strip, 2048);
                    psram_dst += 2048;
                    strip_idx = 0;
                }
            }
        } else if ((op & 0xC0) == 0x40) {
            prev_px = cache[op & 0x3F];
            sram_strip[strip_idx++] = prev_px;
            total_written++;
            if (strip_idx == 1024) {
                memcpy(psram_dst, sram_strip, 2048);
                psram_dst += 2048;
                strip_idx = 0;
            }
        } else if (op == 0x80) {
            if (in_idx + 1 >= comp_len) break;
            uint16_t px = in_buffer[in_idx] | (in_buffer[in_idx + 1] << 8);
            in_idx += 2;
            cache[(px ^ (px >> 3)) & 63] = px;
            prev_px = px;
            sram_strip[strip_idx++] = px;
            total_written++;
            if (strip_idx == 1024) {
                memcpy(psram_dst, sram_strip, 2048);
                psram_dst += 2048;
                strip_idx = 0;
            }
        }
    }
    while (total_written < 65536) {
        sram_strip[strip_idx++] = 0xDEDE;
        total_written++;
        if (strip_idx == 1024) {
            memcpy(psram_dst, sram_strip, 2048);
            psram_dst += 2048;
            strip_idx = 0;
        }
    }
    if (strip_idx > 0) {
        memcpy(psram_dst, sram_strip, strip_idx * 2);
    }
}

// Genera una retícula táctica de radar de 256x256 en RGB565 si la SD no tiene placeholder personalizado
static void render_fallback_tactical_placeholder(uint8_t * dst_bytes) {
    if (!dst_bytes) return;
    uint16_t * px_buf = (uint16_t *)dst_bytes;
    const uint16_t c_bg     = 0x0842; // Fondo táctico oscuro (#080A10)
    const uint16_t c_sub    = 0x10A3; // Sub-rejilla cada 32 px
    const uint16_t c_grid   = 0x1925; // Rejilla principal cada 64 px
    const uint16_t c_border = 0x21A7; // Borde de tesela 256x256
    const uint16_t c_cross  = 0x03EF; // Mira central cian tenue

    for (int y = 0; y < 256; y++) {
        uint16_t * row = &px_buf[y * 256];
        bool is_border_y = (y == 0 || y == 255);
        bool is_grid_y   = ((y & 63) == 0);
        bool is_sub_y    = ((y & 31) == 0);
        for (int x = 0; x < 256; x++) {
            uint16_t col = c_bg;
            if (is_border_y || x == 0 || x == 255) {
                col = c_border;
            } else if (is_grid_y || (x & 63) == 0) {
                col = c_grid;
            } else if (is_sub_y && (x & 3) == 0) {
                col = c_sub;
            } else if ((x & 31) == 0 && (y & 3) == 0) {
                col = c_sub;
            }
            if ((y == 128 && x >= 120 && x <= 136) || (x == 128 && y >= 120 && y <= 136)) {
                col = c_cross;
            }
            row[x] = col;
        }
    }
}

// Carga o recarga en caliente el placeholder desde la MicroSD (/UI/placeholder.qoi o /UI/placeholder.bin)
bool reload_custom_placeholder() {
    if (static_placeholder == NULL) return false;

    static const char* qoi_paths[3] = {
        "/UI/placeholder.qoi",
        "/UI/PLACEHOLDER.QOI",
        "/MAP/PLACEHOLDER.QOI"
    };
    static const char* bin_paths[3] = {
        "/UI/placeholder.bin",
        "/UI/PLACEHOLDER.BIN",
        "/MAP/PLACEHOLDER.BIN"
    };

    bool loaded_ok = false;
    bool take_lock = (sd_spi_mutex != NULL);
    if (take_lock) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);

    // 1. Prioridad: Placeholder comprimido en QOI-565 (~12-25 KB, ultrarrápido de enviar por Wi-Fi)
    for (int i = 0; i < 3 && !loaded_ok; i++) {
        if (sd.exists(qoi_paths[i])) {
            FsFile qf = sd.open(qoi_paths[i], O_READ);
            if (qf) {
                uint32_t fsize = qf.size();
                if (fsize > 20 && fsize <= COMPRESSED_BUF_BYTES) {
                    uint8_t * tmp_qoi = (uint8_t*)heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM);
                    if (tmp_qoi) {
                        if (qf.read(tmp_qoi, fsize) == (int)fsize) {
                            decode_qoi565_fast(tmp_qoi, fsize, static_placeholder);
                            loaded_ok = true;
                            Serial.printf("[CARTO ENGINE] Placeholder QOI cargado desde %s (%u bytes)\n", qoi_paths[i], fsize);
                        }
                        heap_caps_free(tmp_qoi);
                    }
                }
                qf.close();
            }
        }
    }

    // 2. Respaldo: Placeholder en binario RGB565 crudo de 256x256 (131,072 bytes exactos)
    for (int i = 0; i < 3 && !loaded_ok; i++) {
        if (sd.exists(bin_paths[i])) {
            FsFile bf = sd.open(bin_paths[i], O_READ);
            if (bf) {
                if (bf.size() >= 131072 && bf.read(static_placeholder, 131072) == 131072) {
                    loaded_ok = true;
                    Serial.printf("[CARTO ENGINE] Placeholder BIN RGB565 cargado desde %s (131072 bytes)\n", bin_paths[i]);
                }
                bf.close();
            }
        }
    }

    if (take_lock) xSemaphoreGive(sd_spi_mutex);

    if (loaded_ok) {
        placeholder_sd_loaded = true;
    }

    // Si el mapa ya está en pantalla, refrescar al instante las teselas que estén mostrando el placeholder
    if (scr_mapa != NULL && xSemaphoreTakeRecursive(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < 16; i++) {
            if (tiles[i].obj != NULL && tiles[i].dsc.data == static_placeholder) {
                lv_obj_invalidate(tiles[i].obj);
            }
        }
        xSemaphoreGiveRecursive(lvgl_mutex);
    }

    return loaded_ok;
}

static inline bool is_tile_entering_viewport(int tile_idx) {
    int32_t sx0 = -307 + drag_pixel_x + (tiles[tile_idx].px + 1) * 256;
    int32_t sy0 = -261 + drag_pixel_y + (tiles[tile_idx].py + 1) * 256;
    return (sx0 + 256 > -128) && (sx0 < 528) && (sy0 + 256 > -128) && (sy0 < 528);
}

static inline bool try_instant_l2_hydrate(int tile_idx, bool allow_sync_decode = false) {
    bool hydrated = false;
    uint32_t tx = tiles[tile_idx].lx;
    uint32_t ty = tiles[tile_idx].ly;
    uint8_t tz = current_z;

    if (allow_sync_decode && is_tile_entering_viewport(tile_idx)) {
        int v_idx = search_vault_lockfree(tz, tx, ty);
        if (v_idx != -1) {
            uint32_t len = l2_vault[v_idx].length;
            const uint8_t * src_ptr = l2_vault[v_idx].buffer;
            if (l2_vault[v_idx].valid && l2_vault[v_idx].lx == tx && l2_vault[v_idx].ly == ty && l2_vault[v_idx].z == tz) {
                decode_qoi565_fast(src_ptr, len, tiles[tile_idx].buffer);
                hydrated = true;
            }
        }
    }

    portENTER_CRITICAL(&tile_spinlock);
    if (hydrated) {
        tiles[tile_idx].dsc.data = tiles[tile_idx].buffer;
        tiles[tile_idx].needs_load = false;
        tiles[tile_idx].ready_to_refresh = true;
    } else {
        tiles[tile_idx].dsc.data = static_placeholder;
        tiles[tile_idx].ready_to_refresh = true;
        tiles[tile_idx].needs_load = true;
    }
    portEXIT_CRITICAL(&tile_spinlock);
    return hydrated;
}

// ---------------------------------------------------------------------
// 6. DRIVER HÁPTICO DRV2605
// ---------------------------------------------------------------------
void init_haptico() {
    Wire.beginTransmission(0x5A); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
}

void disparar_haptico(uint8_t efecto) {
    Wire.beginTransmission(0x5A); Wire.write(0x04); Wire.write(efecto); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x05); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(0x01); Wire.endTransmission();
}

// ---------------------------------------------------------------------
// 7. INYECCIÓN INVERSA, CÓDIGO DE COLORES Y SEGUIMIENTO TIPO GOOGLE MAPS
// ---------------------------------------------------------------------
void calcular_coordenadas_tactiles(double *lat_out, double *lng_out) {
    int32_t sw = lv_obj_get_width(scr_mapa); int32_t sh = lv_obj_get_height(scr_mapa);
    if (sw == 0) sw = 400; if (sh == 0) sh = 400;
    
    double offset_cx = (sw / 2.0) + 51.0; double offset_cy = (sh / 2.0) + 5.0;
    double px_global_x = ((double)current_x * 256.0) + offset_cx - (double)drag_pixel_x;
    double px_global_y = ((double)current_y * 256.0) + offset_cy - (double)drag_pixel_y;
    double n = pow(2.0, current_z);
    
    *lng_out = (px_global_x / n * 360.0) / 256.0 - 180.0;
    double lat_rad = atan(sinh(3.14159265358979323846 * (1.0 - 2.0 * (px_global_y / 256.0) / n)));
    *lat_out = lat_rad * 180.0 / 3.14159265358979323846;
}

static void posicionar_punto_gps_en_contenedor() {
    if (punto_gps == NULL) return;

    if (!gps_locked || gnss_active_source == 0 || current_lat == 0.0 || current_lng == 0.0) {
        if (!lv_obj_has_flag(punto_gps, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(punto_gps, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    static uint8_t last_painted_source = 0;
    uint8_t src_now = gnss_active_source;
    if (src_now != last_painted_source && src_now != 0) {
        uint32_t hex_col = 0x007AFF;
        if (src_now == 2) hex_col = 0xFFD600;
        else if (src_now == 3) hex_col = 0xFF00EA;

        lv_obj_set_style_bg_color(punto_gps, lv_color_hex(hex_col), LV_PART_MAIN);
        lv_obj_set_style_shadow_color(punto_gps, lv_color_hex(hex_col), LV_PART_MAIN);
        last_painted_source = src_now;
    }

    double n = pow(2.0, current_z);
    double gx = (current_lng + 180.0) / 360.0 * n;
    double lat_rad = current_lat * 3.14159265358979323846 / 180.0;
    double gy = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / 3.14159265358979323846) / 2.0 * n;

    int32_t cx_px = 256 + (int32_t)round((gx - (double)current_x) * 256.0) - 8;
    int32_t cy_px = 256 + (int32_t)round((gy - (double)current_y) * 256.0) - 8;

    static int32_t last_gps_px = -99999;
    static int32_t last_gps_py = -99999;
    if (lv_obj_has_flag(punto_gps, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_clear_flag(punto_gps, LV_OBJ_FLAG_HIDDEN);
    }
    if (cx_px != last_gps_px || cy_px != last_gps_py) {
        lv_obj_set_pos(punto_gps, cx_px, cy_px);
        last_gps_px = cx_px;
        last_gps_py = cy_px;
    }
}

void actualizar_marcador_gps() {
    if (!gps_locked || gnss_active_source == 0 || current_lat == 0.0 || current_lng == 0.0) {
        posicionar_punto_gps_en_contenedor();
        return;
    }

    if (gnss_pos_dirty) {
        if (touch_down_on_map) return;
        gnss_pos_dirty = false;
        if (seguimiento_gnss_activo && !modo_exploracion && !pinch_active && !glide_active && !pausa_lectura_sd) {
            anclar_a_gnss();
            return;
        }
    }

    posicionar_punto_gps_en_contenedor();
}

void anclar_a_gnss() {
    if (!gps_locked || current_lat == 0.0 || current_lng == 0.0) return;

    double n = pow(2.0, current_z);
    double gx = (current_lng + 180.0) / 360.0 * n;
    double lat_rad = current_lat * 3.14159265358979323846 / 180.0;
    double gy = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / 3.14159265358979323846) / 2.0 * n;

    int32_t sw = lv_obj_get_width(scr_mapa); int32_t sh = lv_obj_get_height(scr_mapa);
    if (sw == 0) sw = 400; if (sh == 0) sh = 400;
    double offset_cx = (sw / 2.0) + 51.0; double offset_cy = (sh / 2.0) + 5.0;

    double base_x = (gx * 256.0) - offset_cx; double base_y = (gy * 256.0) - offset_cy;

    uint32_t target_x = (uint32_t)floor(base_x / 256.0);
    uint32_t target_y = (uint32_t)floor(base_y / 256.0);
    int32_t target_drag_x = (int32_t)round((double)target_x * 256.0 - base_x);
    int32_t target_drag_y = (int32_t)round((double)target_y * 256.0 - base_y);

    double current_abs_x = ((double)current_x * 256.0) - (double)drag_pixel_x;
    double current_abs_y = ((double)current_y * 256.0) - (double)drag_pixel_y;
    double target_abs_x = ((double)target_x * 256.0) - (double)target_drag_x;
    double target_abs_y = ((double)target_y * 256.0) - (double)target_drag_y;

    if (abs(current_abs_x - target_abs_x) > 256 || abs(current_abs_y - target_abs_y) > 256) {
        current_x = target_x; current_y = target_y;
        drag_pixel_x = target_drag_x; drag_pixel_y = target_drag_y;

        for(int i=0; i<16; i++) {
            tiles[i].px = (i % 4) - 1; tiles[i].py = (i / 4) - 1; 
            tiles[i].lx = current_x + tiles[i].px; tiles[i].ly = current_y + tiles[i].py;
            lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
            try_instant_l2_hydrate(i, false);
        }
        lv_obj_set_pos(tile_container, -307 + drag_pixel_x, -261 + drag_pixel_y);
        refrescar_linea_ruta();
        if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
    } 
    else {
        int16_t step_dx = (int16_t)(current_abs_x - target_abs_x);
        int16_t step_dy = (int16_t)(current_abs_y - target_abs_y);
        if (step_dx != 0 || step_dy != 0) update_map_topology(step_dx, step_dy);
    }

    posicionar_punto_gps_en_contenedor();
}

void ir_a_coordenada(double lat, double lng) {
    double n = pow(2.0, current_z);
    double gx = (lng + 180.0) / 360.0 * n;
    double lat_rad = lat * 3.14159265358979323846 / 180.0;
    double gy = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / 3.14159265358979323846) / 2.0 * n;

    int32_t sw = lv_obj_get_width(scr_mapa); int32_t sh = lv_obj_get_height(scr_mapa);
    if (sw == 0) sw = 400; if (sh == 0) sh = 400;
    
    double offset_cx = (sw / 2.0) + 51.0; double offset_cy = (sh / 2.0) + 5.0;
    double base_x = (gx * 256.0) - offset_cx; double base_y = (gy * 256.0) - offset_cy;

    current_x = (uint32_t)floor(base_x / 256.0);
    current_y = (uint32_t)floor(base_y / 256.0);
    drag_pixel_x = (int32_t)round((double)current_x * 256.0 - base_x);
    drag_pixel_y = (int32_t)round((double)current_y * 256.0 - base_y);

    for(int i=0; i<16; i++) {
        tiles[i].px = (i % 4) - 1; tiles[i].py = (i / 4) - 1; 
        tiles[i].lx = current_x + tiles[i].px; tiles[i].ly = current_y + tiles[i].py;
        lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        try_instant_l2_hydrate(i, false);
    }
    
    lv_obj_set_pos(tile_container, -307 + drag_pixel_x, -261 + drag_pixel_y);
    refrescar_linea_ruta();
    posicionar_punto_gps_en_contenedor();
    if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
}

void ejecutar_zoom(int8_t salto) {
    char z_msg[36];
    if (salto > 0) {
        if (current_z >= max_sd_z) {
            snprintf(z_msg, sizeof(z_msg), "LIMITE SD: Z%d MAX", max_sd_z);
            mostrar_toast(z_msg);
            return;
        }
    } else if (salto < 0) {
        if (current_z <= min_sd_z) {
            snprintf(z_msg, sizeof(z_msg), "LIMITE SD: Z%d MIN", min_sd_z);
            mostrar_toast(z_msg);
            return;
        }
    } else {
        return;
    }

    int32_t sw = lv_obj_get_width(scr_mapa); int32_t sh = lv_obj_get_height(scr_mapa);
    if (sw == 0) sw = 400; if (sh == 0) sh = 400;
    double offset_cx = (sw / 2.0) + 51.0; double offset_cy = (sh / 2.0) + 5.0;
    double orig_gx = ((double)current_x * 256.0) + offset_cx - (double)drag_pixel_x;
    double orig_gy = ((double)current_y * 256.0) + offset_cy - (double)drag_pixel_y;

    int cur_center_ok = 0, cur_total_ok = 0;
    for (int i = 0; i < 16; i++) {
        if (tiles[i].dsc.data != static_placeholder) {
            cur_total_ok++;
            if ((tiles[i].px == 0 || tiles[i].px == 1) && (tiles[i].py == 0 || tiles[i].py == 1)) {
                cur_center_ok++;
            }
        }
    }
    bool current_matrix_has_map = (cur_center_ok >= 1 || cur_total_ok >= 3);

    uint8_t chosen_z = current_z;
    uint32_t chosen_x = current_x;
    uint32_t chosen_y = current_y;
    int32_t chosen_drag_x = drag_pixel_x;
    int32_t chosen_drag_y = drag_pixel_y;

    if (salto > 0) {
        for (uint8_t z = current_z + 1; z <= max_sd_z; z++) {
            if (valid_z_mask != 0 && !(valid_z_mask & (1UL << z))) continue;

            int diff_z = (int)z - (int)current_z;
            double scale = pow(2.0, diff_z);
            double cand_gx = orig_gx * scale;
            double cand_gy = orig_gy * scale;
            double base_x = cand_gx - offset_cx;
            double base_y = cand_gy - offset_cy;
            uint32_t cand_x = (uint32_t)floor(base_x / 256.0);
            uint32_t cand_y = (uint32_t)floor(base_y / 256.0);

            int c_ok = 0, t_ok = 0;
            check_zone_coverage_at_zoom(z, cand_x, cand_y, c_ok, t_ok);

            if (c_ok >= 1 || t_ok >= 4) {
                chosen_z = z;
                chosen_x = cand_x;
                chosen_y = cand_y;
                chosen_drag_x = (int32_t)round((double)cand_x * 256.0 - base_x);
                chosen_drag_y = (int32_t)round((double)cand_y * 256.0 - base_y);
            } else {
                snprintf(z_msg, sizeof(z_msg), "LIMITE ZONA: MAX Z%d", current_z);
                mostrar_toast(z_msg);
                disparar_haptico(47);
                return;
            }
            break;
        }
    } else {
        for (int z = (int)current_z - 1; z >= (int)min_sd_z; z--) {
            if (valid_z_mask != 0 && !(valid_z_mask & (1UL << z))) continue;

            int diff_z = z - (int)current_z;
            double scale = pow(2.0, diff_z);
            double cand_gx = orig_gx * scale;
            double cand_gy = orig_gy * scale;
            double base_x = cand_gx - offset_cx;
            double base_y = cand_gy - offset_cy;
            uint32_t cand_x = (uint32_t)floor(base_x / 256.0);
            uint32_t cand_y = (uint32_t)floor(base_y / 256.0);

            int c_ok = 0, t_ok = 0;
            check_zone_coverage_at_zoom((uint8_t)z, cand_x, cand_y, c_ok, t_ok);

            if (c_ok >= 1 || t_ok >= 2 || !current_matrix_has_map) {
                chosen_z = (uint8_t)z;
                chosen_x = cand_x;
                chosen_y = cand_y;
                chosen_drag_x = (int32_t)round((double)cand_x * 256.0 - base_x);
                chosen_drag_y = (int32_t)round((double)cand_y * 256.0 - base_y);
                break;
            }
        }

        if (chosen_z == current_z) {
            snprintf(z_msg, sizeof(z_msg), "LIMITE ZONA: MIN Z%d", current_z);
            mostrar_toast(z_msg);
            disparar_haptico(47);
            return;
        }
    }

    if (chosen_z == current_z) return;

    stop_kinetic_glide();
    vector_x = 0; vector_y = 0; touch_hist_count = 0;
    last_pan_Input_ms = millis();

    current_z = chosen_z;
    current_x = chosen_x;
    current_y = chosen_y;
    drag_pixel_x = chosen_drag_x;
    drag_pixel_y = chosen_drag_y;

    for(int i=0; i<16; i++) {
        tiles[i].px = (i % 4) - 1; tiles[i].py = (i / 4) - 1; 
        tiles[i].lx = current_x + tiles[i].px; tiles[i].ly = current_y + tiles[i].py;
        lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        try_instant_l2_hydrate(i, false);
    }
    
    lv_obj_set_pos(tile_container, -307 + drag_pixel_x, -261 + drag_pixel_y);
    posicionar_punto_gps_en_contenedor();
    refrescar_linea_ruta();
    if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
}

void update_map_topology(int16_t dx, int16_t dy) {
    drag_pixel_x = drag_pixel_x + dx; 
    drag_pixel_y = drag_pixel_y + dy;
    if (dx != 0) vector_x = (dx > 0) ? -1 : 1;
    if (dy != 0) vector_y = (dy > 0) ? -1 : 1;
    if (dx != 0 || dy != 0) last_pan_Input_ms = millis();

    bool need_miner_wake = false;
    bool base_tile_shifted = false;

    while (drag_pixel_x <= -256) { 
        drag_pixel_x = drag_pixel_x + 256; current_x++; base_tile_shifted = true;
        for(int i=0; i<16; i++) {
            tiles[i].px -= 1;
            if(tiles[i].px < -1) { 
                tiles[i].px = 2; tiles[i].lx += 4; 
                try_instant_l2_hydrate(i, false);
                need_miner_wake = true;
            }
            lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        }
    }
    while (drag_pixel_x >= 256) { 
        drag_pixel_x = drag_pixel_x - 256; current_x--; base_tile_shifted = true;
        for(int i=0; i<16; i++) {
            tiles[i].px += 1;
            if(tiles[i].px > 2) { 
                tiles[i].px = -1; tiles[i].lx -= 4; 
                try_instant_l2_hydrate(i, false);
                need_miner_wake = true;
            }
            lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        }
    }
    while (drag_pixel_y <= -256) { 
        drag_pixel_y = drag_pixel_y + 256; current_y++; base_tile_shifted = true;
        for(int i=0; i<16; i++) {
            tiles[i].py -= 1;
            if(tiles[i].py < -1) { 
                tiles[i].py = 2; tiles[i].ly += 4; 
                try_instant_l2_hydrate(i, false);
                need_miner_wake = true;
            }
            lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        }
    }
    while (drag_pixel_y >= 256) { 
        drag_pixel_y = drag_pixel_y - 256; current_y--; base_tile_shifted = true;
        for(int i=0; i<16; i++) {
            tiles[i].py += 1;
            if(tiles[i].py > 2) { 
                tiles[i].py = -1; tiles[i].ly -= 4; 
                try_instant_l2_hydrate(i, false);
                need_miner_wake = true;
            }
            lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        }
    }
    lv_obj_set_pos(tile_container, -307 + drag_pixel_x, -261 + drag_pixel_y);

    if (base_tile_shifted) {
        refrescar_linea_ruta();
        posicionar_punto_gps_en_contenedor();
    }

    if ((need_miner_wake || base_tile_shifted) && ghost_miner_handle != NULL) {
        xTaskNotifyGive(ghost_miner_handle);
    }
}

void anim_blink_cb(void * var, int32_t v) { lv_obj_set_style_bg_opa((lv_obj_t*)var, v, LV_PART_MAIN); }

void mostrar_toast(const char * msg) {
    if(lbl_toast == NULL || panel_toast == NULL) return;
    lv_label_set_text(lbl_toast, msg);
    lv_obj_clear_flag(panel_toast, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(panel_toast);
    
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, panel_toast);
    lv_anim_set_values(&a, 255, 0);
    lv_anim_set_time(&a, 500);
    lv_anim_set_delay(&a, 2000); 
    lv_anim_set_exec_cb(&a, [](void * var, int32_t v) {
        lv_obj_set_style_bg_opa((lv_obj_t*)var, v, LV_PART_MAIN);
        lv_obj_set_style_text_opa(lv_obj_get_child((lv_obj_t*)var, 0), v, LV_PART_MAIN);
        if (v == 0) lv_obj_add_flag((lv_obj_t*)var, LV_OBJ_FLAG_HIDDEN);
    });
    lv_anim_start(&a);
    lv_obj_set_style_bg_opa(panel_toast, 255, LV_PART_MAIN);
    lv_obj_set_style_text_opa(lbl_toast, 255, LV_PART_MAIN);
}

// ---------------------------------------------------------------------
// 10. INTERCEPTOR MULTITOUCH CON VENTANA DE 16 MS (COMPATIBLE CON DOUBLE-TAP)
// ---------------------------------------------------------------------
static void exosys_multitouch_read_cb(lv_indev_t * indev, lv_indev_data_t * data) {
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static int16_t tap_anchor_x = 0;
    static int16_t tap_anchor_y = 0;
    static bool was_pressed = false;
    static uint32_t last_valid_touch_ms = 0;

    int16_t tx[2] = {0, 0};
    int16_t ty[2] = {0, 0};
    uint32_t now_ms = millis();

    if (lv_scr_act() != scr_mapa) {
        g_mt_fingers = 0;
        pinch_active = false;
        touch_down_on_map = false;
        uint8_t touched = instance.getPoint(tx, ty, 1);
        if (touched > 0) {
            last_valid_touch_ms = now_ms;
            if (!was_pressed) {
                was_pressed = true;
                tap_anchor_x = tx[0];
                tap_anchor_y = ty[0];
                last_x = tx[0];
                last_y = ty[0];
            } else {
                if (abs(tx[0] - tap_anchor_x) > 6 || abs(ty[0] - tap_anchor_y) > 6) {
                    last_x = tx[0];
                    last_y = ty[0];
                }
            }
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else if (was_pressed && (now_ms - last_valid_touch_ms <= 24UL)) {
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            was_pressed = false;
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_RELEASED;
        }
        return;
    }

    uint8_t touched = instance.getPoint(tx, ty, 2);

    if (touched > 0) {
        last_valid_touch_ms = now_ms;
        was_pressed = true;
        touch_down_on_map = true;
        g_mt_fingers = touched;
        g_mt_x[0] = tx[0];
        g_mt_y[0] = ty[0];
        if (touched >= 2) {
            g_mt_x[1] = tx[1];
            g_mt_y[1] = ty[1];
        }
        last_x = tx[0];
        last_y = ty[0];
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else if (was_pressed && (now_ms - last_valid_touch_ms <= 16UL)) {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        g_mt_fingers = 0;
        was_pressed = false;
        touch_down_on_map = false;
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ---------------------------------------------------------------------
// 11. BUILDERS DE PANTALLAS SECUNDARIAS
// ---------------------------------------------------------------------
void refrescar_lista_waypoints() {
    lv_obj_clean(list_waypoints_obj);
    lv_list_add_text(list_waypoints_obj, seleccionando_ruta_db ? "Seleccione Destino de Ruta" : "Base de Datos Táctica");
    
    for (int i = 0; i < wp_count; i++) {
        lv_obj_t * btn = lv_list_add_btn(list_waypoints_obj, LV_SYMBOL_GPS, wp_db[i].nombre);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x222222), LV_PART_MAIN);
        lv_obj_set_style_text_color(btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        
        lv_obj_add_event_cb(btn, [](lv_event_t * e) {
            wp_selected_idx = (int)(uintptr_t)lv_event_get_user_data(e); 
            
            if (seleccionando_ruta_db) {
                target_route_lat = wp_db[wp_selected_idx].lat;
                target_route_lng = wp_db[wp_selected_idx].lng;
                seleccionando_ruta_db = false;
                lv_scr_load(scr_mapa);
                mostrar_toast("PREPARANDO RUTA...");
                disparar_haptico(1);
                Serial.printf("[RUTEO] Destino fijado desde SD: %f, %f\n", target_route_lat, target_route_lng);
                
                flag_calc_route = true; 
            } else {
                const char* org = "DESCONOCIDO";
                if(wp_db[wp_selected_idx].origen == 0) org = "INYECCION MANUAL";
                else if(wp_db[wp_selected_idx].origen == 1) org = "COORDENADAS";
                else if(wp_db[wp_selected_idx].origen == 2) org = "MEMORIA SD";
                else if(wp_db[wp_selected_idx].origen == 3) org = "BLUETOOTH";

                lv_label_set_text_fmt(lbl_wp_details, "INFO DE WAYPOINT\n\nLAT: %.6f\nLON: %.6f\nORIGEN: %s", 
                    wp_db[wp_selected_idx].lat, wp_db[wp_selected_idx].lng, org);
                
                lv_scr_load(scr_wp_details);
            }
        }, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }
}

void build_wp_screens() {
    panel_toast = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_toast, 250, 40);
    lv_obj_align(panel_toast, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_set_style_bg_color(panel_toast, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_set_style_radius(panel_toast, 5, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_toast, 0, LV_PART_MAIN);
    lv_obj_add_flag(panel_toast, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(panel_toast, LV_OBJ_FLAG_CLICKABLE);
    lbl_toast = lv_label_create(panel_toast);
    lv_obj_set_style_text_color(lbl_toast, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_toast, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_center(lbl_toast);

    scr_wp_coords = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_coords, lv_color_hex(0x000000), LV_PART_MAIN);
    ta_coords = lv_textarea_create(scr_wp_coords);
    lv_obj_set_size(ta_coords, 380, 80);
    lv_obj_align(ta_coords, LV_ALIGN_TOP_MID, 0, 20);
    lv_textarea_set_placeholder_text(ta_coords, "Latitud, Longitud");
    lv_obj_t * kb_coords = lv_keyboard_create(scr_wp_coords);
    lv_obj_set_size(kb_coords, 340, 200); 
    lv_obj_align(kb_coords, LV_ALIGN_BOTTOM_MID, 0, -20); 
    lv_keyboard_set_mode(kb_coords, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb_coords, ta_coords);
    
    lv_obj_add_event_cb(kb_coords, [](lv_event_t * e){
        if(lv_event_get_code(e) == LV_EVENT_READY) {
            const char * txt = lv_textarea_get_text(ta_coords);
            double lat, lng;
            if (sscanf(txt, "%lf,%lf", &lat, &lng) == 2 && lat >= -90.0 && lat <= 90.0 && lng >= -180.0 && lng <= 180.0) {
                guardar_waypoint(lat, lng, 1);
                lv_obj_set_style_border_color(ta_coords, lv_color_hex(0x00FF00), LV_PART_MAIN);
                disparar_haptico(1);
            } else {
                lv_obj_set_style_border_color(ta_coords, lv_color_hex(0xFF003C), LV_PART_MAIN);
                disparar_haptico(47);
            }
        }
    }, LV_EVENT_READY, NULL);

    scr_wp_sd = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_sd, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_t * lbl_sd = lv_label_create(scr_wp_sd);
    lv_label_set_text(lbl_sd, "IMPORTACION SD\nCargue el archivo .GPX\nen la carpeta raiz.");
    lv_obj_set_style_text_color(lbl_sd, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_center(lbl_sd);

    scr_wp_ble = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_ble, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_t * lbl_ble = lv_label_create(scr_wp_ble);
    lv_label_set_text(lbl_ble, "ENLACE BLE\nEsperando sincronizacion\ncon aplicacion...");
    lv_obj_set_style_text_color(lbl_ble, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_center(lbl_ble);

    scr_wp_list = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_list, lv_color_hex(0x111111), LV_PART_MAIN);
    list_waypoints_obj = lv_list_create(scr_wp_list);
    lv_obj_set_size(list_waypoints_obj, 360, 400);
    lv_obj_center(list_waypoints_obj);
    lv_obj_set_style_bg_color(list_waypoints_obj, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_list_add_text(list_waypoints_obj, "Base de Datos Táctica");

    scr_wp_details = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_details, lv_color_hex(0x111111), LV_PART_MAIN);
    
    lbl_wp_details = lv_label_create(scr_wp_details);
    lv_obj_align(lbl_wp_details, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_text_color(lbl_wp_details, lv_color_hex(0x00FF00), LV_PART_MAIN);
    
    lv_obj_t * btn_go_wp = lv_btn_create(scr_wp_details);
    lv_obj_set_size(btn_go_wp, 220, 50);
    lv_obj_align(btn_go_wp, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_color(btn_go_wp, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_t * lbl_btn_go = lv_label_create(btn_go_wp);
    lv_label_set_text(lbl_btn_go, "VER EN MAPA");
    lv_obj_center(lbl_btn_go);
    lv_obj_add_event_cb(btn_go_wp, [](lv_event_t * e){
        if(lv_event_get_code(e) == LV_EVENT_RELEASED && wp_selected_idx != -1) {
            modo_exploracion = true;
            seguimiento_gnss_activo = false;
            lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(btn_gps, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
            ir_a_coordenada(wp_db[wp_selected_idx].lat, wp_db[wp_selected_idx].lng);
            disparar_haptico(1);
            lv_scr_load(scr_mapa);
        }
    }, LV_EVENT_ALL, NULL);
    
    lv_obj_t * btn_edit_name = lv_btn_create(scr_wp_details);
    lv_obj_set_size(btn_edit_name, 220, 50);
    lv_obj_align(btn_edit_name, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(btn_edit_name, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_btn_edit = lv_label_create(btn_edit_name);
    lv_label_set_text(lbl_btn_edit, "EDITAR NOMBRE");
    lv_obj_center(lbl_btn_edit);
    lv_obj_add_event_cb(btn_edit_name, [](lv_event_t * e){
        if(lv_event_get_code(e) == LV_EVENT_RELEASED && wp_selected_idx != -1) {
            lv_textarea_set_text(ta_edit_name, wp_db[wp_selected_idx].nombre);
            lv_scr_load(scr_wp_edit);
        }
    }, LV_EVENT_ALL, NULL);

    lv_obj_t * btn_del_wp = lv_btn_create(scr_wp_details);
    lv_obj_set_size(btn_del_wp, 220, 50);
    lv_obj_align(btn_del_wp, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(btn_del_wp, lv_color_hex(0xFF003C), LV_PART_MAIN);
    lv_obj_t * lbl_btn_del = lv_label_create(btn_del_wp);
    lv_label_set_text(lbl_btn_del, "BORRAR");
    lv_obj_center(lbl_btn_del);
    lv_obj_add_event_cb(btn_del_wp, [](lv_event_t * e){
        if(lv_event_get_code(e) == LV_EVENT_RELEASED && wp_selected_idx != -1) {
            disparar_haptico(47);
            borrar_waypoint(wp_selected_idx);
            refrescar_lista_waypoints();
            lv_scr_load(scr_wp_list);
        }
    }, LV_EVENT_ALL, NULL);

    scr_wp_edit = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_wp_edit, lv_color_hex(0x000000), LV_PART_MAIN);
    ta_edit_name = lv_textarea_create(scr_wp_edit);
    lv_obj_set_size(ta_edit_name, 380, 80);
    lv_obj_align(ta_edit_name, LV_ALIGN_TOP_MID, 0, 20);
    lv_textarea_set_max_length(ta_edit_name, 23);
    lv_obj_t * kb_name = lv_keyboard_create(scr_wp_edit);
    lv_obj_set_size(kb_name, 340, 200); 
    lv_obj_align(kb_name, LV_ALIGN_BOTTOM_MID, 0, -20); 
    lv_keyboard_set_textarea(kb_name, ta_edit_name);
    
    lv_obj_add_event_cb(kb_name, [](lv_event_t * e){
        lv_event_code_t code = lv_event_get_code(e);
        if(code == LV_EVENT_READY && wp_selected_idx != -1) {
            const char * new_name = lv_textarea_get_text(ta_edit_name);
            strncpy(wp_db[wp_selected_idx].nombre, new_name, 23);
            
            char key_base[10];
            sprintf(key_base, "wp_%d", wp_selected_idx);
            wp_prefs.putBytes(key_base, &wp_db[wp_selected_idx], sizeof(Waypoint));
            
            disparar_haptico(1);
            mostrar_toast("NOMBRE ACTUALIZADO");
            refrescar_lista_waypoints();
            lv_scr_load(scr_wp_list);
        } else if (code == LV_EVENT_CANCEL) {
            lv_scr_load(scr_wp_details);
        }
    }, LV_EVENT_ALL, NULL);
}

// ---------------------------------------------------------------------
// 12. CONSTRUCTOR VISUAL PRINCIPAL, MOTOR FÍSICO Y GESTOS TÁCTICOS
// ---------------------------------------------------------------------
void build_ui() {
    lv_display_t * disp = lv_display_get_default();
    if (disp && lv_display_get_refr_timer(disp)) {
        lv_timer_set_period(lv_display_get_refr_timer(disp), 8);
    }
    if (lv_anim_get_timer()) {
        lv_timer_set_period(lv_anim_get_timer(), 8);
    }

    lv_indev_t * indev_iter = lv_indev_get_next(NULL);
    while (indev_iter != NULL) {
        if (lv_indev_get_type(indev_iter) == LV_INDEV_TYPE_POINTER) {
            lv_indev_set_read_cb(indev_iter, exosys_multitouch_read_cb);
            if (lv_indev_get_read_timer(indev_iter)) {
                lv_timer_set_period(lv_indev_get_read_timer(indev_iter), 10);
            }
            break;
        }
        indev_iter = lv_indev_get_next(indev_iter);
    }

    scr_mapa = lv_obj_create(NULL);
    lv_obj_clear_flag(scr_mapa, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr_mapa, lv_color_hex(0x05000A), LV_PART_MAIN);
    lv_obj_add_flag(scr_mapa, LV_OBJ_FLAG_CLICKABLE);

    tile_container = lv_obj_create(scr_mapa);
    lv_obj_set_size(tile_container, 1024, 1024); 
    lv_obj_set_pos(tile_container, -307, -261); 
    lv_obj_set_style_bg_color(tile_container, lv_color_hex(0xEAEAEA), LV_PART_MAIN);
    lv_obj_set_style_border_width(tile_container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tile_container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(tile_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(tile_container, LV_OBJ_FLAG_CLICKABLE);

    uint8_t * arena_ptr = global_arena;
    for(int i=0; i<16; i++) {
        tiles[i].buffer = arena_ptr;
        arena_ptr += 131072;
        tiles[i].dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        tiles[i].dsc.header.cf = LV_COLOR_FORMAT_RGB565; 
        tiles[i].dsc.header.flags = 0;
        tiles[i].dsc.header.w = 256;
        tiles[i].dsc.header.h = 256;
        tiles[i].dsc.header.stride = 256 * 2; 
        tiles[i].dsc.data_size = 131072;
        tiles[i].dsc.data = static_placeholder;
        tiles[i].px = (i % 4) - 1; 
        tiles[i].py = (i / 4) - 1; 
        tiles[i].lx = current_x + tiles[i].px;
        tiles[i].ly = current_y + tiles[i].py;
        tiles[i].needs_load = true;
        tiles[i].ready_to_refresh = false;
        tiles[i].obj = lv_image_create(tile_container);
        lv_obj_set_size(tiles[i].obj, 256, 256);
        lv_obj_set_pos(tiles[i].obj, (tiles[i].px + 1) * 256, (tiles[i].py + 1) * 256);
        lv_obj_remove_flag(tiles[i].obj, LV_OBJ_FLAG_CLICKABLE);
        lv_image_set_src(tiles[i].obj, &tiles[i].dsc);
    }

    lv_timer_create([](lv_timer_t * t) {
        uint8_t max_refreshes_per_tick = (touch_down_on_map || glide_active) ? 1 : 4;
        uint8_t refreshed_now = 0;
        for(int i=0; i<16; i++) {
            if (tiles[i].ready_to_refresh) {
                tiles[i].ready_to_refresh = false;
                lv_image_set_src(tiles[i].obj, &tiles[i].dsc);
                lv_obj_invalidate(tiles[i].obj);
                if (++refreshed_now >= max_refreshes_per_tick) break;
            }
        }

        if (lv_scr_act() != scr_mapa) {
            pinch_active = false;
            stop_kinetic_glide();
            return;
        }

        if (g_mt_fingers >= 2) {
            float dx = (float)(g_mt_x[1] - g_mt_x[0]);
            float dy = (float)(g_mt_y[1] - g_mt_y[0]);
            float dist = sqrtf(dx * dx + dy * dy);

            if (!pinch_active) {
                pinch_active = true;
                pinch_base_dist = dist;
                stop_kinetic_glide();
                touch_hist_count = 0;
            } else {
                float delta_d = dist - pinch_base_dist;
                if (delta_d >= 44.0f) {
                    ejecutar_zoom(1);
                    disparar_haptico(1);
                    pinch_base_dist = dist;
                } else if (delta_d <= -44.0f) {
                    ejecutar_zoom(-1);
                    disparar_haptico(1);
                    pinch_base_dist = dist;
                }
            }
            pinch_cooldown_until = millis() + 80;
            return;
        } else if (pinch_active) {
            pinch_active = false;
            pinch_cooldown_until = millis() + 80;
            return;
        }

        if (glide_active) {
            uint32_t now_ms = millis();
            uint32_t dt_ms = now_ms - glide_last_ms;
            if (dt_ms == 0) return;
            if (dt_ms > 45) dt_ms = 45;
            glide_last_ms = now_ms;

            float dt = (float)dt_ms * 0.001f;
            glide_rem_x += glide_vx * dt;
            glide_rem_y += glide_vy * dt;

            int16_t step_x = (int16_t)glide_rem_x;
            int16_t step_y = (int16_t)glide_rem_y;
            glide_rem_x -= (float)step_x;
            glide_rem_y -= (float)step_y;

            float decay = expf(-3.8f * dt);
            glide_vx *= decay;
            glide_vy *= decay;

            if (step_x != 0 || step_y != 0) {
                update_map_topology(step_x, step_y);
            }

            if ((glide_vx * glide_vx + glide_vy * glide_vy) < 225.0f) {
                stop_kinetic_glide();
                vector_x = 0;
                vector_y = 0;
                if (ghost_miner_handle != NULL) xTaskNotifyGive(ghost_miner_handle);
            }
        }
    }, 8, NULL);

    punto_gps = lv_obj_create(tile_container);
    lv_obj_set_size(punto_gps, 16, 16);
    lv_obj_set_style_bg_color(punto_gps, lv_color_hex(0x007AFF), LV_PART_MAIN);
    lv_obj_set_style_radius(punto_gps, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_color(punto_gps, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(punto_gps, 3, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(punto_gps, lv_color_hex(0x007AFF), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(punto_gps, 10, LV_PART_MAIN);
    lv_obj_add_flag(punto_gps, LV_OBJ_FLAG_HIDDEN); 
    lv_obj_remove_flag(punto_gps, LV_OBJ_FLAG_CLICKABLE);

    uint32_t init_tac_col = get_active_tactical_color();

    punto_rojo = lv_obj_create(scr_mapa);
    lv_obj_set_size(punto_rojo, 12, 12);
    lv_obj_set_style_bg_color(punto_rojo, lv_color_hex(init_tac_col), LV_PART_MAIN); 
    lv_obj_set_style_radius(punto_rojo, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(punto_rojo, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(punto_rojo, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(punto_rojo, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(punto_rojo, LV_OBJ_FLAG_CLICKABLE);

    lbl_wp_manual_inst = lv_label_create(scr_mapa);
    lv_label_set_text(lbl_wp_manual_inst, "SELECCIONE UBICACION");
    lv_obj_set_style_text_color(lbl_wp_manual_inst, lv_color_hex(init_tac_col), LV_PART_MAIN);
    lv_obj_set_style_bg_color(lbl_wp_manual_inst, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lbl_wp_manual_inst, 200, LV_PART_MAIN);
    lv_obj_align(lbl_wp_manual_inst, LV_ALIGN_TOP_MID, 0, 15);
    lv_obj_add_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);

    btn_gps = lv_btn_create(scr_mapa);
    lv_obj_set_size(btn_gps, 50, 50); 
    lv_obj_align(btn_gps, LV_ALIGN_BOTTOM_LEFT, 25, -35); 
    lv_obj_set_style_bg_color(btn_gps, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_gps, 200, LV_PART_MAIN);
    lv_obj_set_style_radius(btn_gps, 25, LV_PART_MAIN);
    lv_obj_add_flag(btn_gps, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t * lbl_gps_text = lv_label_create(btn_gps);
    lv_label_set_text(lbl_gps_text, "GPS");
    lv_obj_center(lbl_gps_text);
    
    lv_obj_add_event_cb(btn_gps, [](lv_event_t * e){
        if(lv_event_get_code(e) == LV_EVENT_RELEASED) {
            hardware_back_router();
        }
    }, LV_EVENT_ALL, NULL);

    btn_open_wp_menu = lv_btn_create(scr_mapa);
    lv_obj_set_size(btn_open_wp_menu, 50, 50);
    lv_obj_align(btn_open_wp_menu, LV_ALIGN_BOTTOM_LEFT, 25, -100);
    lv_obj_set_style_bg_color(btn_open_wp_menu, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_open_wp_menu, 25, LV_PART_MAIN);
    lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t * lbl_wp_btn = lv_label_create(btn_open_wp_menu);
    lv_label_set_text(lbl_wp_btn, LV_SYMBOL_LIST);
    lv_obj_center(lbl_wp_btn);

    panel_wp_menu = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_wp_menu, 300, 410); 
    lv_obj_center(panel_wp_menu);
    lv_obj_set_style_bg_color(panel_wp_menu, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel_wp_menu, 185, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_wp_menu, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_wp_menu, 2, LV_PART_MAIN);
    lv_obj_add_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN);

    const char* options[7] = {
        "INYECCION MANUAL",
        "COORDENADAS GPS",
        "CARGA DESDE SD",
        "ENLACE BLUETOOTH",
        "MIS WAYPOINTS",
        "HALLAR RUTA",
        TACTICAL_COLOR_NAMES[tactical_color_idx % 6]
    };

    for(int i=0; i<7; i++) {
        lv_obj_t * btn = lv_btn_create(panel_wp_menu);
        lv_obj_set_size(btn, 260, 44);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 6 + (i * 51));
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x333333), LV_PART_MAIN);
        
        lv_obj_t * lbl = lv_label_create(btn);
        lv_label_set_text(lbl, options[i]);
        lv_obj_center(lbl);

        if(i == 5) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0xFFB300), LV_PART_MAIN);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), LV_PART_MAIN);
            btn_menu_ruta_ptr = btn;
            lbl_menu_ruta_ptr = lbl;
        } else if (i == 6) {
            btn_menu_color_ptr = btn;
            lbl_menu_color_ptr = lbl;
            aplicar_color_tactico_ui();
        }

        lv_obj_add_event_cb(btn, [](lv_event_t * e) {
            int idx = (int)(uintptr_t)lv_event_get_user_data(e);

            if (idx == 6) {
                tactical_color_idx = (tactical_color_idx + 1) % 6;
                wp_prefs.putUChar("tac_col", tactical_color_idx);
                aplicar_color_tactico_ui();
                disparar_haptico(1);
                mostrar_toast(TACTICAL_COLOR_NAMES[tactical_color_idx]);
                return;
            }
            
            if (idx == 5 && active_route_len > 0) {
                liberar_buffers_ruta();
                if (route_line != NULL) lv_obj_add_flag(route_line, LV_OBJ_FLAG_HIDDEN); 
                
                lv_label_set_text(lbl_menu_ruta_ptr, "HALLAR RUTA");
                lv_obj_set_style_bg_color(btn_menu_ruta_ptr, lv_color_hex(0xFFB300), LV_PART_MAIN);
                lv_obj_set_style_text_color(lbl_menu_ruta_ptr, lv_color_hex(0x000000), LV_PART_MAIN);
                
                mostrar_toast("RUTA ELIMINADA");
                lv_obj_add_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                disparar_haptico(47);
                return;
            }

            lv_obj_add_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN);
            
            if (idx == 0) { 
                modo_inyeccion_tactil = true;
                lv_label_set_text(lbl_wp_manual_inst, "SELECCIONE UBICACION");
                lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                
                lv_anim_init(&anim_blinker);
                lv_anim_set_var(&anim_blinker, punto_rojo);
                lv_anim_set_values(&anim_blinker, 255, 0);
                lv_anim_set_time(&anim_blinker, 250);
                lv_anim_set_playback_time(&anim_blinker, 250);
                lv_anim_set_repeat_count(&anim_blinker, LV_ANIM_REPEAT_INFINITE);
                lv_anim_set_exec_cb(&anim_blinker, anim_blink_cb);
                lv_anim_start(&anim_blinker);
            }
            else if (idx == 1) lv_scr_load(scr_wp_coords);
            else if (idx == 2) lv_scr_load(scr_wp_sd);
            else if (idx == 3) lv_scr_load(scr_wp_ble);
            else if (idx == 4) { seleccionando_ruta_db = false; refrescar_lista_waypoints(); lv_scr_load(scr_wp_list); }
            else if (idx == 5) { lv_obj_clear_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN); } 
        }, LV_EVENT_RELEASED, (void*)(uintptr_t)i);
    }

    lv_obj_add_event_cb(btn_open_wp_menu, [](lv_event_t * e){
        lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(panel_wp_menu, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_RELEASED, NULL);

    panel_route_origen = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_route_origen, 300, 160);
    lv_obj_center(panel_route_origen);
    lv_obj_set_style_bg_color(panel_route_origen, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel_route_origen, 200, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_route_origen, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_route_origen, 2, LV_PART_MAIN);
    lv_obj_add_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN);

    btn_origen_gnss = lv_btn_create(panel_route_origen);
    lv_obj_set_size(btn_origen_gnss, 260, 50);
    lv_obj_align(btn_origen_gnss, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(btn_origen_gnss, lv_color_hex(0x222222), LV_PART_MAIN);
    lbl_origen_gnss = lv_label_create(btn_origen_gnss);
    lv_label_set_text(lbl_origen_gnss, "DESDE MI UBICACION");
    lv_obj_center(lbl_origen_gnss);
    lv_obj_add_event_cb(btn_origen_gnss, [](lv_event_t * e){
        if(gps_locked && current_lat != 0.0) {
            start_route_lat = current_lat;
            start_route_lng = current_lng;
            lv_obj_add_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN);
            
            modo_seleccion_ruta_tactil = true;
            lv_label_set_text(lbl_wp_manual_inst, "SELECCIONE DESTINO");
            lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
            
            lv_anim_init(&anim_blinker);
            lv_anim_set_var(&anim_blinker, punto_rojo);
            lv_anim_set_values(&anim_blinker, 255, 0);
            lv_anim_set_time(&anim_blinker, 250);
            lv_anim_set_playback_time(&anim_blinker, 250);
            lv_anim_set_repeat_count(&anim_blinker, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_exec_cb(&anim_blinker, anim_blink_cb);
            lv_anim_start(&anim_blinker);
        } else {
            mostrar_toast("GNSS NO DISPONIBLE");
            disparar_haptico(47);
        }
    }, LV_EVENT_RELEASED, NULL);

    lv_obj_t * btn_origen_manual = lv_btn_create(panel_route_origen);
    lv_obj_set_size(btn_origen_manual, 260, 50);
    lv_obj_align(btn_origen_manual, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_color(btn_origen_manual, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_o_man = lv_label_create(btn_origen_manual);
    lv_label_set_text(lbl_o_man, "PUNTO A PUNTO (MANUAL)");
    lv_obj_center(lbl_o_man);
    lv_obj_add_event_cb(btn_origen_manual, [](lv_event_t * e){
        lv_obj_add_flag(panel_route_origen, LV_OBJ_FLAG_HIDDEN);
        modo_seleccion_origen_tactil = true;
        lv_label_set_text(lbl_wp_manual_inst, "SELECCIONE PUNTO A (ORIGEN)");
        lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
        
        lv_anim_init(&anim_blinker);
        lv_anim_set_var(&anim_blinker, punto_rojo);
        lv_anim_set_values(&anim_blinker, 255, 0);
        lv_anim_set_time(&anim_blinker, 250);
        lv_anim_set_playback_time(&anim_blinker, 250);
        lv_anim_set_repeat_count(&anim_blinker, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_exec_cb(&anim_blinker, anim_blink_cb);
        lv_anim_start(&anim_blinker);
    }, LV_EVENT_RELEASED, NULL);

    panel_route_menu = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_route_menu, 300, 160);
    lv_obj_center(panel_route_menu);
    lv_obj_set_style_bg_color(panel_route_menu, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel_route_menu, 200, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_route_menu, lv_color_hex(0xFFB300), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_route_menu, 2, LV_PART_MAIN);
    lv_obj_add_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * btn_route_wp = lv_btn_create(panel_route_menu);
    lv_obj_set_size(btn_route_wp, 260, 50);
    lv_obj_align(btn_route_wp, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(btn_route_wp, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_r_wp = lv_label_create(btn_route_wp);
    lv_label_set_text(lbl_r_wp, "DESTINO GUARDADO"); 
    lv_obj_center(lbl_r_wp);
    lv_obj_add_event_cb(btn_route_wp, [](lv_event_t * e){
        lv_obj_add_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN);
        seleccionando_ruta_db = true;
        refrescar_lista_waypoints();
        lv_scr_load(scr_wp_list);
    }, LV_EVENT_RELEASED, NULL);

    lv_obj_t * btn_route_tmp = lv_btn_create(panel_route_menu);
    lv_obj_set_size(btn_route_tmp, 260, 50);
    lv_obj_align(btn_route_tmp, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_color(btn_route_tmp, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_r_tmp = lv_label_create(btn_route_tmp);
    lv_label_set_text(lbl_r_tmp, "DESTINO EN MAPA"); 
    lv_obj_center(lbl_r_tmp);
    lv_obj_add_event_cb(btn_route_tmp, [](lv_event_t * e){
        lv_obj_add_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN);
        modo_seleccion_ruta_tactil = true;
        lv_label_set_text(lbl_wp_manual_inst, "SELECCIONE DESTINO");
        lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
        
        lv_anim_init(&anim_blinker);
        lv_anim_set_var(&anim_blinker, punto_rojo);
        lv_anim_set_values(&anim_blinker, 255, 0);
        lv_anim_set_time(&anim_blinker, 250);
        lv_anim_set_playback_time(&anim_blinker, 250);
        lv_anim_set_repeat_count(&anim_blinker, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_exec_cb(&anim_blinker, anim_blink_cb);
        lv_anim_start(&anim_blinker);
    }, LV_EVENT_RELEASED, NULL);

    panel_osd_info = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_osd_info, 360, 400);
    lv_obj_center(panel_osd_info);
    lv_obj_set_style_bg_color(panel_osd_info, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel_osd_info, 240, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_osd_info, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_osd_info, 2, LV_PART_MAIN);
    lv_obj_add_flag(panel_osd_info, LV_OBJ_FLAG_HIDDEN);

    lbl_osd_info = lv_label_create(panel_osd_info);
    lv_obj_set_width(lbl_osd_info, 320);
    lv_label_set_long_mode(lbl_osd_info, LV_LABEL_LONG_WRAP);
    lv_label_set_text(lbl_osd_info, 
        "DICCIONARIO ESPACIAL\n\n"
        "AZUL: GPS interno del reloj.\n"
        "AMARILLO: GPS recibido del telefono.\n"
        "MAGENTA: Fusion Dual (Reloj + Telefono).\n\n"
        "SATS EN VISTA: Menos de 4 = Perdido. 4 a 7 = Preciso. 8+ = Exactitud militar.\n\n"
        "TIMEOUT: Si pasan 30s sin datos nuevos, el enlace se declara CIEGO."
    );
    lv_obj_set_style_text_font(lbl_osd_info, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_osd_info, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_align(lbl_osd_info, LV_ALIGN_TOP_MID, 0, 10);

    panel_osd = lv_obj_create(scr_mapa);
    lv_obj_set_size(panel_osd, 360, 420);
    lv_obj_center(panel_osd);
    lv_obj_set_style_bg_color(panel_osd, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel_osd, 230, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_osd, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_osd, 2, LV_PART_MAIN);
    lv_obj_add_flag(panel_osd, LV_OBJ_FLAG_HIDDEN);

    lbl_osd = lv_label_create(panel_osd);
    lv_obj_set_width(lbl_osd, 300);
    lv_label_set_long_mode(lbl_osd, LV_LABEL_LONG_WRAP);
    lv_label_set_text(lbl_osd, "INICIANDO HARDWARE ESPACIAL...");
    lv_obj_set_style_text_font(lbl_osd, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(lbl_osd, LV_ALIGN_TOP_LEFT, 0, 10);

    lv_obj_t * btn_info = lv_btn_create(panel_osd);
    lv_obj_set_size(btn_info, 40, 40);
    lv_obj_align(btn_info, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_style_bg_color(btn_info, lv_color_hex(0x00A3FF), LV_PART_MAIN);
    lv_obj_t * lbl_info_icon = lv_label_create(btn_info);
    lv_label_set_text(lbl_info_icon, "?"); 
    lv_obj_center(lbl_info_icon);
    lv_obj_add_event_cb(btn_info, [](lv_event_t * e){
        lv_obj_clear_flag(panel_osd_info, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(panel_osd_info);
        disparar_haptico(1);
    }, LV_EVENT_RELEASED, NULL);
    
    lv_obj_t * btn_glo = lv_btn_create(panel_osd);
    lv_obj_set_size(btn_glo, 320, 60);
    lv_obj_align(btn_glo, LV_ALIGN_BOTTOM_MID, 0, -80);
    lv_obj_set_style_bg_color(btn_glo, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_glo = lv_label_create(btn_glo);
    lv_label_set_text(lbl_glo, "GPS + GALILEO + GLONASS\n(Recomendado: America, Europa, Africa)");
    lv_obj_center(lbl_glo);
    lv_obj_add_event_cb(btn_glo, [](lv_event_t * e){
        configurar_gnss(1);
    }, LV_EVENT_RELEASED, NULL);

    lv_obj_t * btn_bds = lv_btn_create(panel_osd);
    lv_obj_set_size(btn_bds, 320, 60);
    lv_obj_align(btn_bds, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(btn_bds, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_t * lbl_bds = lv_label_create(btn_bds);
    lv_label_set_text(lbl_bds, "GPS + GALILEO + BEIDOU\n(Recomendado: Asia, Oceania)");
    lv_obj_center(lbl_bds);
    lv_obj_add_event_cb(btn_bds, [](lv_event_t * e){
        configurar_gnss(2);
    }, LV_EVENT_RELEASED, NULL);

    lv_timer_create([](lv_timer_t * t) {
        if (!osd_visible) return;

        double hdop = gps.hdop.hdop();

        if(btn_origen_gnss != NULL && lbl_origen_gnss != NULL) {
            if(!gps_locked || current_lat == 0.0) {
                lv_obj_set_style_bg_color(btn_origen_gnss, lv_color_hex(0x222222), LV_PART_MAIN);
                lv_obj_set_style_text_color(lbl_origen_gnss, lv_color_hex(0x555555), LV_PART_MAIN);
            } else {
                lv_obj_set_style_bg_color(btn_origen_gnss, lv_color_hex(0x00A3FF), LV_PART_MAIN);
                lv_obj_set_style_text_color(lbl_origen_gnss, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            }
        }

        const char* estado_str = "CIEGO (Sin senal >30s)";
        uint32_t osd_col = 0xFFB300;
        if (gnss_active_source == 1) {
            estado_str = "RELOJ (3D AZUL)";
            osd_col = 0x007AFF;
        } else if (gnss_active_source == 2) {
            estado_str = "TELEFONO (AMARILLO)";
            osd_col = 0xFFD600;
        } else if (gnss_active_source == 3) {
            estado_str = "FUSION DUAL (MAGENTA)";
            osd_col = 0xFF00EA;
        }

        lv_obj_set_style_text_color(lbl_osd, lv_color_hex(osd_col), LV_PART_MAIN);
        lv_obj_set_style_border_color(panel_osd, lv_color_hex(osd_col), LV_PART_MAIN);

        uint32_t age_ms = 99999;
        if (gnss_active_source == 2 && remote_gnss_last_ms > 0) {
            age_ms = millis() - remote_gnss_last_ms;
        } else if (gps.location.isValid()) {
            age_ms = gps.location.age();
        }

        lv_label_set_text_fmt(lbl_osd, 
            "[ ESTADO DEL ENLACE ESPACIAL ]\n\n"
            "Fuente: %s\n"
            "Constelacion: %s\n"
            "Sats Reloj: %d | Pulsos Tel: %u\n"
            "HDOP: %.1f %s\n"
            "Refresco: %u ms\n\n"
            "LAT: %.6f\n"
            "LON: %.6f\n"
            "ALT: %.1f m\n"
            "Velocidad: %.1f km/h",
            estado_str,
            active_constellation,
            sats_in_view, (unsigned int)remote_gnss_rx_count,
            hdop, (hdop > 0.0 && hdop < 2.0) ? "(Ideal)" : "(Normal)",
            (unsigned int)age_ms,
            gps_locked ? current_lat : 0.0,
            gps_locked ? current_lng : 0.0,
            gps_locked ? fused_alt_m : 0.0,
            gps_locked ? fused_spd_kmh : 0.0
        );
    }, 1000, NULL);

    static uint8_t global_tap_count = 0;
    static uint32_t global_last_tap = 0;
    static double global_tap_lat = 0.0;
    static double global_tap_lng = 0.0;

    lv_timer_create([](lv_timer_t * t) {
        if (global_tap_count > 0 && (millis() - global_last_tap > 310)) { 
            if (global_tap_count >= 2) {
                if (modo_inyeccion_tactil || modo_seleccion_ruta_tactil || modo_seleccion_origen_tactil) {
                    if (modo_inyeccion_tactil) {
                        guardar_waypoint(global_tap_lat, global_tap_lng, 0);
                        disparar_haptico(1);
                        modo_inyeccion_tactil = false;
                        mostrar_toast("WAYPOINT GUARDADO");
                    } else if (modo_seleccion_origen_tactil) {
                        start_route_lat = global_tap_lat;
                        start_route_lng = global_tap_lng;
                        mostrar_toast("ORIGEN FIJADO");
                        disparar_haptico(1);
                        modo_seleccion_origen_tactil = false;
                        lv_obj_clear_flag(panel_route_menu, LV_OBJ_FLAG_HIDDEN); 
                    } else if (modo_seleccion_ruta_tactil) {
                        target_route_lat = global_tap_lat;
                        target_route_lng = global_tap_lng;
                        mostrar_toast("PREPARANDO RUTA...");
                        disparar_haptico(1);
                        modo_seleccion_ruta_tactil = false;
                        flag_calc_route = true; 
                    }
                    
                    lv_anim_del(punto_rojo, NULL);
                    lv_obj_set_style_bg_opa(punto_rojo, 255, LV_PART_MAIN);
                    lv_obj_add_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
                    
                    if (modo_exploracion) {
                        lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
                        lv_obj_clear_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                    } else {
                        lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
                        lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                    }
                } 
                else {
                    modo_exploracion = !modo_exploracion;
                    disparar_haptico(1); 
                    
                    if (modo_exploracion) {
                        seguimiento_gnss_activo = false;
                        aplicar_color_tactico_ui();
                        lv_obj_clear_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN); 
                        lv_obj_clear_flag(btn_gps, LV_OBJ_FLAG_HIDDEN);    
                        lv_obj_clear_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                        mostrar_toast("MODO EXPLORACION");
                    } else {
                        lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);   
                        lv_obj_add_flag(btn_gps, LV_OBJ_FLAG_HIDDEN);    
                        lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                        mostrar_toast("MODO NAVEGACION");
                    }
                }
            }
            global_tap_count = 0; 
        }
    }, 10, NULL);

    lv_obj_add_event_cb(scr_mapa, [](lv_event_t * e){
        static lv_point_t start_point = {0, 0};
        static lv_point_t prev_point = {0, 0};
        static uint32_t press_start = 0;
        
        static bool trig_2s = false;
        static bool trig_5s = false;
        static bool dragged = false;
        static bool was_in_pinch_cooldown = false;
        
        lv_indev_t * indev = lv_indev_get_act();
        if(indev == NULL) return;
        
        lv_event_code_t code = lv_event_get_code(e);

        if (code == LV_EVENT_PRESSED) {
            stop_kinetic_glide();
            uint32_t now_ms = millis();
            last_pan_Input_ms = now_ms;
            lv_point_t pt;
            lv_indev_get_point(indev, &pt);

            start_point = pt;
            prev_point = pt;
            drag_smooth_x = 0.0f;
            drag_smooth_y = 0.0f;
            vector_x = 0;
            vector_y = 0;
            
            press_start = now_ms;
            touch_hist[0] = {(int16_t)start_point.x, (int16_t)start_point.y, press_start};
            touch_hist_count = 1;
            
            trig_2s = false;
            trig_5s = false; 
            dragged = false;
            was_in_pinch_cooldown = false;
        }
        else if (code == LV_EVENT_PRESSING) {
            uint32_t now_ms = millis();
            if (pinch_active || g_mt_fingers >= 2 || now_ms < pinch_cooldown_until) {
                lv_indev_get_point(indev, &prev_point);
                start_point = prev_point;
                touch_hist_count = 0;
                was_in_pinch_cooldown = true;
                return;
            }
            if (was_in_pinch_cooldown) {
                was_in_pinch_cooldown = false;
                lv_indev_get_point(indev, &prev_point);
                start_point = prev_point;
                drag_smooth_x = 0.0f;
                drag_smooth_y = 0.0f;
                touch_hist[0] = {(int16_t)prev_point.x, (int16_t)prev_point.y, now_ms};
                touch_hist_count = 1;
            }

            uint32_t hold_time = now_ms - press_start;

            lv_point_t point;
            lv_indev_get_point(indev, &point);
            if (point.x == 0 && point.y == 0) return;

            int32_t offset_x = point.x - start_point.x;
            int32_t offset_y = point.y - start_point.y;
            int32_t dist_sq = (offset_x * offset_x) + (offset_y * offset_y);

            if (!dragged) {
                if (dist_sq > (9 * 9)) {
                    dragged = true;
                    seguimiento_gnss_activo = false; 
                    prev_point = point;
                } else {
                    if (hold_time >= 2000 && !trig_2s && !modo_exploracion) {
                        trig_2s = true;

                        if (modo_inyeccion_tactil || modo_seleccion_ruta_tactil || modo_seleccion_origen_tactil) {
                            modo_inyeccion_tactil = false; modo_seleccion_ruta_tactil = false; modo_seleccion_origen_tactil = false;
                            lv_anim_del(punto_rojo, NULL);
                            lv_obj_set_style_bg_opa(punto_rojo, 255, LV_PART_MAIN);
                            lv_obj_add_flag(lbl_wp_manual_inst, LV_OBJ_FLAG_HIDDEN);
                            lv_obj_add_flag(punto_rojo, LV_OBJ_FLAG_HIDDEN);
                            lv_obj_add_flag(btn_open_wp_menu, LV_OBJ_FLAG_HIDDEN);
                        }

                        stop_kinetic_glide();
                        touch_hist_count = 0; vector_x = 0; vector_y = 0;
                        seguimiento_gnss_activo = true;

                        if (gps_locked && current_lat != 0.0 && current_lng != 0.0) {
                            disparar_haptico(1);
                            anclar_a_gnss();
                            mostrar_toast("UBICACION Y SEGUIMIENTO ACTIVOS");
                        } else {
                            disparar_haptico(47);
                            mostrar_toast("SIN SENAL GNSS ACTIVA");
                        }
                    }

                    if (hold_time >= 5000 && !trig_5s && !modo_exploracion) {
                        trig_5s = true;
                        disparar_haptico(47); 
                        osd_visible = !osd_visible;
                        if (osd_visible) {
                            lv_obj_clear_flag(panel_osd, LV_OBJ_FLAG_HIDDEN);
                            lv_obj_move_foreground(panel_osd);
                        } else {
                            lv_obj_add_flag(panel_osd, LV_OBJ_FLAG_HIDDEN);
                            lv_obj_add_flag(panel_osd_info, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    return;
                }
            }

            int16_t raw_dx = point.x - prev_point.x;
            int16_t raw_dy = point.y - prev_point.y;
            
            if (abs(raw_dx) > 250 || abs(raw_dy) > 250) {
                prev_point = point;
                touch_hist_count = 0;
                return;
            }

            if (raw_dx != 0 || raw_dy != 0) {
                prev_point = point;

                if (touch_hist_count < 8) {
                    touch_hist[touch_hist_count++] = {(int16_t)point.x, (int16_t)point.y, now_ms};
                } else {
                    for (int k = 0; k < 7; k++) touch_hist[k] = touch_hist[k + 1];
                    touch_hist[7] = {(int16_t)point.x, (int16_t)point.y, now_ms};
                }

                drag_smooth_x += (float)raw_dx;
                drag_smooth_y += (float)raw_dy;
                int16_t step_x = (int16_t)(drag_smooth_x * 0.90f);
                int16_t step_y = (int16_t)(drag_smooth_y * 0.90f);
                if (step_x == 0 && fabsf(drag_smooth_x) >= 1.0f) step_x = (int16_t)drag_smooth_x;
                if (step_y == 0 && fabsf(drag_smooth_y) >= 1.0f) step_y = (int16_t)drag_smooth_y;
                drag_smooth_x -= (float)step_x;
                drag_smooth_y -= (float)step_y;

                if (step_x != 0 || step_y != 0) {
                    update_map_topology(step_x, step_y);
                }
            }
        }
        else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) { 
            uint32_t now_ms = millis();

            if (pinch_active || now_ms < pinch_cooldown_until) {
                global_tap_count = 0;
                touch_hist_count = 0;
                return;
            }

            int16_t rem_dx = (int16_t)roundf(drag_smooth_x);
            int16_t rem_dy = (int16_t)roundf(drag_smooth_y);
            drag_smooth_x = 0.0f;
            drag_smooth_y = 0.0f;
            if (rem_dx != 0 || rem_dy != 0) {
                update_map_topology(rem_dx, rem_dy);
            }

            uint32_t hold_time = now_ms - press_start;
            
            if (trig_2s || trig_5s || hold_time > 800) {
                global_tap_count = 0;
                touch_hist_count = 0;
                return;
            }
            
            if (hold_time < 300 && !dragged) {
                calcular_coordenadas_tactiles(&global_tap_lat, &global_tap_lng);
                global_tap_count++;
                global_last_tap = now_ms;
                touch_hist_count = 0;
                return;
            }

            float vx_px_s = 0.0f;
            float vy_px_s = 0.0f;

            if (touch_hist_count >= 2) {
                const TouchHistorySample &newest = touch_hist[touch_hist_count - 1];
                if (now_ms - newest.t_ms <= 75) {
                    int anchor_idx = 0;
                    for (int k = (int)touch_hist_count - 2; k >= 0; k--) {
                        uint32_t span = newest.t_ms - touch_hist[k].t_ms;
                        anchor_idx = k;
                        if (span >= 35 && span <= 105) break;
                    }
                    uint32_t dt_ms = newest.t_ms - touch_hist[anchor_idx].t_ms;
                    if (dt_ms >= 6 && dt_ms <= 160) {
                        vx_px_s = ((float)(newest.x - touch_hist[anchor_idx].x) * 1000.0f) / (float)dt_ms;
                        vy_px_s = ((float)(newest.y - touch_hist[anchor_idx].y) * 1000.0f) / (float)dt_ms;
                    }
                }
            }
            touch_hist_count = 0;

            float speed_px_s = sqrtf(vx_px_s * vx_px_s + vy_px_s * vy_px_s);
            if (speed_px_s > 65.0f) {
                if (speed_px_s > 3400.0f) {
                    float scale = 3400.0f / speed_px_s;
                    vx_px_s *= scale;
                    vy_px_s *= scale;
                }
                glide_vx = vx_px_s;
                glide_vy = vy_px_s;
                glide_rem_x = 0.0f;
                glide_rem_y = 0.0f;
                glide_last_ms = now_ms;
                glide_active = true;
            } else {
                stop_kinetic_glide();
                vector_x = 0;
                vector_y = 0;
                if (ghost_miner_handle != NULL) xTaskNotifyGive(ghost_miner_handle);
            }
        }
    }, LV_EVENT_ALL, NULL);

    build_wp_screens();
    lv_screen_load(scr_mapa);
}

// ---------------------------------------------------------------------
// 13. ESCÁNER EN 2.º PLANO Y GHOSTMINER LOCK-FREE CON ZERO-COPY SWAP
// ---------------------------------------------------------------------
void scan_sd_zoom_bounds() {
    if (!placeholder_sd_loaded) {
        reload_custom_placeholder();
        placeholder_sd_loaded = true;
    }

    uint8_t found_min = 255;
    uint8_t found_max = 0;
    uint32_t mask = 0;
    char z_path[32];
    char f_name[64];

    for (uint8_t z = 1; z <= 22; z++) {
        if (pausa_lectura_sd) return;
        if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
        snprintf(z_path, sizeof(z_path), "/MAP/Z%d", z);
        if (!sd.exists(z_path)) {
            if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
            continue;
        }

        FsFile z_dir = sd.open(z_path, O_READ);
        if (z_dir && z_dir.isDirectory()) {
            bool has_qoi = false;
            FsFile entry = z_dir.openNextFile();
            while (entry) {
                entry.getName(f_name, sizeof(f_name));
                if (!entry.isDirectory() && strncmp(f_name, "C_", 2) == 0 && strstr(f_name, ".QOI") != NULL) {
                    has_qoi = true;
                    entry.close();
                    break;
                }
                entry.close();
                entry = z_dir.openNextFile();
            }
            z_dir.close();

            if (has_qoi) {
                if (z < found_min) found_min = z;
                if (z > found_max) found_max = z;
                mask |= (1UL << z);
            }
        } else if (z_dir) {
            z_dir.close();
        }
        if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
    }

    if (found_min <= found_max && mask != 0) {
        min_sd_z = found_min;
        max_sd_z = found_max;
        valid_z_mask = mask;
        Serial.printf("[CARTO ENGINE] Zooms SD auditados en 2do plano -> Min: Z%d | Max: Z%d\n", min_sd_z, max_sd_z);
    }
    sd_zoom_bounds_scanned = true;
}

static bool open_cluster_persistent(uint8_t z, uint32_t cx, uint32_t cy, uint32_t &active_cx, uint32_t &active_cy, uint8_t &active_z, int &active_hdr_slot) {
    if (map_file.isOpen() && active_cx == cx && active_cy == cy && active_z == z) {
        if (active_hdr_slot < 0 || !hdr_cache[active_hdr_slot].valid ||
            hdr_cache[active_hdr_slot].cx != cx || hdr_cache[active_hdr_slot].cy != cy || hdr_cache[active_hdr_slot].z != z) {
            active_hdr_slot = search_header_cache(z, cx, cy);
        }
        return true;
    }
    if (map_file.isOpen()) map_file.close();

    char sd_path[64];
    snprintf(sd_path, sizeof(sd_path), "/MAP/Z%d/C_%u_%u.QOI", z, cx, cy);
    if (!map_file.open(sd_path, O_READ)) {
        active_cx = 0xFFFFFFFF;
        active_hdr_slot = -1;
        if (!sd.exists(sd_path)) {
            blacklist_cluster(z, cx, cy);
        }
        return false;
    }

    active_cx = cx;
    active_cy = cy;
    active_z = z;

    int slot = search_header_cache(z, cx, cy);
    if (slot == -1) {
        slot = hdr_cache_head;
        hdr_cache_head = (hdr_cache_head + 1) % MAX_HEADER_CACHE;
        hdr_cache[slot].valid = false;
        if (map_file.seek(0) && map_file.read(hdr_cache[slot].data, 8192) == 8192) {
            hdr_cache[slot].cx = cx;
            hdr_cache[slot].cy = cy;
            hdr_cache[slot].z = z;
            hdr_cache[slot].valid = true;
        } else {
            slot = -1;
        }
    }
    active_hdr_slot = slot;
    return true;
}

static inline bool fetch_pointer_8b(uint32_t req_lx, uint32_t req_ly, int active_hdr_slot, uint32_t &data_offset, uint32_t &data_length) {
    uint32_t pair_idx = ((req_lx % CLUSTER_SIZE) * CLUSTER_SIZE + (req_ly % CLUSTER_SIZE));
    if (active_hdr_slot >= 0 && hdr_cache[active_hdr_slot].valid) {
        data_offset = hdr_cache[active_hdr_slot].data[pair_idx * 2];
        data_length = hdr_cache[active_hdr_slot].data[pair_idx * 2 + 1];
        return true;
    } else {
        uint32_t hdr_pos = pair_idx * 8;
        uint32_t ptr_pair[2] = {0, 0};
        if (map_file.seek(hdr_pos) && map_file.read(ptr_pair, 8) == 8) {
            data_offset = ptr_pair[0];
            data_length = ptr_pair[1];
            return true;
        } else {
            data_offset = 0;
            data_length = 0;
            return false;
        }
    }
}

static inline bool any_active_tile_pending() {
    for (int j = 0; j < 16; j++) {
        if (tiles[j].needs_load) return true;
    }
    return false;
}

static inline void write_vault_slot_lockfree(uint8_t z, uint32_t lx, uint32_t ly, const uint8_t * src, uint32_t len) {
    int slot = l2_head;
    l2_head = (l2_head + 1) % MAX_QOI_TILES;
    l2_vault[slot].valid = false;
    __sync_synchronize();
    memcpy(l2_vault[slot].buffer, src, len);
    l2_vault[slot].lx = lx;
    l2_vault[slot].ly = ly;
    l2_vault[slot].z = z;
    l2_vault[slot].length = len;
    __sync_synchronize();
    l2_vault[slot].valid = true;
}

static bool prefetch_single_coord(uint8_t pf_z, uint32_t target_x, uint32_t target_y, uint32_t &active_cx, uint32_t &active_cy, uint8_t &active_z, int &active_hdr_slot) {
    if (pausa_lectura_sd) return false;
    if (search_vault_lockfree(pf_z, target_x, target_y) != -1) return false;
    if (is_tile_known_empty(pf_z, target_x, target_y)) return false;

    uint32_t cx = target_x / CLUSTER_SIZE;
    uint32_t cy = target_y / CLUSTER_SIZE;
    if (is_blacklisted(pf_z, cx, cy)) return false;

    bool ok = false;
    if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
    if (!pausa_lectura_sd && open_cluster_persistent(pf_z, cx, cy, active_cx, active_cy, active_z, active_hdr_slot)) {
        uint32_t data_offset = 0, data_length = 0;
        if (fetch_pointer_8b(target_x, target_y, active_hdr_slot, data_offset, data_length)) {
            if (data_offset < 8192 || data_length <= 20) {
                mark_tile_known_empty(pf_z, target_x, target_y);
            } else if (data_length <= QOI_CHUNK_SIZE) {
                if (map_file.seek(data_offset) && map_file.read(compressed_buffer, data_length) == data_length) {
                    write_vault_slot_lockfree(pf_z, target_x, target_y, compressed_buffer, data_length);
                    ok = true;
                }
            }
        }
    }
    if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
    return ok;
}

void raw_reader_task(void *pvParameters) {
    uint32_t active_cx = 0xFFFFFFFF;
    uint32_t active_cy = 0xFFFFFFFF;
    uint8_t active_z = 0xFF;
    int active_hdr_slot = -1;
    uint8_t streak_count = 0;

    static const int8_t ring_dx[16] = {
         0,  1, -1,  2,  -2, -2,  3,  3,   0,  1, -1,  2,  -2, -2,  3,  3
    };
    static const int8_t ring_dy[16] = {
        -2, -2, -2, -2,   0,  1,  0,  1,   3,  3,  3,  3,  -1,  2, -1,  2
    };

    while(1) {
        if (pausa_lectura_sd) {
            if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
            if (map_file.isOpen()) map_file.close();
            active_cx = 0xFFFFFFFF;
            active_cy = 0xFFFFFFFF;
            active_z = 0xFF;
            active_hdr_slot = -1;
            ghost_miner_parked = true;
            if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        ghost_miner_parked = false;

        if (!sd_zoom_bounds_scanned) {
            scan_sd_zoom_bounds();
        }

        int best_idx = -1;
        uint32_t min_score = 0xFFFFFFFF;
        uint32_t req_lx = 0, req_ly = 0;
        uint8_t req_z = current_z;
        int32_t snap_drag_x = drag_pixel_x;
        int32_t snap_drag_y = drag_pixel_y;
        int8_t snap_vx = vector_x;
        int8_t snap_vy = vector_y;

        int32_t target_scr_cx = 200 + (snap_vx * 96);
        int32_t target_scr_cy = 200 + (snap_vy * 96);

        portENTER_CRITICAL(&tile_spinlock);
        req_z = current_z;
        for(int j = 0; j < 16; j++) {
            if(tiles[j].needs_load) {
                int32_t sx0 = -307 + snap_drag_x + (tiles[j].px + 1) * 256;
                int32_t sy0 = -261 + snap_drag_y + (tiles[j].py + 1) * 256;
                bool is_visible = (sx0 + 256 > -128) && (sx0 < 528) && (sy0 + 256 > -128) && (sy0 < 528);

                int32_t d_cx = (sx0 + 128) - target_scr_cx;
                int32_t d_cy = (sy0 + 128) - target_scr_cy;
                uint32_t score = (uint32_t)(d_cx * d_cx + d_cy * d_cy);
                if (!is_visible) score += 10000000U;

                if (score < min_score) {
                    min_score = score;
                    best_idx = j;
                    req_lx = tiles[j].lx;
                    req_ly = tiles[j].ly;
                }
            }
        }
        portEXIT_CRITICAL(&tile_spinlock);

        if (best_idx != -1) {
            bool file_read_ok = false;
            bool tile_genuinely_empty = false;

            int vault_idx = search_vault_lockfree(req_z, req_lx, req_ly);
            if (vault_idx != -1) {
                uint32_t v_len = l2_vault[vault_idx].length;
                const uint8_t * v_ptr = l2_vault[vault_idx].buffer;
                if (l2_vault[vault_idx].valid && l2_vault[vault_idx].lx == req_lx && l2_vault[vault_idx].ly == req_ly && l2_vault[vault_idx].z == req_z) {
                    decode_qoi565_fast(v_ptr, v_len, shadow_buffer);
                    file_read_ok = true;
                }
            }

            if (!file_read_ok) {
                if (is_tile_known_empty(req_z, req_lx, req_ly)) {
                    tile_genuinely_empty = true;
                } else {
                    uint32_t cx = req_lx / CLUSTER_SIZE;
                    uint32_t cy = req_ly / CLUSTER_SIZE;

                    if (is_blacklisted(req_z, cx, cy)) {
                        tile_genuinely_empty = true;
                    } else if (!pausa_lectura_sd) {
                        if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
                        if (!pausa_lectura_sd) {
                            if (open_cluster_persistent(req_z, cx, cy, active_cx, active_cy, active_z, active_hdr_slot)) {
                                uint32_t data_offset = 0, data_length = 0;
                                if (fetch_pointer_8b(req_lx, req_ly, active_hdr_slot, data_offset, data_length)) {
                                    if (data_offset < 8192 || data_length <= 20) {
                                        mark_tile_known_empty(req_z, req_lx, req_ly);
                                        tile_genuinely_empty = true;
                                    } else if (data_length <= COMPRESSED_BUF_BYTES) {
                                        bool read_ok = (map_file.seek(data_offset) && map_file.read(compressed_buffer, data_length) == data_length);
                                        if (!read_ok) {
                                            map_file.close();
                                            active_cx = 0xFFFFFFFF;
                                            if (open_cluster_persistent(req_z, cx, cy, active_cx, active_cy, active_z, active_hdr_slot)) {
                                                read_ok = (map_file.seek(data_offset) && map_file.read(compressed_buffer, data_length) == data_length);
                                            }
                                        }
                                        if (read_ok) {
                                            if (data_length <= QOI_CHUNK_SIZE) {
                                                write_vault_slot_lockfree(req_z, req_lx, req_ly, compressed_buffer, data_length);
                                            }
                                            decode_qoi565_fast(compressed_buffer, data_length, shadow_buffer);
                                            file_read_ok = true;
                                        }
                                    } else {
                                        tile_genuinely_empty = true;
                                    }
                                }
                            } else if (is_blacklisted(req_z, cx, cy)) {
                                tile_genuinely_empty = true;
                            }
                        }
                        if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);
                    }
                }
            }

            portENTER_CRITICAL(&tile_spinlock);
            if (tiles[best_idx].lx == req_lx && tiles[best_idx].ly == req_ly && current_z == req_z) {
                if (file_read_ok) {
                    uint8_t * old_buf = tiles[best_idx].buffer;
                    tiles[best_idx].buffer = shadow_buffer;
                    shadow_buffer = old_buf;
                    tiles[best_idx].dsc.data = tiles[best_idx].buffer;
                    tiles[best_idx].needs_load = false;
                    tiles[best_idx].ready_to_refresh = true;
                } else if (tile_genuinely_empty) {
                    tiles[best_idx].dsc.data = static_placeholder;
                    tiles[best_idx].needs_load = false;
                    tiles[best_idx].ready_to_refresh = true;
                }
            }
            portEXIT_CRITICAL(&tile_spinlock);

            if (touch_down_on_map || glide_active || (millis() - last_pan_Input_ms) < 120) {
                streak_count = 0;
                vTaskDelay(pdMS_TO_TICKS(4));
            } else if (++streak_count >= 5) {
                streak_count = 0;
                vTaskDelay(1);
            }
            continue;
        }

        streak_count = 0;

        bool actively_panning = touch_down_on_map || glide_active || (millis() - last_pan_Input_ms) < 140;
        bool prefetched = false;

        if (!actively_panning) {
            uint8_t pf_z = current_z;
            uint32_t base_cx = current_x;
            uint32_t base_cy = current_y;
            for (int k = 0; k < 16 && !prefetched && !any_active_tile_pending(); k++) {
                uint32_t tx = (uint32_t)((int32_t)base_cx + ring_dx[k]);
                uint32_t ty = (uint32_t)((int32_t)base_cy + ring_dy[k]);
                prefetched = prefetch_single_coord(pf_z, tx, ty, active_cx, active_cy, active_z, active_hdr_slot);
            }
        }

        if (prefetched) {
            vTaskDelay(pdMS_TO_TICKS(4));
        } else {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15));
        }
    }
}

// ---------------------------------------------------------------------
// 15. NÚCLEO MATEMÁTICO A* CON PROYECCIÓN PERPENDICULAR A ARISTAS Y CULLING
// ---------------------------------------------------------------------
struct RouteEdge { uint32_t target; float weight; uint32_t next; };
struct AStarNode { uint32_t id; float f_score; };

float fast_distance(float lat1, float lon1, float lat2, float lon2) {
    float x = (lon2 - lon1) * cosf((lat1 + lat2) * 0.00872664626f);
    float y = (lat2 - lat1);
    return sqrtf(x * x + y * y) * 111319.9f;
}

static inline void latlon_to_norm_mercator(float lat, float lon, float &norm_x, float &norm_y) {
    norm_x = (lon + 180.0f) * (1.0f / 360.0f);
    float lat_rad = lat * 0.017453292519943295f;
    float s = sinf(lat_rad);
    if (s > 0.9999f) s = 0.9999f;
    if (s < -0.9999f) s = -0.9999f;
    norm_y = 0.5f - (0.25f * logf((1.0f + s) / (1.0f - s)) * 0.3183098861837907f);
}

void refrescar_linea_ruta() {
    if(active_route_len == 0 || route_line == NULL || route_pixels_pool == NULL || route_norm_pool == NULL) return;
    
    double scale_px = (double)(1UL << current_z) * 256.0;
    double global_container_x = (double)(current_x - 1) * 256.0;
    double global_container_y = (double)(current_y - 1) * 256.0;

    uint32_t out_idx = 0;
    int32_t last_px = -999999;
    int32_t last_py = -999999;
    bool prev_inside = false;
    int32_t prev_raw_px = 0;
    int32_t prev_raw_py = 0;

    for(uint32_t i = 0; i < active_route_len; i++) {
        int32_t px = (int32_t)round(((double)route_norm_pool[i].norm_x * scale_px) - global_container_x);
        int32_t py = (int32_t)round(((double)route_norm_pool[i].norm_y * scale_px) - global_container_y);

        if (px < -8192) px = -8192; else if (px > 9216) px = 9216;
        if (py < -8192) py = -8192; else if (py > 9216) py = 9216;

        bool inside = (px >= -384 && px <= 1408 && py >= -384 && py <= 1408);

        if (inside) {
            if (!prev_inside && i > 0) {
                route_pixels_pool[out_idx].x = (lv_coord_t)prev_raw_px;
                route_pixels_pool[out_idx].y = (lv_coord_t)prev_raw_py;
                last_px = prev_raw_px;
                last_py = prev_raw_py;
                out_idx++;
            }
            if (out_idx == 0 || i == active_route_len - 1 || (abs(px - last_px) + abs(py - last_py)) >= 3) {
                route_pixels_pool[out_idx].x = (lv_coord_t)px;
                route_pixels_pool[out_idx].y = (lv_coord_t)py;
                last_px = px;
                last_py = py;
                out_idx++;
            }
        } else if (prev_inside) {
            route_pixels_pool[out_idx].x = (lv_coord_t)px;
            route_pixels_pool[out_idx].y = (lv_coord_t)py;
            last_px = px;
            last_py = py;
            out_idx++;
        }

        prev_inside = inside;
        prev_raw_px = px;
        prev_raw_py = py;
    }

    if (out_idx < 2) {
        int32_t p0x = (int32_t)round(((double)route_norm_pool[0].norm_x * scale_px) - global_container_x);
        int32_t p0y = (int32_t)round(((double)route_norm_pool[0].norm_y * scale_px) - global_container_y);
        int32_t p1x = (int32_t)round(((double)route_norm_pool[active_route_len - 1].norm_x * scale_px) - global_container_x);
        int32_t p1y = (int32_t)round(((double)route_norm_pool[active_route_len - 1].norm_y * scale_px) - global_container_y);
        route_pixels_pool[0].x = (lv_coord_t)constrain(p0x, -4096, 5120);
        route_pixels_pool[0].y = (lv_coord_t)constrain(p0y, -4096, 5120);
        route_pixels_pool[1].x = (lv_coord_t)constrain(p1x, -4096, 5120);
        route_pixels_pool[1].y = (lv_coord_t)constrain(p1y, -4096, 5120);
        out_idx = 2;
    }

    lv_line_set_points(route_line, route_pixels_pool, out_idx);
}

static uint8_t last_astar_fail_reason = 0;

// Widgets de la barra de carga para el cálculo de rutas
static lv_obj_t * panel_route_prog = NULL;
static lv_obj_t * lbl_route_prog = NULL;
static lv_obj_t * bar_route_prog = NULL;

static void actualizar_progreso_ruta_ui(uint8_t pct, const char* etapa_txt) {
    if (scr_mapa == NULL) return;
    if (xSemaphoreTakeRecursive(lvgl_mutex, pdMS_TO_TICKS(60))) {
        uint32_t col = get_active_tactical_color();
        if (panel_route_prog == NULL) {
            panel_route_prog = lv_obj_create(scr_mapa);
            lv_obj_set_size(panel_route_prog, 300, 88);
            lv_obj_center(panel_route_prog);
            lv_obj_set_style_bg_color(panel_route_prog, lv_color_hex(0x081014), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(panel_route_prog, 240, LV_PART_MAIN);
            lv_obj_set_style_border_width(panel_route_prog, 2, LV_PART_MAIN);
            lv_obj_set_style_radius(panel_route_prog, 10, LV_PART_MAIN);
            lv_obj_clear_flag(panel_route_prog, LV_OBJ_FLAG_SCROLLABLE);

            lbl_route_prog = lv_label_create(panel_route_prog);
            lv_obj_set_style_text_font(lbl_route_prog, &lv_font_montserrat_14, LV_PART_MAIN);
            lv_obj_align(lbl_route_prog, LV_ALIGN_TOP_MID, 0, 6);

            bar_route_prog = lv_bar_create(panel_route_prog);
            lv_obj_set_size(bar_route_prog, 256, 14);
            lv_obj_align(bar_route_prog, LV_ALIGN_BOTTOM_MID, 0, -10);
            lv_bar_set_range(bar_route_prog, 0, 100);
            lv_obj_set_style_bg_color(bar_route_prog, lv_color_hex(0x18262C), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(bar_route_prog, 255, LV_PART_MAIN);
            lv_obj_set_style_radius(bar_route_prog, 6, LV_PART_MAIN);
            lv_obj_set_style_radius(bar_route_prog, 6, LV_PART_INDICATOR);
        }

        lv_obj_set_style_border_color(panel_route_prog, lv_color_hex(col), LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl_route_prog, lv_color_hex(col), LV_PART_MAIN);
        lv_obj_set_style_bg_color(bar_route_prog, lv_color_hex(col), LV_PART_INDICATOR);

        lv_label_set_text_fmt(lbl_route_prog, "%s (%u%%)", etapa_txt, (unsigned int)pct);
        lv_bar_set_value(bar_route_prog, pct, LV_ANIM_OFF);
        lv_obj_clear_flag(panel_route_prog, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(panel_route_prog);
        lv_refr_now(NULL);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

static void ocultar_progreso_ruta_ui() {
    if (panel_route_prog == NULL) return;
    if (xSemaphoreTakeRecursive(lvgl_mutex, pdMS_TO_TICKS(60))) {
        lv_obj_add_flag(panel_route_prog, LV_OBJ_FLAG_HIDDEN);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

static inline float project_point_onto_segment_m2(
    float plat, float plon,
    float alat, float alon,
    float blat, float blon,
    float lat_scale_m, float lon_scale_m,
    float &out_t, float &out_proj_lat, float &out_proj_lon
) {
    float vx = (blon - alon) * lon_scale_m;
    float vy = (blat - alat) * lat_scale_m;
    float wx = (plon - alon) * lon_scale_m;
    float wy = (plat - alat) * lat_scale_m;

    float len2 = vx * vx + vy * vy;
    float t = 0.0f;
    if (len2 > 1e-4f) {
        t = (wx * vx + wy * vy) / len2;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;
    }
    out_t = t;
    out_proj_lat = alat + t * (blat - alat);
    out_proj_lon = alon + t * (blon - alon);

    float dx = wx - t * vx;
    float dy = wy - t * vy;
    return dx * dx + dy * dy;
}

// ---------------------------------------------------------------------
// MOTOR A* MULTI-SECTOR CON EVASIÓN BIDIRECCIONAL DE ISLAS Y PROGRESO
// ---------------------------------------------------------------------
static bool run_astar_multi_hwy_files(
    const char (*file_list)[64],
    int file_count,
    float start_lat,
    float start_lng,
    float end_lat,
    float end_lng,
    float max_anchor_dist_m,
    bool filter_corridor_capsule,
    uint8_t prog_base,
    uint8_t prog_span
) {
    if (file_count <= 0) return false;

    const uint32_t MAX_NODES_CAP = 38000;
    const uint32_t MAX_EDGES_CAP = 75000;
    const uint32_t HASH_BUCKETS  = 32768;
    const uint32_t closed_bytes  = (MAX_NODES_CAP + 7U) >> 3;
    const uint32_t aligned_closed = (closed_bytes + 3U) & ~3U;

    uint32_t offset = 0;
    RouteNode* macro_nodes         = (RouteNode*)(global_arena + offset); offset += MAX_NODES_CAP * sizeof(RouteNode);
    uint32_t* macro_first_edge     = (uint32_t*)(global_arena + offset);  offset += MAX_NODES_CAP * sizeof(uint32_t);
    RouteEdge* macro_edges         = (RouteEdge*)(global_arena + offset); offset += MAX_EDGES_CAP * sizeof(RouteEdge);
    float* g_score                 = (float*)(global_arena + offset);     offset += MAX_NODES_CAP * sizeof(float);
    uint32_t* came_from            = (uint32_t*)(global_arena + offset);  offset += MAX_NODES_CAP * sizeof(uint32_t);
    uint8_t* closed_bits           = (uint8_t*)(global_arena + offset);   offset += aligned_closed;
    uint8_t* goal_bits             = (uint8_t*)(global_arena + offset);   offset += aligned_closed;
    uint8_t* excluded_start_bits   = (uint8_t*)(global_arena + offset);   offset += aligned_closed;
    uint8_t* excluded_end_bits     = (uint8_t*)(global_arena + offset);   offset += aligned_closed;
    uint8_t* io_buf                = (uint8_t*)(global_arena + offset);   offset += 8192;
    uint16_t* hash_heads           = (uint16_t*)(global_arena + offset);  offset += HASH_BUCKETS * sizeof(uint16_t);
    uint16_t* hash_next            = (uint16_t*)(global_arena + offset);  offset += MAX_NODES_CAP * sizeof(uint16_t);

    uint32_t* local_to_global = came_from;

    memset(macro_first_edge, 0xFF, MAX_NODES_CAP * sizeof(uint32_t));
    memset(hash_heads, 0xFF, HASH_BUCKETS * sizeof(uint16_t));
    memset(hash_next, 0xFF, MAX_NODES_CAP * sizeof(uint16_t));

    const float mid_lat_rad = ((start_lat + end_lat) * 0.5f) * 0.0174532925f;
    const float lon_scale_m = cosf(mid_lat_rad) * 111319.9f;
    const float lat_scale_m = 111319.9f;
    const float direct_dist_m = fast_distance(start_lat, start_lng, end_lat, end_lng);

    const float capsule_radius_m = max(2600.0f, direct_dist_m * 0.55f);
    const float capsule_r2 = capsule_radius_m * capsule_radius_m;

    const float pad_deg = max(0.035f, (capsule_radius_m / 111319.9f) * 1.25f);
    const float box_s = min(start_lat, end_lat) - pad_deg;
    const float box_n = max(start_lat, end_lat) + pad_deg;
    const float box_w = min(start_lng, end_lng) - pad_deg;
    const float box_e = max(start_lng, end_lng) + pad_deg;

    uint32_t total_nodes = 0;
    uint32_t edges_stored = 0;
    int files_loaded = 0;

    for (int f_idx = 0; f_idx < file_count; f_idx++) {
        const char* fname = file_list[f_idx];
        if (!sd.exists(fname)) continue;

        FsFile f = sd.open(fname, O_READ);
        if (!f) continue;

        uint32_t headers[2] = {0, 0};
        if (f.read(headers, 8) != 8) {
            f.close();
            continue;
        }
        uint32_t file_nodes = headers[0];
        uint32_t file_edges = headers[1];

        if (file_nodes == 0 || file_edges == 0 || file_nodes > MAX_NODES_CAP) {
            f.close();
            continue;
        }

        uint8_t load_pct = prog_base + (uint8_t)(((f_idx + 1) * (prog_span / 2)) / max(1, file_count));
        actualizar_progreso_ruta_ui(load_pct, (file_count > 1) ? "UNIENDO SECTORES" : "CARGANDO RED VIAL");

        bool apply_filter = filter_corridor_capsule || (file_nodes > (MAX_NODES_CAP - total_nodes - 500));

        uint32_t n_read = 0;
        while (n_read < file_nodes) {
            uint32_t batch = (file_nodes - n_read < 682) ? (file_nodes - n_read) : 682;
            if (f.read(io_buf, batch * 12) != batch * 12) break;
            const uint8_t* ptr = io_buf;

            for (uint32_t i = 0; i < batch; i++) {
                float nlat, nlon;
                memcpy(&nlat, ptr, 4);
                memcpy(&nlon, ptr + 4, 4);
                ptr += 12;

                if (apply_filter) {
                    if (nlat < box_s || nlat > box_n || nlon < box_w || nlon > box_e) {
                        local_to_global[n_read++] = 0xFFFFFFFF;
                        continue;
                    }
                    float t_cap, plat_cap, plon_cap;
                    float d2_cap = project_point_onto_segment_m2(
                        nlat, nlon, start_lat, start_lng, end_lat, end_lng,
                        lat_scale_m, lon_scale_m, t_cap, plat_cap, plon_cap
                    );
                    if (d2_cap > capsule_r2) {
                        local_to_global[n_read++] = 0xFFFFFFFF;
                        continue;
                    }
                }

                int32_t base_qlat = (int32_t)roundf(nlat * 25000.0f);
                int32_t base_qlon = (int32_t)roundf(nlon * 25000.0f);
                uint32_t matched_g = 0xFFFFFFFF;

                if (file_count > 1) {
                    for (int32_t dlat = -1; dlat <= 1 && matched_g == 0xFFFFFFFF; dlat++) {
                        for (int32_t dlon = -1; dlon <= 1; dlon++) {
                            uint32_t h_chk = ((uint32_t)(base_qlat + dlat) * 73856093U ^
                                              (uint32_t)(base_qlon + dlon) * 19349663U) & (HASH_BUCKETS - 1U);
                            uint16_t cur = hash_heads[h_chk];
                            while (cur != 0xFFFF) {
                                if (fabsf(macro_nodes[cur].lat - nlat) <= 0.000025f &&
                                    fabsf(macro_nodes[cur].lon - nlon) <= 0.000025f) {
                                    matched_g = (uint32_t)cur;
                                    break;
                                }
                                cur = hash_next[cur];
                            }
                            if (matched_g != 0xFFFFFFFF) break;
                        }
                    }
                }

                if (matched_g == 0xFFFFFFFF) {
                    if (total_nodes < MAX_NODES_CAP) {
                        matched_g = total_nodes++;
                        macro_nodes[matched_g].lat = nlat;
                        macro_nodes[matched_g].lon = nlon;
                        if (file_count > 1) {
                            uint32_t h_ins = ((uint32_t)base_qlat * 73856093U ^
                                              (uint32_t)base_qlon * 19349663U) & (HASH_BUCKETS - 1U);
                            hash_next[matched_g] = hash_heads[h_ins];
                            hash_heads[h_ins] = (uint16_t)matched_g;
                        }
                    }
                }
                local_to_global[n_read++] = matched_g;
            }
            yield();
        }

        if (n_read < file_nodes) {
            f.close();
            continue;
        }

        uint32_t e_read = 0;
        while (e_read < file_edges) {
            uint32_t batch = (file_edges - e_read < 682) ? (file_edges - e_read) : 682;
            if (f.read(io_buf, batch * 12) != batch * 12) break;
            const uint8_t* ptr = io_buf;

            for (uint32_t i = 0; i < batch; i++) {
                uint32_t lu, lv;
                float dist;
                memcpy(&lu, ptr, 4);
                memcpy(&lv, ptr + 4, 4);
                memcpy(&dist, ptr + 8, 4);
                ptr += 12;
                e_read++;

                if (lu >= file_nodes || lv >= file_nodes) continue;
                uint32_t gu = local_to_global[lu];
                uint32_t gv = local_to_global[lv];
                if (gu == 0xFFFFFFFF || gv == 0xFFFFFFFF || gu == gv) continue;
                if (dist < 0.5f) dist = 0.5f;

                bool exists_uv = false;
                for (uint32_t e = macro_first_edge[gu]; e != 0xFFFFFFFF; e = macro_edges[e].next) {
                    if (macro_edges[e].target == gv) {
                        if (dist < macro_edges[e].weight) macro_edges[e].weight = dist;
                        exists_uv = true;
                        break;
                    }
                }
                if (!exists_uv && edges_stored < MAX_EDGES_CAP) {
                    macro_edges[edges_stored] = {gv, dist, macro_first_edge[gu]};
                    macro_first_edge[gu] = edges_stored++;
                }
            }
            yield();
        }

        f.close();
        files_loaded++;

        if (total_nodes >= (MAX_NODES_CAP - 800) || edges_stored >= (MAX_EDGES_CAP - 1500)) {
            break;
        }
    }

    uint32_t forward_edges_count = edges_stored;
    if (files_loaded == 0 || total_nodes < 2 || forward_edges_count == 0) {
        last_astar_fail_reason = max(last_astar_fail_reason, (uint8_t)1);
        return false;
    }

    for (uint32_t u = 0; u < total_nodes && edges_stored < MAX_EDGES_CAP; u++) {
        for (uint32_t e = macro_first_edge[u]; e != 0xFFFFFFFF; e = macro_edges[e].next) {
            if (e >= forward_edges_count) continue;
            uint32_t v = macro_edges[e].target;
            float dist = macro_edges[e].weight;

            bool has_reverse = false;
            for (uint32_t rev = macro_first_edge[v]; rev != 0xFFFFFFFF; rev = macro_edges[rev].next) {
                if (macro_edges[rev].target == u) {
                    has_reverse = true;
                    break;
                }
            }
            if (!has_reverse && edges_stored < MAX_EDGES_CAP) {
                macro_edges[edges_stored] = {u, dist * 1.35f + 4.0f, macro_first_edge[v]};
                macro_first_edge[v] = edges_stored++;
            }
        }
        if ((u & 4095U) == 0) yield();
    }

    const float max_anchor_d2 = max_anchor_dist_m * max_anchor_dist_m;
    const uint32_t active_closed_bytes = (total_nodes + 7U) >> 3;

    memset(excluded_start_bits, 0x00, active_closed_bytes);
    memset(excluded_end_bits,   0x00, active_closed_bytes);

    AStarNode* pq_array = (AStarNode*)vault_arena;
    uint32_t max_pq = VAULT_ARENA_BYTES / sizeof(AStarNode);

    float best_start_proj_lat = start_lat;
    float best_start_proj_lon = start_lng;
    float best_end_proj_lat   = end_lat;
    float best_end_proj_lon   = end_lng;
    uint32_t final_end_node   = 0xFFFFFFFF;
    float end_corridor_d2     = 0.0f;
    bool reached              = false;

    for (int attempt = 0; attempt < 6; attempt++) {
        uint8_t astar_pct = prog_base + (prog_span / 2) + (uint8_t)(((attempt + 1) * (prog_span / 2)) / 6);
        actualizar_progreso_ruta_ui(min((uint8_t)96, astar_pct), "TRAZANDO RUTA OPTIMA");

        float min_start_seg_d2 = 1e30f;
        float min_end_seg_d2   = 1e30f;
        uint32_t best_end_u    = 0xFFFFFFFF;
        uint32_t best_end_v    = 0xFFFFFFFF;
        uint32_t best_start_u  = 0xFFFFFFFF;
        uint32_t best_start_v  = 0xFFFFFFFF;

        for (uint32_t u = 0; u < total_nodes; u++) {
            float ulat = macro_nodes[u].lat;
            float ulon = macro_nodes[u].lon;
            bool u_excl_s = (excluded_start_bits[u >> 3] & (1U << (u & 7U))) != 0;
            bool u_excl_e = (excluded_end_bits[u >> 3]   & (1U << (u & 7U))) != 0;

            for (uint32_t e = macro_first_edge[u]; e != 0xFFFFFFFF; e = macro_edges[e].next) {
                if (e >= forward_edges_count) continue;
                uint32_t v = macro_edges[e].target;
                float vlat = macro_nodes[v].lat;
                float vlon = macro_nodes[v].lon;

                if (!u_excl_s && !(excluded_start_bits[v >> 3] & (1U << (v & 7U)))) {
                    float t_s, plat_s, plon_s;
                    float d2_s = project_point_onto_segment_m2(
                        start_lat, start_lng, ulat, ulon, vlat, vlon, lat_scale_m, lon_scale_m, t_s, plat_s, plon_s
                    );
                    if (d2_s < min_start_seg_d2) {
                        min_start_seg_d2 = d2_s;
                        best_start_proj_lat = plat_s;
                        best_start_proj_lon = plon_s;
                        best_start_u = u;
                        best_start_v = v;
                    }
                }

                if (!u_excl_e && !(excluded_end_bits[v >> 3] & (1U << (v & 7U)))) {
                    float t_e, plat_e, plon_e;
                    float d2_e = project_point_onto_segment_m2(
                        end_lat, end_lng, ulat, ulon, vlat, vlon, lat_scale_m, lon_scale_m, t_e, plat_e, plon_e
                    );
                    if (d2_e < min_end_seg_d2) {
                        min_end_seg_d2 = d2_e;
                        best_end_proj_lat = plat_e;
                        best_end_proj_lon = plon_e;
                        best_end_u = u;
                        best_end_v = v;
                    }
                }
            }
            if ((u & 4095U) == 0) yield();
        }

        if (best_start_u == 0xFFFFFFFF || best_end_u == 0xFFFFFFFF ||
            min_start_seg_d2 > max_anchor_d2 || min_end_seg_d2 > max_anchor_d2) {
            if (attempt == 0) last_astar_fail_reason = max(last_astar_fail_reason, (uint8_t)2);
            break;
        }

        last_astar_fail_reason = max(last_astar_fail_reason, (uint8_t)3);

        const float start_corridor_m = sqrtf(min_start_seg_d2) + 60.0f;
        const float start_corridor_d2 = start_corridor_m * start_corridor_m;

        const float end_corridor_m = sqrtf(min_end_seg_d2) + 55.0f;
        end_corridor_d2 = end_corridor_m * end_corridor_m;

        for (uint32_t i = 0; i < total_nodes; i++) {
            g_score[i] = 1e30f;
            came_from[i] = 0xFFFFFFFF;
        }
        memset(closed_bits, 0x00, active_closed_bytes);
        memset(goal_bits,   0x00, active_closed_bytes);

        auto fast_h_goal = [&](uint32_t node_id) -> float {
            float dx = (best_end_proj_lon - macro_nodes[node_id].lon) * lon_scale_m;
            float dy = (best_end_proj_lat - macro_nodes[node_id].lat) * lat_scale_m;
            return sqrtf(dx * dx + dy * dy);
        };

        uint32_t pq_size = 0;
        auto push_pq = [&](uint32_t id, float f_val) {
            if (pq_size >= max_pq) return;
            uint32_t i = pq_size++;
            while (i > 0) {
                uint32_t p = (i - 1) >> 1;
                if (pq_array[p].f_score <= f_val) break;
                pq_array[i] = pq_array[p];
                i = p;
            }
            pq_array[i] = {id, f_val};
        };

        auto pop_pq = [&]() -> AStarNode {
            AStarNode root = pq_array[0];
            AStarNode last = pq_array[--pq_size];
            uint32_t i = 0;
            while ((i << 1) + 1 < pq_size) {
                uint32_t a = (i << 1) + 1;
                uint32_t b = a + 1;
                uint32_t c = (b < pq_size && pq_array[b].f_score < pq_array[a].f_score) ? b : a;
                if (pq_array[c].f_score >= last.f_score) break;
                pq_array[i] = pq_array[c];
                i = c;
            }
            pq_array[i] = last;
            return root;
        };

        const float base_s_dist = sqrtf(min_start_seg_d2);
        for (uint32_t u = 0; u < total_nodes; u++) {
            float ulat = macro_nodes[u].lat;
            float ulon = macro_nodes[u].lon;
            bool u_excl_s = (excluded_start_bits[u >> 3] & (1U << (u & 7U))) != 0;
            bool u_excl_e = (excluded_end_bits[u >> 3]   & (1U << (u & 7U))) != 0;

            for (uint32_t e = macro_first_edge[u]; e != 0xFFFFFFFF; e = macro_edges[e].next) {
                if (e >= forward_edges_count) continue;
                uint32_t v = macro_edges[e].target;
                float vlat = macro_nodes[v].lat;
                float vlon = macro_nodes[v].lon;
                float w = macro_edges[e].weight;

                if (!u_excl_s && !(excluded_start_bits[v >> 3] & (1U << (v & 7U)))) {
                    float t_s, plat_s, plon_s;
                    float d2_s = project_point_onto_segment_m2(
                        start_lat, start_lng, ulat, ulon, vlat, vlon, lat_scale_m, lon_scale_m, t_s, plat_s, plon_s
                    );
                    if (d2_s <= start_corridor_d2) {
                        float perp_pen = max(0.0f, sqrtf(d2_s) - base_s_dist);
                        float cost_to_v = perp_pen + (1.0f - t_s) * w;
                        if (cost_to_v < g_score[v]) {
                            g_score[v] = cost_to_v;
                            came_from[v] = 0xFFFFFFFF;
                            push_pq(v, cost_to_v + fast_h_goal(v));
                        }
                        float cost_to_u = perp_pen + t_s * w;
                        if (cost_to_u < g_score[u]) {
                            g_score[u] = cost_to_u;
                            came_from[u] = 0xFFFFFFFF;
                            push_pq(u, cost_to_u + fast_h_goal(u));
                        }
                    }
                }

                if (!u_excl_e && !(excluded_end_bits[v >> 3] & (1U << (v & 7U)))) {
                    float t_e, plat_e, plon_e;
                    float d2_e = project_point_onto_segment_m2(
                        end_lat, end_lng, ulat, ulon, vlat, vlon, lat_scale_m, lon_scale_m, t_e, plat_e, plon_e
                    );
                    if (d2_e <= end_corridor_d2) {
                        goal_bits[u >> 3] |= (uint8_t)(1U << (u & 7U));
                        goal_bits[v >> 3] |= (uint8_t)(1U << (v & 7U));
                    }
                }
            }
            if ((u & 4095U) == 0) yield();
        }

        goal_bits[best_end_u >> 3] |= (uint8_t)(1U << (best_end_u & 7U));
        goal_bits[best_end_v >> 3] |= (uint8_t)(1U << (best_end_v & 7U));

        uint32_t loop_cnt = 0;
        uint32_t visited_in_comp = 0;

        while (pq_size > 0) {
            uint32_t curr = pop_pq().id;
            if (goal_bits[curr >> 3] & (1U << (curr & 7U))) {
                final_end_node = curr;
                reached = true;
                break;
            }

            uint8_t mask = (uint8_t)(1U << (curr & 7U));
            uint32_t byte_idx = curr >> 3;
            if (closed_bits[byte_idx] & mask) continue;
            closed_bits[byte_idx] |= mask;
            visited_in_comp++;

            float curr_g = g_score[curr];

            for (uint32_t e = macro_first_edge[curr]; e != 0xFFFFFFFF; e = macro_edges[e].next) {
                uint32_t target = macro_edges[e].target;
                if (closed_bits[target >> 3] & (1U << (target & 7U))) continue;

                float tentative_g = curr_g + macro_edges[e].weight;
                if (tentative_g < g_score[target]) {
                    came_from[target] = curr;
                    g_score[target] = tentative_g;
                    push_pq(target, tentative_g + fast_h_goal(target));
                }
            }
            if ((++loop_cnt & 4095U) == 0) yield();
        }

        if (reached) break;

        uint32_t best_reached_end = 0xFFFFFFFF;
        float best_reached_d2 = 1e30f;
        for (uint32_t i = 0; i < total_nodes; i++) {
            if (closed_bits[i >> 3] & (1U << (i & 7U))) {
                float dxe = (macro_nodes[i].lon - end_lng) * lon_scale_m;
                float dye = (macro_nodes[i].lat - end_lat) * lat_scale_m;
                float d2e = dxe * dxe + dye * dye;
                if (d2e < best_reached_d2) {
                    best_reached_d2 = d2e;
                    best_reached_end = i;
                }
            }
        }

        if (best_reached_end != 0xFFFFFFFF && best_reached_d2 <= (180.0f * 180.0f) && visited_in_comp >= 12) {
            final_end_node = best_reached_end;
            best_end_proj_lat = macro_nodes[best_reached_end].lat;
            best_end_proj_lon = macro_nodes[best_reached_end].lon;
            reached = true;
            break;
        }

        if (visited_in_comp > 60) {
            for (uint32_t b = 0; b < active_closed_bytes; b++) {
                excluded_end_bits[b] |= goal_bits[b];
            }
            excluded_end_bits[best_end_u >> 3] |= (uint8_t)(1U << (best_end_u & 7U));
            excluded_end_bits[best_end_v >> 3] |= (uint8_t)(1U << (best_end_v & 7U));
        } else {
            for (uint32_t b = 0; b < active_closed_bytes; b++) {
                excluded_start_bits[b] |= closed_bits[b];
            }
            excluded_start_bits[best_start_u >> 3] |= (uint8_t)(1U << (best_start_u & 7U));
            excluded_start_bits[best_start_v >> 3] |= (uint8_t)(1U << (best_start_v & 7U));
        }
    }

    if (!reached || final_end_node == 0xFFFFFFFF) return false;

    if (came_from[final_end_node] != 0xFFFFFFFF) {
        uint32_t prev_n = came_from[final_end_node];
        float t_chk, plat_chk, plon_chk;
        float d2_chk = project_point_onto_segment_m2(
            end_lat, end_lng,
            macro_nodes[prev_n].lat, macro_nodes[prev_n].lon,
            macro_nodes[final_end_node].lat, macro_nodes[final_end_node].lon,
            lat_scale_m, lon_scale_m,
            t_chk, plat_chk, plon_chk
        );
        if (d2_chk <= end_corridor_d2 && t_chk < 0.85f) {
            best_end_proj_lat = plat_chk;
            best_end_proj_lon = plon_chk;
            final_end_node = prev_n;
        }
    }

    uint32_t raw_count = 0;
    uint32_t c = final_end_node;
    while (c != 0xFFFFFFFF && raw_count < total_nodes) {
        raw_count++;
        c = came_from[c];
    }
    if (raw_count == 0) return false;

    uint32_t max_graph_pts = MAX_ROUTE_POINTS_POOL - 4;
    uint32_t graph_pts = (raw_count > max_graph_pts) ? max_graph_pts : raw_count;
    uint32_t total_alloc = graph_pts + 4;

    liberar_buffers_ruta();
    route_norm_pool = (ProjectedRouteNode*)heap_caps_malloc(total_alloc * sizeof(ProjectedRouteNode), MALLOC_CAP_SPIRAM);
    route_pixels_pool = (lv_point_precise_t*)heap_caps_malloc(total_alloc * sizeof(lv_point_precise_t), MALLOC_CAP_SPIRAM);
    if (!route_norm_pool || !route_pixels_pool) {
        liberar_buffers_ruta();
        return false;
    }

    latlon_to_norm_mercator(start_lat, start_lng, route_norm_pool[0].norm_x, route_norm_pool[0].norm_y);
    latlon_to_norm_mercator(best_start_proj_lat, best_start_proj_lon, route_norm_pool[1].norm_x, route_norm_pool[1].norm_y);

    c = final_end_node;
    if (raw_count > max_graph_pts) {
        for (uint32_t i = 0; i < max_graph_pts; i++) {
            uint32_t target_step = (i * raw_count) / max_graph_pts;
            uint32_t cur_step = (i == 0) ? 0 : (((i - 1) * raw_count) / max_graph_pts);
            while (cur_step < target_step && c != 0xFFFFFFFF) {
                c = came_from[c];
                cur_step++;
            }
            if (c == 0xFFFFFFFF) break;
            latlon_to_norm_mercator(
                macro_nodes[c].lat,
                macro_nodes[c].lon,
                route_norm_pool[1 + max_graph_pts - i].norm_x,
                route_norm_pool[1 + max_graph_pts - i].norm_y
            );
        }
    } else {
        for (uint32_t i = 0; i < graph_pts; i++) {
            latlon_to_norm_mercator(
                macro_nodes[c].lat,
                macro_nodes[c].lon,
                route_norm_pool[1 + graph_pts - i].norm_x,
                route_norm_pool[1 + graph_pts - i].norm_y
            );
            c = came_from[c];
        }
    }

    latlon_to_norm_mercator(best_end_proj_lat, best_end_proj_lon, route_norm_pool[graph_pts + 2].norm_x, route_norm_pool[graph_pts + 2].norm_y);
    latlon_to_norm_mercator(end_lat, end_lng, route_norm_pool[graph_pts + 3].norm_x, route_norm_pool[graph_pts + 3].norm_y);
    active_route_len = graph_pts + 4;

    return true;
}

void calcular_y_dibujar_ruta() {
    double start_lat = start_route_lat;
    double start_lng = start_route_lng;

    if(start_lat == 0.0 || start_lng == 0.0 || target_route_lat == 0.0 || target_route_lng == 0.0) { 
        calcular_coordenadas_tactiles(&start_lat, &start_lng);
    }

    actualizar_progreso_ruta_ui(5, "PREPARANDO MEMORIA");

    stop_kinetic_glide();
    ghost_miner_parked = false;
    pausa_lectura_sd = true;
    if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
    uint32_t wait_t0 = millis();
    while (!ghost_miner_parked && (millis() - wait_t0 < 500)) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (sd_spi_mutex) xSemaphoreTake(sd_spi_mutex, portMAX_DELAY);
    if (map_file.isOpen()) map_file.close();

    static lv_point_precise_t dummy_point[1] = {{0, 0}};
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        for(int i=0; i<16; i++) {
            tiles[i].dsc.data = static_placeholder; 
            lv_image_set_src(tiles[i].obj, &tiles[i].dsc);
        }
        if (route_line != NULL) {
            lv_obj_add_flag(route_line, LV_OBJ_FLAG_HIDDEN);
            lv_line_set_points(route_line, dummy_point, 1);
        }
        lv_timer_handler(); 
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    actualizar_progreso_ruta_ui(12, "BUSCANDO SECTORES");

    liberar_buffers_ruta();
    bool reached = false; 
    last_astar_fail_reason = 0;

    int32_t start_sx = (int32_t)((start_lng + 180.0) / 0.02);
    int32_t start_sy = (int32_t)((start_lat + 90.0) / 0.02);
    int32_t end_sx   = (int32_t)((target_route_lng + 180.0) / 0.02);
    int32_t end_sy   = (int32_t)((target_route_lat + 90.0) / 0.02);
    int32_t mid_sx   = (start_sx + end_sx) / 2;
    int32_t mid_sy   = (start_sy + end_sy) / 2;

    struct SectorCandidate {
        int32_t sx;
        int32_t sy;
        float prio;
    };
    SectorCandidate cands[64];
    int cand_count = 0;

    int32_t min_sx = min(start_sx, end_sx) - 1;
    int32_t max_sx = max(start_sx, end_sx) + 1;
    int32_t min_sy = min(start_sy, end_sy) - 1;
    int32_t max_sy = max(start_sy, end_sy) + 1;

    // -----------------------------------------------------------------
    // ETAPA 1 (15% -> 65%): CORREDOR MULTI-SECTOR COSIDO (RUTA MÁS CORTA)
    // -----------------------------------------------------------------
    if ((max_sx - min_sx) <= 14 && (max_sy - min_sy) <= 14) {
        float ax = (float)start_sx, ay = (float)start_sy;
        float bx = (float)end_sx,   by = (float)end_sy;
        float vx = bx - ax,         vy = by - ay;
        float seg_len2 = vx * vx + vy * vy;

        for (int32_t sx = min_sx; sx <= max_sx; sx++) {
            for (int32_t sy = min_sy; sy <= max_sy; sy++) {
                float wx = (float)sx - ax;
                float wy = (float)sy - ay;
                float t = (seg_len2 > 0.0f) ? ((wx * vx + wy * vy) / seg_len2) : 0.0f;
                if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
                float dx = wx - t * vx;
                float dy = wy - t * vy;
                float d2_seg = dx * dx + dy * dy;

                if (d2_seg <= 3.5f && cand_count < 64) {
                    cands[cand_count++] = {sx, sy, d2_seg * 10.0f + t};
                }
            }
        }

        for (int i = 0; i < cand_count - 1; i++) {
            for (int j = i + 1; j < cand_count; j++) {
                if (cands[j].prio < cands[i].prio) {
                    SectorCandidate tmp = cands[i];
                    cands[i] = cands[j];
                    cands[j] = tmp;
                }
            }
        }

        static char files_to_merge[24][64];
        int merge_count = 0;
        for (int i = 0; i < cand_count && merge_count < 22; i++) {
            snprintf(files_to_merge[merge_count], 64, "/SECTORS/S_%d_%d.HWY", (int)cands[i].sx, (int)cands[i].sy);
            if (sd.exists(files_to_merge[merge_count])) {
                merge_count++;
            }
        }
        if (sd.exists("/ARTERIAL.HWY") && merge_count < 24) {
            strncpy(files_to_merge[merge_count++], "/ARTERIAL.HWY", 64);
        }

        if (merge_count > 0) {
            reached = run_astar_multi_hwy_files(
                files_to_merge,
                merge_count,
                (float)start_lat, (float)start_lng,
                (float)target_route_lat, (float)target_route_lng,
                4500.0f,
                true,
                15, 50
            );
        }
    }

    // -----------------------------------------------------------------
    // ETAPA 2 (65% -> 80%): SECTOR INDIVIDUAL SIN FILTRO DE CÁPSULA
    // -----------------------------------------------------------------
    if (!reached && abs(start_sx - end_sx) <= 2 && abs(start_sy - end_sy) <= 2) {
        int32_t single_sx[3] = { mid_sx, start_sx, end_sx };
        int32_t single_sy[3] = { mid_sy, start_sy, end_sy };
        static char single_file[1][64];

        for (int c = 0; c < 3 && !reached; c++) {
            snprintf(single_file[0], 64, "/SECTORS/S_%d_%d.HWY", (int)single_sx[c], (int)single_sy[c]);
            if (sd.exists(single_file[0])) {
                reached = run_astar_multi_hwy_files(
                    single_file, 1,
                    (float)start_lat, (float)start_lng,
                    (float)target_route_lat, (float)target_route_lng,
                    5000.0f,
                    false,
                    65, 15
                );
            }
        }
    }

    // -----------------------------------------------------------------
    // ETAPA 3 (80% -> 96%): RESPALDO GARANTIZADO EN /ARTERIAL.HWY PURO
    // -----------------------------------------------------------------
    if (!reached && sd.exists("/ARTERIAL.HWY")) {
        static char art_only[1][64] = { "/ARTERIAL.HWY" };
        reached = run_astar_multi_hwy_files(
            art_only, 1,
            (float)start_lat, (float)start_lng,
            (float)target_route_lat, (float)target_route_lng,
            55000.0f,
            false,
            80, 16
        );
    }

    actualizar_progreso_ruta_ui(100, reached ? "RUTA COMPLETADA" : "FINALIZANDO");

    shadow_buffer = shadow_arena_base;

    uint8_t* vault_ptr = vault_arena;
    for(int i=0; i<MAX_QOI_TILES; i++) {
        l2_vault[i].buffer = vault_ptr;
        l2_vault[i].valid = false;
        vault_ptr += QOI_CHUNK_SIZE;
    }
    for(int i=0; i<MAX_HEADER_CACHE; i++) {
        hdr_cache[i].valid = false;
    }
    memset(empty_tile_ring, 0, sizeof(empty_tile_ring));
    memset(sd_blacklist, 0, sizeof(sd_blacklist));
    empty_tile_head = 0;
    bl_head = 0;
    l2_head = 0;
    __sync_synchronize();

    if (sd_spi_mutex) xSemaphoreGive(sd_spi_mutex);

    ocultar_progreso_ruta_ui();

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        uint8_t* arena_ptr = global_arena;
        for(int i=0; i<16; i++) {
            tiles[i].buffer = arena_ptr;
            tiles[i].dsc.data = static_placeholder; 
            lv_image_set_src(tiles[i].obj, &tiles[i].dsc);
            tiles[i].needs_load = true;
            tiles[i].ready_to_refresh = false;
            arena_ptr += 131072;
        }

        if(reached && active_route_len >= 2) {
            uint32_t r_col = get_active_tactical_color();
            if(route_line == NULL) {
                route_line = lv_line_create(tile_container);
                static lv_style_t style_line; lv_style_init(&style_line);
                lv_style_set_line_width(&style_line, 5);
                lv_style_set_line_color(&style_line, lv_color_hex(r_col)); 
                lv_style_set_line_rounded(&style_line, true);
                lv_style_set_line_opa(&style_line, LV_OPA_COVER);
                lv_obj_add_style(route_line, &style_line, 0);
                lv_obj_remove_flag(route_line, LV_OBJ_FLAG_CLICKABLE);
            }
            lv_obj_set_style_line_color(route_line, lv_color_hex(r_col), LV_PART_MAIN);
            lv_obj_set_style_line_opa(route_line, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_clear_flag(route_line, LV_OBJ_FLAG_HIDDEN); 
            refrescar_linea_ruta();
            lv_obj_move_foreground(route_line);
            if (punto_gps != NULL) lv_obj_move_foreground(punto_gps);
            mostrar_toast("RUTA OPTIMA ESTABLECIDA");

            if (btn_menu_ruta_ptr != NULL && lbl_menu_ruta_ptr != NULL) {
                lv_label_set_text(lbl_menu_ruta_ptr, "BORRAR RUTA");
                lv_obj_set_style_bg_color(btn_menu_ruta_ptr, lv_color_hex(0xFF003C), LV_PART_MAIN); 
                lv_obj_set_style_text_color(lbl_menu_ruta_ptr, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            }
        } else {
            char err_msg[40];
            if (last_astar_fail_reason == 0) {
                snprintf(err_msg, sizeof(err_msg), "FALTA S_%d_%d / HWY", (int)mid_sx, (int)mid_sy);
            } else if (last_astar_fail_reason == 1) {
                snprintf(err_msg, sizeof(err_msg), "HWY EXCEDE RAM RELOJ");
            } else if (last_astar_fail_reason == 2) {
                snprintf(err_msg, sizeof(err_msg), "SIN CALLES EN S_%d_%d", (int)mid_sx, (int)mid_sy);
            } else {
                snprintf(err_msg, sizeof(err_msg), "SIN CONEXION VIAL");
            }
            mostrar_toast(err_msg);
        }

        update_map_topology(0, 0);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }

    pausa_lectura_sd = false;
    if (ghost_miner_handle) xTaskNotifyGive(ghost_miner_handle);
}

void init_carto_engine() {
    wp_prefs.begin("exoned-wp", false);
    wp_count = wp_prefs.getUChar("wp_count", 0);
    tactical_color_idx = wp_prefs.getUChar("tac_col", 0) % 6;
    for(int i = 0; i < wp_count; i++) {
        char key_base[10]; sprintf(key_base, "wp_%d", i);
        wp_prefs.getBytes(key_base, &wp_db[i], sizeof(Waypoint));
    }

    if (sd_spi_mutex == NULL) {
        sd_spi_mutex = xSemaphoreCreateMutex();
    }

    init_haptico();
    configurar_gnss(1);

    global_arena = (uint8_t*)heap_caps_malloc(GLOBAL_ARENA_BYTES, MALLOC_CAP_SPIRAM);
    vault_arena = (uint8_t*)heap_caps_malloc(VAULT_ARENA_BYTES, MALLOC_CAP_SPIRAM);
    shadow_arena_base = (uint8_t*)heap_caps_malloc(131072, MALLOC_CAP_SPIRAM);
    shadow_buffer = shadow_arena_base;
    static_placeholder = (uint8_t*)heap_caps_malloc(131072, MALLOC_CAP_SPIRAM); 
    compressed_buffer = (uint8_t*)heap_caps_malloc(COMPRESSED_BUF_BYTES, MALLOC_CAP_SPIRAM);

    for(int i = 0; i < MAX_HEADER_CACHE; i++) {
        hdr_cache[i].data = (uint32_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM);
        hdr_cache[i].valid = false;
    }

    if(!global_arena || !vault_arena || !compressed_buffer || !shadow_arena_base) {
        Serial.println("[MOTOR] ERROR CRITICO: Falla al asegurar los bancos de memoria.");
    }

    memset(global_arena, 0x00, GLOBAL_ARENA_BYTES);

    // 1. Generar retícula táctica base en PSRAM y luego intentar cargar placeholder custom de la SD
    render_fallback_tactical_placeholder(static_placeholder);
    reload_custom_placeholder();

    uint8_t * vault_ptr = vault_arena;
    for(int i=0; i<MAX_QOI_TILES; i++) {
        l2_vault[i].buffer = vault_ptr;
        l2_vault[i].valid = false;
        vault_ptr += QOI_CHUNK_SIZE;
    }

    xTaskCreatePinnedToCore(raw_reader_task, "GhostMiner", 32768, NULL, 2, &ghost_miner_handle, 0);
}

void carto_loop_handler() {
    while (GPSSerial.available() > 0) {
        char c = GPSSerial.read();
        gps.encode(c);

        if (c == '\n') {
            nmea_buffer[nmea_idx] = '\0'; 
            if (nmea_buffer[3] == 'G' && nmea_buffer[4] == 'S' && nmea_buffer[5] == 'V') {
                int commas = 0;
                int msg_num = 0;
                for(int i=0; i<nmea_idx; i++) {
                    if(nmea_buffer[i] == ',') {
                        commas++;
                        if (commas == 2 && i+1 < nmea_idx) msg_num = nmea_buffer[i+1] - '0';
                        if (commas == 3 && i+1 < nmea_idx && msg_num == 1) {
                            sats_in_view = atoi(&nmea_buffer[i+1]);
                            max_snr = 0; 
                        }
                        if((commas == 7 || commas == 11 || commas == 15 || commas == 19) && i + 1 < nmea_idx) {
                            int snr_val = atoi(&nmea_buffer[i+1]);
                            if(snr_val > max_snr && snr_val != 99) max_snr = snr_val; 
                        }
                    }
                }
            }
            nmea_idx = 0; 
        } else if (c != '\r' && nmea_idx < 126) {
            nmea_buffer[nmea_idx++] = c;
        }
    }

    // --- FILTRO DE PÉRDIDA DE SEÑAL SATELITAL (30 SEGUNDOS) Y FUSIÓN DUAL ---
    uint32_t now_ms = millis();
    static uint32_t last_valid_watch_gps_ms = 0;

    bool watch_updated = gps.location.isUpdated() &&
                         gps.location.isValid() &&
                         (gps.location.lat() != 0.0 || gps.location.lng() != 0.0);

    if (watch_updated) {
        last_valid_watch_gps_ms = now_ms;
    }

    bool watch_valid = (last_valid_watch_gps_ms > 0) &&
                       ((now_ms - last_valid_watch_gps_ms) <= 30000UL) &&
                       gps.location.isValid();

    bool phone_valid = (remote_gnss_rx_count > 0) &&
                       (remote_gnss_last_ms > 0) &&
                       ((now_ms - remote_gnss_last_ms) <= 30000UL) &&
                       (remote_gnss_lat != 0.0f || remote_gnss_lon != 0.0f);

    static uint32_t last_seen_phone_pkt = 0;
    bool phone_updated = phone_valid && (remote_gnss_rx_count != last_seen_phone_pkt);

    if (watch_valid && phone_valid) {
        gnss_active_source = 3; // COMBINADOS (MAGENTA NEÓN)
        if (watch_updated || phone_updated || !gps_locked) {
            last_seen_phone_pkt = remote_gnss_rx_count;
            double w_lat = gps.location.lat();
            double w_lng = gps.location.lng();
            double p_lat = (double)remote_gnss_lat;
            double p_lng = (double)remote_gnss_lon;

            float sep_m = fast_distance((float)w_lat, (float)w_lng, (float)p_lat, (float)p_lng);
            if (sep_m <= 150.0f) {
                double hdop = gps.hdop.hdop();
                double w_weight = (hdop > 0.0 && hdop <= 1.8) ? 0.65 : 0.40;
                double p_weight = 1.0 - w_weight;
                current_lat = (w_lat * w_weight) + (p_lat * p_weight);
                current_lng = (w_lng * w_weight) + (p_lng * p_weight);
            } else {
                current_lat = w_lat;
                current_lng = w_lng;
            }
            fused_alt_m = gps.altitude.isValid() ? gps.altitude.meters() : (double)remote_gnss_alt;
            fused_spd_kmh = gps.speed.isValid() ? gps.speed.kmph() : (double)remote_gnss_spd;
            current_satellites = gps.satellites.value();
            gps_locked = true;
            gnss_pos_dirty = true;
        }
    } else if (watch_valid) {
        gnss_active_source = 1; // SOLO RELOJ (AZUL)
        if (watch_updated || !gps_locked) {
            current_lat = gps.location.lat();
            current_lng = gps.location.lng();
            fused_alt_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
            fused_spd_kmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
            current_satellites = gps.satellites.value();
            gps_locked = true;
            gnss_pos_dirty = true;
        }
    } else if (phone_valid) {
        gnss_active_source = 2; // SOLO TELÉFONO (AMARILLO)
        if (phone_updated || !gps_locked) {
            last_seen_phone_pkt = remote_gnss_rx_count;
            current_lat = (double)remote_gnss_lat;
            current_lng = (double)remote_gnss_lon;
            fused_alt_m = (double)remote_gnss_alt;
            fused_spd_kmh = (double)remote_gnss_spd;
            gps_locked = true;
            gnss_pos_dirty = true;
        }
    } else {
        if (gps_locked) {
            gps_locked = false;
            gnss_active_source = 0;
            current_satellites = 0;
            if (punto_gps != NULL && xSemaphoreTakeRecursive(lvgl_mutex, 0) == pdTRUE) {
                lv_obj_add_flag(punto_gps, LV_OBJ_FLAG_HIDDEN);
                xSemaphoreGiveRecursive(lvgl_mutex);
            }
        }
        gnss_active_source = 0;
    }

    if (gps_locked && !primer_enganche_logrado && current_lat != 0.0 && current_lng != 0.0 && !pausa_lectura_sd) {
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            primer_enganche_logrado = true;
            seguimiento_gnss_activo = true;
            if (!osd_visible) {
                lv_obj_add_flag(panel_osd, LV_OBJ_FLAG_HIDDEN);
            }
            anclar_a_gnss();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    }

    if (flag_calc_route) {
        flag_calc_route = false;
        calcular_y_dibujar_ruta();
    }
}

void invalidate_map_cache() {
    for(int i = 0; i < MAX_QOI_TILES; i++) {
        l2_vault[i].valid = false;
    }
    for(int i = 0; i < MAX_HEADER_CACHE; i++) {
        hdr_cache[i].valid = false;
    }
    memset(empty_tile_ring, 0, sizeof(empty_tile_ring));
    memset(sd_blacklist, 0, sizeof(sd_blacklist));
    empty_tile_head = 0;
    bl_head = 0;
    l2_head = 0;
    __sync_synchronize();
    Serial.println("[CARTO ENGINE] Cache L2 (20 ranuras de 40KB) purgada tras interrupcion de video.");
}