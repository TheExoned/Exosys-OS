# EXOSYS OS — Tactical Real-Time Operating System & Spatial Ecosystem

![Platform](https://img.shields.io/badge/Hardware-ESP32--S3%20Dual--Core%20240MHz-00E5FF?style=for-the-badge&logo=espressif)
![Memory](https://img.shields.io/badge/Memory-8MB%20Octal%20PSRAM%20%7C%20512KB%20SRAM-00FF41?style=for-the-badge)
![Kernel](https://img.shields.io/badge/Kernel-FreeRTOS%20SMP-FFB300?style=for-the-badge)
![License](https://img.shields.io/badge/License-GNU%20AGPLv3%20%2F%20Commercial-FF003C?style=for-the-badge)

**Exosys OS** is a deterministic, bare-metal Real-Time Operating System and distributed pre-compilation ecosystem engineered to push dual-core microcontrollers (ESP32-S3) beyond their theoretical hardware limits. 

Traditionally, autonomous instrumentation capable of multi-constellation GNSS telemetry, rasterized topological mapping, heuristic A* pathfinding over tens of thousands of nodes, and smooth video playback requires Application Processors (ARM Cortex-A MPUs) with Memory Management Units (MMUs) consuming **> 1,500 mW**. Exosys OS achieves equivalent spatial and graphical throughput on a resource-constrained MCU without a GPU or MMU while consuming **110–150 mW**.

---

## 1. Architectural Paradigm: Compute Asymmetry & Memory Multiplexing

Microcontrollers suffer from three physical bottlenecks when running complex graphical and spatial workloads concurrently:
1. **SPI Bus Contention:** Reading map clusters and video frames from an external MicroSD card while pushing pixels to a DMA display over shared or bandwidth-limited buses causes severe frame drops and bus collisions.
2. **PSRAM Latency & Heap Fragmentation:** External Octal SPI PSRAM (8 MB) is large but slow compared to internal SRAM (512 KB). Dynamic allocations (`malloc`/`new`) during real-time pan/zoom operations fragment the heap and trigger kernel panics.
3. **FAT32 File-System Overhead:** Storing high-resolution maps (Zoom 15–20) as individual tile files generates millions of entries, pushing `SdFat` `open()` latencies above **15 ms** per tile.

Exosys OS eliminates these bottlenecks through **Three-Tier Compute Asymmetry** (PC JIT Forge + Android Tactical Hub + ESP32-S3 SMP Kernel) and **Static Monolithic Memory Arenas**.

```text
+-----------------------------------------------------------------------------------+
|                        TIER 1 & 2: EXTERNAL PRE-COMPILATION                       |
|  [PC Forge JIT (Python + Numba)]          [Android Tactical Link (Kotlin)]        |
|  - OSM Urban Pruning (-89% dead weight)   - Hardware MediaCodec .EXND V32 Forge   |
|  - 32x32 Column-Major .QOI Clustering     - QOI-565 Static Placeholder Regulator  |
|  - Multi-Sector .HWY Road Graph Builder   - BLE GATT GNSS Injection & Remote UI   |
+-----------------------------------------+-----------------------------------------+
                                          |
                        TCP (:8892) / UDP (:8889) / BLE / SD
                                          |
+-----------------------------------------v-----------------------------------------+
|                     TIER 3: EXOSYS RTOS (ESP32-S3 @ 240 MHz)                      |
|                                                                                   |
|  +----------------------------------+       +----------------------------------+  |
|  |       CORE 0 (PRO_CPU)           |       |       CORE 1 (APP_CPU)           |  |
|  |  - GhostMiner Predictive I/O     |       |  - CartoEngine Kinetic Physics   |  |
|  |  - IRAM QOI-565 Fast Decoder     |       |  - BackgroundEngine (.EXND V32)  |  |
|  |  - LwIP TCP/UDP & NimBLE Stack   |       |  - Multi-Sector A* Pathfinding   |  |
|  +----------------+-----------------+       +-----------------+----------------+  |
|                   |                                           |                   |
|                   +-------------+---------------+-------------+                   |
|                                 | Mutex & Spinlocks                               |
|                                 v                                                 |
|  +-----------------------------------------------------------------------------+  |
|  |                    8 MB OCTAL PSRAM (MONOLITHIC ARENAS)                     |  |
|  |  - global_arena (2.09 MB): L1 Map Tiles (16x 256x256) <-> A* Graph Overlay  |  |
|  |  - vault_arena  (819 KB) : L2 Lock-Free QOI Cache (20 slots) <-> .EXND VRAM |  |
|  |  - shadow_buffer (128 KB): Zero-Copy Pointer Swap Staging Bank              |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 2. Core Synchronization & Symmetric Multiprocessing (SMP)

Exosys OS enforces strict hardware isolation between the two Xtensa LX7 cores using FreeRTOS task pinning, recursive mutexes (`lvgl_mutex`, `sd_spi_mutex`), and hardware spinlocks (`tile_spinlock`).

| Execution Unit | Assigned Subsystem | Technical Operation | Architectural Justification | GUI / User Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Core 0 (`PRO_CPU`)** | **GhostMiner** (`raw_reader_task`) | Audits user pan velocity vectors (`vector_x`, `vector_y`), fetches `Cx,y.QOI` clusters from MicroSD, and decodes tiles into the L2 vault. | FAT32 I/O and SPI bus transactions are blocking. Isolating them on Core 0 prevents UI thread starvation. | Zero stuttering during high-speed kinetic map panning; tiles hydrate ahead of the viewport. |
| **Core 0 (`PRO_CPU`)** | **RF & Network Stack** | Executes 802.11 Wi-Fi MAC/PHY, LwIP TCP/UDP sockets (`:8892` / `:8889`), and NimBLE GATT services. | RF interrupts require microsecond-level servicing dictated by ESP-IDF silicon design. | Seamless background injection of 7.4 MB `.EXND` videos and remote screen control without watchdog resets. |
| **Core 1 (`APP_CPU`)** | **CartoEngine & LVGL** | Runs sub-pixel exponential decay physics, capacitive multitouch pinch-to-zoom, and DMA display transfers. | Deterministic frame pacing requires uninterrupted CPU cycles and zero SD bus wait states. | Smartphone-grade 60 FPS kinetic touch response and instant UI transitions. |
| **Core 1 (`APP_CPU`)** | **BackgroundEngine** | Streams `.EXND` differential video from internal `FFat` flash, applying Delta RLE and 2x FSR spatial scaling. | Synchronizes directly with `lvgl_mutex` to invalidate dirty bounding boxes without tearing. | Smooth 24 FPS animated wallpapers running beneath real-time telemetry HUDs. |
| **Core 1 (`APP_CPU`)** | **GNSS & A* Router** | Injects bare-metal UBX binary frames to the U-Blox MIA-M10Q and executes multi-sector A* routing. | Requires exclusive, high-bandwidth access to `global_arena` during destructive memory overlay. | Computes optimal routes across 38,000 road nodes and renders Mercator polylines in milliseconds. |

---

## 3. Core Subsystems & Technical Innovations

### 3.1. CartoEngine, GhostMiner & Modified QOI-565 Codec
* **1024:1 File Reduction via Monolithic Clustering:** Instead of storing individual tiles, maps are packed into 32x32 sector clusters (`/MAP/Z#/C_x_y.QOI`) with an 8,192-byte Column-Major header table (`uint32_t offset`, `uint32_t length`). Four header cache slots in PSRAM allow O(1) sector lookups without repeated SD seeks.
* **IRAM-Accelerated QOI-565:** Standard QOI operates on 24/32-bit RGB888/RGBA, requiring costly per-pixel bit-shifting on 16-bit displays. Exosys OS implements a custom **QOI-565** specification that compresses pre-swapped 16-bit RGB565 pixels directly using three bitwise opcodes (`RUN 0x00`, `INDEX 0x40` with a 64-entry hash LUT, and `LITERAL 0x80`). Compiled with `IRAM_ATTR`, `decode_qoi565_fast()` decodes into a 2 KB internal SRAM strip (`sram_strip[1024]`) before bursting to PSRAM.
* **Zero-Copy Pointer Swapping:** When GhostMiner finishes decoding a tile into `shadow_buffer`, it does not copy the 131,072-byte image into the active LVGL descriptor. Inside a critical spinlock, it swaps the 32-bit memory pointers (`tiles[best_idx].buffer = shadow_buffer`). Cost: **1 CPU clock cycle**.

### 3.2. Destructive VRAM Overlay for Heuristic A* Pathfinding
Running A* across a city-scale road network requires megabytes of RAM for node coordinates, adjacency lists, g-scores, and priority queues—memory that does not exist when 16 map tiles occupy 2.09 MB of PSRAM.
* **The Overlay Protocol:** When a route is requested, CartoEngine parks GhostMiner (`pausa_lectura_sd = true`), points all 16 LVGL tile descriptors to a single 128 KB `static_placeholder` buffer, and **destructively repurposes the 2,097,152-byte `global_arena`** as a contiguous mathematical workspace.
* **Multi-Sector Stitching & Capsule Culling:** `run_astar_multi_hwy_files()` stitches up to 24 `.HWY` sector files plus `/ARTERIAL.HWY` in real time, using a 32,768-bucket spatial hash table to weld boundary nodes. It allocates up to **38,000 nodes** and **75,000 bidirectional edges** inside `global_arena`, while repurposing `vault_arena` (819 KB) as a binary min-heap priority queue.
* **Sub-Millisecond Recovery:** Once the optimal path is projected into normalized Mercator coordinates (`route_norm_pool`), the engine zeroes the L2 cache flags and wakes GhostMiner, restoring the live raster map underneath the route polyline in milliseconds.

### 3.3. `.EXND` V32 Differential Video Engine & Bus Arbitration
Playing video from a MicroSD card while a map engine runs on the same SPI bus causes fatal collisions.
* **SPI-to-FFat Asynchronous Offloading:** When a wallpaper is selected, `secure_spi_lock()` parks GhostMiner and transfers the `.EXND` binary in 8 KB slices from the MicroSD to the internal 8 MB `FFat` flash partition. Playback occurs 100% from internal flash, leaving the SPI bus free.
* **Zero-Math Delta RLE + 2x FSR Geometry:** `.EXND` V32 encodes frames at half logical resolution (205x251) using ordered 4x4 Bayer dithering. Subsequent frames store only modified pixels within a tight bounding box (`skip` and `write_len` pairs). During decoding, each 16-bit pixel is expanded into a 2x2 physical block (410x502) directly inside `vault_arena`, cutting storage weight and bus bandwidth by **75%** while only invalidating dirty screen rows (`lv_obj_invalidate_area`).

### 3.4. Tri-State GNSS Sensor Fusion & Bare-Metal UBX Injection
* **Hardware Reconfiguration:** Communicates with the U-Blox MIA-M10Q over UART using raw binary UBX frames (`0xB5 0x62`) to dynamically switch satellite constellations (`GPS + Galileo + GLONASS` for Americas/Europe/Africa vs. `GPS + Galileo + BeiDou` for Asia/Oceania) and parse real-time SNR and HDOP metrics.
* **Dual-Source Weighted Fusion:** Continuously evaluates signal validity across a 30-second hardware watchdog:
  * **Mode 1 — Internal U-Blox 3D Fix (Blue):** Autonomous watch navigation.
  * **Mode 2 — Mobile Tactical Link (Yellow):** Fallback GNSS injected from the Android companion app via BLE/UDP when indoor or under heavy canopy.
  * **Mode 3 — Dual HDOP-Weighted Fusion (Neon Magenta):** Combines watch and phone coordinates using dynamic HDOP weighting when both fixes are locked within a 150 m coherence radius.

### 3.5. Android Tactical Hub & PC JIT Forge
* **Android Studio Companion (`ExndWallpaperStudio` / `BleCoreService`):** Extracts video frames via hardware `MediaCodec` YUV420 pipelines, executes a 324-step perceptual pain ladder to fit any clip under 7.4 MB, regulates 256x256 static map placeholders under a strict 38 KB QOI ceiling, and injects files over raw TCP sockets (`:8892`). Includes remote screen casting and accessibility touch injection.
* **PC Forge JIT (`Numba @njit(nogil=True)`):** Interrogates OpenStreetMap vector databases in two passes (`landuse`/`place` polygons + `highway` bounding boxes) to prune **89% of uninhabited rural/ocean tiles** at high zoom levels. Uses a Token-Bucket governor with micro-jitter across multi-provider fallback servers (Google -> Carto -> Esri) to compile state-wide `Cx,y.QOI` clusters at **0.18 ms/sector**.

---

## 4. Benchmark Comparison

| Metric / Subsystem | Commercial Smartwatch (MPU) | Standard Embedded MCU | Exosys OS Architecture (ESP32-S3) |
| :--- | :--- | :--- | :--- |
| **Power Consumption** | > 1,500 mW (< 18 h battery) | 50–90 mW | **110–150 mW (Multi-day autonomy)** |
| **Map Storage Strategy** | Cloud-dependent / SQLite | Millions of loose PNG/BIN files | **OSM Urban Pruning (-89%) + 32x32 `.QOI` Clusters** |
| **SD File Open Overhead** | N/A (eMMC ext4) | > 15 ms per tile (FAT32 degradation) | **1024:1 File Reduction + 8 KB Column-Major Header Cache** |
| **Video Playback Engine** | Hardware GPU H.264 | Unfeasible / SPI bus lockup | **`.EXND` V32 in `FFat` + Delta RLE + 2x FSR Upscaling** |
| **Autonomous A* Routing** | Offloaded to smartphone API | Out-of-Memory (RAM exhaustion) | **Destructive 2 MB VRAM Overlay (38k nodes / 75k edges)** |
| **Hardware BOM Cost** | > $400–$600 USD | $15–$30 USD | **< $60 USD (Full Tactical Instrument)** |

---

## 5. Target Applications & Use Cases

1. **Off-Grid Search and Rescue (SAR) & Tactical Navigation:** Zero-dependency offline cartography, multi-constellation GNSS telemetry, and autonomous road/trail routing in disaster zones or areas without cellular infrastructure.
2. **Ultra-Low-Power Edge Instrumentation:** High-refresh-rate HMI and spatial logging for agricultural monitoring, environmental telemetry, and long-range LoRa mesh nodes.
3. **Next-Generation Wearable & IoT Frameworks:** Drop-in architectural blueprint (`.EXND` and `CartoEngine`) for running 60 FPS GUIs and video on low-cost microcontrollers (ESP32-S3, RP2350, STM32) without adding expensive MPUs or GPUs.

---

## 6. Repository Structure

```text
Exosys-OS/
├── boards/                 # Custom ESP32-S3 board definitions (LilyGo T-Watch Ultra / S3)
├── lib/                    # Hardware-specific audio & peripheral libraries
├── src/
│   ├── main.cpp            # FreeRTOS SMP orchestration, hardware init & UI router
│   ├── CartoEngine.cpp/.h  # GhostMiner, QOI-565 IRAM codec, L2 Vault & Multi-Sector A*
│   ├── BackgroundEngine.cpp/.h # .EXND V32 decoder, FFat async transfer & TCP server (:8892)
│   ├── NetworkEngine.cpp/.h    # Wi-Fi, UDP telemetry (:8889) & BLE GATT synchronization
│   ├── KinEngine.cpp/.h        # IMU gesture recognition & kinematic processing
│   ├── PowerEngine.cpp/.h      # PMU AXP2101 management & dynamic clock scaling
│   ├── TimeEngine.cpp/.h       # RTC synchronization & tactical alarms
│   └── Settings.cpp/.h         # NVS persistence & system configuration UI
├── support/                # Build scripts & SDL2 emulator hooks
├── variants/               # Pinout & hardware variant definitions
└── platformio.ini          # Build flags, partition tables & dependency graph
```

---

## 7. Licensing (Dual-License Model)

**Copyright (C) 2026 Axel Daniel Luna Carmona (TheExoned). All Rights Reserved.**

Exosys OS and its proprietary engines (`CartoEngine`, `BackgroundEngine .EXND`, `GhostMiner`) are distributed under a **Dual-Licensing Model**:

1. **Open-Source / Non-Commercial & Academic Use (GNU AGPLv3):**  
   This software is free for personal, academic, and open-source use under the terms of the **GNU Affero General Public License v3.0** (see [`LICENSE`](./LICENSE)). Any modification, derivative work, or network/fleet deployment must remain open-source under the exact same AGPLv3 terms and disclose its complete Corresponding Source code.
2. **Commercial & Proprietary Licensing:**  
   If you are a corporation or commercial entity wishing to integrate Exosys OS, `.EXND`, or `CartoEngine` into closed-source firmware, commercial wearables, fleet hardware, or proprietary products without AGPLv3 copyleft obligations, you must acquire a **Commercial License**.  
   *For commercial licensing inquiries, contact the author directly via GitHub [@TheExoned](https://github.com/TheExoned).*
