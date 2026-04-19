# AVI Flight Computer - Architecture Documentation

## 1. Overall System Architecture

The AVI Flight Computer is a modular embedded avionics system built around an STM32F411 microcontroller. The architecture follows a layered approach where hardware peripherals are abstracted through an OSAL (Operating System Abstraction Layer), enabling high-level modules to remain portable and testable.

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Application Layer                           │
│                    (main.c - 50 Hz loop)                         │
├─────────────────────────────────────────────────────────────────────┤
│  Data Processing   │  Altitude Engine   │  Apogee Detection       │
│  (CSV Format)   │  (Barometric)     │  (State Machine)       │
├─────────────────────────────────────────────────────────────────────┤
│                          Library Layer                             │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐  │
│  │   IMU   │ │ MS5837  │ │  SD_SPI  │ │FatFS    │ │ OSAL     │  │
│  │ (SPI)   │ │ (I2C)   │ │ (SPI)    │ │         │ │Hal Abstr. │  │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                     Hardware Peripherals                           │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐    │
│  │   SPI1  │ │   I2C1  │ │  USB    │ │  SDIO   │ │  GPIO   │    │
│  │(IMU,SD)│ │(Baro)   │ │  CDC    │ │  (CS)   │ │  (LED)  │    │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └──────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

## 2. Hardware Block Diagram

### MCU and Peripherals

```
                    ┌──────────────────────┐
                    │   STM32F411CEU6       │
                    │   (BlackPill)         │
                    │                      │
                    │   96 MHz            │
                    │   Cortex-M4F         │
                    └─────────┬────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
    ┌────▼─────┐        ┌────▼────┐        ┌────▼──────┐
    │   SPI1   │        │  I2C1   │        │   USB     │
    │          │        │        │        │   OTG     │
    │  MOSI ───┼───┐    │   SCL   │        │   D+      │
    │  MISO ───┼───┼──► │   SDA   │        │   D-      │
    │  SCK  ───┼───┘    │        │        │           │
    │  NSS ◄───┤CS     │        │        │  (CDC)    │
    └────┬─────┘        └────┬────┘        └───────────┘
         │                    │
    ┌────▼───────────────────▼────┐
    │      SPI Bus (Shared)        │
    │  ┌─────────┐    ┌─────────┐ │
    │  │  IMU   │    │   SD    │ │
    │  │LSM6DSO │    │  Card   │ │
    │  │(SPI)   │    │ (SPI)   │ │
    │  └──┬─────┘    └──┬─────┘ │
    │     │CS            │CS      │
    │   ┌─┴──┐       ┌─┴──┐    │
    │   │PA4 │        │PA3 │    │
    └───┴────┴────────┴────┴────┘
          │
    ┌────▼────────────────────┐
    │    Barometer          │
    │    MS5837-30BA        │
    │    (I2C)              │
    │    Address: 0x76      │
    └───────────────────────┘
```

### Pin Assignment Summary

| Pin | Function | Peripheral | Notes |
|-----|----------|------------|-------|
| PA4 | CS_IMU | SPI1 NSS | IMU chip select |
| PA3 | CS_SD | GPIO Output | SD card chip select |
| PA5 | SCK | SPI1 | Clock |
| PA6 | MISO | SPI1 | Master input |
| PA7 | MOSI | SPI1 | Master output |
| PB6 | SCL | I2C1 | Barometer clock |
| PB7 | SDA | I2C1 | Barometer data |
| PA9 | USB D+ | USB OTG | CDC data+ |
| PA10 | USB D- | USB OTG | CDC data- |

## 3. Data Flow Architecture

### Sensor to Storage Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   IMU       │     │  Barometer  │     │  Altitude   │
│  (LSM6DSO)  │     │  (MS5837)   │     │  Engine     │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                    │                    │
       │ SPI                │ I2C               │
       ▼                    ▼                    ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  imu_read  │     │ ms5837_read │     │altitude_upd │
│   (g, dps) │     │ (Pa, °C)    │     │(mm, mm/s)   │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                    │                    │
       └────────────────────┼────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────┐
│                    CSV Formatting                      │
│              Format: t;ax;ay;az;wx;wy;wz;T;p            │
└──���─��─────────────────────┬──────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              │                         │
              ▼                         ▼
     ┌─────────────────┐      ┌─────────────────┐
     │   USB CDC       │      │     FatFS        │
     │  (Debug Out)    │      │    (SD Card)    │
     │   115200 baud   │      │   datos_N.csv   │
     └─────────────────┘      └─────────────────┘
```

### Apogee Detection State Machine

```
         ┌──────────┐
         │  IDLE   │ (Initial state, waiting for launch)
         └────┬────┘
              │ altitude > APOGEE_MIN_ALT (100m)
              │ velocity > 0 (ascent detected)
              ▼
         ┌──────────┐
         │ ASCENT  │ (Monitoring for peak)
         └────┬────┘
              │ velocity < 0 (descent detected)
              │ for APOGEE_WINDOW_SAMPLES (10) consecutive
              ▼
         ┌──────────┐
         │DETECTED │ (Parachute deployment triggered)
         └──────────┘
```

## 4. Key Design Decisions

### 4.1 Why STM32F411?

The STM32F411CE was selected for several reasons:

- **Price**: Inexpensive (< €5), widely available on BlackPill boards
- **Performance**: 96 MHz Cortex-M4F with FPU provides sufficient headroom for sensor processing at 50+ Hz
- **Peripherals**: Integrated USB OTG (no external crystal needed), SPI, I2C
- **Flash**: 512 KB flash sufficient for firmware and calibration storage
- **RAM**: 128 KB RAM adequate for dual-buffer CSV logging

**Alternative considered**: STM32F103 (BluePill) - Rejected due to lack of USB OTG and lower performance.

### 4.2 SPI vs I2C for Sensors

The IMU uses SPI while the barometer uses I2C. This decision was based on:

| Sensor | Protocol | Rationale |
|--------|----------|-----------|
| LSM6DSO | SPI | Higher bandwidth (up to 10 MHz), full-duplex, lower latency for high-rate IMU logging |
| MS5837 | I2C | Only I2C interface available, lower bandwidth acceptable for barometer (10-40 Hz) |

**Trade-off**: Using shared SPI for both sensors requires careful chip-select management, but eliminates the need for a second independent SPI bus.

### 4.3 Why FatFS for SD Card?

FatFS was selected over a raw logging approach for several reasons:

- **PC Compatibility**: CSV files can be read directly on any computer without specialized software
- **Recovery**: File system provides built-in integrity checking
- **Simplicity**: Easy to implement, well-tested, small footprint

**Trade-off**: FatFS has larger code footprint and occasional write latency spikes. For future high-rate data acquisition, consider switching to the `sdlog_raw` module.

### 4.4 Why CSV for Data Format?

CSV was chosen as the primary data format:

- **Universal**: No proprietary tools required
- **Debug-Friendly**: Human-readable during development
- **Import Support**: Direct import into Excel, Python (pandas), MATLAB

**Trade-off**: Text encoding is less space-efficient than binary. Consider `sdlog_raw` for binary format in future iterations.

## 5. Memory Layout

### Flash Memory Map

```
0x08000000  ┌────────────────────────────┐
            │  Main Flash (512 KB)      │
            │                          │
            │  Sector 0:   16 KB      │
            │  Sector 1:   16 KB      │
            │  Sector 2:   16 KB      │
            │  Sector 3:   16 KB      │
            │  Sector 4:   64 KB     │
            │  Sector 5:  128 KB ◄───┼── CONFIG_FLASH_CALIB_ADDR
            │  Sector 6:  128 KB     │
            │  Sector 7:  128 KB     │
            └────────────────────────────┘
```

### Calibration Storage (Sector 5)

| Offset | Size | Description |
|--------|------|-------------|
| 0x00 | 4 bytes | Magic: 0x43414C49 ("CALI") |
| 0x04 | 12 bytes | Accelerometer offsets (x, y, z) |
| 0x10 | 12 bytes | Gyroscope offsets (x, y, z) |
| 0x1C | Remaining | Reserved for future use |

### SRAM Memory Map

```
0x20000000  ┌────────────────────────────┐
            │  SRAM1 + SRAM2 (128 KB)  │
            │                          │
            │  Stack (down)            │
            │  ──────────────────────  │
            │  CSV Line Buffer         │
            │  FatFS work area        │
            │  Heap (up)              │
            └────────────────────────────┘
```

## 6. Timing and Loop Structure

### Main Loop Timing

The main loop operates at approximately 50 Hz (20 ms period):

```c
while (1) {
    // Read IMU (SPI)      ~500 us
    // Read Baro (I2C)    ~1 ms
    // Compute Altitude    ~50 us
    // Apogee Check      ~20 us
    // Format CSV        ~100 us
    // USB Transmit     ~500 us (blocking)
    // SD Write (if)   ~2-5 ms (deferred)

    HAL_Delay(20);      // Target: ~20 ms loop period
}
```

### Worst-Case Timing Budget

| Operation | Typical | Worst Case |
|-----------|---------|----------|
| IMU Read | 0.5 ms | 2 ms |
| Baro Read | 1.0 ms | 3 ms |
| CSV Format | 0.1 ms | 0.1 ms |
| USB Transmit | 0.5 ms | 5 ms |
| SD Write | 2 ms | 20 ms |
| **Total** | ~4 ms | ~30 ms |

**Note**: SD writes are deferred using FatFS `f_sync()` to avoid blocking the main loop. The actual write may complete in the background.

### Interrupt Architecture

| Interrupt | Priority | Purpose |
|-----------|----------|---------|
| SysTick | 15 (lowest) | System tick for `HAL_Delay()` |
| USB OTG | 10 (medium) | USB CDC data handling |
| SPI1 | 5 (high) | IMU data ready (if using DMA) |
| I2C1 | 5 (high) | Barometer data ready (if using DMA) |

The current implementation uses polling rather than interrupts to simplify development and ensure deterministic timing. Future iterations may migrate to DMA-driven I2C/SPI for lower CPU load.

## 7. Initialization Sequence

```
Power-On Reset
     │
     ▼
HAL_Init() ──── Configure systick, flash, low-level init
     │
     ▼
SystemClock_Config() ─── HSE 25 MHz → PLL → 96 MHz / 48 MHz USB
     │
     ▼
MX_GPIO_Init() ─────── Configure CS pins, pull-ups
     │
     ├──────────────────────────┐
     │                          │
     ▼                          ▼
MX_I2C1_Init()            MX_SPI1_Init()
(Barometer bus)           (IMU/SD bus)
     │                          │
     ▼                          ▼
USB CDC Init              SD Card Init
(VID/PID, CDC class)     (SPI low-speed first)
     │                          │
     ▼                          ▼
MS5837 Init              IMU Init
(Reset + PROM read)       (Register config)
     │                          │
     ▼                          ▼
Altitude Init            IMU Calibration
(base pressure)        (if configured)
     │
     ▼
Main Loop
(forever)
```

## 8. Error Handling Strategy

| Error | Detection | Recovery |
|-------|----------|---------|
| IMU not found | SPI timeout | Continue without IMU |
| Barometer error | I2C timeout | Continue without altitude |
| SD card not found | Init failure | Disable logging to SD |
| USB CDC disconnect | Not detected | Continue (buffered) |
| Flash write fail | Calibration write error | Use defaults |

The system is designed to continue operating with degraded functionality if individual sensors fail.