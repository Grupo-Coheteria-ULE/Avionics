# AVI Flight Computer - Module Reference

This document provides detailed reference documentation for each library module in the `lib/` directory.

## 1. Apogee Detection (`lib/apogee`)

### Purpose

Implements a state machine that detects the rocket's apogee (maximum altitude) by monitoring barometric altitude and vertical velocity. The detection algorithm requires sustained negative velocity (descent) for a configurable number of consecutive samples before triggering the apogee event.

### Data Structure

```c
typedef struct {
    apogee_state_t state;           // IDLE, ASCENT, DETECTED
    uint8_t pos_count;          // Consecutive descent samples
    int32_t last_altitude_mm;   // Last altitude reading
    int32_t last_velocity_mm_s; // Last velocity reading
} apogee_ctx_t;
```

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `APOGEE_MIN_ALT_MM` | 100000 | Minimum altitude required to enter ASCENT state (100 m) |
| `APOGEE_WINDOW_SAMPLES` | 10 | Consecutive descent samples needed for detection |

### Public API

| Function | Description |
|----------|-------------|
| `void apogee_init(apogee_ctx_t *ctx)` | Initializes the apogee detection context. Resets state to IDLE. |
| `bool apogee_update(apogee_ctx_t *ctx, uint32_t dt_ms, int32_t altitude_mm, int32_t velocity_mm_s)` | Updates detection with new sensor data. Returns `true` if apogee detected in this update. |
| `apogee_state_t apogee_get_state(const apogee_ctx_t *ctx)` | Returns the current state (IDLE, ASCENT, or DETECTED). |

### State Machine Flow

```
IDLE ──(alt > 100m && vel > 0)──► ASCENT ──(vel < 0 for 10 samples)──► DETECTED
```

### Dependencies

- None (standalone module)

---

## 2. Altitude Calculation (`lib/altitude`)

### Purpose

Computes relative altitude from barometric pressure using the international barometric formula. Also calculates vertical velocity by differentiating altitude over time.

### Barometric Formula

The altitude is computed using the standard formula:

```
altitude = 44330 * (1 - (pressure / base_pressure)^0.1903)
```

Results are scaled to millimeters for higher precision.

### Data Structure

```c
typedef struct {
    float base_pressure_pa;      // Baseline pressure in Pascals
    int32_t last_altitude_mm;     // Last computed altitude
    int32_t last_velocity_mm_s;   // Last computed velocity
    uint32_t last_timestamp_ms;   // Last update timestamp
    bool initialized;            // Initialization flag
} altitude_ctx_t;
```

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `ALTITUDE_EXPONENT` | 0.1903 | Barometric formula exponent |
| `ALTITUDE_SCALE_M` | 44330 | Scale factor for meters |
| `ALTITUDE_SCALE_MM` | 44330000 | Scale factor for millimeters |

### Public API

| Function | Description |
|----------|-------------|
| `void altitude_init(altitude_ctx_t *ctx, float pressure_pa)` | Initializes with baseline pressure. |
| `int32_t altitude_update(altitude_ctx_t *ctx, float pressure_pa, uint32_t now_ms)` | Updates altitude and computes velocity. Returns altitude in mm. |
| `int32_t altitude_get(const altitude_ctx_t *ctx)` | Returns the last computed altitude in mm. |
| `int32_t altitude_get_velocity(const altitude_ctx_t *ctx)` | Returns the last vertical velocity in mm/s. |

### Dependencies

- None (standalone module)

---

## 3. IMU Driver (`lib/imu`)

### Purpose

Provides a generic IMU driver with support for the LSM6DSO (SPI). Implements sensor initialization, data reading, and calibration with flash-backed offset storage. The module includes a calibration routine that computes accelerometer and gyroscope offsets from stationary samples and persists them to flash memory.

### Data Types

```c
// Raw sensor data
typedef struct {
    float ax, ay, az;       // Linear acceleration [g]
    float wx, wy, wz;       // Angular velocity [deg/s]
    float temperature;       // Temperature [deg C]
} imu_raw_t;

// Calibration data stored in flash
typedef struct {
    uint32_t magic;         // Validation: 0x43414C49
    float acc_off[3];      // Accelerometer offsets
    float gyro_off[3];     // Gyroscope offsets
} imu_cal_t;
```

### Public API

| Function | Description |
|----------|-------------|
| `int imu_init(void)` | Initializes IMU hardware (configures registers). Returns 0 on success. |
| `int imu_read(imu_raw_t *out)` | Reads a single sample with calibration applied. Returns 0 on success. |
| `int imu_calibrate(uint32_t num_samples)` | Runs calibration (blocking). Stores results in flash if successful. |
| `int imu_load_cal(void)` | Loads calibration from flash. Returns 0 if valid data found. |
| `int imu_save_cal(void)` | Saves current calibration to flash. |
| `const imu_cal_t *imu_get_cal(void)` | Returns pointer to current calibration data. |

### Calibration Process

1. Collect `num_samples` while sensor is stationary
2. Compute average accelerometer offsets (X, Y should be ~0, Z should be ~1g)
3. Compute average gyroscope offsets (all should be ~0)
4. Store results in flash sector 5

### Dependencies

- `lib/osal` (SPI abstraction)
- Flash HAL functions (for calibration storage)

---

## 4. Barometer Driver (`lib/ms5837`)

### Purpose

Driver for the MS5837-30BA pressure sensor. The MS5837 is a high-resolution barometric pressure sensor with I2C interface, commonly used in aerospace applications.

### Data Types

```c
// Barometer readings
typedef struct {
    float pressure;         // Pressure in Pascals
    float temperature;     // Temperature in degrees C
} ms5837_data_t;

// Driver instance
typedef struct {
    void *i2c_bus;         // osal_i2c_t* (opaque)
    uint16_t prom[7];       // PROM calibration coefficients
    uint32_t last_D1;      // Raw pressure ADC
    uint32_t last_D2;      // Raw temperature ADC
    bool initialized;
} ms5837_t;
```

### Public API

| Function | Description |
|----------|-------------|
| `int ms5837_init(ms5837_t *dev, void *i2c_bus)` | Initializes the barometer (reset + reads PROM). Returns 0 on success. |
| `int ms5837_read(ms5837_t *dev, ms5837_data_t *out)` | Reads pressure and temperature. Returns 0 on success. |

### Measurement Process

1. Send conversion command (D1 for pressure, D2 for temperature)
2. Wait for conversion (typical: 2.5 ms for 4096 oversampling)
3. Read raw ADC values
4. Apply PROM calibration coefficients to compute pressure/temperature

### Dependencies

- `lib/osal` (I2C abstraction)

---

## 5. SD Card SPI Driver (`lib/sd_spi`)

### Purpose

Provides block-level access to SD cards operating in SPI mode. Supports SDSC and SDHC cards with 512-byte logical sector interface. Designed to sit above the OSAL SPI abstraction and below filesystems or raw loggers.

### Features

- SD card initialization in SPI mode
- SDSC and SDHC detection
- Single/multi-block read and write
- Sector count from CSD register
- Optional ACMD23 pre-erase hint

### Data Types

```c
typedef enum {
    SD_SPI_OK,
    SD_SPI_ERR,
    SD_SPI_TIMEOUT,
    SD_SPI_PARAM,
    SD_SPI_NOCARD,
    SD_SPI_CRC_ERR
} sd_spi_status_t;

typedef enum {
    SD_SPI_CARD_UNKNOWN = 0,
    SD_SPI_CARD_SDSC = 1,
    SD_SPI_CARD_SDHC = 2
} sd_spi_card_t;

typedef struct {
    osal_spi_t *bus;         // SPI bus abstraction
    uint32_t timeout_ms;     // Operation timeout
    sd_spi_card_t card_type;
    uint32_t sector_count;
    bool initialized;
} sd_spi_t;
```

### Public API

| Function | Description |
|----------|-------------|
| `void sd_spi_attach(sd_spi_t *sd, osal_spi_t *bus, uint32_t timeout_ms)` | Attaches driver to SPI bus. |
| `sd_spi_status_t sd_spi_init(sd_spi_t *sd)` | Initializes SD card in SPI mode. |
| `sd_spi_status_t sd_spi_read(sd_spi_t *sd, uint32_t lba, void *buf, uint32_t count)` | Reads sectors. |
| `sd_spi_status_t sd_spi_write(sd_spi_t *sd, uint32_t lba, const void *buf, uint32_t count)` | Writes sectors. |
| `sd_spi_status_t sd_spi_preerase(sd_spi_t *sd, uint32_t nblocks)` | Sends pre-erase hint. |
| `uint32_t sd_spi_get_sector_count(const sd_spi_t *sd)` | Returns sector count. |
| `sd_spi_card_t sd_spi_get_card_type(const sd_spi_t *sd)` | Returns card type. |

### Initialization Sequence

1. Configure SPI for low speed (100-400 kHz)
2. Send ≥74 clock pulses with CS high
3. CMD0 (GO_IDLE_STATE)
4. CMD8 (SEND_IF_COND)
5. ACMD41 loop (SD_SEND_OP_COND)
6. CMD58 (READ_OCR) - check CCS bit
7. Optional CMD16 (SET_BLOCKLEN)
8. Read CSD for capacity
9. Switch to high speed

### Dependencies

- `lib/osal` (SPI abstraction)

---

## 6. CSV Logger (`lib/csv_logger`)

### Purpose

Formats sensor data as CSV lines and provides debug output to USB CDC. This is the primary interface between the sensor acquisition pipeline and both the debug serial output and SD card logging.

### Data Types

```c
#define CSV_LINE_MAX_LEN 128

typedef struct {
    uint32_t timestamp_ms;
    imu_raw_t imu;           // ax, ay, az, wx, wy, wz
    float temperature;        // from barometer
    float pressure;          // from barometer
} csv_sample_t;
```

### CSV Format

```
t;ax;ay;az;wx;wy;wz;T;p
```

| Column | Field | Units |
|--------|-------|-------|
| t | Timestamp | milliseconds |
| ax | Acceleration X | g |
| ay | Acceleration Y | g |
| az | Acceleration Z | g |
| wx | Angular velocity X | deg/s |
| wy | Angular velocity Y | deg/s |
| wz | Angular velocity Z | deg/s |
| T | Temperature | deg C |
| p | Pressure | Pa |

### Public API

| Function | Description |
|----------|-------------|
| `int csv_format_line(const csv_sample_t *sample, char *buf)` | Formats sample as CSV. Returns length. |
| `void csv_debug_init(void)` | Initializes serial debug backend. |
| `void csv_debug_print(const csv_sample_t *sample)` | Sends CSV line over USB CDC. |

### Configuration

The CSV separator is configured in `include/config.h`:

```c
#define CONFIG_CSV_SEP ';'
```

### Dependencies

- `lib/imu` (sensor data types)

---

## 7. Raw Page Logger (`lib/sdlog_raw`)

### Purpose

Implements a deterministic raw page logger for high-rate data acquisition on SD cards. Unlike the CSV/FatFS approach, this module writes fixed-size binary pages to optimize write performance and reduce latency.

### Features

- Fixed-size on-disk pages (configurable, typically 4096 bytes)
- Multi-buffer RAM staging (double-buffering)
- Optional wrap-around logging
- Optional superblock-based resume after reset
- Optional CRC generation
- Optional ACMD23 pre-erase

### Data Types

```c
typedef struct {
    uint32_t meta_lba;          // Superblock LBA (0 = disabled)
    uint32_t data_lba;          // First data LBA
    uint32_t end_lba;          // End of data area
    uint32_t page_bytes;        // Page size (multiple of 512)
    void *buffers;             // External RAM buffers
    uint32_t num_buffers;      // Number of buffers
    bool wrap;                // Enable wrap-around
    bool use_crc;             // Enable CRC
    bool use_preerase;         // Enable ACMD23
    uint32_t meta_period_pages;// Superblock commit period
} sdlog_cfg_t;

typedef struct {
    sd_spi_t *sd;
    sdlog_cfg_t cfg;
    uint32_t page_sectors;
    uint32_t payload_cap;
    // ... runtime state
} sdlog_t;
```

### Page Header (32 bytes)

```
Offset  Size  Field
0x00    4    Magic (0x53444C47 "SDLG")
0x04    4    Version
0x08    8    Timestamp t0
0x10    4    Sequence number
0x14    4    Payload bytes
0x18    4    CRC32 (optional)
0x1C    16   Reserved
```

### Public API

| Function | Description |
|----------|-------------|
| `sdlog_status_t sdlog_init(sdlog_t *log, sd_spi_t *sd, const sdlog_cfg_t *cfg)` | Initializes logger. |
| `sdlog_status_t sdlog_append(sdlog_t *log, const void *data, uint32_t len, uint64_t t0)` | Appends to buffer (non-blocking). |
| `sdlog_status_t sdlog_poll(sdlog_t *log, uint32_t max_pages, uint32_t *pages_written)` | Writes ready pages to SD. |
| `sdlog_status_t sdlog_flush(sdlog_t *log, uint32_t max_pages)` | Closes and flushes. |
| `uint32_t sdlog_get_dropped_bytes(const sdlog_t *log)` | Returns dropped bytes count. |

### Dependencies

- `lib/sd_spi` (SD card interface)
- `lib/osal` (for optional delay_ms)

---

## 8. OS Abstraction Layer (`lib/osal`)

### Purpose

Provides a thin abstraction layer over STM32 HAL services. Reduces direct dependencies between high-level modules and HAL, improving readability, portability, and testability.

### Modules

- **Time Utilities**: `osal_delay_ms()`, `osal_ticks_ms()`
- **I2C Abstraction**: Blocking I2C read/write with register support
- **SPI Abstraction**: Full-duplex SPI with chip-select callbacks
- **UART Abstraction**: Blocking and interrupt-driven UART

### Data Types

```c
// I2C wrapper
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint32_t timeout_ms;
} osal_i2c_t;

// SPI wrapper
typedef struct {
    void *hspi;
    uint32_t timeout_ms;
    void (*cs_select)(void *user);
    void (*cs_deselect)(void *user);
    void *cs_user;
} osal_spi_t;

// UART wrapper
typedef struct {
    void *huart;
    uint32_t timeout_ms;
} osal_uart_t;
```

### Status Codes

| Code | Description |
|------|-------------|
| `OSAL_OK` | Success |
| `OSAL_ERR` | Generic error |
| `OSAL_TIMEOUT` | Timeout |
| `OSAL_E_NULL` | Null pointer |
| `OSAL_E_PARAM` | Invalid parameter |

### Public API

#### Time

| Function | Description |
|----------|-------------|
| `void osal_delay_ms(uint32_t ms)` | Blocking delay. |
| `uint32_t osal_ticks_ms(void)` | Get tick count. |

#### I2C

| Function | Description |
|----------|-------------|
| `void osal_i2c_init(osal_i2c_t *bus, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)` | Initialize wrapper. |
| `osal_status_t osal_i2c_write(osal_i2c_t *bus, uint16_t addr8, const uint8_t *data, uint16_t len)` | Raw write. |
| `osal_status_t osal_i2c_read(osal_i2c_t *bus, uint16_t addr8, uint8_t *data, uint16_t len)` | Raw read. |
| `osal_status_t osal_i2c_reg_read(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, uint8_t *data, uint16_t len)` | Register read. |
| `osal_status_t osal_i2c_reg_write(osal_i2c_t *bus, uint16_t addr8, uint8_t reg, const uint8_t *data, uint16_t len)` | Register write. |

#### SPI

| Function | Description |
|----------|-------------|
| `void osal_spi_init(osal_spi_t *bus, void *hspi, uint32_t timeout_ms, cs_select, cs_deselect, cs_user)` | Initialize wrapper. |
| `osal_status_t osal_spi_txrx(osal_spi_t *bus, const uint8_t *tx, uint8_t *rx, uint16_t len)` | Full-duplex transfer. |
| `osal_status_t osal_spi_reg_read(osal_spi_t *bus, uint8_t reg, uint8_t *data, uint16_t len)` | Register read. |
| `osal_status_t osal_spi_reg_write(osal_spi_t *bus, uint8_t reg, const uint8_t *data, uint16_t len)` | Register write. |

#### UART

| Function | Description |
|----------|-------------|
| `void osal_uart_init(osal_uart_t *bus, void *huart, uint32_t timeout_ms)` | Initialize wrapper. |
| `osal_status_t osal_uart_write(osal_uart_t *bus, const uint8_t *data, uint16_t len)` | Blocking write. |
| `osal_status_t osal_uart_read(osal_uart_t *bus, uint8_t *data, uint16_t len)` | Blocking read. |
| `osal_status_t osal_uart_write_it(osal_uart_t *bus, const uint8_t *data, uint16_t len)` | Interrupt write. |
| `osal_status_t osal_uart_read_it(osal_uart_t *bus, uint8_t *data, uint16_t len)` | Interrupt read. |

### Dependencies

- STM32 HAL (hardware abstraction)
- `main.h` (STM32 device definitions)

---

## Module Dependency Graph

```
                    ┌────────────────┐
                    │   main.c      │
                    └───────┬────────┘
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
    ┌──────────┐       ┌──────────┐       ┌──────────┐
    │  imu   │       │ ms5837  │       │ apogee  │
    └────┬───┘       └────┬───┘       └────┬───┘
         │                │                │
         ▼                ▼                ▼
    ┌──────────┐       ┌──────────┐       ┌──────────┐
    │  osal  │       │  osal  │       │altitude│
    └────┬───┘       └────┬───┘       └────┬───┘
         │                │                │
    ┌────▼───┐       ┌────▼───┐       ▼
    │ sd_spi │       │sd_spi │   (stand-alone)
    └────┬───┘       └────┬───┘
         │                │
    ┌────▼──────────────────────┐
    │     csv_logger           │
    │     sdlog_raw          │
    └───────────────────────┘
```

### Summary Table

| Module | Purpose | Key Functions | Dependencies |
|--------|--------|---------------|--------------|
| `apogee` | Apogee detection | `apogee_init()`, `apogee_update()` | None |
| `altitude` | Altitude calculation | `altitude_init()`, `altitude_update()` | None |
| `imu` | IMU driver | `imu_init()`, `imu_read()`, `imu_calibrate()` | `osal` |
| `ms5837` | Barometer driver | `ms5837_init()`, `ms5837_read()` | `osal` |
| `sd_spi` | SD card SPI | `sd_spi_init()`, `sd_spi_read()`, `sd_spi_write()` | `osal` |
| `csv_logger` | CSV formatting | `csv_format_line()`, `csv_debug_print()` | `imu` |
| `sdlog_raw` | Raw page logger | `sdlog_init()`, `sdlog_append()`, `sdlog_poll()` | `sd_spi` |
| `osal` | HAL abstraction | `osal_i2c_*()`, `osal_spi_*()`, `osal_uart_*()` | None (HAL) |