# AVI Flight Computer

An embedded avionics system for model rockets, designed by GC_ULE (Grupo de Cohetería - Universidad Laboral de Errenteria). The system runs on an STM32F411 BlackPill microcontroller and provides high-rate IMU data logging (accelerometer + gyroscope), barometric altitude measurement, apogee detection, and SD card storage with CSV output.

## Hardware Requirements

| Component | Part | Interface | Notes |
|-----------|------|----------|-------|
| MCU | STM32F411CE (BlackPill) | - | 96 MHz ARM Cortex-M4F |
| IMU | LSM6DSO | SPI | 3-axis accelerometer + gyroscope |
| Barometer | MS5837-30BA | I2C | Pressure + temperature |
| Storage | microSD Card | SPI | FatFS for CSV data logging |
| Debug | USB CDC | - | Virtual COM port at 115200 baud |

## Quick Start

```bash
# Clone the repository
git clone https://github.com/your-repo/AVI_FlightComputer.git
cd AVI_FlightComputer

# Build the project
pio run

# Upload via DFU (ensure BOOT0 is HIGH, then press NRST)
pio run --target upload

# Monitor serial output
pio device monitor
```

## Key Commands

| Command | Description |
|---------|------------|
| `pio run` | Build the project |
| `pio run --target upload` | Upload firmware via DFU |
| `pio run --target erase` | Erase flash before uploading |
| `pio device monitor` | Open USB CDC serial monitor |
| `pio run --target clean` | Clean build artifacts |

## Architecture Overview

The flight software uses a simple ~50 Hz main loop that reads sensors, computes altitude and velocity, checks for apogee, formats CSV data, and writes to both USB CDC (debug) and SD card (FatFS).

```
Sensors (SPI/I2C) → Data Processing → Storage & Telemetry
       ↓                    ↓              ↓
  IMU + Baro          Altitude          SD Card
                    + Apogee          (FatFS)
                         ↓
                    USB CDC
                   (Debug Out)
```

For detailed architecture documentation, see [docs/architecture.md](docs/architecture.md).

For module reference documentation, see [docs/modules.md](docs/modules.md).

## Project Structure

```
├── src/                  # Main application and HAL/USB/FatFS sources
├── include/              # Configuration and HAL headers
├── lib/                  # Modular avionics libraries
│   ├── apogee/           # Apogee detection state machine
│   ├── altitude/         # Barometric altitude calculation
│   ├── imu/              # IMU driver with flash calibration
│   ├── ms5837/           # Barometer driver
│   ├── sd_spi/           # SD card SPI interface
│   ├── csv_logger/       # CSV formatting and debug output
│   ├── sdlog_raw/        # Raw page logger
│   └── osal/             # OS abstraction layer
├── docs/                 # Architecture and module documentation
└── platformio.ini       # PlatformIO build configuration
```

## Configuration

Edit `include/config.h` to enable/disable features:

| Flag | Description |
|------|-------------|
| `CONFIG_IMU_CALIBRATE` | Run IMU calibration on startup (stores offsets in flash) |
| `CONFIG_SERIAL_DEBUG` | Enable USB CDC debug output |
| `CONFIG_CDC` | Use USB CDC instead of hardware UART |

## Author

Developed by **Alejandro Gil Getino** for Grupo de Coheteria de la universidad de León.