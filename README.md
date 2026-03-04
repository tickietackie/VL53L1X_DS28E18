# Proximity_DS28E18

Arduino library to access I2C proximity and range sensors through the DS28E18 1-Wire ↔ I2C bridge. This package adapts popular sensor drivers to run over a DS28E18 device (single or many on a OneWireBus) so you can place I2C sensors remotely on a simple 1-Wire network.

Overview
- Bridge: DS28E18 1-Wire to I2C bridge (requires a 1-Wire master such as Adafruit DS248x)
- Supported sensors in this repository:
  - `VCNL4040` — proximity + ambient light sensor (VCNL4040_DS28E18)
  - `VL53L1X` — ST time-of-flight distance sensor (VL53L1X_DS28E18)
- Modes: single DS28E18 device or multiple devices via `OneWireBus`

Highlights
- Use remote I2C sensors over a 1-Wire bus (simple wiring, long runs)
- Full feature access for each supported sensor (configuration, measurement modes, interrupts)
- Example sketches for each sensor in `example/`

Requirements
- Arduino-compatible MCU
- Adafruit DS248x (or compatible) 1-Wire master
- DS28E18 1-Wire-to-I2C bridge
- Sensor(s): VCNL4040 and/or VL53L1X

Installation

PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/DanielSart/VCNL4040_DS28E18.git
  https://github.com/DanielSart/DS28E18_DS2482_Library.git
  adafruit/Adafruit DS248x @ ^1.0.1
```

Arduino IDE

1. Download this repository as ZIP
2. Sketch → Include Library → Add .ZIP Library
3. Also install dependencies: `DS28E18_DS2482_Library` and `Adafruit DS248x`

Quick Start — Common Setup

Basic wiring (I2C master → DS2482 → 1-Wire → DS28E18 → I2C sensor):

```
MCU ──I2C──> DS2482 ──1-Wire──> DS28E18 ──I2C──> <sensor>
```

Common initialization (example pattern used by both sensor examples):

```cpp
#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "DS28E18.h"

Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);

void setup_common() {
  Wire.begin();
  ds2482.begin(&Wire, 0x18);
  ds28e18.begin(true);
  ds28e18.skipROM();
  ds28e18.initializeGPIO();
  ds28e18.resetDeviceStatus();
}
```

VCNL4040 — Proximity & Ambient Light
------------------------------------

Features
- Proximity (0–65535), configurable integration time and resolution
- Ambient light (ALS) and white channel readings
- LED drive and duty cycle control, interrupt thresholds and persistence

Quick Start (VCNL4040)

```cpp
#include "VCNL4040_DS28E18.h"

VCNL4040_DS28E18 vcnl4040(ds28e18);

void setup() {
  setup_common();
  if (vcnl4040.begin()) {
    Serial.println("VCNL4040 Ready!");
    vcnl4040.powerOnAmbient();
  }
}

void loop() {
  Serial.print("Prox: ");
  Serial.print(vcnl4040.getProximity());
  Serial.print(" ALS: ");
  Serial.println(vcnl4040.getAmbient());
  delay(100);
}
```

API (selected)
- `begin()` — Initialize and configure defaults
- `isConnected()` / `getID()` — Device presence and ID (0x0186)
- `getProximity()` / `powerOnProximity()` / `setProxIntegrationTime()`
- `getAmbient()` / `powerOnAmbient()` / `setAmbientIntegrationTime()`
- `getWhite()` / `enableWhiteChannel()`

See `src/VCNL4040_DS28E18.h` for the complete API.

VL53L1X — Time-of-Flight Distance Sensor
----------------------------------------

Features
- Precise ranging (mm), selectable distance modes (`Short`, `Medium`, `Long`)
- Continuous and single-shot ranging modes
- Timing budget and inter-measurement period configuration

Quick Start (VL53L1X)

```cpp
#include "VL53L1X_DS28E18.h"

VL53L1X_DS28E18 tof(ds28e18);

void setup() {
  setup_common();
  if (tof.begin()) {
    Serial.println("VL53L1X Ready!");
    tof.setDistanceMode(VL53L1X_DS28E18::Short);
    tof.startContinuous(50); // 50 ms period
  }
}

void loop() {
  uint16_t dist = tof.read();
  Serial.print("Distance (mm): ");
  Serial.println(dist);
  delay(50);
}
```

API (selected)
- `begin()` / `isConnected()` / `getModelID()` (should be 0xEACC)
- `setDistanceMode()` / `getDistanceMode()`
- `setMeasurementTimingBudget()` / `getMeasurementTimingBudget()`
- `startContinuous()` / `stopContinuous()`
- `read()` / `readSingle()` / `dataReady()`

See `src/VL53L1X_DS28E18.h` for the complete API.

Examples
- VCNL4040 example: [example/VCNL4040_DS28E18_Example/](example/VCNL4040_DS28E18_Example)
- VCNL4040 simple example: [example/VCNL4040_DS28E18_Simple_Example/](example/VCNL4040_DS28E18_Simple_Example)
- VL53L1X example: [example/VL53L1X_DS28E18_Example/](example/VL53L1X_DS28E18_Example)

Contributing
- Bug reports, improvements and pull requests are welcome. Please follow repository contribution guidelines.

License
- MIT
