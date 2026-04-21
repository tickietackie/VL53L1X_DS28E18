# VL53L1X_DS28E18

Arduino library to access the VL53L1X time-of-flight distance sensor through the DS28E18 1-Wire ↔ I2C bridge. Adapts the VL53L1X driver to run over a DS28E18 device (single or many on a OneWireBus) so you can place the sensor remotely on a simple 1-Wire network.

Overview
- Bridge: DS28E18 1-Wire to I2C bridge (requires a 1-Wire master such as Adafruit DS248x)
- Sensor: `VL53L1X` — ST time-of-flight distance sensor
- Modes: single DS28E18 device or multiple devices via `OneWireBus`

Highlights
- Use a remote VL53L1X over a 1-Wire bus (simple wiring, long runs)
- Full feature access: distance modes, timing budget, continuous and single-shot ranging
- Example sketches in `example/`

Requirements
- Arduino-compatible MCU
- Adafruit DS248x (or compatible) 1-Wire master
- DS28E18 1-Wire-to-I2C bridge
- VL53L1X sensor

Installation

PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/tickietackie/VL53L1X_DS28E18.git
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
MCU ──I2C──> DS2482 ──1-Wire──> DS28E18 ──I2C──> VL53L1X
```

Common initialization:

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

VL53L1X — Time-of-Flight Distance Sensor
----------------------------------------

Features
- Precise ranging (mm), selectable distance modes (`Short`, `Medium`, `Long`)
- Continuous and single-shot ranging modes
- Timing budget and inter-measurement period configuration

Quick Start

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
- VL53L1X example: [example/VL53L1X_DS28E18_Example/](example/VL53L1X_DS28E18_Example)

Contributing
- Bug reports, improvements and pull requests are welcome. Please follow repository contribution guidelines.

License
- MIT
