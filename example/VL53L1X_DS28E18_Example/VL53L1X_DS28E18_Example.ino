#include <Wire.h>
#include <Adafruit_DS248x.h>
#include "DS28E18.h"
#include "VL53L1X_DS28E18.h"

// --- Instances ---
Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);
VL53L1X_DS28E18 vl53l1x(ds28e18);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("--- DS28E18 + VL53L1X Example ---");

  if (!ds2482.begin(&Wire, 0x18))
  {
    Serial.println("DS2482 not found!");
    while (1)
      ;
  }
  Serial.println("DS2482 initialized.");

  if (!ds28e18.begin(true))
  {
    Serial.println("DS28E18 init failed!");
    while (1)
      ;
  }

  // Configure for single sensor
  ds28e18.skipROM();

  // Configure GPIOs (I2C Pullups on DS28E18 side)
  if (!ds28e18.initializeGPIO())
  {
    Serial.println("DS28E18 GPIO Init Failed");
    while (1)
      ;
  }

  // Clear any POR flags
  ds28e18.resetDeviceStatus();


  Serial.println("Initializing VL53L1X...");

  // Set a generous timeout for the slow 1-Wire->I2C bridge
  vl53l1x.setTimeout(5000);


  if (!vl53l1x.begin())
  {
    Serial.println("VL53L1X init failed!");
    uint16_t mid = vl53l1x.getModelID();
    Serial.print("Model ID: 0x");
    Serial.println(mid, HEX);
    while (1)
      ;
  }

  uint16_t modelId = vl53l1x.getModelID();
  Serial.print("VL53L1X Model ID: 0x");
  Serial.println(modelId, HEX);
  Serial.println("VL53L1X initialized successfully!");


  // Configure Long distance mode, 50ms timing budget
  vl53l1x.setDistanceMode(VL53L1X_DS28E18::Long);
  vl53l1x.setMeasurementTimingBudget(50000);

  // Start continuous ranging at 100ms intervals
  vl53l1x.startContinuous(100);

  Serial.println("System Ready. Starting measurements...\n");
}

void loop()
{
  uint16_t distance = vl53l1x.read();
  uint8_t status = vl53l1x.getRangeStatus();

  if (vl53l1x.timeoutOccurred())
  {
    Serial.println("TIMEOUT");
    return;
  }

  Serial.print("Status: ");
  Serial.print(status);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.print(" mm");

  if (status == 0)
    Serial.print(" (OK)");
  else if (status == VL53L1X_DS28E18::SignalFail || status == VL53L1X_DS28E18::OutOfBoundsFail)
    Serial.print(" (low signal / out of range)");
  else
    Serial.print(" (noise/error)");

  Serial.print(" | Sig: ");
  Serial.print(vl53l1x.getSignalRate());
  Serial.print(" | Amb: ");
  Serial.println(vl53l1x.getAmbientRate());

  delay(100);
}
