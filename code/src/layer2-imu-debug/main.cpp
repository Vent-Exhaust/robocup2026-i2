#include <Arduino.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire.begin();
  Wire.setClock(400000);

  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO085 — check wiring/address");
    while (1) delay(10);
  }
  Serial.println("BNO085 found");

  // ARVR stabilized rotation vector gives stable yaw (fused + game-rotation corrected)
  bno.enableReport(SH2_ARVR_STABILIZED_RV, 10000); // 10ms = 100Hz
}

// Convert quaternion to yaw in degrees
float quaternionToYaw(float r, float i, float j, float k) {
  // Yaw (rotation about Z axis)
  float sinYaw = 2.0f * (r * k + i * j);
  float cosYaw = 1.0f - 2.0f * (j * j + k * k);
  return atan2(sinYaw, cosYaw) * (180.0f / M_PI);
}

void loop() {
  if (!bno.getSensorEvent(&sensorValue)) return;

  if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
    float yaw = quaternionToYaw(
      sensorValue.un.arvrStabilizedRV.real,
      sensorValue.un.arvrStabilizedRV.i,
      sensorValue.un.arvrStabilizedRV.j,
      sensorValue.un.arvrStabilizedRV.k
    );
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" deg  (accuracy: ");
    Serial.print(sensorValue.status);
    Serial.println(")");
  }
}

// #include <Arduino.h>
// #include <Wire.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   while (!Serial); // Wait for Serial to be ready (Teensy)
//   Serial.println("I2C Scanner Starting...");
// }

// void loop() {
//   Serial.println("Scanning...");
//   int devices = 0;
//   for (uint8_t address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     uint8_t error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println();
//       devices++;
//     }
//   }
//   if (devices == 0) {
//     Serial.println("No I2C devices found.");
//   }
//   Serial.println("Scan complete.");
//   delay(5000); // Wait 5 seconds before next scan
// }

// #include <Arduino.h>

// void setup() {
//   Serial.begin(115200);
//   while (!Serial && millis() < 3000);
  
//   pinMode(19, OUTPUT);
//   pinMode(18, OUTPUT);
//   Serial.println("Toggling pin 19...");
// }

// void loop() {
//   digitalWrite(19, HIGH);
//   digitalWrite(18, LOW);
//   delayMicroseconds(100);
//   digitalWrite(19, LOW);
//   digitalWrite(18, HIGH);
//   delayMicroseconds(100);
// }

// #include <Arduino.h>
// #include <Wire.h>

// void setup() {
//   delay(1000);  // Wait for BNO085 to boot before touching the bus
//   Wire.begin();
//   Wire.setClock(100000);
//   Serial.begin(115200);
//   delay(500);
//   Serial.println("Scanning...");
// }

// void loop() {
//   Serial.println("Scanning...");
//   int devices = 0;
//   for (uint8_t address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     uint8_t error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("Found: 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//       devices++;
//     }
//   }
//   if (devices == 0) Serial.println("No devices found.");
//   delay(5000);
// }