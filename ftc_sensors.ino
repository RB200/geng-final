#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// PD-controlled ball-on-rail controller. Uses an IIRF to remove some noise, but more filtering is needed

const uint8_t DISTANCE_REGISTER = 0x24;
const uint8_t SENSOR_ADDRESS = 0x08;
double length = 0.22; // meters
double xDes = length/2; 
double lastX = 0;

double Kp = 400;
double Kd = 125;

static double x = xDes;
Servo servo;

bool readDistanceMm(uint8_t address, uint32_t &distanceMm) {
  Wire.beginTransmission(address);
  Wire.write(DISTANCE_REGISTER);

  // Keep bus control between setting the register pointer and reading the bytes.
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesRequested = 4;
  uint8_t bytesRead = Wire.requestFrom(address, bytesRequested);
  if (bytesRead != bytesRequested) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  uint8_t data[4];
  for (uint8_t i = 0; i < bytesRequested; i++) {
    data[i] = Wire.read();
  }

  uint32_t rawDistance = ((uint32_t)data[0])
                       | ((uint32_t)data[1] << 8)
                       | ((uint32_t)data[2] << 16)
                       | ((uint32_t)data[3] << 24);

  // The Waveshare protocol defines distance as 24-bit millimeters. Masking keeps
  // this usable even if the fourth byte is status/reserved on a firmware variant.
  distanceMm = rawDistance & 0x00FFFFFFUL;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  servo.attach(39);
}

static unsigned long lastTimeMs = 0;

void loop() {
  uint32_t distanceMm = 0;
  unsigned long nowMs = millis();

  if (lastTimeMs == 0) {
    lastTimeMs = nowMs;
    return;
  }

  double dt = (nowMs - lastTimeMs) / 1000.0;
  if (dt <= 0) {
    return;
  }


  if (readDistanceMm(SENSOR_ADDRESS, distanceMm)) { // if there's a valid reading

    x = 0.5 * (distanceMm / 1000.0) + 0.5 * x; // IIR filter, need more filtering

    double error = xDes - x; // compute error
    double xd = (x - lastX) / dt; // compute velocity (extremely noisy bc not enough filtering)
    Serial.print("Distance (mm): ");
    Serial.println(distanceMm);
    double theta = 1613 + (Kp * error) - (Kd * xd); // compute desired servo value using a PD controller w/ center position at 1613 us.

    Serial.println(theta);


    lastX = x; // update last time and pos for velocity calculation
    lastTimeMs = nowMs;
    theta = constrain(theta, 1550,1670); // constrain so platform doesn't overextend
    Serial.println(theta);

    servo.writeMicroseconds(theta);
  }

  delay(100);
}