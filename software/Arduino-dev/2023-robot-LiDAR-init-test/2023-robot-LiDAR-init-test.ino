#include <Wire.h>
#include "Adafruit_VL6180X.h"

const int LIDAR_COUNT = 4;
const int LIDAR_ADDR_BASE = 0x50;

// GPIO pin numbers for the CS line on each lidar sensor
const int lidar_cs_pins[LIDAR_COUNT] = {28, 26, 27, 9};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];

void setup() {
  Serial.begin(9600);

  initSensors();
}

void loop() {
  // put your main code here, to run repeatedly:
  size_t i = 0;
  for (; i < LIDAR_COUNT; ++i) {
    uint8_t range = lidar_sensors[i].readRange();
    uint8_t status = lidar_sensors[i].readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
      Serial.print(i);
      Serial.print("] Range: ");
      Serial.println(range);
    }
    else {
      Serial.print(i);
      Serial.print("] Error: ");
      Serial.println(status);
    }
  }
}

void initSensors() {
  // Setup CS pins
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    pinMode(lidar_cs_pins[i], OUTPUT);
  }

  // Reset all sensors
  // for (size_t i = 0; i < LIDAR_COUNT; ++i) {
  //   digitalWrite(lidar_cs_pins[i], LOW);
  // }
  // delay(10);

  // // "Unreset" sensors
  // for (size_t i = 0; i < LIDAR_COUNT; ++i) {
  //   digitalWrite(lidar_cs_pins[i], HIGH);
  // }
  // delay(10);

  // Disable all sensors except the first
  for (size_t i = 1; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }

  // Set address for each sensor
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
    if (!lidar_sensors[i].begin(&Wire2)) {
      Serial.print("Failed init on sensor ");
      Serial.println(i);
    }
    else {
      lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
    }
    delay(10);
  }
}
