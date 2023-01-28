//Notes:
//---360 Degrees in one revolution for Encoders (Old Robot)

#include <Wire.h>
#include <Encoder.h>

#include "Adafruit_VL6180X.h"

#define ENCODER_RIGHT_1 40
#define ENCODER_RIGHT_2 41
#define ENCODER_LEFT_1 22
#define ENCODER_LEFT_2 23

Adafruit_VL6180X vl = Adafruit_VL6180X();

Encoder rightEncoder(ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder(ENCODER_LEFT_1, ENCODER_LEFT_2);

uint8_t i = 0;

void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  Wire.begin();
  // Wire.beginTransmission(0x70);
  // Wire.write(1);
  // Wire.endTransmission();

  // selectSensor(1);

  // Serial.println("Adafruit VL6180x test!");
  // if (! vl.begin()) {
  //   Serial.println("Failed to find sensor");
  //   while (1);
  // }

  // selectSensor(0);

  // Serial.println("Adafruit VL6180x test!");
  // if (! vl.begin()) {
  //   Serial.println("Failed to find sensor");
  //   while (1);
  // }

  for(int k = 0; k < 8; k++)
  {
    selectSensor(k);

    Serial.println("Adafruit VL6180x test!");
    if (! vl.begin()) {
      Serial.println("Failed to find sensor");
      while (1);
    }
  }

  Serial.println("Sensor found!");
}

void loop() {
  // float lux = vl.readLux(VL6180X_ALS_GAIN_5);


  
  // Serial.print("Left: "); Serial.println(leftEncoder.read());
  // Serial.print("Right: "); Serial.println(rightEncoder.read());

  // uint8_t range = vl.readRange();
  // uint8_t status = vl.readRangeStatus();

  // if (status == VL6180X_ERROR_NONE) {
  //   Serial.print("Range: "); Serial.println(range);
  // }


  selectSensor(i % 8);
  Serial.print("Lidar " + (i % 8)); Serial.println(vl.readRange());


  // selectSensor(1);
  // Serial.print("Lidar 2: "); Serial.println(vl.readRange());
  // Some error occurred, print it out!
  
  // if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
  //   Serial.println("System error");
  // }
  // else if (status == VL6180X_ERROR_ECEFAIL) {
  //   Serial.println("ECE failure");
  // }
  // else if (status == VL6180X_ERROR_NOCONVERGE) {
  //   Serial.println("No convergence");
  // }
  // else if (status == VL6180X_ERROR_RANGEIGNORE) {
  //   Serial.println("Ignoring range");
  // }
  // else if (status == VL6180X_ERROR_SNR) {
  //   Serial.println("Signal/Noise error");
  // }
  // else if (status == VL6180X_ERROR_RAWUFLOW) {
  //   Serial.println("Raw reading underflow");
  // }
  // else if (status == VL6180X_ERROR_RAWOFLOW) {
  //   Serial.println("Raw reading overflow");
  // }
  // else if (status == VL6180X_ERROR_RANGEUFLOW) {
  //   Serial.println("Range reading underflow");
  // }
  // else if (status == VL6180X_ERROR_RANGEOFLOW) {
  //   Serial.println("Range reading overflow");
  // }
  delay(500);
}

void selectSensor(uint8_t i){

  if(i < 7)
    return;

    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();

}