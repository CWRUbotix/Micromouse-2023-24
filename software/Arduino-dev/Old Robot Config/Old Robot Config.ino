//Notes:
//---360 Degrees in one revolution for Encoders (Old Robot)

#include <Wire.h>
#include <Encoder.h>

#include "Adafruit_VL6180X.h"

#define ENCODER_RIGHT_1 40
#define ENCODER_RIGHT_2 41
#define ENCODER_LEFT_1 22
#define ENCODER_LEFT_2 23

#define MOTORLEFT_1 2
#define MOTORLEFT_2 3
#define MOTORRIGHT_1 4
#define MOTORRIGHT_2 5

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

  for(uint8_t k = 0; k < 2; k++)
  {
    selectSensor(k);

    Serial.println("Adafruit VL6180x test!");
    if (! vl.begin()) {
      Serial.print("Failed to find sensor: "); Serial.println(k);
      while (1);
    }
  }
  Serial.println("Sensor found!");


  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);

  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 255);

  delay(2000);

  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 0);
}

void loop() {

  // Serial.print("Left: "); Serial.println(leftEncoder.read());
  // Serial.print("Right: "); Serial.println(rightEncoder.read());

  // uint8_t range = vl.readRange();
  // uint8_t status = vl.readRangeStatus();

  // if (status == VL6180X_ERROR_NONE) {
  //   Serial.print("Range: "); Serial.println(range);
  // }

  selectSensor(0);
  Serial.print("Lidar 1: "); Serial.println(vl.readRange());

  selectSensor(1);
  Serial.print("Lidar 2: "); Serial.println(vl.readRange());


 
  delay(500);
}

void selectSensor(uint8_t i){

  if(i > 7)
    return;

    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();


}