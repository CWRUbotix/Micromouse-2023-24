#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"

const int LIDAR_COUNT = 4;
const int LIDAR_ADDR_BASE = 0x50;

// GPIO pin numbers for the CS line on each lidar sensor
const int lidar_cs_pins[LIDAR_COUNT] = {28, 26, 27, 9};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];

#define ENCODER_LEFT_1 23
#define ENCODER_LEFT_2 22
#define ENCODER_RIGHT_1 1
#define ENCODER_RIGHT_2 2

#define MOTORLEFT_1 3
#define MOTORLEFT_2 19
#define MOTORRIGHT_1 18
#define MOTORRIGHT_2 4

double lidarFL = 0;
double lidarFR = 0;
double lidarBL = 0;
double lidarBR = 0;

Encoder rightEncoder(ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder(ENCODER_LEFT_1, ENCODER_LEFT_2);

const double wheelSeparation = 9.5;
const double wheelRadius = 3;
const double turnRatio = (wheelSeparation / 2.0) / wheelRadius / 360 * 380 * 12;
const double angleTolerance = 5;

const double lidarSeparation = 4;
const double distTolerance = 200;

double Input, Output, Setpoint;
PID turningPID(&Input, &Output, &Setpoint, 0.8, 0.1, 0.1, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);

  init_sensors();
  //turn(90);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void init_sensors() {
  /* // Reset all sensors
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }
  delay(10); */
  
  /* // "Unreset" sensors
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
  }
  delay(10); */

  // Disable all sensors except the first
  for (size_t i = 1; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }
    
  // Set address for each sensor
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
    lidar_sensors[i].begin(); // Ignoring return value!
    lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
    delay(10);
  }
}

void turn(double angle) {
  // If angle is negative (CW), turn right
  if(angle < 0) {
    turnRight(abs(angle));
  }
  // If angle is positive (CCW), turn left
  else {
    turnLeft(angle);
  }
  // Get error from side lidar sensors
  double error = getAngle();
  // If current angle is not within tolerance, perform an adjustment turn to compensate
  if(abs(error) > angleTolerance) {
    turn(error * -0.8);
  }
}

double getAngle() {
  // If left side is in range, use left measurements
  if(lidarFL < distTolerance && lidarBL < distTolerance) {
    return atan2(lidarBL - lidarFL, lidarSeparation);
  }
  // If right side is in range, use right measurements
  else if(lidarFR < distTolerance && lidarBR < distTolerance) {
    return atan2(lidarFR - lidarBR, lidarSeparation);
  }
  // If both sides are out of range, return 0
  else
    return 0;
}

void turnRight(double angle) {
  // Get baseline encoder reading
  int initial = leftEncoder.read();

  // Turn right wheel backwards
  analogWrite(MOTORRIGHT_1, 255);
  analogWrite(MOTORRIGHT_2, 127);

  // Turn left wheel forwards
  analogWrite(MOTORLEFT_1, 255);
  analogWrite(MOTORLEFT_2, 127);

  // Wait until necessary angle is reached
  while(leftEncoder.read() - initial < angle * turnRatio) {}

  // Stop both motors
  analogWrite(MOTORLEFT_1, 0);
  analogWrite(MOTORLEFT_2, 0);
  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 0);
}

void turnLeft(double angle) {
  // Get baseline encoder reading
  int initial = rightEncoder.read();

  // Turn right wheel backwards
  analogWrite(MOTORLEFT_1, 255);
  analogWrite(MOTORLEFT_2, 127);

  // Turn left wheel forwards
  analogWrite(MOTORRIGHT_1, 127);
  analogWrite(MOTORRIGHT_2, 255);

  // Wait until necessary angle is reached
  while(rightEncoder.read() - initial < angle * turnRatio) {}

  // Stop both motors
  analogWrite(MOTORLEFT_1, 0);
  analogWrite(MOTORLEFT_2, 0);
  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 0);
}

void turnRightWithPID(double angle) {
  // Set up PID
  Input = leftEncoder.read();
  Setpoint = angle * turnRatio;
  turningPID.SetMode(AUTOMATIC);
  // Get baseline encoder reading
  int initial = leftEncoder.read();

  // Wait until necessary angle is reached
  while(leftEncoder.read() - initial < angle * turnRatio - angleTolerance) {
    // Turn right wheel backwards
    analogWrite(MOTORRIGHT_1, 127 + turningPID.Compute() / 2);
    analogWrite(MOTORRIGHT_2, 127);

    // Turn left wheel forwards
    analogWrite(MOTORLEFT_1, 127);
    analogWrite(MOTORLEFT_2, 127 + turningPID.Compute() / 2);
  }

  // Stop both motors
  analogWrite(MOTORLEFT_1, 0);
  analogWrite(MOTORLEFT_2, 0);
  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 0);

  // Turn off PID
  turningPID.SetMode(MANUAL);
}

void turnLeftWithPID(double angle) {
  // Set up input
  Input = rightEncoder.read();
  Setpoint = angle * turnRatio;
  turningPID.SetMode(AUTOMATIC);
  // Get baseline encoder reading
  int initial = rightEncoder.read();

  // Wait until necessary angle is reached
  while(rightEncoder.read() - initial < angle * turnRatio - angleTolerance) {
    // Turn right wheel backwards
    analogWrite(MOTORLEFT_1, 127 + turningPID.Compute() / 2);
    analogWrite(MOTORLEFT_2, 127);

    // Turn left wheel forwards
    analogWrite(MOTORRIGHT_1, 127);
    analogWrite(MOTORRIGHT_2, 127 + turningPID.Compute() / 2);
  }

  // Stop both motors
  analogWrite(MOTORLEFT_1, 0);
  analogWrite(MOTORLEFT_2, 0);
  analogWrite(MOTORRIGHT_1, 0);
  analogWrite(MOTORRIGHT_2, 0);

  // Turn off PID
  turningPID.SetMode(MANUAL);
}

// P = (I1^2)*R1 + (I0^2 - 2*I0*I1+I1^2)*R2 -> P' = 2I1*R1 - 2I0*R2 + 2I1*R2 = 0 -> 2I1*R1 - 2(I0 - I1)*R2 = 2I1*R1 - 2I2*R2 = 0