#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "micromouse_pins_2023.h"

const int LIDAR_COUNT = 4;
const int LIDAR_ADDR_BASE = 0x50;

// GPIO pin numbers for the CS line on each lidar sensor
const int lidar_cs_pins[LIDAR_COUNT] = {28, 26, 27, 9};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];

typedef enum motor_t {
  LEFT_MOTOR = 0,
  RIGHT_MOTOR
} motor_t;

const int POWER_DEADBAND = 6;

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

double initial, output, target;
PID turningPID(&initial, &output, &target, 0.05, 0, 0, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);

  initSensors();
  turn(90);
}

void loop() {
  // put your main code here, to run repeatedly:
}

/** 
 * Reset all lidar sensors and set addresses
 */
void initSensors() {
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

/**
 * Convert a value in range [-128..127] to a motor power value
 * 
 * @param p The input power [-128..127]
 * @return The output power [0..255]
 */
uint8_t convertPower(int8_t p) {
    if (p == 0) {
        return 255;
    }
    if (p < 0) {
        if (p == -128) {
            p = -127;
        }
        p = -p;
    }
    return 255 - (((uint8_t)p) * 2);
}

/**
 * Set motor power in range [-128..127] for a specified motor
 * 
 * @param m The motor to modify
 * @param power The power and direction of the motor [-128..127]
 *              Positive is "forward"
 *              Negative is "backward"
 */
void setMotor (motor_t m, int8_t power) {
  int m1, m2;

  // Determine motor
  switch (m) {
  case LEFT_MOTOR:
    m1 = MOTORLEFT_1;
    m2 = MOTORLEFT_2;
    break;
  case RIGHT_MOTOR:
    m1 = MOTORRIGHT_1;
    m2 = MOTORRIGHT_2;
    break;
  default:
    return;
  }

  // Set power
  if (power < POWER_DEADBAND && power > -POWER_DEADBAND) {
    analogWrite(m1, 255);
    analogWrite(m2, 255);
  }
  else if (power > 0) {
    analogWrite(m1, convertPower(power));
    analogWrite(m2, 255);
  }
  else {
    analogWrite(m1, 255);
    analogWrite(m2, convertPower(power));
  }
}

/** 
 * Turn robot by a given angle (in degrees)
 * 
 * @param angle The angle to turn (in degrees)
 *              Positive is CCW
 *              Negative is CW
 */
void turn(double angle) {
  // If angle is negative (CW), turn right
  if(angle < 0) {
    turnRightWithPID(abs(angle));
  }
  // If angle is positive (CCW), turn left
  else {
    turnLeftWithPID(angle);
  }
  // Get error from side lidar sensors
  double error = getAngle();
  // If current angle is not within tolerance, perform an adjustment turn to compensate
  if(abs(error) > angleTolerance) {
    turn(error * -0.8);
  }
}

/**
 * Get angle of robot from center line of maze path
 *
 * @return The angle of the robot from the center line of the maze path
 *         Positive is CCW
 *         Negative is CW
 */
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

/**
 * Turn robot right (CW) by a given positive angle (in degrees) without PID
 * 
 * @param angle The angle to turn (in degrees)
 */
void turnRight(double angle) {
  // Get baseline encoder reading
  int initial = leftEncoder.read();

  // Turn right wheel backwards
  setMotor(RIGHT_MOTOR, -128);

  // Turn left wheel forwards
  setMotor(LEFT_MOTOR, 127);

  // Wait until necessary angle is reached
  while(leftEncoder.read() - initial < angle * turnRatio) {}

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

/**
 * Turn robot left (CCW) by a given positive angle (in degrees) without PID
 * 
 * @param angle The angle to turn (in degrees)
 */
void turnLeft(double angle) {
  // Get baseline encoder reading
  int initial = rightEncoder.read();

  // Turn right wheel backwards
  setMotor(LEFT_MOTOR, -128);

  // Turn left wheel forwards
  setMotor(RIGHT_MOTOR, 127);

  // Wait until necessary angle is reached
  while(rightEncoder.read() - initial < angle * turnRatio) {}

  // Stop both motors
  setMotor(LEFT_MOTOR, 0);
  setMotor(RIGHT_MOTOR, 0);
}

/**
 * Turn robot right (CW) by a given positive angle (in degrees) using PID
 * 
 * @param angle The angle to turn (in degrees)
 */
void turnRightWithPID(double angle) {
  // Set up PID
  initial = leftEncoder.read();
  target = angle * turnRatio;
  turningPID.SetMode(AUTOMATIC);

  // Wait until necessary angle is reached
  while(abs(leftEncoder.read() - initial) < target - angleTolerance) {
    // Update PID
    turningPID.Compute();

    // Turn right wheel backwards
    setMotor(RIGHT_MOTOR, output / -2);

    // Turn left wheel forwards
    setMotor(LEFT_MOTOR, output / 2);
  }

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);

  // Turn off PID
  turningPID.SetMode(MANUAL);
}

/**
 * Turn robot left (CCW) by a given positive angle (in degrees) using PID
 * 
 * @param angle The angle to turn (in degrees)
 */
void turnLeftWithPID(double angle) {
  // Set up PID
  initial = rightEncoder.read();
  target = angle * turnRatio;
  turningPID.SetMode(AUTOMATIC);

  // Wait until necessary angle is reached
  while(abs(rightEncoder.read() - initial) < target - angleTolerance) {
    // Update PID
    turningPID.Compute();
    
    // Turn left wheel backwards
    setMotor(LEFT_MOTOR, output / -2);

    // Turn right wheel forwards
    setMotor(RIGHT_MOTOR, output / 2);
  }

  // Stop both motors
  setMotor(LEFT_MOTOR, 0);
  setMotor(RIGHT_MOTOR, 0);

  // Turn off PID
  turningPID.SetMode(MANUAL);
}
