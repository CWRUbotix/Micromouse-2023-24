/* --- Includes --- */
// Wire.h for I2C communication with TCA chip and LiDAR
#include <Wire.h>
// Arduino standard encoder library
#include <Encoder.h>
// Adafruit library for VL6180X (short range) and VL53L0X (long ) LiDAR sensor
#include "Adafruit_VL6180X.h"
#include "Adafruit_VL53L0X.h"
// Adafruit library for VL53L0X (long range)
// Pin definitions
#include "micromouse_pins_2023.h"
// Robot Logic and Algorithm definitions
#include "Algo/FMicro.cpp"
/* ---- Defines ---- */
typedef enum motor_t {
    LEFT_MOTOR = 0,
    RIGHT_MOTOR = 1
} motor_t;

// #define TEST
#undef log
#undef logf
#undef logln

#ifdef TEST
  #define log(...) Serial.print(__VA_ARGS__)
  #define logf(...) Serial.printf(__VA_ARGS__)
  #define logln(...) Serial.println(__VA_ARGS__)
  #define LOGGING 1
#else
  #define log(...)
  #define logf(...)
  #define logln(...)
  #define LOGGING 0
#endif

#define POWER_DEADBAND 12

#define LIDAR_COUNT 5
#define LONG_RANGE_LIDAR_COUNT 1
#define LIDAR_ADDR_BASE 0x50

// The physical distance between the sensors
// TODO: Chech that values are consistent with new robot
#define LIDAR_SEPARATION_FB 74.5 // 74.5 mm between sensors front to back
#define LIDAR_SEPARATION_LR 70 // 70 mm between sensors across robot

int ANGLE_TOLERANCE = 0;
int speed = 32;

const double encoderTicks = 12;
const double gearRatio = 75;
const double wheelSeparation = 7.5; // 7.5 cm between wheels
const double wheelRadius = 1.72; // 3.44 cm diameter
const double turnRatio = (wheelSeparation / 2.0) / wheelRadius / 360 * gearRatio * encoderTicks; // degree to encoder tick conversion ratio

// The LiDAR sensors return a running average of readings,
//  so when we move past a wall, the LiDAR returns a value greater than the previous value but less than an overflow.
// (After a certain amount of time, the running average overflows the maximum and only then does the LiDAR throw a read error)
// If the LiDAR is greater than this value, we assume that it's not sensing the wall.
#define SENSOR_RANGE_MAX 110
#define LONG_RANGE_SENSOR_RANGE_MAX 110 // TODO: Check whether this is accurate

// When centered, there should be ~60mm in front of the front sensor
#define LIDAR_FRONT_TARGET 60

// Squares are 10in by 10in, but we work in mm. 10in = 254 mm
#define SQUARE_SIZE 254

/* ---- User Variables ---- */

// GPIO pin numbers for the CS line on each LiDAR sensor
// TODO: Check inputs are correct
const int lidar_cs_pins[LIDAR_COUNT + LONG_RANGE_LIDAR_COUNT] = {LIDAR_FrontShort, LIDAR_FrontLeft, LIDAR_FrontRight, LIDAR_BackLeft, LIDAR_BackRight, LIDAR_FrontLong};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];
Adafruit_VL53L0X long_range_lidar_sensors[LONG_RANGE_LIDAR_COUNT];

bool forward_errored, front_left_errored, front_right_errored, back_left_errored, back_right_errored, long_range_errored;
uint8_t forward;
uint8_t front_left;
uint8_t front_right;
uint8_t back_left;
uint8_t back_right;
uint8_t long_range;

Encoder rightEncoder (ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder (ENCODER_LEFT_1, ENCODER_LEFT_2);

/**
 * Convert a value in range [-127..127] to a motor power value
 *
 * @param p The input power [-127..127]
 * @return Output power [0..255]
 */
uint8_t convertPower(int8_t p) {
  if (p == 0) {
    return 255;
  }
  if (p < 0) {
    p = -p;
  }
  return 255 - (((uint8_t) p) * 2);
}

/**
 * Set motor power for a specified motor
 *
 * @param m The motor to modify
 * @param power The power and direction of the motor
 *              (range: [-127..127])
 *              Positive is "forward"
 *              Negative is "backward"
 */
void setMotor (motor_t m, int power) {
  if (power < -127) {
    power = -127;
  }else if (power > 127) {
    power = 127;
  }
  int m1, m2;

  // Determine motor
  if (m == LEFT_MOTOR) {
    m1 = MOTORLEFT_1;
    m2 = MOTORLEFT_2;
  }else if (m == RIGHT_MOTOR) {
    m1 = MOTORRIGHT_1;
    m2 = MOTORRIGHT_2;
  }else {
    return;
  }
  // Set power
  if (power < POWER_DEADBAND && power > -POWER_DEADBAND) {
      analogWrite(m1, 255);
      analogWrite(m2, 255);
  } else if (power > 0) {
      analogWrite(m1, convertPower(power));
      analogWrite(m2, 255);
  } else {
      analogWrite(m1, 255);
      analogWrite(m2, convertPower(power));
  }
}

int wallLeft() {
  front_left = lidar_sensors[3].readRange();
  logf("Left: %d\n", !(lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX));
  return !(lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX);
}

int wallRight() {
  front_right = lidar_sensors[2].readRange();
  logf("Right: %d\n", !(lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX));
  return !(lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX);
}

int wallFront() {
  logf("Front: %d\n", !(lidar_sensors[5].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX));
  return !(lidar_sensors[5].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX);
}

// TODO: Update to check number of squares using SQUARE_SIZE
// TODO: Update for specific lidar technology
int numSquares() {
  long_range = lidar_sensors[6].readRange();
  if(!(lidar_sensors[6].readRangeStatus() != VL53L0X_ERROR_NONE || long_range > LONG_RANGE_SENSOR_RANGE_MAX));
    return long_range / SQUARE_SIZE;
  return 0;
}

void updateSensors () {
  // Read the right LIDAR sensors and update their values
  back_right = lidar_sensors[1].readRange();
  back_right_errored = lidar_sensors[1].readRangeStatus() != VL6180X_ERROR_NONE || back_right > SENSOR_RANGE_MAX;
  front_right = lidar_sensors[2].readRange();
  front_right_errored = lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX;

  // Read the left LIDAR sensors and update their values
  back_left = lidar_sensors[0].readRange();
  back_left_errored = lidar_sensors[0].readRangeStatus() != VL6180X_ERROR_NONE || back_left > SENSOR_RANGE_MAX;
  front_left = lidar_sensors[3].readRange();
  front_left_errored = lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX;

  // Read the front short LIDAR sensor and update their value
  forward = lidar_sensors[4].readRange();
  forward_errored = lidar_sensors[4].readRangeStatus() != VL6180X_ERROR_NONE || forward > SENSOR_RANGE_MAX;

  // Read the long range LIDAR sensor and update their value
  long_range = lidar_sensors[5].readRange();
  long_range_errored = long_range_lidar_sensors[0].readRangeStatus() != VL53L0X_ERROR_NONE || long_range > LONG_RANGE_SENSOR_RANGE_MAX;
}

// p_controller(80.0, currentAngle, 0, -127.0, 127.0);
double p_controller(double p, double current, double goal, double min, double max) {
  double out = (goal - current) * p;
  if (out > max) {
    out = max;
  }else if (out < min) {
    out = min;
  }
  return out;
}

/**
 * Counter-clockwise is a positive angle
 * @return angle given by lidar sensors
 */

double getAngle()
{
  // arctan((lidarDistanceBL - lidarDistanceFL) / lidarSeparation);
  // Average left and right sensors
  double leftAngle = -atan2(front_left - back_left, LIDAR_SEPARATION_FB);
  double rightAngle = atan2(front_right - back_right, LIDAR_SEPARATION_FB);
  // logf("left angle: %f\tright angle: %f; ", leftAngle * 180.0 / PI, rightAngle * 180.0 / PI);

  if ((back_left_errored || front_left_errored) && (back_right_errored || front_right_errored)) {
    // If we have no good data, assume we're going straight
    // logf("Using 0 as angle\n");
    return 0;
  } else if (back_left_errored || front_left_errored) {
    // logf("Using right angle\n");
    return rightAngle;
  } else if (back_right_errored || front_right_errored) {
    // logf("Using left angle\n");
    return leftAngle;
  } else {
    // logf("Averaging angles\n");
    return (leftAngle + rightAngle) / 2;
  }
}

/**
 * Turn robot by a given angle in place
 *
 * @param angle     The angle to turn (in degrees)
 * @param direction The direction to turn (LEFT or RIGHT)
 */
void turn(double angle, turning_direction_t direction) {
  // Encoder to turn
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  // target point
  double target = angle * turnRatio;

  // Direction constant
  int dir = 1;

  if (direction == LEFT)
  {
    // Turn left
    turnEncoder = &rightEncoder;
    otherTurnEncoder = &leftEncoder;
  } else {
    // Turn righ
    turnEncoder = &leftEncoder;
    otherTurnEncoder = &rightEncoder;
    dir = -1;
  }

  turnEncoder->write(0);
  otherTurnEncoder->write(0);

  // Turn right wheel backwards if left, forwards if right
  // Scale by 0.7 to compensate for over-volted motors
  setMotor(RIGHT_MOTOR, 45.125 * dir * 0.7);
  // Turn left wheel forwards if left, backwards if right
  setMotor(LEFT_MOTOR, -45.125 * dir * 0.7);

  // Turn until within margin of error

  int encoderAverage;
  do {
    encoderAverage = (turnEncoder->read() - otherTurnEncoder->read()) / 2;
  } while (turnEncoder->read() < target - ANGLE_TOLERANCE);

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

/**
 * Turn robot by a given angle along a circular path
 * 
 * @param angle     The angle to turn (in degrees)
 * @param direction The direction to turn (LEFT/RIGHT)
 */
void movingTurn(double angle, turning_direction_t direction) {
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  double turnRatio = (SQUARE_SIZE + wheelSeparation) / 2.0 / wheelRadius / 360 * gearRatio * encoderTicks; // degree to encoder tick conversion ratio

  double target = angle * turnRatio;

  double FAST_SPEED = 45.125 * (SQUARE_SIZE + wheelSeparation) / wheelSeparation * 0.35;
  double SLOW_SPEED = 45.125 * (SQUARE_SIZE - wheelSeparation) / wheelSeparation * 0.35;

  if(direction == LEFT){
    turnEncoder = &rightEncoder;
    otherTurnEncoder = &leftEncoder;
    target = target * 10.5 / 12; // correction
    turnEncoder->write(0);
    otherTurnEncoder->write(0);
    setMotor(RIGHT_MOTOR, FAST_SPEED);
    setMotor(LEFT_MOTOR, SLOW_SPEED);
  }
  else{
    turnEncoder = &leftEncoder;
    otherTurnEncoder = &rightEncoder;
    target = target * 9 / 12; // correction
    turnEncoder->write(0);
    otherTurnEncoder->write(0);
    setMotor(RIGHT_MOTOR, SLOW_SPEED);
    setMotor(LEFT_MOTOR, FAST_SPEED);
  }

  do {
    // wait
  } while (turnEncoder->read() < target - ANGLE_TOLERANCE);

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

/*
void movingTurn(double angle, turning_direction_t dir){
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  leftEncoder.write(0);
  rightEncoder.write(0);

  double ratio =  (SQUARE_SIZE/10 + wheelSeparation)/(SQUARE_SIZE/10 - wheelSeparation);
  double max = 50;
  int target = 1800;


  double FAST_SPEED = max*ratio/(ratio + 1);
  double SLOW_SPEED = max*1/(ratio + 1);
  
  if(dir == LEFT){
    turnEncoder = &rightEncoder;
    otherTurnEncoder = &leftEncoder;
    setMotor(RIGHT_MOTOR, FAST_SPEED);
    setMotor(LEFT_MOTOR, SLOW_SPEED);
  }
  else{
    turnEncoder = &leftEncoder;
    otherTurnEncoder = &rightEncoder;
    setMotor(RIGHT_MOTOR, SLOW_SPEED);
    setMotor(LEFT_MOTOR, FAST_SPEED);
  }
  int encoderAverage = 0;
    do {
      encoderAverage = (turnEncoder->read() - otherTurnEncoder->read()) / 2;
    } while (encoderAverage < target);

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}*/

// Rotate 90 degrees right
void turnRight(){
  logln("Turning Right");
  turn(90.0 + getAngle() * 180.0 / PI, RIGHT);
}

// Rotate 90 degrees left
void turnLeft(){
  logln("Turning Left");
  turn(90.0 - getAngle() * 180.0 / PI, LEFT);
}

// Rotate 90 degrees right while moving forward
void movingTurnRight(){
  logln("Moving Turn Right");
  movingTurn(90.0 + getAngle() * 180.0 / PI, RIGHT);
}

// Rotate 90 degrees left while moving forward
void movingTurnLeft(){
  logln("Moving Turn Left");
  movingTurn(90.0 - getAngle() * 180.0 / PI, LEFT);
}

// Rotate 45 degrees right
void turnRight45(){
  turn(45.0 + getAngle() * 180.0 / PI, RIGHT);
}

// Rotate 45 degrees left
void turnLeft45(){
  turn(45.0 - getAngle() * 180.0 / PI, LEFT);
}

void turn180(){
  turn(180.0 - getAngle() * 180.0 / PI, LEFT);
}

// Moves the robot forward 1 square in the direction the robot is currently facing
int moveForward(int number) {
  // Reset encoders
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Create angle variables
  double currentAngle, angularVelocity;

  // Create speed variables
  // (currentDistance is the distance inside the current square.)
  double currentDistance = 0;
  double goalDistance = SQUARE_SIZE * number;
  double velocity;

  // Center variables
  double centerVelocity, centerOffset;

  // loop quickly
  while (1) {
    // update currentDistance and currentAngle
    {
      // read LiDARs
      updateSensors();
      // Update angle
      currentAngle = getAngle();

      // Update current distance
      // 4560 ticks per revolution (380:1 gearbox * 12 ticks per rev normally)
      // num revolutions * pi * diameter
      long leftRevs = leftEncoder.read();
      long rightRevs = rightEncoder.read();
      // ((Num ticks of both wheels / 2) / num ticks per revolution) * PI * diameter 
      // Becomes revolutions * mm per revolution
      currentDistance = (((leftRevs + rightRevs) / 2.0) / (gearRatio * encoderTicks)) * PI * 2 * wheelRadius;
    }

    // logf("Moving forward. current: %d, ultra: %d, goal: %d, cond: %d, %d\n", currentDistance, ultrasonic, goalDistance, ultrasonic < 150, ultrasonic > 95);

    // We're also not allowed to break out of the loop (stop going forward), if we're farther than 95 mm from a wall
    // If we think we're there, but we're not, go farther
    if (currentDistance >= goalDistance && long_range < 150 && long_range > 95) {
      logf("Moving goalDistance forward.\n");
      // Increase goal distance such that the long_range ends up (60mm) away from the wall in front of us
      goalDistance += long_range - LIDAR_FRONT_TARGET;
      redLights();
    }

    // check if currentDistance and currentAngle are within tolerance
    // For the lidar, 60 is 60 mm from the wall. This is about
    // the distance when the robot is centered in the tile
    if (currentDistance >= goalDistance || (!long_range_errored && long_range < LIDAR_FRONT_TARGET)) {
      setMotor(LEFT_MOTOR, 0);
      setMotor(RIGHT_MOTOR, 0);
      logf("Stopped. Long Range Lidar: %d, %d, %lf\n", !long_range_errored, long_range < LIDAR_FRONT_TARGET, long_range);
      if(digitalRead(RED_LED) == HIGH)
        digitalWrite(RED_LED, LOW);
      greenLights();
      greenLights();
      break;
    }
 
    // How far away from the center we are
    // Right is positive
    // (ASCII art by Zach)
    // |              | <--  MAZE    |
    // |              |   CENTERLINE |
    // |              |              |
    // |      ERROR   |              |
    // |           \  |              |
    // |           |<>|              |
    // |       +-------+ <--- X ---> |
    // |       | FRONT |             |
    // |       |L  |  R|             |
    // |       |   |   |             |
    // |       | ROBOT |             |
    // |       +-------+             |
    // |       |<- R ->|             |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // | <------ MAZE WIDTH -------> |
    //
    //              MAZE WIDTH - R
    // ERROR = X - ----------------
    //                    2
    centerOffset = (double)front_right - (double)front_left;
    if (front_left_errored && front_right_errored) {
        // logf("both errored, setting offset to 0\n");
        centerOffset = 0;
    }else if (front_left_errored) {
        // If we don't have a left value
        // (We're targeting to an offset of 0)
        // sensors are 84 mm apart, maze is 240mm wide
        centerOffset = (double)front_right - (240 - LIDAR_SEPARATION_LR) / 2.0;
    }else if (front_right_errored) {
        centerOffset = (240 - LIDAR_SEPARATION_LR) / 2.0 - (double)front_left;
    }

    // logf("left %d; right %d: center offset: %f\n", front_left, front_right, centerOffset);

    // p, current, goal, min, max
    // When the angle is 0.1, we need to bump left power by like 5, so P of 20
    // (A positive angle means that we're turned left)
    angularVelocity = p_controller(80.0, currentAngle, 0, -127.0, 127.0);

    // With a distance of 254 (one square), we've chose a P of 12.25
    //  so it saturates velocity for the majority of the distance
    velocity = p_controller(12.25, currentDistance, goalDistance, -speed, speed);

    // With a center off set of 10mm, that's a velocity of 5
    centerVelocity = p_controller(0.5, centerOffset, 0, -50, 50);

    angularVelocity += centerVelocity;

    // NOTE: We're assuming that at angles close to 0, angularVelocity has a linear relationship with velocity.

    // update motor values
    // Scale by 0.7 to compensate for over-volted motors
    int velocityLeft = (int)(-angularVelocity / 2.0 * 0.7) + velocity;
    int velocityRight = (int)(angularVelocity / 2.0 * 0.7) + velocity;

    setMotor(LEFT_MOTOR, velocityLeft);
    setMotor(RIGHT_MOTOR, velocityRight);
  }
  return 0;
}

/* ---- SETUP ---- */
void setup(void) {
  // Debug led on the board itself
  pinMode(DEBUG_LED, OUTPUT);

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(DEBUG_LED, HIGH);

  // Start serial
  Serial.begin(115200);
  logln("Serial ready!");

  // Starts I2C on the default pins (18 (SDA), 19 (SCL))
  // (I think, I can't find docs on it)
  I2C_LIDAR.begin();
  logln("I2C ready!");

  // Setup LiDARs
  // short range lidars
  for (size_t i = 1; i < LIDAR_COUNT + LONG_RANGE_LIDAR_COUNT; i++) {
    pinMode(lidar_cs_pins[i], OUTPUT);
  }
  // Disable all sensors except the first
  for (size_t i = 2; i < LIDAR_COUNT + LONG_RANGE_LIDAR_COUNT; i++) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }

  // Set address for each sensor
  // Write the CS line high (turning it on)
  // Set the address
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
    // Pass pointer to the Wire2 object since we're running on I2C bus 2
    if (!lidar_sensors[i].begin(&I2C_LIDAR)) {
      log("Failed init on sensor ");
      logln(i);
    } else {
      lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
    }
  }
  logln("LiDAR sensors ready!");

  // Setup motors
  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_2, OUTPUT);

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
  logln("Motors ready!");

  pinMode(START_BUTTON, INPUT_PULLUP);
  int t = 0;
  // Spin until start button is pressed
  // t is ms
  // On for 300 (0-300) off for 500 (300-800)
  while(!digitalRead(START_BUTTON)) {
    if (t == 0) {
      digitalWrite(YELLOW_LED, HIGH);
    }else if (t == 300) {
      digitalWrite(YELLOW_LED, LOW);
    }

    t = (t + 10) % 800;
    delay(10);
  }
  digitalWrite(YELLOW_LED, LOW);

  delay(20); // Switch "debounce"
  
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  long pressTime = millis();
  long endPressTime = pressTime;
  while (digitalRead(START_BUTTON)) {
    /* spin, waiting for button release */
    endPressTime = millis();
    if (endPressTime - pressTime > 1000) {
      speed = 25;
      digitalWrite(LED2, LOW);
    }
    if (endPressTime - pressTime > 3000) {
      speed = 40;
      digitalWrite(LED1, LOW);
    }
  } 
  if (endPressTime - pressTime < 1000) {
  }else if (endPressTime - pressTime < 3000) {
  }else {
  }
  delay(500);
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  initialize();
}

void coolLights(){
  if(digitalRead(BLUE_LED) == LOW)
    digitalWrite(BLUE_LED, HIGH);
  else
    digitalWrite(BLUE_LED, LOW);
    delay(500);
}

void redLights(){
  if(digitalRead(RED_LED) == LOW)
    digitalWrite(RED_LED, HIGH);
  else
    digitalWrite(RED_LED, LOW);
    delay(10);
}

void greenLights(){
  if(digitalRead(GREEN_LED) == LOW)
    digitalWrite(GREEN_LED, HIGH);
  else
    digitalWrite(GREEN_LED, LOW);
    delay(50);
}
/* ---- MAIN ---- */
void loop() {
// updateSensors();
doRun();
}