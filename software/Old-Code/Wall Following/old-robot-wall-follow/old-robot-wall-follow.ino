// Includes
// Wire.h for I2C communication with TCA chip and LiDAR
#include <Wire.h>
// Arduino standard encoder library
#include <Encoder.h>
// Adafruit library for VL6180X LiDAR sensor
#include "Adafruit_VL6180X.h"
// PID library for control loops
#include <PID_v1.h>

// Pin definitions
#define ENCODER_RIGHT_1 40
#define ENCODER_RIGHT_2 41
#define ENCODER_LEFT_1 22
#define ENCODER_LEFT_2 23

#define MOTORLEFT_1 3
#define MOTORLEFT_2 2
#define MOTORRIGHT_1 4
#define MOTORRIGHT_2 5

//typedef enum motor enum_motor;
// enum Motor {LEFT_MOTOR, RIGHT_MOTOR};
int LEFT_MOTOR = 0;
int RIGHT_MOTOR = 1;

// The physical distance between the sensors
#define LIDAR_SEPERATION 74 // 74 mm between sensors

// Squares are 10in by 10in, but we work in mm. 10in = 254mm
#define SQUARE_SIZE 254

// Create encoders and sensors
Adafruit_VL6180X lidar = Adafruit_VL6180X();

Encoder rightEncoder (ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder (ENCODER_LEFT_1, ENCODER_LEFT_2);

// Global variables to store the last seen values for different sensors
// Remember: these are in mm! (They're accurate to +/- 5mm)
// If there's space in front of them, they cap at 255.
uint8_t back_right;
uint8_t front_right;
// TODO: other sensors when we have them

// Spin forever blinking 13 quickly
void abort () {
  setMotor(LEFT_MOTOR, 0);
  setMotor(RIGHT_MOTOR, 0);

  while (1 == 1) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
}

// Spin forever blinking slowly
void success () {
  setMotor(LEFT_MOTOR, 0);
  setMotor(RIGHT_MOTOR, 0);

  while (1 == 1) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
}

void updateSensors () {
  selectSensor(0);
  back_right = lidar.readRange();
  selectSensor(1);
  front_right = lidar.readRange();
}

// TODO: we probably need to account for accumulated error as a result of being not in the center

void selectSensor(uint8_t i){
  if(i > 7)
    return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setMotor (int m, int8_t power) {
    if (m == LEFT_MOTOR) {
        // TODO: double check that 0, 0 is break (and not free spin) for motors

        // These values can be tuned
        if (power < 12 && power > -12) {
          // Serial.printf("Power %d; Writing LEFT motor 1 and 2 to 255\n", power);
          analogWrite(MOTORLEFT_1, 255);
          analogWrite(MOTORLEFT_2, 255);
        }else if (power > 0) {
          // Serial.printf("Power %d; Writing LEFT motor 1 to 255 and left motor 2 to %d\n", power, 255 - power * 2);
          analogWrite(MOTORLEFT_1, 255);
          analogWrite(MOTORLEFT_2, 255 - power * 2);
        }else if (power < 0) {
          // Serial.printf("Power %d; Writing LEFT motor 1 to %d and left motor 2 to 255\n", power, 255 + power * 2);
          analogWrite(MOTORLEFT_1, 255 + power * 2);
          analogWrite(MOTORRIGHT_2, 255);
        }
    }else if (m == RIGHT_MOTOR) {
        if (power < 12 && power > -12) {
          // Serial.printf("Power %d; Writing RIGHT motor 1 and 2 to 255\n", power);
          analogWrite(MOTORRIGHT_1, 255);
          analogWrite(MOTORRIGHT_2, 255);
        }else if (power > 0) {
          // Serial.printf("Power %d; Writing RIGHT motor 1 to 255 and right motor 2 to %d\n", power, 255 - power * 2);
          analogWrite(MOTORRIGHT_1, 255);
          analogWrite(MOTORRIGHT_2, 255 - power * 2);
        }else if (power < 0) {
          // Serial.printf("Power %d; Writing RIGHT motor 1 to %d and left motor 2 to 255\n", power, 255 + power * 2);
          analogWrite(MOTORRIGHT_1, 255 + power * 2);
          analogWrite(MOTORRIGHT_2, 255);
        }
    }
}

double timestamps[1000], goalAngles[1000], currentAngles[1000], angularVelocitys[1000], goalDistances[1000], currentDistances[1000], velocitys[1000];
int count = 0;

void moveOneForward () {
    // Create a PID controller for angle
    double goalAngle, currentAngle, angularVelocity;
    goalAngle = 0;
    // When the angle is 0.1, we need to bump left power by like 5, so diff of 2
    // (When reading with just the right sensors, like right now, a positive angle means that we're turned left)
    // TODO: increase angle gain
    PID anglePID(&currentAngle, &angularVelocity, &goalAngle, 20.0, 0, 0, DIRECT);
    anglePID.SetOutputLimits(-127, 127);
    anglePID.SetSampleTime(10);
    anglePID.SetMode(AUTOMATIC);


    int leftRevs = leftEncoder.read();
    int rightRevs = rightEncoder.read();
    double oldDistance = (leftRevs + rightRevs) / 2.0 / 360.0 * PI * 60.0;

    // Create a PID controller for speed
    // (currentDistance is the distance inside the current square. It resets to 0 at the end of )
    double goalDistance, currentDistance, velocity;
    goalDistance = oldDistance + SQUARE_SIZE; // I hope this is mm
    // Okay so with I and D as zero, with a distance of 254 (one square), we want a speed of around 25.4, so P should be around 0.1
    // The itegral part compensates for steady state errors. Do we have any steady state errors?
    // The derivative term minimizes overshoot.
    // 0.25 is workable, overshoots a tiny bit
    PID positionPID(&currentDistance, &velocity, &goalDistance, 0.5, 0, 0, DIRECT);
    // The library outputs a value between 0 and 255, by default
    positionPID.SetOutputLimits(20, 70);
    positionPID.SetMode(AUTOMATIC);
    positionPID.SetSampleTime(10);

    double back_right_avg = back_right;
    double front_right_avg = front_right;

    // loop quickly
    while (1) {
        // update currentDistance and currentAngle
        {
            // read LiDAR
            updateSensors();

            // low pass filter
            back_right_avg = back_right_avg * 0.9 + back_right * 0.1;
            front_right_avg = front_right_avg * 0.9 + front_right * 0.1;

            // do math
            // arctan((lidarDistanceBL - lidarDistanceFL) / lidarSeparation);
            // For right now, we only have one pair of sensors, so we only get one angle
            currentAngle = atan2(front_right_avg - back_right_avg, LIDAR_SEPERATION);

            // Update current distance
            // rev / 360 is num revolutions
            // num revolutions * pi * diameter (Zach says 60mm)
            int leftRevs = leftEncoder.read();
            int rightRevs = rightEncoder.read();
            currentDistance = (leftRevs + rightRevs) / 2.0 / 360.0 * PI * 60.0;
        }

        // check if currentDistance and currentAngle are within tolerance
        if (abs(currentDistance - goalDistance) < 20) {
          break;
        }
        
        // update PID
        anglePID.Compute(); // updates angularVelocity
        positionPID.Compute(); // updates velocity

        setMotor(LEFT_MOTOR, -angularVelocity / 2.0 + 30);
        setMotor(RIGHT_MOTOR, angularVelocity / 2.0 + 30);

        // update motor values
        // int angularVelocityLeft = -angularVelocity / 2.0;
        // int angularVelocityRight = angularVelocity / 2.0;

        // angularVelocityLeft += velocity;
        // angularVelocityRight += velocity;
        // // int angularVelocityLeft = velocity;
        // // int angularVelocityRight = velocity;
        // setMotor(LEFT_MOTOR, angularVelocityLeft);
        // setMotor(RIGHT_MOTOR, angularVelocityRight);

        if (count > 999) {
          abort();
        }

        if (count < 1000) {
          goalAngles[count] = goalAngle;
          currentAngles[count] = currentAngle;
          angularVelocitys[count] = angularVelocity;
          goalDistances[count] = goalDistance;
          currentDistances[count] = currentDistance;
          velocitys[count] = velocity;
          timestamps[count] = millis();
          count++;
        }

        if (Serial.available()) {
          Serial.printf("Dumping data. Count was at %d.\n", count);
          for (int i = 0; i < count; i++) {
            Serial.printf("%d %f %f %f %f %f %f\n", millis(), goalAngles[i], currentAngles[i], angularVelocitys[i], goalDistances[i], currentDistances[i], velocitys[i]);
          }

          abort();
        }

        delay(10);
    }
}

void setup () {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // Start serial
    Serial.begin(115200);
    Serial.println("Serial ready!");

    // Starts I2C on the default pins (18 (SDA), 19 (SCL))
    // (I think, I can't find docs on it)
    Wire.begin();
    Serial.println("I2C ready!");

    // Setup LiDARs (only 0 and 1, on the right side for now)
    for (uint8_t k = 0; k < 2; k++) {
        selectSensor(k);

        if (!lidar.begin()) {
            // Serial.printf("Failed to find sensor: %d", k);
            abort();
        }
    }
    Serial.println("LiDAR sensors ready!");

    // Setup motors
    pinMode(MOTORLEFT_1, OUTPUT);
    pinMode(MOTORLEFT_2, OUTPUT);
    pinMode(MOTORRIGHT_1, OUTPUT);
    pinMode(MOTORRIGHT_2, OUTPUT);

    setMotor(RIGHT_MOTOR, 0);
    setMotor(LEFT_MOTOR, 0);
    Serial.println("Motors ready!");

    // Get initial values for sensors
    updateSensors();

    moveOneForward();

    setMotor(LEFT_MOTOR, 0);
    setMotor(RIGHT_MOTOR, 0);

    while (!Serial.available());

    Serial.printf("Dumping data. Count was at %d.\n", count);
    for (int i = 0; i < count; i++) {
      Serial.printf("%d %f %f %f %f %f %f\n", timestamps[i], goalAngles[i], currentAngles[i], angularVelocitys[i], goalDistances[i], currentDistances[i], velocitys[i]);
    }

    abort();
}

void loop () {
    // We don't have any looping code
    delay(100000);
}
