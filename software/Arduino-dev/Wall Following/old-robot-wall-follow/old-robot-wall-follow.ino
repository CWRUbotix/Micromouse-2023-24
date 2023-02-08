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

#define MOTORLEFT_1 2
#define MOTORLEFT_2 3
#define MOTORRIGHT_1 4
#define MOTORRIGHT_2 5

// The physical distance between the sensors
#define LIDAR_SEPERATION 74 // 74 mm between sensors

// Create encoders and sensors
Adafruit_VL6180X lidar = Adafruit_VL6180X();

Encoder rightEncoder (ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder (ENCODER_LEFT_1, ENCODER_LEFT_2);

// Global variables to store the last seen values for different sensors
uint8_t back_right;
uint8_t front_right;
// TODO: other sensors when we have them

void updateSensors () {
    selectSensor(0);
    back_right = lidar.readRange();
    selectSensor(1);
    front_right = lidar.readRange();
}

// TODO: we probably need to account for accumulated error as a result of being not in the center

enum motor {
    LEFT_MOTOR,
    RIGHT_MOTOR
};

void setMotor (motor m, int8_t power) {
    if (m == LEFT_MOTOR) {
        // TODO: confirm that forward and backwards are right
        // TODO: confirm that I don't have an off-by-one, and that 0 power = 0 written to both
        // // max forward
        // analogWrite(MOTORLEFT_1, 0);
        // analogWrite(MOTORLEFT_2, 255);

        // // max backward
        // analogWrite(MOTORLEFT_1, 0);
        // analogWrite(MOTORLEFT_2, 255);

        analogWrite(MOTORLEFT_1, power + 127);
        analogWrite(MOTORLEFT_2, 255 - (power + 127));
    }else if (m == RIGHT_MOTOR) {
        analogWrite(MOTORRIGHT_1, power + 127);
        analogWrite(MOTORRIGHT_2, 255 - (power + 127));
    }
}


void moveOneForward () {
    // Create a PID controller for angle
    double goalAngle, currentAngle, angularVelocity;
    PID anglePID(&goalAngle, &currentAngle, &angularVelocity, 2, 0, 1, DIRECT);
    // Create a PID controller for speed
    // (currentDistance is the distance inside the current square. It resets to 0 at the end of )
    double goalDistance, currentDistance, velocity;
    PID positionPID(&goalDistance, &currentDistance, &velocity, 2, 5, 1, DIRECT);

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
            leftRevs = leftEncoder.read();
            rightRevs = rightEncoder.read();
            currentDistance = (leftRevs + rightRevs) / 2 / 360 * PI * 60;
        }

        // check if currentDistance and currentAngle are within tolerance
            // return
            // TODO:

        // update PID
        anglePID.calculate();
        positionPID.calculate();

        // update motor values
        // take pid output, divide by two, - one, etc
        // angularVelocity =
        // angularVelocityLeft =
        // angularVelocityRight =

        // angularVelocityLeft += velocity
        // angularVelocityRight += velocity
        // setMotor(LEFT, angularVelocityLeft)
        // setMotor(RIGHT, angularVelocityRight)
    }
}

void setup () {
    // Wait for serial to become available
    // TODO: Will this stall forever if USB isn't plugged in?
    Serial.begin(115200);
    while (!Serial.available()) {
        delay(1);
    }
    Serial.println("Serial ready!");

    // Starts I2C on the default pins (18 (SDA), 19 (SCL))
    // (I think, I can't find docs on it)
    Wire.begin();
    Serial.println("I2C ready!");

    // Setup LiDARs (only 0 and 1, on the right side for now)
    for (uint8_t k = 0; k < 2; k++) {
        selectSensor(k);

        Serial.println("Adafruit VL6180x test!");
        if (!lidar.begin()) {
            Serial.print("Failed to find sensor: ");
            Serial.println(k);
            while (1);
        }
    }
    Serial.println("LiDAR sensors ready!");

    // Setup motors
    pinMode(MOTORLEFT_1, OUTPUT);
    pinMode(MOTORLEFT_2, OUTPUT);
    pinMode(MOTORRIGHT_1, OUTPUT);
    pinMode(MOTORRIGHT_2, OUTPUT);

    setMotor(RIGHT, 0);
    setMotor(LEFT, 0);

    // Get initial values for sensors
    updateSensors();

    moveOneForward();
}

void loop () {
    // We don't have any looping code
    delay(100000);
}
