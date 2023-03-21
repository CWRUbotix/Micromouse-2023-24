// Includes
// Wire.h for I2C communication with TCA chip and LiDAR
#include <Wire.h>
// Arduino standard encoder library
#include <Encoder.h>
// Adafruit library for VL6180X LiDAR sensor
#include "Adafruit_VL6180X.h"
// PID library for control loops
#include <PID_v1.h>

#include "micromouse_pins_2023.h"

int LEFT_MOTOR = 0;
int RIGHT_MOTOR = 1;

const int LIDAR_COUNT = 4;
const int LIDAR_ADDR_BASE = 0x50;

// GPIO pin numbers for the CS line on each lidar sensor
const int lidar_cs_pins[LIDAR_COUNT] = {28, 26, 27, 9};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];


// The physical distance between the sensors
#define LIDAR_SEPERATION 123 // 123 mm between sensors

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
    // selectSensor(0);
    // back_right = lidar.readRange();
    // selectSensor(1);
    // front_right = lidar.readRange();
}

// TODO: we probably need to account for accumulated error as a result of being not in the center

/**
 * Convert a value in range [-128..127] to a motor power value
 * 
 * @param p The input power [-128..127]
 * @return Output power [0..255]
 */
uint8_t convert_power(int8_t p)
{
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

void setMotor (int m, int8_t power) {
    static const int deadband = 12;
    if (m == LEFT_MOTOR) {
        if (power < deadband && power > -deadband) {
          analogWrite(MOTORLEFT_1, 255);
          analogWrite(MOTORLEFT_2, 255);
        }else if (power > 0) {
          analogWrite(MOTORLEFT_1, 255);
          analogWrite(MOTORLEFT_2, convert_power(power));
        }else if (power < 0) {
          analogWrite(MOTORLEFT_1, convert_power(power));
          analogWrite(MOTORRIGHT_2, 255);
        }
    }else if (m == RIGHT_MOTOR) {
        if (power < deadband && power > -deadband) {
          analogWrite(MOTORRIGHT_1, 255);
          analogWrite(MOTORRIGHT_2, 255);
        }else if (power > 0) {
          analogWrite(MOTORRIGHT_1, 255);
          analogWrite(MOTORRIGHT_2, convert_power(power));
        }else if (power < 0) {
          analogWrite(MOTORRIGHT_1, convert_power(power));
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
    positionPID.SetOutputLimits(0, 255);
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
            // rev / 4560 is num revolutions (380:1 gearbox * 12 ticks per rev normally)
            // num revolutions * pi * diameter (Zach says 60mm)
            int leftRevs = leftEncoder.read();
            int rightRevs = rightEncoder.read();
            Serial.printf("left: %d\tright: %d\n", leftRevs, rightRevs);
            currentDistance = (leftRevs + rightRevs) / 2.0 / 4560 * PI * 60.0;
            Serial.printf("currentDistance: %f\n", currentDistance);
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
          // abort();
          setMotor(LEFT_MOTOR, 0);
          setMotor(RIGHT_MOTOR, 0);

          while (1 == 1) {
            digitalWrite(13, HIGH);
            delay(100);

            if (Serial.available()) {
              Serial.printf("Dumping data. Count was at %d.\n", count);
              for (int i = 0; i < count; i++) {
                Serial.printf("%d %f %f %f %f %f %f\n", millis(), goalAngles[i], currentAngles[i], angularVelocitys[i], goalDistances[i], currentDistances[i], velocitys[i]);
              }

              abort();
            }

            digitalWrite(13, LOW);
            delay(100);
          }
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

    // Setup LiDARs
    for (size_t i = 0; i < LIDAR_COUNT; ++i) {
      pinMode(lidar_cs_pins[i], OUTPUT);
    }
    // Disable all sensors except the first
    for (size_t i = 1; i < LIDAR_COUNT; ++i) {
      digitalWrite(lidar_cs_pins[i], LOW);
    }

    // Set address for each sensor
    // Write the CS line high (turning it on)
    // Set the address
    for (size_t i = 0; i < LIDAR_COUNT; ++i) {
      digitalWrite(lidar_cs_pins[i], HIGH);
      // Pass pointer to the Wire2 object since we're running on I2C bus 2
      if (!lidar_sensors[i].begin(&Wire2)) {
        Serial.print("Failed init on sensor ");
        Serial.println(i);
      }
      else {
        lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
      }
      delay(10);
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
