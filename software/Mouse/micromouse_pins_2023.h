#ifndef MICROMOUSE_PINS_2023_H
#define MICROMOUSE_PINS_2023_H

// LEDS
#define LED0 1 // R1
#define LED1 2 // R2
#define LED2 3 // R3
#define LED3 4 // R4
#define YELLOW_LED LED0
#define BLUE_LED LED1
#define GREEN_LED LED2
#define RED_LED LED3

#define DEBUG_LED 13  // The LED on the teensy board

// I2C BUSES
#define I2C_LIDAR Wire

// LIDAR /CS PINS
#define LIDAR_CS1 20
#define LIDAR_CS2 23
#define LIDAR_CS3 21
#define LIDAR_CS4 22
#define LIDAR_CS5 16
#define LIDAR_CS6 14
#define LIDAR_CS7 15

// Map the pins to where the sensor is on the robot
#define LIDAR_FrontShort LIDAR_CS1
#define LIDAR_FrontLeft LIDAR_CS3
#define LIDAR_FrontRight LIDAR_CS2
#define LIDAR_BackLeft LIDAR_CS7
#define LIDAR_BackRight LIDAR_CS6
#define LIDAR_FrontLong LIDAR_CS4
#define LIDAR_Button LIDAR_CS5

// Encoders
#define ENCODER_LEFT_1 10
#define ENCODER_LEFT_2 12
#define ENCODER_RIGHT_1 9
#define ENCODER_RIGHT_2 11

// Motors
#define MOTORLEFT_1 8
#define MOTORLEFT_2 7
#define MOTORRIGHT_1 5
#define MOTORRIGHT_2 6

#define START_BUTTON LIDAR_Button
#endif

