#ifndef MICROMOUSE_PINS_2023_H
#define MICROMOUSE_PINS_2023_H

// LEDS
#define LED0 5
#define LED1 6
#define LED2 10
#define LED3 11

#ifndef _LED_ARR
#define _LED_ARR
const int LEDS[] = {LED0, LED1, LED2, LED3};
#endif

// I2C BUSES
#define I2C_LIDAR Wire2
#define I2C_GYRO Wire1

// LIDAR /CS PINS
#define LIDAR_CS1 28
#define LIDAR_CS2 26
#define LIDAR_CS3 27
#define LIDAR_CS4 8
#define LIDAR_CS5 7
#define LIDAR_CS6 9

#ifndef _LIDAR_ARR
#define _LIDAR_ARR
const int LIDARS[] = {LIDAR_CS1, LIDAR_CS2, LIDAR_CS3, LIDAR_CS4, LIDAR_CS5, LIDAR_CS6};
#endif

#define SONIC_TRIG1 40
#define SONIC_TRIG2 38
#define SONIC_TRIG3 36
#define SONIC_ECHO1 41 
#define SONIC_ECHO2 39
#define SONIC_ECHO3 37

#define ENCODER_LEFT1 23
#define ENCODER_LEFT2 22
#define ENCODER_RIGHT1 1
#define ENCODER_RIGHT2 2

#define MOTOR_LEFT1 3
#define MOTOR_LEFT2 19
#define MOTOR_RIGHT1 18
#define MOTOR_RIGHT2 4
#endif

