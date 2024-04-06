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

//#define DEBUG_LED 13  // The LED on the teensy board

// I2C BUSES
#define I2C_LIDAR Wire

// LIDAR /CS PINS -> done
#define LIDAR_CS1 20
#define LIDAR_CS2 23
#define LIDAR_CS3 21
#define LIDAR_CS4 22
#define LIDAR_CS5 16
#define LIDAR_CS6 14

/*
#define SONIC_TRIG1 40
#define SONIC_TRIG2 38
#define SONIC_TRIG3 36
#define SONIC_ECHO1 41 
#define SONIC_ECHO2 39
#define SONIC_ECHO3 37
*/
#define SONIC_ECHO3 37 // TODO

// Done
#define ENCODER_LEFT_1 10
#define ENCODER_LEFT_2 12
#define ENCODER_RIGHT_1 9
#define ENCODER_RIGHT_2 11

// Done
#define MOTORLEFT_1 8
#define MOTORLEFT_2 7
#define MOTORRIGHT_1 5
#define MOTORRIGHT_2 6

#define START_BUTTON SONIC_ECHO3
#endif

