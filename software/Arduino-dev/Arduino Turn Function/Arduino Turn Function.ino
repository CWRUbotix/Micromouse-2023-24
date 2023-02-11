#include <Encoder.h>

#define ENCODER_LEFT_1 23
#define ENCODER_LEFT_2 22
#define ENCODER_RIGHT_1 1
#define ENCODER_RIGHT_2 2

#define MOTORLEFT_1 3
#define MOTORLEFT_2 19
#define MOTORRIGHT_1 18
#define MOTORRIGHT_2 4

Encoder rightEncoder(ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder(ENCODER_LEFT_1, ENCODER_LEFT_2);

double wheelSeparation = 9.5;
double wheelRadius = 3;
double turnRatio = (wheelSeparation / 2.0) / wheelRadius / 360 * 380 * 12;

void setup() {
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);

  turnLeft(90);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void turnRight(double angle) {
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