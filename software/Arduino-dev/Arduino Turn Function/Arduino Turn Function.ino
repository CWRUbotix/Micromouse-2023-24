#include <Encoder.h>

#define ENCODER_RIGHT_1 40
#define ENCODER_RIGHT_2 41
#define ENCODER_LEFT_1 22
#define ENCODER_LEFT_2 23

#define MOTORLEFT_1 2
#define MOTORLEFT_2 3
#define MOTORRIGHT_1 4
#define MOTORRIGHT_2 5

Encoder rightEncoder(ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder(ENCODER_LEFT_1, ENCODER_LEFT_2);

double wheelSeparation = 6;
double wheelRadius = 3;
double turnValue = 90 * (wheelSeparation / 2.0) / wheelRadius;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void turnRight() {
  int initial = leftEncoder.read();

  // Turn right wheel backwards
  analogWrite(MotorRight_1, 255);
  analogWrite(MotorRight_2, 0);

  // Turn left wheel forwards
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorLeft_2, 255);

  // Wait until necessary angle is reached
  while(leftEncoder.read() - initial < turnValue) {}

  // Stop both motors
  analogWrite(MotorRight_1, 0);
  analogWrite(MotorLeft_2, 0);
}

void turnLeft() {
  int initial = rightEncoder.read();

  // Turn right wheel backwards
  analogWrite(MotorLeft_1, 255);
  analogWrite(MotorLeft_2, 0);

  // Turn left wheel forwards
  analogWrite(MotorRight_1, 0);
  analogWrite(MotorRight_2, 255);

  // Wait until necessary angle is reached
  while(rightEncoder.read() - initial < turnValue) {}

  // Stop both motors
  analogWrite(MotorLeft_1, 0);
  analogWrite(MotorRight_2, 0);
}