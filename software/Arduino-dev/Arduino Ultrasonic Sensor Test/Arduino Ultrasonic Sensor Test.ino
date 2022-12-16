//Sensor 1
//Echo to 2
const int echoPin1 = 2;
//Trig to 4
const int trigPin1 = 4;

//Sensor 2
//Echo to 3
const int echoPin2 = 3;
//Trig to 5
const int trigPin2 = 5;

//Store time at which last pulse was emitted
long lastPulse = 0;
//Store pulse start time for each sensor
volatile long time[] = {0, 0};
//Store distance calculation for each sensor
volatile double distance[] = {0, 0};
//Store which sensor to pulse
int sensorNum = 0;

void setup() {
  //Sets output pins
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  //Sets input pins with interrupts
  pinMode(echoPin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin1), readPulse1, CHANGE);
  pinMode(echoPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin2), readPulse2, CHANGE);
  //Begin serial monitor
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test");
}

void loop() {
  if(micros() - lastPulse >= 10000) {
    //Send out a pulse to all ultrasonic sensors
    pulse(sensorNum);
    //Record the time at which the pulse was sent
    lastPulse = micros();
    //Print distance from first ultrasonic sensor
    Serial.print("Distance ");
    Serial.print(sensorNum + 1);
    Serial.print(": ");
    Serial.print(distance[sensorNum]);
    Serial.println(" cm");
    sensorNum = (sensorNum + 1) % 2;
  }
}

double pulseAll() {
  //Set the trigPin to high (active) for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  //Return the trigPin to low (inactive) state
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
}

double pulse(int sensorNum) {
  //Set the trigPin to high (active) for 10 microseconds
  digitalWrite(trigPin1 + sensorNum, HIGH);
  delayMicroseconds(10);
  //Return the trigPin to low (inactive) state
  digitalWrite(trigPin1 + sensorNum, LOW);
}

void readPulse(int sensorNum) {
  //Start timing at pulse start
  if(digitalRead(echoPin1 + sensorNum) == HIGH) {
    time[sensorNum] = micros();
  }
  //Calculate the distance traveled from the travel time at pulse end
  else {
    distance[sensorNum] = (micros() - time[sensorNum]) * 0.034 / 2;
  }
}

void readPulse1() {
  readPulse(0);
}

void readPulse2() {
  readPulse(1);
}