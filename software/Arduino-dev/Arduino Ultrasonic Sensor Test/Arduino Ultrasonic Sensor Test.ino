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
volatile long lastPulse = 0;
//Store distance calculation for each sensor
volatile double distance[] = {0, 0};
//Store which sensor to pulse
int sensorNum = 0;
//Store whether the sensor is done reading data
volatile bool sensorDoneReading = true;

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
  if(sensorDoneReading) {
    //Send out a pulse to all ultrasonic sensors
    pulse(sensorNum);
    //Record the time at which the pulse was sent
    lastPulse = micros();
    //Set the sensor to reading data
    sensorDoneReading = false;
    //Print distance from first ultrasonic sensor
    Serial.print("Distance ");
    Serial.print(sensorNum + 1);
    Serial.print(": ");
    Serial.print(distance[sensorNum]);
    Serial.println(" cm");
    //Change sensor to read
    sensorNum = (sensorNum + 1) % 2;
  }
}

void pulseAll() {
  //Set the trigPin to high (active) for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  //Return the trigPin to low (inactive) state
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
}

void pulse(int sensorNum) {
  //Set the trigPin to high (active) for 10 microseconds
  digitalWrite(trigPin1 + sensorNum, HIGH);
  delayMicroseconds(10);
  //Return the trigPin to low (inactive) state
  digitalWrite(trigPin1 + sensorNum, LOW);
}

void readPulse(int sensorNum) {
  //Start timing at pulse start
  if(digitalRead(echoPin1 + sensorNum) == HIGH) {
    lastPulse = micros();
  }
  //Calculate the distance traveled from the travel time at pulse end
  else {
    distance[sensorNum] = (micros() - lastPulse) * 0.034 / 2;
    sensorDoneReading = true;
  }
}

void readPulse1() {
  readPulse(0);
}

void readPulse2() {
  readPulse(1);
}