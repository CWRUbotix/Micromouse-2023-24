#define PIN_M1A 6
#define PIN_M1B 7
#define PIN_M2A 8
#define PIN_M2B 15
#define PIN_E1A 39
#define PIN_E1B 36
#define PIN_E2A 13
#define PIN_E2B 9

void setup() {
  // put your setup code here, to run once:
pinMode(PIN_M1A, OUTPUT);
pinMode(PIN_M2A, OUTPUT);
pinMode(PIN_M1B, OUTPUT);
pinMode(PIN_M2B, OUTPUT);
pinMode(PIN_E1A, INPUT);
pinMode(PIN_E2A, INPUT);
pinMode(PIN_E1B, INPUT);
pinMode(PIN_E2B, INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PIN_M1A, 255);
  analogWrite(PIN_M1B, 0);
  analogWrite(PIN_M2A, 0);
  analogWrite(PIN_M2B, 255);
  delay(1000);
  analogWrite(PIN_M1A, 0);
  analogWrite(PIN_M1B, 0);
  analogWrite(PIN_M2A, 0);
  analogWrite(PIN_M2B, 0);
  delay(1000);

}
