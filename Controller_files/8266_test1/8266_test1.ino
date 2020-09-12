#include <Servo.h>

//Define PINS
#define R1 13 //D7
#define G1 12 //D6
#define R2 2//D4
#define G2 14//D5
#define R3 5//D1
#define G3 16//D0
#define SL 4//D2
#define SR 0//D3

Servo ServoL; 
Servo ServoR;

void setup() {
  //Connect
  pinMode(R1, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(G2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(G3, OUTPUT);
  ServoL.attach(SL);
  ServoR.attach(SR);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(R1, HIGH);
  digitalWrite(G1, LOW);
  digitalWrite(R2, HIGH);
  digitalWrite(G2, LOW);
  digitalWrite(R3, HIGH);
  digitalWrite(G3, LOW);
  ServoL.write(0);
  ServoR.write(180);
  delay(2000);
  digitalWrite(R1, LOW);
  digitalWrite(G1, HIGH);
  digitalWrite(R2, LOW);
  digitalWrite(G2, HIGH);
  digitalWrite(R3, LOW);
  digitalWrite(G3, HIGH);
  ServoL.write(180);
  ServoR.write(0);
  delay(2000);
}
