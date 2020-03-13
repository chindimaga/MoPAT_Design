//Guining Pertin - 06-03-2020
//NodeMCU servo control

//Include libraries
#include <Servo.h>

//Define pins
#define rser_pin 16
#define lser_pin 5

//Create servo objects
Servo rser;
Servo lser;

void setup() {
  //Set pin
  rser.attach(16);
  lser.attach(5);
  //Start serial
  Serial.begin(9600);
}

void loop() {
    rser.write(70);
    lser.write(105);
}
