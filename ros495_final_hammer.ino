// #include <Servo.h>

// Servo myservo1;
// int servo1 = 12;

// void setup() {
// 	myservo1.setPeriodHertz(50);    // standard 50 hz servo
// 	myservo1.attach(servo1, 500, 2500); 
//   myservo1.write(1);
// }

// void loop() {
//   myservo1.write(1);
//   delay(1000);
//   myservo1.write(100);
//   delay(1000);

// }
/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  myservo.write(0);                  // sets the servo position according to the scaled value
  delay(1000);                           // waits for the servo to get there
  myservo.write(35);                  // sets the servo position according to the scaled value
  delay(1000);
}
