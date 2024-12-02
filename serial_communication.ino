String command;
#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  delay(1000);
  Serial.println("Type Command (raise, hit)");
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("raise")) {
      myservo.write(120);
    }

    else if (command.equals("hit")) {
      myservo.write(175);
      delay(500);
    }
    Serial.print("Command: ");
    Serial.println(command);
  }
}
