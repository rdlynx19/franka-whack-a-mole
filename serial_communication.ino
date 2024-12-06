String command;
#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  delay(1000);
  myservo.write(50);
  Serial.println("Type Command (raise, hit)");
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("raise")) {
      myservo.write(50);
    }

    else if (command.equals("hit")) {
      myservo.write(50);
      delay(500);
      myservo.write(100);
      delay(500);
      myservo.write(50);
      Serial.println("hit finish");
    }
    Serial.print("Command: ");
    Serial.println(command);
  }
}
