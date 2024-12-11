String command;
#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  // delay(100);
  myservo.write(50);
  // Serial.println("Type Command (raise, hit)");
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("r")) {
      myservo.write(50);
      Serial.println("d");
    }

    else if (command.equals("h")) {
      //myservo.write(50);
      //delay(500);
      myservo.write(100);
      delay(200);
      myservo.write(50);
      Serial.println("d");
    }
    // Serial.print("Command: ");
    // Serial.println(command);
  }
}

