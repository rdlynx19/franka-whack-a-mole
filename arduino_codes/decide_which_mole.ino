// Define pin ranges and matching logic
const int outputPins[] = {8, 9, 10, 11, 12};  // Pins 7-11
const int inputPins[] = {3, 4, 5, 6, 7};     // Pins 1-5
int count = 0;
int currentIndex = 0; 
int val;
unsigned long previous_time;
String command;

void setup() {
  // Initialize output pins
  for (int i = 0; i < 5; i++) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW); 
    randomSeed(analogRead(0));  // A0 is typically used for random seeding
  }

  // Initialize input pins
  for (int i = 0; i < 5; i++) {
    pinMode(inputPins[i], INPUT);  // Set input pins as INPUT
  }

  Serial.begin(9600);  // Initialize serial communication for debugging
  previous_time = millis();
  digitalWrite(outputPins[currentIndex],HIGH);
}



void loop() {
   if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("blue")) {
      digitalWrite(outputPins[0], LOW);
      digitalWrite(outputPins[2], LOW);
      digitalWrite(outputPins[3], LOW);
      digitalWrite(outputPins[4], LOW);
      digitalWrite(outputPins[1], HIGH);
    }

    else if (command.equals("red")) {
      digitalWrite(outputPins[0], LOW);
      digitalWrite(outputPins[1], LOW);
      digitalWrite(outputPins[2], LOW);
      digitalWrite(outputPins[4], LOW);
      digitalWrite(outputPins[3], HIGH);
      
    }
    else if (command.equals("green")) {
      digitalWrite(outputPins[0], LOW);
      digitalWrite(outputPins[1], LOW);
      digitalWrite(outputPins[3], LOW);
      digitalWrite(outputPins[4], LOW);
      digitalWrite(outputPins[2], HIGH);
  }
}
}

    
