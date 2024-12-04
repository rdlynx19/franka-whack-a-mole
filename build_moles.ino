// Define pin ranges and matching logic
const int outputPins[] = {6, 7, 8};  // Pins 7-11
const int inputPins[] = {2, 3, 4};     // Pins 1-5
int count = 0;
int currentIndex = 0; 
int val;
unsigned long previous_time;

void setup() {
  // Initialize output pins
  for (int i = 0; i < 3; i++) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW); 
    randomSeed(analogRead(0));  // A0 is typically used for random seeding
  }

  // Initialize input pins
  for (int i = 0; i < 3; i++) {
    pinMode(inputPins[i], INPUT);  // Set input pins as INPUT
  }

  Serial.begin(9600);  // Initialize serial communication for debugging
  previous_time = millis();
  digitalWrite(outputPins[currentIndex],HIGH);
}



void loop() {
  Serial.println(currentIndex);
  val = digitalRead(inputPins[currentIndex]);   // read the input pin
  if(millis() - previous_time >= 8000 || val==0){
    previous_time = millis();
    digitalWrite(outputPins[currentIndex], LOW);
    currentIndex = random(0,3);
    digitalWrite(outputPins[currentIndex], HIGH);
    }
}

    
