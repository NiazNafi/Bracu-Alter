
#define LED_1_PIN 2
#define LED_2_PIN 3
#define LED_3_PIN 4
#define LED_4_PIN 5
void powerOffAllLEDs() {
    digitalWrite(LED_1_PIN, LOW);
    digitalWrite(LED_2_PIN, LOW);
    digitalWrite(LED_3_PIN, LOW);
    digitalWrite(LED_4_PIN, LOW);
}
void setup() {
  Serial.begin(9600);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  
}

void loop() {

  if (Serial.available() > 0) {
    powerOffAllLEDs();
    char command = Serial.read(); // Read a single character
    Serial.print("Received command: ");
    Serial.println(command);
    
    if (command == 'w') {
      Serial.println("Moving Forward");
      digitalWrite(LED_1_PIN,HIGH);
    } else if (command == 's') {
      Serial.println("Moving Backward");
      digitalWrite(LED_2_PIN,HIGH);
    } else if (command == 'a') {
      Serial.println("Turning Left");
      digitalWrite(LED_3_PIN,HIGH);
    } else if (command == 'd') {
      Serial.println("Turning Right");
      digitalWrite(LED_4_PIN,HIGH);
    } else {
      Serial.println("Invalid Command");
    }

  }
}
