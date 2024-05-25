#define dirPin 2
#define stepPin 3
#define enaPin 4
#define STPR 200

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin, OUTPUT); 
  Serial.begin(9600); 
}

void loop() {
  if (Serial.available() > 0) {
    char a = Serial.read();
    Serial.print(a);
    if (a == 'a') {
      xMoveSteps(true, STPR * 2);
      Serial.println("Going forward");
    }
    if (a == 'b') {
      xMoveSteps(false, STPR * 2);
      Serial.println("Going Backward");
    }
    //delay (1000); //delay 4 seconds
    
    //delay (1000);
  }
}

void xMoveSteps(bool dir, int steps) {
  digitalWrite(enaPin, LOW); // enable movement or lock
  digitalWrite(dirPin, dir);
  for (int i = 0; i < 10; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  digitalWrite(enaPin, HIGH); // disable movement or free
}
