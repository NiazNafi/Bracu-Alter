#define enaPin_base 22
#define dirPin_base 23
#define stepPin_base 24
#define enaPin_j1 25
#define dirPin_j1 26
#define stepPin_j1 27
#define enaPin_j2 28
#define dirPin_j2 29
#define stepPin_j2 30
#define enaPin_j3 31
#define dirPin_j3 32
#define stepPin_j3 33
#define enaPin_j4 34
#define dirPin_j4 35
#define stepPin_j4 36
#define enaPin_clw 37
#define dirPin_clw 38
#define stepPin_clw 39

#define STPR 200

void setup() {
  pinMode(stepPin_base, OUTPUT);
  pinMode(dirPin_base, OUTPUT);
  pinMode(enaPin_base, OUTPUT);

  pinMode(stepPin_j1, OUTPUT);
  pinMode(dirPin_j1, OUTPUT);
  pinMode(enaPin_j1, OUTPUT);

  pinMode(stepPin_j2, OUTPUT);
  pinMode(dirPin_j2, OUTPUT);
  pinMode(enaPin_j2, OUTPUT);

  pinMode(stepPin_j3, OUTPUT);
  pinMode(dirPin_j3, OUTPUT);
  pinMode(enaPin_j3, OUTPUT);

  pinMode(stepPin_j4, OUTPUT);
  pinMode(dirPin_j4, OUTPUT);
  pinMode(enaPin_j4, OUTPUT);

  pinMode(stepPin_clw, OUTPUT);
  pinMode(dirPin_clw, OUTPUT);
  pinMode(enaPin_clw, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(enaPin_base, LOW);
  digitalWrite(enaPin_j1, LOW);
  digitalWrite(enaPin_j2, LOW);
  digitalWrite(enaPin_j3, LOW);
  digitalWrite(enaPin_j4, LOW);
  digitalWrite(enaPin_clw, LOW);
  
  if (Serial.available()) {
    String a = Serial.readString();
    a.trim();
    
    if (a == "bl") {
      xMoveSteps(true, dirPin_base, stepPin_base, enaPin_base);
      Serial.println("Base left-->Right");
    }
    else if (a == "br") {
      xMoveSteps(false, dirPin_base, stepPin_base, enaPin_base);
      Serial.println("Base Right-->Left");
    }
    else if (a == "j1u") {
      xMoveSteps(true, dirPin_j1, stepPin_j1, enaPin_j1);
      Serial.println("Joint1 Down-->Up");
    }
    else if (a == "j1d") {
      xMoveSteps(false, dirPin_j1, stepPin_j1, enaPin_j1);
      Serial.println("Joint1 Up-->Down");
    }
    else if (a == "j2u") {
      xMoveSteps(true, dirPin_j2, stepPin_j2, enaPin_j2);
      Serial.println("Joint2 Down-->Up");
    }
    else if (a == "j2d") {
      xMoveSteps(false, dirPin_j2, stepPin_j2, enaPin_j2);
      Serial.println("Joint2 Up-->Down");
    }
    else if (a == "j3u") {
      xMoveSteps(true, dirPin_j3, stepPin_j3, enaPin_j3);
      Serial.println("Joint3 Down-->Up");
    }
    else if (a == "j3d") {
      xMoveSteps(false, dirPin_j3, stepPin_j3, enaPin_j3);
      Serial.println("Joint3 Up-->Down");
    }
    else if (a == "j4u") {
      xMoveSteps(true, dirPin_j4, stepPin_j4, enaPin_j4);
      Serial.println("Joint4 Down-->Up");
    }
    else if (a == "j4d") {
      xMoveSteps(false, dirPin_j4, stepPin_j4, enaPin_j4);
      Serial.println("Joint4 Up-->Down");
    }
    else if (a == "clwu") {
      xMoveSteps(true, dirPin_clw, stepPin_clw, enaPin_clw);
      Serial.println("Claw Open");
    }
    else if (a == "clwd") {
      xMoveSteps(false, dirPin_clw, stepPin_clw, enaPin_clw);
      Serial.println("Claw Close");
    }
  }
}

void xMoveSteps(bool dir, int dirPin, int stepPin, int enaPin) {
  digitalWrite(dirPin, dir);
  for (int i = 0; i < 100; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  digitalWrite(enaPin, HIGH); // disable movement or free
}
