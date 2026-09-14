#include <Servo.h>
Servo gripperServo;

const int triggerPin = 2;   // reads Teensy's GPIO signal

void setup() {
  pinMode(triggerPin, INPUT);
  gripperServo.attach(A0);
  gripperServo.write(20);
}

void loop() {
  if (digitalRead(triggerPin) == HIGH) {
    gripperServo.write(20);   // closed — raise this until servo stops straining
  } else {
    gripperServo.write(45);   // open — lower if it strains at the open end
  }
}