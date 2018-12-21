#include <Servo.h>

Servo servo;
int servoPin = 3;
int radius = 90;

void setup() {
  Serial.begin(115200);
  servo.attach(servoPin);
  servo.write(radius);
}

void loop() {
  if (Serial.available()) {
    radius = Serial.read();
    servo.write(radius);
  }
}
