#include <Arduino.h>
#include <Servo.h>

Servo esc;

void setup() {
    esc.attach(12, 1000, 2000);
    esc.writeMicroseconds(1000);
    delay(3000);
}

void loop() {
    esc.writeMicroseconds(1300);
    delay(5000);

    esc.writeMicroseconds(1000);
    delay(3000);
}
