#include <Arduino.h>

#define LED PIN_PA3


void setup() {
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, 1);
  delay(200);
  digitalWrite(LED, 0);
  delay(200);
}

