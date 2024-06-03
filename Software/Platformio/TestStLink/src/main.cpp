#include <Arduino.h>

uint8_t LED_Pin = PC13;

void setup() {

  pinMode(LED_Pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_Pin, 1);
  delay(500);
  digitalWrite(LED_Pin, 0);
  delay(500);
}

