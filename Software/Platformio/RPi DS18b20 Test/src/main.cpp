#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2 // Data-Pin für beide DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1, sensor2;

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starte DS18B20 Multi-Sensor Messung...");

  sensors.begin();

  if (sensors.getDeviceCount() < 2) {
    Serial.println("Fehler: Weniger als 2 Sensoren erkannt!");
    while (1) delay(1000);
  }

  // Die ersten beiden Sensoren auslesen
  sensors.getAddress(sensor1, 0);
  sensors.getAddress(sensor2, 1);

  Serial.print("Sensor 1 Adresse: ");
  printAddress(sensor1);
  Serial.println();

  Serial.print("Sensor 2 Adresse: ");
  printAddress(sensor2);
  Serial.println();
}

void loop() {
  sensors.requestTemperatures(); // Alle Sensoren gleichzeitig messen

  float temp1 = sensors.getTempC(sensor1);
  float temp2 = sensors.getTempC(sensor2);

  Serial.print("Sensor 1: ");
  Serial.print(temp1);
  Serial.println(" °C");

  Serial.print("Sensor 2: ");
  Serial.print(temp2);
  Serial.println(" °C");

  Serial.println("-----------------");
  delay(1000);
}
