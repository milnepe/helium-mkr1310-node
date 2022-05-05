/*
  Helium LoRaWAN soil sensor node with low power sleep
  Reads moisture and BME280 sensors then sends readings as byte array

  Based on Arduino MKRWAN LoraSendReceive example

  Author: Peter Milne
  Copywrite 2022 Peter Milne
  Released under GNU GENERAL PUBLIC LICENSE
  Version 3, 29 June 2007
    
*/

#include <MKRWAN.h>
#include <forcedClimate.h>
#include "ArduinoLowPower.h"
#include "arduino_secrets.h"

#define DEBUG

#define SEND_INTERVAL 60 // Send interval in mins
#define BUFFER_SIZE  10 // Data buffer size in bytes


#define WET 106
#define DRY 608

// Enter keys in arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;


int moisturePin = A0; // Moisture sensor pin

LoRaModem modem;

ForcedClimate climateSensor = ForcedClimate(Wire, 0x76);

int16_t counter = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Begin debugging...");
#endif

  climateSensor.begin();

  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    while (1) {}
  };
#ifdef DEBUG
  Serial.print("Firmware version: ");
  Serial.println(modem.version());
  Serial.print("Device EUI: ");
  Serial.println(modem.deviceEUI());
#endif

  delay(5000);

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    while (1) {}
  }

  /*
    DataRate  Modulation  SF  BW  bit/s
    0   LoRa  12  125   250
    1   LoRa  11  125   440
    2   LoRa  10  125   980
    3   LoRa  9   125   1'760
    4   LoRa  8   125   3'125
    5   LoRa  7   125   5'470
    6   LoRa  7   250   11'000
  */
  modem.dataRate(5);

  // Enable Adjustable Data Rate
  modem.setADR(true);
}

void loop() {
  delay(5000);

  // Buffer sensor readings
  uint8_t buffer[BUFFER_SIZE] = {0};  // init to zero!

  // Read moisture sensor
  int16_t moisture_raw = analogRead(moisturePin);
  if (moisture_raw > DRY) {
    moisture_raw = DRY;
  } else if (moisture_raw < WET) {
    moisture_raw = WET;
  }  
  int16_t moisture_pc = map(moisture_raw, DRY, WET, 0, 100);
  int_to_byte_array(moisture_pc, &buffer[0]);
  delay(50);

  climateSensor.takeForcedMeasurement();
  int16_t temperature_raw = (int16_t)(climateSensor.getTemperatureCelcius() * 100.00);
  int16_t humidity_raw = (int16_t)(climateSensor.getRelativeHumidity() * 100.00);
  int16_t pressure_raw = (int16_t)(climateSensor.getPressure() * 10.00);
  int_to_byte_array(humidity_raw, &buffer[4]);
  int_to_byte_array(temperature_raw, &buffer[2]);
  int_to_byte_array(pressure_raw, &buffer[6]);

  int_to_byte_array(++counter, &buffer[8]);
  delay(5);

#ifdef DEBUG
  Serial.print("Readings int16_t: ");
  Serial.print(moisture_raw);
  Serial.print(" ");
  Serial.print(moisture_pc);
  Serial.print(" ");
  Serial.print(temperature_raw);
  Serial.print(" ");
  Serial.print(humidity_raw);
  Serial.print(" ");
  Serial.print(pressure_raw);
  Serial.print(" ");
  Serial.println(counter);

  Serial.print("Readings byte array: ");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  int err = 0;
  modem.beginPacket();
  modem.write(buffer, BUFFER_SIZE);  // write bytes
  err = modem.endPacket(true);
#ifdef DEBUG
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message");
  }
#endif
  LowPower.sleep(SEND_INTERVAL * 60 * 1000);
}

// Convert 16-bit int to 8-bit array (Big endian)
// where buf points to start byte in array
void int_to_byte_array(int16_t n, uint8_t* buf) {
  *buf = n >> 8;
  *++buf = n & 0xFF;
}
