#include <Arduino.h>
#include <LoRa.h>
#include <ArduinoJson.h>

const int capacity = JSON_OBJECT_SIZE(2);
StaticJsonDocument<capacity> signal_data;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while(true);
  }
  // LoRa settings
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    // read and print LoRa signal data
    signal_data["rssi"] = LoRa.packetRssi();
    signal_data["snr"] = LoRa.packetSnr();
    serializeJson(signal_data, Serial);
    Serial.print("\n");
  }
}