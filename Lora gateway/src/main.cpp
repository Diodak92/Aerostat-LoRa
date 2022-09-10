#include <Arduino.h>
#include <LoRa.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
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
  }
}