#include <Arduino.h>
#include <LoRa.h>
#include <ArduinoJson.h>

const int capacity_signal_data = JSON_OBJECT_SIZE(2);
const int capacity_output_data = JSON_OBJECT_SIZE(1);
StaticJsonDocument<capacity_signal_data> signal_data;
StaticJsonDocument<capacity_output_data> output_data;

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }
  Serial.println(message);
  // read and print LoRa signal data
  signal_data["rssi"] = LoRa.packetRssi();
  signal_data["snr"] = LoRa.packetSnr();
  serializeJson(signal_data, Serial);
  Serial.print("\n");
}

void onTxDone() {
  //Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

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
  // interrupt functions
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

void loop() {
   //while (Serial.available() == 0) {}     //wait for data available
   //String incomin_message = Serial.readString();  //read until timeout
   //Serial.print(incomin_message);
   delay(1000);
}