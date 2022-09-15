#include <Arduino.h>
#include <Servo.h>

// convert raw accelerometer reading and return acceleration in [m/s2]
float convertRawAccl(int aRaw, int range)
{
  float sensitivity = 32768.0 / (range * 9.8);
  return aRaw / sensitivity;
}

// convert raw gyro reading and return rotation in [deg/s]
float convertRawGyro(int gRaw, int range)
{
  float sensitivity = 32768.0 / range;
  return gRaw / sensitivity;
}

// read battery voltage
float read_battery_voltage(const uint8_t ADC_BATTERY)
{
  // read analog value
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.2V):
  return sensorValue * (4.2 / 1023.0);
}

// round floating poin number
double round2(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}

// set vale position
int set_valce_position(bool valve_state = false, const int open_pos = 90, const int closed_pos = 180)
{
  if (valve_state)
  {
    return open_pos;
  }
  else
    return closed_pos;
}

// LoRa functions
void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
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

  //Serial.print("Node Receive: ");
  //Serial.println(message);
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