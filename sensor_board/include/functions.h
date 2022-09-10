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