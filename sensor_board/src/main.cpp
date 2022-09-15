#include <Arduino.h>
#include <ArduinoJson.h>
#include <LoRa.h>
#include <HoneywellTruStabilitySPI.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BMP3XX.h>
#include <BMI160Gen.h>
#include <Servo.h>
#include <functions.h>

// BME680 hardware setup
#define SEALEVELPRESSURE_HPA (1013.25)
#define SLAVE_SELECT_PIN A1 // SPI SS pin
#define SERVO_PIN A3 // servo pin
#define ADC_BATTERY A5 // baterry pin

// BMI160 I2C addres 
const int bmi160_addr = 105;
int accl_range, gyro_range;
// create json object for storing sensor data
const int capacity_out = JSON_OBJECT_SIZE(16);
// create json object for storing input data
const int capacity_in = JSON_OBJECT_SIZE(2);
DynamicJsonDocument<>
StaticJsonDocument<capacity_out> sensorData;
StaticJsonDocument<capacity_in> inputData;

// construct a 0 : 100 mbar sensor using the default SPI slave select pin (SS)
TruStabilityPressureSensor chamber_pressure_sensor(SLAVE_SELECT_PIN, 0.0, 100.0);
// construct a BME688 sensor object
Adafruit_BME680 bme;
// construct a BMP390 sensor object
Adafruit_BMP3XX bmp;
// construct a servo ovbect for gas reliese valve
Servo gas_reliese_valve;

void setup()
{
  LoRa.begin(868E6); // start LoRa module
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  // start serial comunication
  //Serial.begin(115200);
  //while(!Serial);

  // confugure gauge pressure sensor
  SPI.begin();                     // start SPI communication
  chamber_pressure_sensor.begin(); // TruStability sensor initialization

  // start BMP390 comunication
  if (!bmp.begin_I2C(0X77)) {  // hardware I2C mode, can pass in address & alt Wire
    LoRa.beginPacket();
    LoRa.println("Could not find a valid BMP390 sensor, check wiring!");
    LoRa.endPacket();
    while(true);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  // start BME688 comunication
  if (!bme.begin(0x76))
  {
    LoRa.beginPacket();
    LoRa.println(F("Could not find a valid BME688 sensor, check wiring!"));
    LoRa.endPacket();
    while(true);
  }

  // Set up oversampling and filter initialization for BME688
  //Serial.println("Initializing BME680 device...");
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_8X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  //Serial.println("Initializing gas sensor done.");

  // configure IMU
  //Serial.println("Initializing IMU device...");
  Wire.begin(); // start I2C bus
  BMI160.begin(BMI160GenClass::I2C_MODE, Wire, bmi160_addr);
  BMI160.detachInterrupt();
  BMI160.setAccelerometerRange(2); // 2G
  BMI160.setAccelerometerRate(100); //100 Hz
  BMI160.setGyroRange(250); // 125 deg/s
  BMI160.setGyroRate(100); //100 Hz
  accl_range = BMI160.getAccelerometerRange();
  gyro_range = BMI160.getGyroRange();
  //Serial.println("Initializing IMU device done.");

  // attach sevro object
  gas_reliese_valve.attach(SERVO_PIN);
  // setup default valve position
  gas_reliese_valve.write(set_valce_position());
}

void loop()
{
  // read IMU data
  int axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);
  // convert the raw accl and gyro data
  sensorData["ax"] = round2(convertRawAccl(axRaw, accl_range));
  sensorData["ay"] = round2(convertRawAccl(ayRaw, accl_range));
  sensorData["az"] = round2(convertRawAccl(azRaw, accl_range));
  sensorData["gx"] = round2(convertRawGyro(gxRaw, gyro_range));
  sensorData["gy"] = round2(convertRawGyro(gyRaw, gyro_range));
  sensorData["gz"] = round2(convertRawGyro(gzRaw, gyro_range));

  // read data from Honeywell TruStability pressure sensor
  // the sensor returns 0 when new data is ready
  if (chamber_pressure_sensor.readSensor() == 0)
  {
    sensorData["t0"] = round2(chamber_pressure_sensor.temperature());
    sensorData["p0"] = round2(chamber_pressure_sensor.pressure());
  }

  // Tell BME680 to begin measurement
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    LoRa.beginPacket();
    LoRa.println(F("Failed to begin reading!"));
    LoRa.endPacket();
    return;
  }

  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.
  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()){
    LoRa.beginPacket();
    LoRa.println(F("Failed to complete reading!"));
    LoRa.endPacket();
    return;
  }
  
  // read and append data from BME680
  sensorData["t1"] = round2(bme.temperature);                          // [oC]
  sensorData["p1"] = round2(bme.pressure / 100.0);                     // [hPa]
  sensorData["hum"] = round2(bme.humidity);                            // [%]
  sensorData["gas"] = round2(bme.gas_resistance / 1000.0);             // [KOhms]
  sensorData["alt0"] = round2(bme.readAltitude(SEALEVELPRESSURE_HPA)); // [m]

  // try read data from BMP390
  if (! bmp.performReading()) {
    LoRa.beginPacket();
    LoRa.println("Failed to perform reading!");
    LoRa.endPacket();
    return;
  }

  // read and append data from BMP390
  sensorData["t2"] = round2(bmp.temperature); // [oC]
  sensorData["p2"] = round2(bmp.pressure / 100.0);
  sensorData["alt1"] = round2(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  
  // The MKR 1310 board does not have a ADC_BATTERY pin
  // In order to measure battery voltage connect V_bat via voltage divider to any A5 pin 
  // read and save battery voltage
  // sensorData["bat"] = read_battery_voltage(ADC_BATTERY);

  // Send data over LoRa network
  LoRa.beginPacket();
  // Serialize data to JSON
  serializeJson(sensorData, LoRa);
  LoRa.print("\n");
  LoRa.endPacket();

  // wait for respone and read message
  //deserializeJson(message_in, LoRa.);
  //bool valve_control = message_in["valve"];
  //gas_reliese_valve.write(set_valce_position(valve_control));

  // wait for next iter
  delay(500);
}
