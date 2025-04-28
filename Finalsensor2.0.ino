#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_seesaw.h"

// ==== BME280 Setup ====
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C connection

// ==== Soil Sensor Setup ====
Adafruit_seesaw ss;

// ==== Wind Sensor Setup ====
const int wind_sensor_pin = A0;
const int baseline = 78;              // Adjust this based on no-wind baseline reading
const float calibration_factor = 10.0; // Adjust this for your specific sensor

// ==== General Settings ====
unsigned long delayTime = 1000;

// ==== Setup Function ====
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial

  Serial.println("Initializing Sensors...");

  // Initialize BME280
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    while (1) delay(10);
  } else {
    Serial.println("BME280 sensor initialized.");
  }

  // Initialize Soil Sensor
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! Seesaw Soil Sensor not found");
    while (1) delay(10);
  } else {
    Serial.print("Seesaw Soil Sensor started! Version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  // Initialize Wind Sensor Pin
  pinMode(wind_sensor_pin, INPUT);

  Serial.println("-- Setup Complete --");
  Serial.println();
}

// ==== Main Loop ====
void loop() {
  readSensors();
  delay(delayTime);
}

// ==== Read and Print All Sensor Values ====
void readSensors() {
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  float soilTemp = ss.getTemp();
  uint16_t soilMoisture = ss.touchRead(0);

  float windSpeed = readWindSensor();

  // Print everything on one line
  Serial.print("BME280 Temp: "); Serial.print(temperature); Serial.print(" °C, ");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa, ");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.print(" m, ");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.print(" %, ");
  Serial.print("Soil Temp: "); Serial.print(soilTemp); Serial.print(" °C, ");
  Serial.print("Soil Moisture: "); Serial.print(soilMoisture); Serial.print(", ");
  Serial.print("Wind Speed: "); Serial.print(windSpeed); Serial.println(" m/s");
}

// ==== Wind Sensor Function ====
float readWindSensor() {
  int wind_speed_raw = analogRead(wind_sensor_pin);
  float wind_speed_calibrated = (wind_speed_raw - baseline) / calibration_factor;
  
  if (wind_speed_calibrated < 0) wind_speed_calibrated = 0; // No negative wind speeds
  
  return wind_speed_calibrated;
}
