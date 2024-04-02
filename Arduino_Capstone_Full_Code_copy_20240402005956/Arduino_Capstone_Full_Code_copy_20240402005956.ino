
#include <ArduinoBLE.h>
#include "Adafruit_PM25AQI.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <SoftwareSerial.h>
SoftwareSerial pmSerial(1,2);


const int batteryPin = A3;

// Function to read the battery voltage
float readBatteryVoltage() {
    int sensorValue = analogRead(batteryPin);
    float voltage = sensorValue * (5.0 / 1023.0);
    return voltage;
}

// Function to calculate battery percentage
float batteryPercentage(float voltage) {
    float maxVoltage = 4.8; // Maximum voltage of 4 fully charged AA batteries
    float minVoltage = 0.04; // Minimum voltage of 4 discharged AA batteries
    float percentage = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100;
    percentage = constrain(percentage, 0, 100); // Ensure the percentage is between 0 and 100
    return percentage;
}

// Bluetooth LE Service and Characteristic UUIDs
BLEService customService("12345678-1234-1234-1234-123456789ABC");
BLEStringCharacteristic commandCharacteristic("ABCD1234-1234-1234-1234-123456789ABC", BLERead | BLEWrite, 20);
BLEStringCharacteristic responseCharacteristic("ABCD5678-1234-1234-1234-123456789ABC", BLENotify, 256); // Increased size for sensor data

// Humidity and temperature sensor (SHT31)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// PM2.5 sensor setup
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// Temperature sensor setup
#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // Initialize sensors
  sensors.begin();
  if (!sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  if (! aqi.begin_UART(&Serial1)) {
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("ArduinoBLETest");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(commandCharacteristic);
  customService.addCharacteristic(responseCharacteristic);
  BLE.addService(customService);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}
void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.println("Connected to central device!");
    while (central.connected()) {
      if (commandCharacteristic.written()) {
        String command = commandCharacteristic.value();
        } else  {
          PM25_AQI_Data data;
            // Request temperature from Dallas sensor
            sensors.requestTemperatures(); 
            if (! aqi.read(&data)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  }
  float voltage = readBatteryVoltage();
  float percentage = batteryPercentage(voltage);
    delay(1000); // Wait for a second before reading again
            float solarTemperature = sensors.getTempCByIndex(0);
            
            // Read humidity and temperature from SHT31
            float ambientHumidity = sht31.readHumidity();
            float ambientTemperature = sht31.readTemperature(); // Ambient temperature
            float pm25 = data.particles_25um;
            
            // Prepare sensor data string
            String sensorData =  String(solarTemperature) + "," + String(ambientTemperature) + "," + String(ambientHumidity) + "," + String(pm25) + "," + String(percentage);
            Serial.println(sensorData);
            delay(5000);
            // Send the sensor data over Bluetooth
            responseCharacteristic.writeValue(sensorData);
        }
      }
    }
    Serial.println("Disconnected from central device.");
  }



