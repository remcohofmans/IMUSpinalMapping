/*
 * SensorManager.h
 * Handles the ICM-20948 sensors initialization and data acquisition
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define NO_OF_UNITS 2

class SensorManager {
public:
  SensorManager();
  
  // Initialize sensors and return the number of active sensors found
  bool initialize();
  
  // Read data from all active sensors
  void readAllSensors();
  
  // Getters for sensor data and status
  bool isSensorActive(int sensorId) const;
  int getActiveCount() const;
  
  // Raw sensor data getters
  void getRawAccel(int sensorId, float &x, float &y, float &z) const;
  void getRawGyro(int sensorId, float &x, float &y, float &z) const;
  void getRawMag(int sensorId, float &x, float &y, float &z) const;
  float getTemperature(int sensorId) const;
  
  // Configure sensor settings
  void configureForCalibration();
  void configureForNormalOperation();

private:
  Adafruit_ICM20948 icm[NO_OF_UNITS];
  bool sensorActive[NO_OF_UNITS];
  int activeCount;
  
  // Current sensor readings
  struct SensorData {
    float accel[3]; // X, Y, Z
    float gyro[3];  // X, Y, Z
    float mag[3];   // X, Y, Z
    float temp;     // Temperature
  };
  
  SensorData sensorData[NO_OF_UNITS];
};

#endif // SENSOR_MANAGER_H
