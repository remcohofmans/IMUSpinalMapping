/*
 * SensorManager.h
 * Handles the ICM-20948 sensors initialization and data acquisition
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "Adafruit_ICM20948.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_Sensor.h>
#include "AHRS_mahony_filter.h"
#include <Wire.h>

#define NO_OF_UNITS 1
#define TCAADDR 0x70

struct SensorConfig {
  uint8_t channel;
  uint8_t address;
};

class SensorManager {
public:
  SensorManager();
  
  void tcaSelect(uint8_t i);

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
  unsigned long getReadingTimestamp(int sensorId) const; 
  
  // Configure sensor settings
  void configureForCalibration();
  void configureForNormalOperation();

private:
  bool sensorActive[NO_OF_UNITS];
  int activeCount;

  static const SensorConfig sensorConfigs[NO_OF_UNITS];
  
  // Current sensor readings
  struct SensorData {
    float accel[3]; // X, Y, Z
    float gyro[3];  // X, Y, Z
    float mag[3];   // X, Y, Z
    float temp;     // Temperature
    unsigned long timestamp; 
  };
  
  // Initialize ICM20948 IMU sensors
  Adafruit_ICM20948 icm[NO_OF_UNITS];

  // Create pointers to its sensor modalities: accelerometer, gyroscope, and magnetometer
  // (these pointers allow access to sensor data using the Adafruit Unified Sensor API)
  Adafruit_Sensor* accelerometers[NO_OF_UNITS];
  Adafruit_Sensor* gyroscopes[NO_OF_UNITS];
  Adafruit_Sensor* magnetometers[NO_OF_UNITS];

  SensorData sensorData[NO_OF_UNITS];
};

#endif // SENSOR_MANAGER_H
