/*
 * CalibrationManager.h
 * Handles sensor calibration operations and data
 */

#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include "SensorManager.h"
#include "FilterManager.h"

// Forward declaration to avoid circular dependency
class FilterManager;

// Calibration data structure
struct CalibrationData {
  // Accelerometer
  float accel_offset[3] = {0, 0, 0};  // X, Y, Z
  float accel_scale[3] = {1, 1, 1};   // X, Y, Z
  
  // Gyroscope
  float gyro_offset[3] = {0, 0, 0};   // X, Y, Z
  float gyro_scale[3] = {1, 1, 1};    // X, Y, Z
  
  // Magnetometer (includes hard and soft iron correction)
  float mag_offset[3] = {0, 0, 0};    // X, Y, Z (hard iron)
  float mag_scale[3] = {1, 1, 1};     // X, Y, Z (soft iron)
  float mag_transform[3][3] = {       // Soft iron transform matrix
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // Temperature compensation
  float temp_ref = 25.0;              // Reference temperature
  float temp_coef_accel[3] = {0, 0, 0}; // Temperature coefficients for accel
  float temp_coef_gyro[3] = {0, 0, 0};  // Temperature coefficients for gyro
};

class CalibrationManager {
public:
  CalibrationManager();
  
  // Initialize with references to other managers
  void initialize(SensorManager* sensorMgr, FilterManager* filterMgr);
  
  // Calibration procedures
  void performFullCalibration();
  void calibrateGyros();
  void calibrateAccelerometers();
  void calibrateMagnetometers();
  void calibrateTemperatures();
  
  // Apply calibration to sensor readings
  void calibrateAccelData(int sensorId, float raw_x, float raw_y, float raw_z, 
                         float temp, float &cal_x, float &cal_y, float &cal_z);
  void calibrateGyroData(int sensorId, float raw_x, float raw_y, float raw_z, 
                        float temp, float &cal_x, float &cal_y, float &cal_z);
  void calibrateMagData(int sensorId, float raw_x, float raw_y, float raw_z, 
                       float &cal_x, float &cal_y, float &cal_z);
  
  // Temperature compensation
  float compensateForTemperature(float value, float temp_coef, float temp, float temp_ref);
  
  // Utility functions
  void printCalibrationData();
  
  // Access calibration data (for storage)
  CalibrationData* getCalibrationData(int sensorId);
  void setCalibrationData(int sensorId, const CalibrationData& data);
  size_t getCalibrationDataSize() const;
  CalibrationData* getAllCalibrationData();

private:
  SensorManager* sensorManager;
  FilterManager* filterManager;
  CalibrationData calibrationData[NO_OF_UNITS];
};

#endif // CALIBRATION_MANAGER_H
