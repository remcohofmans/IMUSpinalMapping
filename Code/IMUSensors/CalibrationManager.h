/*
 * CalibrationManager.h
 * Handles sensor calibration operations and data
 */

#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include "SensorManager.h"
#include "FilterManager.h"

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
  float mag_scale[3] = {1, 1, 1};     // X, Y, Z (soft iron scales - for backwards compatibility)
  float soft_iron_matrix[3][3] = {     // Soft iron transformation matrix
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
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
  void initialize(SensorManager* sensorMgr);
  
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
  
  // Utility functions
  void printCalibrationData();

  void transformSensorAxes(float &x, float &y, float &z, int axisMapping[3], int axisSigns[3]);
  
  // Access calibration data (for storage)
  CalibrationData* getCalibrationData(int sensorId);
  void setGyroOffset (int sensorId, float offsetX, float offsetY, float offsetZ);
  void setCalibrationData(int sensorId, const CalibrationData& data);
  size_t getCalibrationDataSize() const;
  CalibrationData* getAllCalibrationData();
  
private:
  SensorManager* sensorManager;
  FilterManager* filterManager;
  CalibrationData calibrationData[NO_OF_UNITS];
    
  // Helper function for temperature compensation
  float compensateForTemperature(float value, float temp_coef, float temp, float temp_ref);
  
  // Helper function for eigenvalue decomposition
  void eigenDecomposition(float cov[3][3], float eigenvalues[3], float eigenvectors[3][3]);
};

#endif // CALIBRATION_MANAGER_H