/*
 * FilterManager.h
 * Handles filtering and sensor fusion for IMU data
 */

#ifndef FILTER_MANAGER_H
#define FILTER_MANAGER_H

#include "SensorManager.h"

// Kalman filter state structure
struct KalmanState {
  float x;      // State
  float p;      // Estimation error covariance
  float q;      // Process noise covariance
  float r;      // Measurement noise covariance
  float k;      // Kalman gain
};

class FilterManager {
public:
  FilterManager();
  
  // Initialize with sensor manager
  void initialize(SensorManager* sensorMgr);
  
  // Reset timers for complementary filter
  void resetTimers();
  
  // Process and filter sensors
  void processAllSensors();
  
  // Individual filter implementations
  float applyMovingAverage(float new_value, float buffer[], int &buffer_idx);
  float applyKalmanFilter(float measurement, KalmanState &state);
  
  // Apply complementary filter for orientation
  void updateComplementaryFilter(int sensorId, float accel_x, float accel_y, float accel_z,
                              float gyro_x, float gyro_y, float gyro_z,
                              float mag_x, float mag_y, float mag_z);
  
  // Get filtered data values
  void getFilteredAccel(int sensorId, float &x, float &y, float &z);
  void getFilteredGyro(int sensorId, float &x, float &y, float &z);
  void getFilteredMag(int sensorId, float &x, float &y, float &z);
  void getOrientation(int sensorId, float &roll, float &pitch, float &yaw);

private:
  SensorManager* sensorManager;
  
  // Filter coefficient (0.98 = 98% gyro, 2% accel)
  static constexpr float ALPHA = 0.98f;
  
  // Moving average filter buffers
  static const int FILTER_SAMPLES = 10;
  float accel_x_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float accel_y_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float accel_z_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float gyro_x_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float gyro_y_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float gyro_z_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float mag_x_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float mag_y_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  float mag_z_buffer[NO_OF_UNITS][FILTER_SAMPLES];
  int buffer_index[NO_OF_UNITS];
  
  // Kalman filter states
  KalmanState kalman_accel_x[NO_OF_UNITS];
  KalmanState kalman_accel_y[NO_OF_UNITS];
  KalmanState kalman_accel_z[NO_OF_UNITS];
  KalmanState kalman_gyro_x[NO_OF_UNITS];
  KalmanState kalman_gyro_y[NO_OF_UNITS];
  KalmanState kalman_gyro_z[NO_OF_UNITS];
  
  // Complementary filter variables
  float comp_angle_x[NO_OF_UNITS];  // Roll
  float comp_angle_y[NO_OF_UNITS];  // Pitch
  float comp_angle_z[NO_OF_UNITS];  // Yaw
  unsigned long last_time[NO_OF_UNITS];
  
  // Storage for filtered sensor data
  struct FilteredData {
    float accel[3];  // X, Y, Z
    float gyro[3];   // X, Y, Z
    float mag[3];    // X, Y, Z
  };
  
  FilteredData filteredData[NO_OF_UNITS];
  
  // Initialize filter states
  void initializeFilters();

  void applySpinalConstraints(int sensorId);

};

#endif // FILTER_MANAGER_H
