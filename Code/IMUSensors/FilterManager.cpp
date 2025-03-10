/*
 * FilterManager.cpp
 * Implementation of the FilterManager class
 */

#include "FilterManager.h"
#include <math.h>

FilterManager::FilterManager() {
  // Initialize buffers to zero
  for (int i = 0; i < NO_OF_UNITS; i++) {
    for (int j = 0; j < FILTER_SAMPLES; j++) {
      accel_x_buffer[i][j] = 0;
      accel_y_buffer[i][j] = 0;
      accel_z_buffer[i][j] = 0;
      gyro_x_buffer[i][j] = 0;
      gyro_y_buffer[i][j] = 0;
      gyro_z_buffer[i][j] = 0;
      mag_x_buffer[i][j] = 0;
      mag_y_buffer[i][j] = 0;
      mag_z_buffer[i][j] = 0;
    }
    buffer_index[i] = 0;
    
    // Initialize complementary filter variables
    comp_angle_x[i] = 0;
    comp_angle_y[i] = 0;
    comp_angle_z[i] = 0;
    last_time[i] = 0;
  }
}

void FilterManager::initialize(SensorManager* sensorMgr) {
  sensorManager = sensorMgr;
  initializeFilters();
}

void FilterManager::initializeFilters() {
  // Initialize Kalman filter states for all sensors
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensorManager->isSensorActive(i)) continue;
    
    // Accelerometer Kalman filters
    kalman_accel_x[i].x = 0;
    kalman_accel_x[i].p = 1;
    kalman_accel_x[i].q = 0.01;
    kalman_accel_x[i].r = 0.1;
    
    kalman_accel_y[i].x = 0;
    kalman_accel_y[i].p = 1;
    kalman_accel_y[i].q = 0.01;
    kalman_accel_y[i].r = 0.1;
    
    kalman_accel_z[i].x = 0;
    kalman_accel_z[i].p = 1;
    kalman_accel_z[i].q = 0.01;
    kalman_accel_z[i].r = 0.1;
    
    // Gyroscope Kalman filters
    kalman_gyro_x[i].x = 0;
    kalman_gyro_x[i].p = 1;
    kalman_gyro_x[i].q = 0.001;
    kalman_gyro_x[i].r = 0.03;

    kalman_gyro_y[i].x = 0;
    kalman_gyro_y[i].p = 1;
    kalman_gyro_y[i].q = 0.001;
    kalman_gyro_y[i].r = 0.03;

    kalman_gyro_z[i].x = 0;
    kalman_gyro_z[i].p = 1;
    kalman_gyro_z[i].q = 0.001;
    kalman_gyro_z[i].r = 0.03;
  }
}

void FilterManager::resetTimers() {
  unsigned long now = millis();
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (sensorManager->isSensorActive(i)) {
      last_time[i] = now;
    }
  }
}

float FilterManager::applyMovingAverage(float new_value, float buffer[], int &buffer_idx) {
  // Add new value to buffer
  buffer[buffer_idx] = new_value;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  
  // Advance buffer index with wraparound
  buffer_idx = (buffer_idx + 1) % FILTER_SAMPLES;
  
  return sum / FILTER_SAMPLES;
}

float FilterManager::applyKalmanFilter(float measurement, KalmanState &state) {
  // Prediction
  // x = x (no state transition model for constant value)
  // p = p + q
  state.p = state.p + state.q;
  
  // Update
  state.k = state.p / (state.p + state.r);
  state.x = state.x + state.k * (measurement - state.x);
  state.p = (1 - state.k) * state.p;
  
  return state.x;
}

void FilterManager::updateComplementaryFilter(int sensorId, float accel_x, float accel_y, float accel_z,
                                            float gyro_x, float gyro_y, float gyro_z,
                                            float mag_x, float mag_y, float mag_z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    return;
  }
  
  // Calculate dt
  unsigned long now = millis();
  float dt = (now - last_time[sensorId]) / 1000.0;
  last_time[sensorId] = now;
  
  // Prevent large time jumps
  if (dt > 0.2f) dt = 0.2f;
  
  // Calculate angles from accelerometer (roll & pitch only, not yaw)
  float accel_angle_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
  float accel_angle_y = atan2(-accel_x, accel_z) * 180.0 / M_PI;
  
  // Calculate yaw from magnetometer (tilt-compensated compass)
  
  // Tilt compensation for magnetometer
  float cosRoll = cos(comp_angle_x[sensorId] * M_PI / 180.0);
  float sinRoll = sin(comp_angle_x[sensorId] * M_PI / 180.0);
  float cosPitch = cos(comp_angle_y[sensorId] * M_PI / 180.0);
  float sinPitch = sin(comp_angle_y[sensorId] * M_PI / 180.0);
  
  // Tilt compensated magnetic field X
  float tiltCompensatedX = mag_x * cosPitch + mag_y * sinRoll * sinPitch + mag_z * cosPitch * sinPitch;
  // Tilt compensated magnetic field Y
  float tiltCompensatedY = mag_y * cosRoll - mag_z * sinRoll;
  // Magnetic yaw
  float mag_angle_z = atan2(-tiltCompensatedY, tiltCompensatedX) * 180.0 / M_PI;
  
  // Complementary filter for roll and pitch
  comp_angle_x[sensorId] = ALPHA * (comp_angle_x[sensorId] + gyro_x * dt * 180.0 / M_PI) + (1.0 - ALPHA) * accel_angle_x;
  comp_angle_y[sensorId] = ALPHA * (comp_angle_y[sensorId] + gyro_y * dt * 180.0 / M_PI) + (1.0 - ALPHA) * accel_angle_y;
  
  // Complementary filter for yaw using magnetometer
  float gyro_angle_z = comp_angle_z[sensorId] + gyro_z * dt * 180.0 / M_PI;
  
  // Normalize mag_angle_z to be within ±180° of the current gyro-based yaw
  while (mag_angle_z - gyro_angle_z > 180) mag_angle_z -= 360;
  while (mag_angle_z - gyro_angle_z < -180) mag_angle_z += 360;
  
  // Yaw uses magnetometer for reference
  comp_angle_z[sensorId] = ALPHA * gyro_angle_z + (1.0 - ALPHA) * mag_angle_z;
}

void FilterManager::processAllSensors() {
  for (int sensorId = 0; sensorId < NO_OF_UNITS; sensorId++) {
    if (!sensorManager->isSensorActive(sensorId)) continue;
    
    // Get raw sensor readings
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    
    sensorManager->getRawAccel(sensorId, accel_x, accel_y, accel_z);
    sensorManager->getRawGyro(sensorId, gyro_x, gyro_y, gyro_z);
    sensorManager->getRawMag(sensorId, mag_x, mag_y, mag_z);
    float temp = sensorManager->getTemperature(sensorId);
    
    // Apply Moving Average filter
    float ma_accel_x = applyMovingAverage(accel_x, accel_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_accel_y = applyMovingAverage(accel_y, accel_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_accel_z = applyMovingAverage(accel_z, accel_z_buffer[sensorId], buffer_index[sensorId]);
    
    float ma_gyro_x = applyMovingAverage(gyro_x, gyro_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_gyro_y = applyMovingAverage(gyro_y, gyro_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_gyro_z = applyMovingAverage(gyro_z, gyro_z_buffer[sensorId], buffer_index[sensorId]);
    
    float ma_mag_x = applyMovingAverage(mag_x, mag_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_mag_y = applyMovingAverage(mag_y, mag_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_mag_z = applyMovingAverage(mag_z, mag_z_buffer[sensorId], buffer_index[sensorId]);
    
    // Apply Kalman filter to accelerometer and gyroscope data
    filteredData[sensorId].accel[0] = applyKalmanFilter(ma_accel_x, kalman_accel_x[sensorId]);
    filteredData[sensorId].accel[1] = applyKalmanFilter(ma_accel_y, kalman_accel_y[sensorId]);
    filteredData[sensorId].accel[2] = applyKalmanFilter(ma_accel_z, kalman_accel_z[sensorId]);
    
    filteredData[sensorId].gyro[0] = applyKalmanFilter(ma_gyro_x, kalman_gyro_x[sensorId]);
    filteredData[sensorId].gyro[1] = applyKalmanFilter(ma_gyro_y, kalman_gyro_y[sensorId]);
    filteredData[sensorId].gyro[2] = applyKalmanFilter(ma_gyro_z, kalman_gyro_z[sensorId]);
    
    // Magnetometer uses just moving average, no Kalman
    filteredData[sensorId].mag[0] = ma_mag_x;
    filteredData[sensorId].mag[1] = ma_mag_y;
    filteredData[sensorId].mag[2] = ma_mag_z;
    
    // Update orientation using complementary filter
    updateComplementaryFilter(
      sensorId,
      filteredData[sensorId].accel[0], filteredData[sensorId].accel[1], filteredData[sensorId].accel[2],
      filteredData[sensorId].gyro[0], filteredData[sensorId].gyro[1], filteredData[sensorId].gyro[2],
      filteredData[sensorId].mag[0], filteredData[sensorId].mag[1], filteredData[sensorId].mag[2]
    );
  }
}

void FilterManager::getFilteredAccel(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0;
    return;
  }
  
  x = filteredData[sensorId].accel[0];
  y = filteredData[sensorId].accel[1];
  z = filteredData[sensorId].accel[2];
}

void FilterManager::getFilteredGyro(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0;
    return;
  }
  
  x = filteredData[sensorId].gyro[0];
  y = filteredData[sensorId].gyro[1];
  z = filteredData[sensorId].gyro[2];
}

void FilterManager::getFilteredMag(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0;
    return;
  }
  
  x = filteredData[sensorId].mag[0];
  y = filteredData[sensorId].mag[1];
  z = filteredData[sensorId].mag[2];
}

void FilterManager::getOrientation(int sensorId, float &roll, float &pitch, float &yaw) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    roll = pitch = yaw = 0;
    return;
  }
  
  roll = comp_angle_x[sensorId];
  pitch = comp_angle_y[sensorId];
  yaw = comp_angle_z[sensorId];
}