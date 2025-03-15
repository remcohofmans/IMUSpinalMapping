/*
 * FilterManager.cpp
 * Enhanced filtering and sensor fusion for spinal IMU monitoring
 */

#include "FilterManager.h"
#include "CalibrationManager.h"  
#include <math.h>

FilterManager::FilterManager() : 
  lastTime(0),
  adaptiveFilteringEnabled(true),
  anatomicalConstraintsEnabled(true),
  useQuaternions(true) {
  
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
    euler_angles[i].roll = 0;
    euler_angles[i].pitch = 0;
    euler_angles[i].yaw = 0;
    
    // Initialize quaternion to identity
    quaternions[i].w = 1.0f;
    quaternions[i].x = 0.0f;
    quaternions[i].y = 0.0f;
    quaternions[i].z = 0.0f;
    
    // Initialize sensor-to-segment alignment matrices to identity
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        alignmentMatrix[i][r][c] = (r == c) ? 1.0f : 0.0f;
      }
    }
    
    // Initialize magnetic disturbance detection
    magDisturbance[i] = false;
    lastMagMagnitude[i] = 0.0f;
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
    
    // Gyroscope Kalman filters - configured for spine movements
    // X-axis rotation rate - Axial Rotation
    kalman_gyro_x[i].x = 0;
    kalman_gyro_x[i].p = 1;
    kalman_gyro_x[i].q = 0.0006;  // Higher restriction axial rotation
    kalman_gyro_x[i].r = 0.035;   // Slightly increased measurement noise
    
    // Y-axis rotation rate - Flexion/Extension
    kalman_gyro_y[i].x = 0;
    kalman_gyro_y[i].p = 1;
    kalman_gyro_y[i].q = 0.0009;  // Lower restriction for flexion/extension
    kalman_gyro_y[i].r = 0.03;    // Standard measurement noise
    
    // Z-axis rotation rate - Lateral Bending
    kalman_gyro_z[i].x = 0;
    kalman_gyro_z[i].p = 1;
    kalman_gyro_z[i].q = 0.0007;  // Moderate restriction for lateral bending
    kalman_gyro_z[i].r = 0.03;    // Standard measurement noise
  }
}

void FilterManager::resetTimers() {
  unsigned long now = millis();
  lastTime = now;
}

void FilterManager::configureFiltering(bool enableAdaptiveFiltering, bool enableAnatomicalConstraints, bool useQuaternionMode) {
  adaptiveFilteringEnabled = enableAdaptiveFiltering;
  anatomicalConstraintsEnabled = enableAnatomicalConstraints;
  useQuaternions = useQuaternionMode;
}

// ===== Enhanced Filtering Algorithms =====

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
  state.p = state.p + state.q;
  
  // Update
  state.k = state.p / (state.p + state.r);
  state.x = state.x + state.k * (measurement - state.x);
  state.p = (1 - state.k) * state.p;
  
  return state.x;
}

// ===== Quaternion and Matrix Operations =====

void FilterManager::normalize_quaternion(Quaternion &q) {
  float magnitude = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (magnitude > 0.0f) {
    q.w /= magnitude;
    q.x /= magnitude;
    q.y /= magnitude;
    q.z /= magnitude;
  } else {
    // If magnitude is zero, set to identity quaternion
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
  }
}

void FilterManager::euler_to_quaternion(const EulerAngles &euler, Quaternion &q) {
  // Convert Euler angles to quaternion
  // Using Z-Y-X convention (yaw, pitch, roll) which corresponds to
  // Lateral Bending, Flexion/Extension, Axial Rotation in the spine model
  
  // Convert degrees to radians
  float roll_rad = euler.roll * M_PI / 180.0f;  // Axial Rotation
  float pitch_rad = euler.pitch * M_PI / 180.0f; // Flexion/Extension
  float yaw_rad = euler.yaw * M_PI / 180.0f;    // Lateral Bending
  
  // Calculate trig identities
  float cr = cos(roll_rad * 0.5f);
  float sr = sin(roll_rad * 0.5f);
  float cp = cos(pitch_rad * 0.5f);
  float sp = sin(pitch_rad * 0.5f);
  float cy = cos(yaw_rad * 0.5f);
  float sy = sin(yaw_rad * 0.5f);
  
  // Compute quaternion components using Z-Y-X convention
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  
  // Ensure normalized quaternion
  normalize_quaternion(q);
}

void FilterManager::quaternion_to_euler(const Quaternion &q, EulerAngles &euler) {
  // Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
  // This uses the Z-Y-X convention (yaw, pitch, roll) which corresponds to
  // Lateral Bending, Flexion/Extension, Axial Rotation in the spine model
  
  // Calculate roll (x-axis rotation) - Axial Rotation
  float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  euler.roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
  
  // Calculate pitch (y-axis rotation) - Flexion/Extension
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1.0f) {
    // Use 90 degrees if out of range (north or south pole)
    euler.pitch = copysign(90.0f, sinp) * 180.0f / M_PI;
  } else {
    euler.pitch = asin(sinp) * 180.0f / M_PI;
  }
  
  // Calculate yaw (z-axis rotation) - Lateral Bending
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  euler.yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

void FilterManager::matrix_to_quaternion(float R[3][3], Quaternion &q) {
  // Method to convert rotation matrix to quaternion
  float trace = R[0][0] + R[1][1] + R[2][2];
  
  if (trace > 0) {
    float S = sqrt(trace + 1.0f) * 2.0f;
    q.w = 0.25f * S;
    q.x = (R[2][1] - R[1][2]) / S;
    q.y = (R[0][2] - R[2][0]) / S;
    q.z = (R[1][0] - R[0][1]) / S;
  } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
    float S = sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
    q.w = (R[2][1] - R[1][2]) / S;
    q.x = 0.25f * S;
    q.y = (R[0][1] + R[1][0]) / S;
    q.z = (R[0][2] + R[2][0]) / S;
  } else if (R[1][1] > R[2][2]) {
    float S = sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
    q.w = (R[0][2] - R[2][0]) / S;
    q.x = (R[0][1] + R[1][0]) / S;
    q.y = 0.25f * S;
    q.z = (R[1][2] + R[2][1]) / S;
  } else {
    float S = sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
    q.w = (R[1][0] - R[0][1]) / S;
    q.x = (R[0][2] + R[2][0]) / S;
    q.y = (R[1][2] + R[2][1]) / S;
    q.z = 0.25f * S;
  }
  
  normalize_quaternion(q);
}

void FilterManager::slerp(const Quaternion &q1, const Quaternion &q2, float t, Quaternion &result) {
  // Spherical Linear Interpolation (SLERP) between two quaternions
  // q1: starting quaternion
  // q2: ending quaternion
  // t: interpolation factor (0.0 to 1.0)
  // result: interpolated quaternion
  
  // Calculate dot product (cosine of angle between quaternions)
  float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
  
  // Ensure we interpolate along the shortest path
  Quaternion q2_adj = q2;
  if (dot < 0.0f) {
    q2_adj.w = -q2.w;
    q2_adj.x = -q2.x;
    q2_adj.y = -q2.y;
    q2_adj.z = -q2.z;
    dot = -dot;
  }
  
  // If quaternions are close, use linear interpolation to avoid divide-by-zero
  const float DOT_THRESHOLD = 0.9995f;
  if (dot > DOT_THRESHOLD) {
    result.w = q1.w + t * (q2_adj.w - q1.w);
    result.x = q1.x + t * (q2_adj.x - q1.x);
    result.y = q1.y + t * (q2_adj.y - q1.y);
    result.z = q1.z + t * (q2_adj.z - q1.z);
    normalize_quaternion(result);
    return;
  }
  
  // Calculate interpolation parameters
  float theta_0 = acos(dot);  // Angle between quaternions
  float theta = theta_0 * t;  // Angle for interpolation
  
  float sin_theta = sin(theta);
  float sin_theta_0 = sin(theta_0);
  
  // Calculate interpolation coefficients
  float s0 = cos(theta) - dot * sin_theta / sin_theta_0;
  float s1 = sin_theta / sin_theta_0;
  
  // Interpolate
  result.w = s0 * q1.w + s1 * q2_adj.w;
  result.x = s0 * q1.x + s1 * q2_adj.x;
  result.y = s0 * q1.y + s1 * q2_adj.y;
  result.z = s0 * q1.z + s1 * q2_adj.z;
  
  // Normalize result
  normalize_quaternion(result);
}

// ===== Sensor Fusion Implementation =====

void FilterManager::processAllSensors() {
  if (!sensorManager) {
    Serial.println("WARNING: SensorManager not initialized!");
    return;
  }
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;
  
  // Prevent large time jumps
  if (dt > 0.2f) dt = 0.2f;
  if (dt < 0.001f) dt = 0.001f; // Also prevent very small dt
  
  for (int sensorId = 0; sensorId < NO_OF_UNITS; sensorId++) {
    if (!sensorManager->isSensorActive(sensorId)) continue;
    
    // Get sensor readings
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float temperature;
    
    // Get raw readings
    sensorManager->getRawAccel(sensorId, accel_x, accel_y, accel_z);
    sensorManager->getRawGyro(sensorId, gyro_x, gyro_y, gyro_z);
    sensorManager->getRawMag(sensorId, mag_x, mag_y, mag_z);
    temperature = sensorManager->getTemperature(sensorId);
    
    // Apply calibration
    float cal_accel_x, cal_accel_y, cal_accel_z;
    float cal_gyro_x, cal_gyro_y, cal_gyro_z;
    float cal_mag_x, cal_mag_y, cal_mag_z;
    
    if (calibrationManager) {
      calibrationManager->calibrateAccelData(sensorId, accel_x, accel_y, accel_z, 
                                           temperature, cal_accel_x, cal_accel_y, cal_accel_z);
      calibrationManager->calibrateGyroData(sensorId, gyro_x, gyro_y, gyro_z, 
                                          temperature, cal_gyro_x, cal_gyro_y, cal_gyro_z);
      calibrationManager->calibrateMagData(sensorId, mag_x, mag_y, mag_z, 
                                         cal_mag_x, cal_mag_y, cal_mag_z);
    } else {
      // No calibration manager, use raw values
      cal_accel_x = accel_x;
      cal_accel_y = accel_y;
      cal_accel_z = accel_z;
      cal_gyro_x = gyro_x;
      cal_gyro_y = gyro_y;
      cal_gyro_z = gyro_z;
      cal_mag_x = mag_x;
      cal_mag_y = mag_y;
      cal_mag_z = mag_z;
    }
    
    // Apply Moving Average filter
    float ma_accel_x = applyMovingAverage(cal_accel_x, accel_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_accel_y = applyMovingAverage(cal_accel_y, accel_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_accel_z = applyMovingAverage(cal_accel_z, accel_z_buffer[sensorId], buffer_index[sensorId]);
    
    float ma_gyro_x = applyMovingAverage(cal_gyro_x, gyro_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_gyro_y = applyMovingAverage(cal_gyro_y, gyro_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_gyro_z = applyMovingAverage(cal_gyro_z, gyro_z_buffer[sensorId], buffer_index[sensorId]);
    
    float ma_mag_x = applyMovingAverage(cal_mag_x, mag_x_buffer[sensorId], buffer_index[sensorId]);
    float ma_mag_y = applyMovingAverage(cal_mag_y, mag_y_buffer[sensorId], buffer_index[sensorId]);
    float ma_mag_z = applyMovingAverage(cal_mag_z, mag_z_buffer[sensorId], buffer_index[sensorId]);
    
    // Apply Kalman filter
    filteredData[sensorId].accel[0] = applyKalmanFilter(ma_accel_x, kalman_accel_x[sensorId]);
    filteredData[sensorId].accel[1] = applyKalmanFilter(ma_accel_y, kalman_accel_y[sensorId]);
    filteredData[sensorId].accel[2] = applyKalmanFilter(ma_accel_z, kalman_accel_z[sensorId]);
    
    filteredData[sensorId].gyro[0] = applyKalmanFilter(ma_gyro_x, kalman_gyro_x[sensorId]);
    filteredData[sensorId].gyro[1] = applyKalmanFilter(ma_gyro_y, kalman_gyro_y[sensorId]);
    filteredData[sensorId].gyro[2] = applyKalmanFilter(ma_gyro_z, kalman_gyro_z[sensorId]);
    
    filteredData[sensorId].mag[0] = ma_mag_x;
    filteredData[sensorId].mag[1] = ma_mag_y;
    filteredData[sensorId].mag[2] = ma_mag_z;
    
    // Detect magnetic disturbances
    detectMagneticDisturbance(sensorId, ma_mag_x, ma_mag_y, ma_mag_z);
    
    // Apply sensor fusion algorithm
    if (useQuaternions) {
      updateQuaternionOrientation(sensorId, dt);
    } else {
      updateEulerAngles(sensorId, dt);
    }
    
    // Apply anatomical constraints if enabled
    if (anatomicalConstraintsEnabled) {
      applySpinalConstraints(sensorId);
    }
  }
}

void FilterManager::detectMagneticDisturbance(int sensorId, float mag_x, float mag_y, float mag_z) {
  // Calculate current magnitude of magnetic field
  float magMagnitude = sqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
  
  // If this is the first reading, initialize
  if (lastMagMagnitude[sensorId] == 0.0f) {
    lastMagMagnitude[sensorId] = magMagnitude;
    magDisturbance[sensorId] = false;
    return;
  }
  
  // Check for substantial change in magnitude, which indicates disturbance
  const float DISTURBANCE_THRESHOLD = 0.15f; // 15% change
  float percentChange = fabsf(magMagnitude - lastMagMagnitude[sensorId]) / lastMagMagnitude[sensorId];
  
  magDisturbance[sensorId] = (percentChange > DISTURBANCE_THRESHOLD);
  
  // Gradually update reference magnitude with a low-pass filter
  // This allows adaptation to slowly changing environments
  if (!magDisturbance[sensorId]) {
    lastMagMagnitude[sensorId] = 0.95f * lastMagMagnitude[sensorId] + 0.05f * magMagnitude;
  }
}

void FilterManager::updateEulerAngles(int sensorId, float dt) {
  // Get filtered sensor data
  float accel_x = filteredData[sensorId].accel[0];
  float accel_y = filteredData[sensorId].accel[1];
  float accel_z = filteredData[sensorId].accel[2];
  
  float gyro_x = filteredData[sensorId].gyro[0];
  float gyro_y = filteredData[sensorId].gyro[1];
  float gyro_z = filteredData[sensorId].gyro[2];
  
  float mag_x = filteredData[sensorId].mag[0];
  float mag_y = filteredData[sensorId].mag[1];
  float mag_z = filteredData[sensorId].mag[2];
  
  // ----- Adapted to the spinal coordinate system -----
  // X-axis points down the spine
  // Y-axis points to the left
  // Z-axis points out of the back
  
  // Calculate magnitude of acceleration for tilt reliability estimation
  float accel_mag = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  
  // Check alignment with gravity for adaptive filtering
  float x_alignment = fabs(accel_x / accel_mag);
  float reliable_accel = (accel_mag > 9.5f && accel_mag < 10.1f); // Near 9.8 m/s²
  
  // Calculate tilt angles from accelerometer
  float accel_roll = atan2(accel_y, accel_z) * 180.0f / M_PI;
  float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0f / M_PI;
  
  // Apply tilt compensation to magnetometer readings
  float cos_roll = cos(euler_angles[sensorId].roll * M_PI / 180.0f);
  float sin_roll = sin(euler_angles[sensorId].roll * M_PI / 180.0f);
  float cos_pitch = cos(euler_angles[sensorId].pitch * M_PI / 180.0f);
  float sin_pitch = sin(euler_angles[sensorId].pitch * M_PI / 180.0f);
  
  // Rotate magnetometer readings to horizontal plane
  float mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch;
  float mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;
  
  // Calculate yaw (heading) from tilt-compensated magnetometer
  float mag_yaw = atan2(-mag_y_comp, mag_x_comp) * 180.0f / M_PI;
  
  // Handle magnetic declination if needed
  mag_yaw += MAGNETIC_DECLINATION;
  
  // Calculate new angles using complementary filter with adaptive weights
  float alpha_roll = ALPHA;
  float alpha_pitch = ALPHA;
  float alpha_yaw = ALPHA;
  
  if (adaptiveFilteringEnabled) {
    // Adjust weights based on sensor reliability
    
    // For roll - less reliable when X aligned with gravity
    if (x_alignment > 0.9f) {
      alpha_roll = 0.98f; // More weight to gyro
    } else if (!reliable_accel) {
      alpha_roll = 0.95f; // Less weight to potentially noisy accelerometer
    }
    
    // For pitch - generally reliable with accelerometer
    if (!reliable_accel) {
      alpha_pitch = 0.95f; // Less weight to potentially noisy accelerometer
    }
    
    // For yaw - less reliable when magnetic disturbance detected
    if (magDisturbance[sensorId]) {
      alpha_yaw = 0.999f; // Almost entirely rely on gyro during disturbance
    }
  }
  
  // Integrate gyro rates
  float gyro_roll = euler_angles[sensorId].roll + gyro_x * dt;
  float gyro_pitch = euler_angles[sensorId].pitch + gyro_y * dt;
  float gyro_yaw = euler_angles[sensorId].yaw + gyro_z * dt;
  
  // Normalize mag_yaw to prevent jumps at ±180°
  while (mag_yaw - gyro_yaw > 180.0f) mag_yaw -= 360.0f;
  while (mag_yaw - gyro_yaw < -180.0f) mag_yaw += 360.0f;
  
  // Apply complementary filter
  float new_roll = alpha_roll * gyro_roll + (1.0f - alpha_roll) * accel_roll;
  float new_pitch = alpha_pitch * gyro_pitch + (1.0f - alpha_pitch) * accel_pitch;
  float new_yaw = alpha_yaw * gyro_yaw + (1.0f - alpha_yaw) * mag_yaw;
  
  // Normalize angles to -180° to +180° range
  while (new_roll > 180.0f) new_roll -= 360.0f;
  while (new_roll < -180.0f) new_roll += 360.0f;
  while (new_pitch > 180.0f) new_pitch -= 360.0f;
  while (new_pitch < -180.0f) new_pitch += 360.0f;
  while (new_yaw > 180.0f) new_yaw -= 360.0f;
  while (new_yaw < -180.0f) new_yaw += 360.0f;
  
  // Update Euler angles
  euler_angles[sensorId].roll = new_roll;   // Axial rotation
  euler_angles[sensorId].pitch = new_pitch; // Flexion/Extension
  euler_angles[sensorId].yaw = new_yaw;     // Lateral bending
  
  // Update quaternion from Euler angles for consistency
  euler_to_quaternion(euler_angles[sensorId], quaternions[sensorId]);
}

void FilterManager::updateQuaternionOrientation(int sensorId, float dt) {
  // Get filtered sensor data
  float accel_x = filteredData[sensorId].accel[0];
  float accel_y = filteredData[sensorId].accel[1];
  float accel_z = filteredData[sensorId].accel[2];
  
  float gyro_x = filteredData[sensorId].gyro[0]; // rad/s
  float gyro_y = filteredData[sensorId].gyro[1];
  float gyro_z = filteredData[sensorId].gyro[2];
  
  float mag_x = filteredData[sensorId].mag[0];
  float mag_y = filteredData[sensorId].mag[1];
  float mag_z = filteredData[sensorId].mag[2];
  
  // Normalize accelerometer data
  float accel_mag = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  if (accel_mag > 0.0f) {
    accel_x /= accel_mag;
    accel_y /= accel_mag;
    accel_z /= accel_mag;
  }
  
  // Normalize magnetometer data
  float mag_mag = sqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
  if (mag_mag > 0.0f) {
    mag_x /= mag_mag;
    mag_y /= mag_mag;
    mag_z /= mag_mag;
  }
  
  // Check for reliable sensor data
  bool reliable_accel = (accel_mag > 9.5f && accel_mag < 10.1f); // Near 9.8 m/s²
  bool reliable_mag = !magDisturbance[sensorId];
  
  // Calculate reference direction of Earth's magnetic field
  // First, get the heading vector in the horizontal plane
  // This involves creating a right-handed orthogonal system using the down direction (from accelerometer)
  // and the magnetic field direction
  
  // Down direction (gravity) is negative of the normalized accelerometer vector
  float down_x = -accel_x;
  float down_y = -accel_y;
  float down_z = -accel_z;
  
  // East direction is perpendicular to both magnetic field and down direction
  float east_x = mag_y * down_z - mag_z * down_y;
  float east_y = mag_z * down_x - mag_x * down_z;
  float east_z = mag_x * down_y - mag_y * down_x;
  
  // Normalize east direction
  float east_mag = sqrt(east_x*east_x + east_y*east_y + east_z*east_z);
  if (east_mag > 0.0f) {
    east_x /= east_mag;
    east_y /= east_mag;
    east_z /= east_mag;
  }
  
  // North direction is perpendicular to both east and down (cross product)
  float north_x = down_y * east_z - down_z * east_y;
  float north_y = down_z * east_x - down_x * east_z;
  float north_z = down_x * east_y - down_y * east_x;
  
  // Create rotation matrix from reference frame to sensor frame
  float R[3][3];
  R[0][0] = north_x; R[0][1] = east_x; R[0][2] = down_x;
  R[1][0] = north_y; R[1][1] = east_y; R[1][2] = down_y;
  R[2][0] = north_z; R[2][1] = east_z; R[2][2] = down_z;
  
  // Convert rotation matrix to quaternion (observation quaternion)
  Quaternion q_obs;
  matrix_to_quaternion(R, q_obs);
  
  // Use Madgwick or Mahony algorithm for quaternion updates
  // This is a simplified implementation of Madgwick's algorithm
  
  // Integrate gyroscope data to get orientation change
  float half_dt = dt * 0.5f;
  Quaternion q_dot;
  q_dot.w = -half_dt * (gyro_x * quaternions[sensorId].x + gyro_y * quaternions[sensorId].y + gyro_z * quaternions[sensorId].z);
  q_dot.x = half_dt * (gyro_x * quaternions[sensorId].w + gyro_z * quaternions[sensorId].y - gyro_y * quaternions[sensorId].z);
  q_dot.y = half_dt * (gyro_y * quaternions[sensorId].w - gyro_z * quaternions[sensorId].x + gyro_x * quaternions[sensorId].z);
  q_dot.z = half_dt * (gyro_z * quaternions[sensorId].w + gyro_y * quaternions[sensorId].x - gyro_x * quaternions[sensorId].y);
  
  // Update quaternion based on gyro
  Quaternion q_gyro;
  q_gyro.w = quaternions[sensorId].w + q_dot.w;
  q_gyro.x = quaternions[sensorId].x + q_dot.x;
  q_gyro.y = quaternions[sensorId].y + q_dot.y;
  q_gyro.z = quaternions[sensorId].z + q_dot.z;
  
  // Normalize gyro quaternion
  normalize_quaternion(q_gyro);
  
  // Determine fusion weight (beta) based on sensor reliability
  float beta = 0.1f; // Default value
  
  if (adaptiveFilteringEnabled) {
    if (!reliable_accel || !reliable_mag) {
      beta = 0.01f; // Much less weight to observations if sensors unreliable
    }
  }
  
  // SLERP between gyro quaternion and observation quaternion
  slerp(q_gyro, q_obs, beta, quaternions[sensorId]);
  
  // Normalize final quaternion
  normalize_quaternion(quaternions[sensorId]);
  
  // Update Euler angles from quaternion for convenience
  quaternion_to_euler(quaternions[sensorId], euler_angles[sensorId]);
}

void FilterManager::applySpinalConstraints(int sensorId) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    return;
  }
  
  // Get current angles
  float axial_rotation, flexion_extension, lateral_bending;
  
  if (useQuaternions) {
    axial_rotation = euler_angles[sensorId].roll;
    flexion_extension = euler_angles[sensorId].pitch;
    lateral_bending = euler_angles[sensorId].yaw;
  } else {
    axial_rotation = euler_angles[sensorId].roll;
    flexion_extension = euler_angles[sensorId].pitch;
    lateral_bending = euler_angles[sensorId].yaw;
  }
  
  // Apply constraints based on sensor position
  if (sensorId == 0) {
    // Thoracic limits
    // Axial Rotation (around X)
    if (axial_rotation > 46.8f) axial_rotation = 46.8f;
    if (axial_rotation < -46.8f) axial_rotation = -46.8f;
    
    // Flexion/Extension (around Y)
    if (flexion_extension > 26.0f) flexion_extension = 26.0f;   // Flexion limit
    if (flexion_extension < -22.0f) flexion_extension = -22.0f; // Extension limit
    
    // Lateral Bending (around Z)
    if (lateral_bending > 30.0f) lateral_bending = 30.0f;
    if (lateral_bending < -30.0f) lateral_bending = -30.0f;
  } 
  else if (sensorId == 1) {
    // Lumbar limits
    // Axial Rotation (around X)
    if (axial_rotation > 15.3f) axial_rotation = 15.3f;
    if (axial_rotation < -15.3f) axial_rotation = -15.3f;
    
    // Flexion/Extension (around Y)
    if (flexion_extension > 65.0f) flexion_extension = 65.0f;   // Flexion limit
    if (flexion_extension < -31.0f) flexion_extension = -31.0f; // Extension limit
    
    // Lateral Bending (around Z)
    if (lateral_bending > 30.0f) lateral_bending = 30.0f;
    if (lateral_bending < -30.0f) lateral_bending = -30.0f;
  }
  
  // Apply coupling between lateral bending and axial rotation
  // This mimics the natural coupling in spinal movement
  if (abs(lateral_bending) > 10.0f) {
    // When lateral bending exceeds 10 degrees, couple with axial rotation
    float coupling_factor = (sensorId == 0) ? 0.3f : 0.1f;
    float coupled_rotation = axial_rotation + 
      (lateral_bending - (lateral_bending > 0 ? 10.0f : -10.0f)) * coupling_factor;
    
    // Apply limits again
    if (sensorId == 0) {
      if (coupled_rotation > 46.8f) coupled_rotation = 46.8f;
      if (coupled_rotation < -46.8f) coupled_rotation = -46.8f;
    } else {
      if (coupled_rotation > 15.3f) coupled_rotation = 15.3f;
      if (coupled_rotation < -15.3f) coupled_rotation = -15.3f;
    }
    
    axial_rotation = coupled_rotation;
  }
  
  // Update the angles
  if (useQuaternions) {
    euler_angles[sensorId].roll = axial_rotation;
    euler_angles[sensorId].pitch = flexion_extension;
    euler_angles[sensorId].yaw = lateral_bending;
    
    // Convert back to quaternion
    euler_to_quaternion(euler_angles[sensorId], quaternions[sensorId]);
  } else {
    euler_angles[sensorId].roll = axial_rotation;
    euler_angles[sensorId].pitch = flexion_extension;
    euler_angles[sensorId].yaw = lateral_bending;
  }
}

void FilterManager::getFilteredAccel(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0.0f;
    return;
  }
  
  x = filteredData[sensorId].accel[0];
  y = filteredData[sensorId].accel[1];
  z = filteredData[sensorId].accel[2];
}

void FilterManager::getFilteredGyro(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0.0f;
    return;
  }
  
  x = filteredData[sensorId].gyro[0];
  y = filteredData[sensorId].gyro[1];
  z = filteredData[sensorId].gyro[2];
}

void FilterManager::getFilteredMag(int sensorId, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    x = y = z = 0.0f;
    return;
  }
  
  x = filteredData[sensorId].mag[0];
  y = filteredData[sensorId].mag[1];
  z = filteredData[sensorId].mag[2];
}

void FilterManager::getOrientation(int sensorId, float &roll, float &pitch, float &yaw) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    roll = pitch = yaw = 0.0f;
    return;
  }
  
  roll = euler_angles[sensorId].roll;   // Axial Rotation
  pitch = euler_angles[sensorId].pitch; // Flexion/Extension
  yaw = euler_angles[sensorId].yaw;     // Lateral Bending
}

void FilterManager::calibrateSensorToSegmentAlignment(int sensorId) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    return;
  }
  
  Serial.println("=== Sensor-to-Segment Alignment Calibration ===");
  Serial.println("This procedure will align the sensor axes with anatomical axes.");
  Serial.println("Please follow the instructions carefully:");
  
  // Step 1: Neutral position calibration
  Serial.println("\nSTEP 1: Place the person in a neutral standing position");
  Serial.println("Press any key when ready...");
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read(); // Clear buffer
  
  delay(3000); // Give time to stabilize
  
  // Read current orientation
  float roll1, pitch1, yaw1;
  getOrientation(sensorId, roll1, pitch1, yaw1);
  
  // Step 2: Forward flexion calibration
  Serial.println("\nSTEP 2: Have the person perform a forward flexion (bend forward)");
  Serial.println("Press any key when at maximum flexion...");
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
  
  delay(1000);
  
  // Read flexed orientation
  float roll2, pitch2, yaw2;
  getOrientation(sensorId, roll2, pitch2, yaw2);
  
  // Calculate the primary flexion axis
  float flex_axis[3] = {0, 0, 0};
  
  // The dominant change should be in pitch (Y-axis rotation)
  if (fabs(pitch2 - pitch1) > fabs(roll2 - roll1) && fabs(pitch2 - pitch1) > fabs(yaw2 - yaw1)) {
    // Correct alignment - Y is the flexion axis
    Serial.println("Y-axis correctly aligned with flexion/extension");
    flex_axis[1] = 1.0f;
  } else if (fabs(roll2 - roll1) > fabs(pitch2 - pitch1) && fabs(roll2 - roll1) > fabs(yaw2 - yaw1)) {
    // X-axis is capturing flexion - need to realign
    Serial.println("X-axis is capturing flexion/extension - realigning");
    flex_axis[0] = 1.0f;
  } else {
    // Z-axis is capturing flexion - need to realign
    Serial.println("Z-axis is capturing flexion/extension - realigning");
    flex_axis[2] = 1.0f;
  }
  
  // Step 3: Lateral bending calibration
  Serial.println("\nSTEP 3: Have the person perform a lateral bend (to the side)");
  Serial.println("Press any key when at maximum lateral bend...");
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
  
  delay(1000);
  
  // Read laterally bent orientation
  float roll3, pitch3, yaw3;
  getOrientation(sensorId, roll3, pitch3, yaw3);
  
  // Calculate the primary lateral bending axis
  float lateral_axis[3] = {0, 0, 0};
  
  // The dominant change should be in yaw (Z-axis rotation)
  if (fabs(yaw3 - yaw1) > fabs(roll3 - roll1) && fabs(yaw3 - yaw1) > fabs(pitch3 - pitch1)) {
    // Correct alignment - Z is the lateral bending axis
    Serial.println("Z-axis correctly aligned with lateral bending");
    lateral_axis[2] = 1.0f;
  } else if (fabs(roll3 - roll1) > fabs(yaw3 - yaw1) && fabs(roll3 - roll1) > fabs(pitch3 - pitch1)) {
    // X-axis is capturing lateral bending - need to realign
    Serial.println("X-axis is capturing lateral bending - realigning");
    lateral_axis[0] = 1.0f;
  } else {
    // Y-axis is capturing lateral bending - need to realign
    Serial.println("Y-axis is capturing lateral bending - realigning");
    lateral_axis[1] = 1.0f;
  }
  
  // Calculate third axis as cross product of first two
  float axial_axis[3];
  axial_axis[0] = flex_axis[1] * lateral_axis[2] - flex_axis[2] * lateral_axis[1];
  axial_axis[1] = flex_axis[2] * lateral_axis[0] - flex_axis[0] * lateral_axis[2];
  axial_axis[2] = flex_axis[0] * lateral_axis[1] - flex_axis[1] * lateral_axis[0];
  
  // Normalize axial axis
  float mag = sqrt(axial_axis[0]*axial_axis[0] + axial_axis[1]*axial_axis[1] + axial_axis[2]*axial_axis[2]);
  if (mag > 0.0f) {
    axial_axis[0] /= mag;
    axial_axis[1] /= mag;
    axial_axis[2] /= mag;
  }
  
  // Create alignment rotation matrix
  alignmentMatrix[sensorId][0][0] = axial_axis[0];
  alignmentMatrix[sensorId][0][1] = axial_axis[1];
  alignmentMatrix[sensorId][0][2] = axial_axis[2];
  
  alignmentMatrix[sensorId][1][0] = flex_axis[0];
  alignmentMatrix[sensorId][1][1] = flex_axis[1];
  alignmentMatrix[sensorId][1][2] = flex_axis[2];
  
  alignmentMatrix[sensorId][2][0] = lateral_axis[0];
  alignmentMatrix[sensorId][2][1] = lateral_axis[1];
  alignmentMatrix[sensorId][2][2] = lateral_axis[2];
  
  Serial.println("\nSensor-to-segment alignment calibration complete!");
  Serial.println("The sensor axes are now aligned with anatomical axes:");
  Serial.println("X-axis = Axial Rotation");
  Serial.println("Y-axis = Flexion/Extension");
  Serial.println("Z-axis = Lateral Bending");
}