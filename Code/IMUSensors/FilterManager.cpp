/*
 * FilterManager.cpp
 * Enhanced filtering and sensor fusion for spinal IMU monitoring
 */

#include "FilterManager.h"
#include "CalibrationManager.h"  
#include <math.h>
#include <cppQueue.h>


FilterManager::FilterManager() : 
  adaptiveFilteringEnabled(true),
  anatomicalConstraintsEnabled(false),
  useQuaternions(false) {
  
  // Initialize buffers to zero and tracking flags to false
  for (int i = 0; i < NO_OF_UNITS; i++) {

    // Initialize buffer status flags to false
    accel_buffer_initialized[i] = false;
    gyro_buffer_initialized[i] = false;
    mag_buffer_initialized[i] = false;
    
    // Initialize complementary filter variables
    euler_angles[i].roll = 0;
    euler_angles[i].pitch = 0;
    euler_angles[i].yaw = 0;
    
    // Initialize quaternion to identity
    quaternions[i].w = 1.0f;
    quaternions[i].x = 0.0f;
    quaternions[i].y = 0.0f;
    quaternions[i].z = 0.0f;
    
    // Initialize magnetic disturbance detection
    magDisturbance[i] = false;
    lastMagMagnitude[i] = 0.0f;
  }
}

void FilterManager::initialize(SensorManager* sensorMgr, CalibrationManager* calMgr) {
  sensorManager = sensorMgr;
  calibrationManager = calMgr;
  initializeFilters();
}

void FilterManager::initializeFilters() {
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensorManager->isSensorActive(i)) continue;

    // Kalman filters
    // Accelerometer
    // Accelerometer - balance between stability and responsiveness
    kalman_accel_x[i] = {0, 1, 0.001f, 0.001f, 0};
    kalman_accel_y[i] = {0, 1, 0.001f, 0.001f, 0};
    kalman_accel_z[i] = {0, 1, 0.001f, 0.001f, 0};

    // Gyroscope - spine-specific optimizations
    kalman_gyro_x[i] = {0, 1, 0.0002f, 0.0005f, 0};
    kalman_gyro_y[i] = {0, 1, 0.0008f, 0.0008f, 0};
    kalman_gyro_z[i] = {0, 1, 0.0004f, 0.0008f, 0};  

    // Magnetometer - less critical for relative spine movement
    kalman_mag_x[i] = {0, 1, 0.0005f, 0.002f, 0};  
    kalman_mag_y[i] = {0, 1, 0.0005f, 0.002f, 0};  
    kalman_mag_z[i] = {0, 1, 0.0005f, 0.002f, 0};  

    // Initialize Euler angles from first sensor reading    
    // First get raw sensor data
    sensorManager->readAllSensors();
    float ax, ay, az, mx, my, mz, gx, gy, gz;

    sensorManager->getRawAccel(i, ax, ay, az);
    sensorManager->getRawMag(i, mx, my, mz);
    sensorManager->getRawGyro(i, gx, gy, gz);
    float temp = sensorManager->getTemperature(i);

    // Then calibrate it
    float c_ax, c_ay, c_az, c_mx, c_my, c_mz, c_gx, c_gy, c_gz;

    calibrationManager->calibrateAccelData(i, ax, ay, az, temp, c_ax, c_ay, c_az);
    calibrationManager->calibrateMagData(i, mx, my, mz, c_mx, c_my, c_mz);
    calibrationManager->calibrateGyroData(i, gx, gy, gz, temp, c_gx, c_gy, c_gz);

    // Initialize buffers with first readings if not already initialized
    if (!accel_buffer_initialized[i]) {
      for (int j = 0; j < FILTER_SAMPLES; j++) {
        // Replace array indexing with set method
        accel_x_buffer[i].set(j, c_ax);
        accel_y_buffer[i].set(j, c_ay);
        accel_z_buffer[i].set(j, c_az);
      }
      accel_buffer_initialized[i] = true;
    }

    if (!gyro_buffer_initialized[i]) {
      for (int j = 0; j < FILTER_SAMPLES; j++) {
        // Replace array indexing with set method
        gyro_x_buffer[i].set(j, c_gx);
        gyro_y_buffer[i].set(j, c_gy);
        gyro_z_buffer[i].set(j, c_gz);
      }
      gyro_buffer_initialized[i] = true;
    }

    if (!mag_buffer_initialized[i]) {
      for (int j = 0; j < FILTER_SAMPLES; j++) {
        // Replace array indexing with set method
        mag_x_buffer[i].set(j, c_mx);
        mag_y_buffer[i].set(j, c_my);
        mag_z_buffer[i].set(j, c_mz);
      }
      mag_buffer_initialized[i] = true;
    }

    // Apply axis mapping before calculating angles
    // calibrationManager->transformSensorAxes(ax, ay, az, axisMapping, axisSigns);
    // calibrationManager->transformSensorAxes(mx, my, mz, axisMapping, axisSigns);

    // Normalize accel
    float accel_mag = sqrt(c_ax * c_ax + c_ay * c_ay + c_az * c_az);
    float EPSILON = 0.01f;
    if (accel_mag > EPSILON) {
      c_ax /= accel_mag;
      c_ay /= accel_mag;
      c_az /= accel_mag;
    } else {
      // Default gravity direction if accelerometer reading is invalid
      c_ax = 0.0f;
      c_ay = 0.0f;
      c_az = 1.0f;
    }

    // Calculate initial pitch and roll using the aircraft formulas
    EulerAngles initial;

    // Calculate pitch using square root method to decouple from roll angle
    // This prevents pitch from being affected by device rotation around X-axis
    // When device is rolled 90°, Z approaches 0 while Y becomes large
    // Using sqrt(ay²+az²) ensures stable pitch calculation in all orientations
    initial.pitch = atan2(-c_ax, sqrt(c_ay * c_ay + c_az * c_az));

    // Calculate roll as rotation between Y and Z axes
    // Note: This calculation becomes unstable when pitch approaches ±90°!
    // due to gimbal lock inherent in Euler angle representations
    initial.roll = atan2(-c_ay, c_az);

    // Tilt-compensated magnetometer yaw
    float pitch_rad = initial.pitch;
    float roll_rad = initial.roll;

    float cos_pitch = cos(pitch_rad);
    float sin_pitch = sin(pitch_rad);
    float cos_roll = cos(roll_rad);
    float sin_roll = sin(roll_rad);

    // The magnitude of the Earth's magnetic field can vary due to local interference (e.g., nearby ferromagnetic materials or electronic devices),
    // sensor noise, temperature fluctuations, and calibration drift.
    // However, for yaw (heading) calculation, we only care about the direction of the magnetic field — not its strength.
    // Normalizing the magnetometer vector eliminates magnitude variability, resulting in a unit vector that preserves direction only.
    float mag_norm = sqrt(c_mx * c_mx + c_my * c_my + c_mz * c_mz);
    if (mag_norm > 0.001f) {
      c_mx /= mag_norm;
      c_my /= mag_norm;
      c_mz /= mag_norm;
    } else {
      // Default to north if magnetometer reading is invalid
      c_mx = 1.0f;
      c_my = 0.0f;
      c_mz = 0.0f;
    }

    // Apply tilt compensation to project magnetic field vector onto horizontal plane
    // First, rotate around X-axis (roll) and then around Y-axis (pitch)
    // This transforms magnetometer readings as if the device were perfectly horizontal
    float mx_h = c_mx * cos_pitch + c_mz * sin_pitch;
    float my_h = c_mx * sin_roll * sin_pitch + c_my * cos_roll - c_mz * sin_roll * cos_pitch;

    // Calculate heading (yaw) using the horizontally projected magnetic field components
    // The magnetic declination adjusts for difference between magnetic and true geographic north
    initial.yaw = atan2(-my_h, mx_h) * RAD_TO_DEG + MAGNETIC_DECLINATION;
    initial.yaw = fmod((initial.yaw + 360.0f), 360.0f); // Yaw angle between 0 and 360 degrees -> standard compass convention
    if (initial.yaw > 180.0f) { // Convert to -180 to 180 degree range
      initial.yaw -= 360.0f;
    }

    euler_angles[i] = initial;

    // Optionally initialize quaternions as well
    if (useQuaternions) {
      eulerToQuaternion(initial, quaternions[i]);
    }
  }
}

void FilterManager::configureFiltering(bool enableAdaptiveFiltering, bool enableAnatomicalConstraints, bool useQuaternionMode) {
  adaptiveFilteringEnabled = enableAdaptiveFiltering;
  anatomicalConstraintsEnabled = enableAnatomicalConstraints;
  useQuaternions = useQuaternionMode;
}

// ===== Enhanced Filtering Algorithms =====

float FilterManager::applyMovingAverage(float new_value, SensorQueue& queue) {

  // Enqueue the new value
  if (queue.isFull()) {
    float temp;
    queue.pop(&temp); // Remove the oldest value if the queue is full
  }
  queue.push(new_value); // Pass the value directly, not a pointer to it
  
  // Compute the weighted average
  float sum = 0;
  float weight_sum = 0;
  float temp;
  for (int i = 0; i < queue.getCount(); i++) {
    queue.peekIdx(&temp, i);
    sum += temp * weights[i];
    weight_sum += weights[i];
  }

  return sum / weight_sum;
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

void FilterManager::normalizeQuaternion(Quaternion &q) {
  float magnitude = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (magnitude > 0.0f) {
    q.w /= magnitude;   // Computationally expensive (how long takes one sample?)
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

void FilterManager::eulerToQuaternion(const EulerAngles &euler, Quaternion &q) {
  // Convert Euler angles to quaternion
  // Using Z-Y-X convention (yaw, pitch, roll) which corresponds to
  // Lateral Bending, Flexion/Extension, Axial Rotation in the spine model
  
  // Convert degrees to radians
  float roll_rad = euler.roll * DEG_TO_RAD;  // Axial Rotation
  float pitch_rad = euler.pitch * DEG_TO_RAD; // Flexion/Extension
  float yaw_rad = euler.yaw * DEG_TO_RAD;    // Lateral Bending
  
  // Calculate trigonometric identities
  float cr = cos(roll_rad * 0.5f); // Use multiplication by 0.5, instead of div by 2 for computational efficiency
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
  normalizeQuaternion(q);
}

void FilterManager::quaternionToEuler(const Quaternion &q, EulerAngles &euler) {
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

void FilterManager::matrixToQuaternion(float R[3][3], Quaternion &q) {
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
  
  normalizeQuaternion(q);
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
    normalizeQuaternion(result);
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
  normalizeQuaternion(result);
}

// ===== Sensor Fusion Implementation -> exploit on-board DMP =====

void FilterManager::processAllSensors() {
  if (!sensorManager) return;

  unsigned long currentTime = millis();

  for (int sensorId = 0; sensorId < NO_OF_UNITS; sensorId++) {
    if (!sensorManager->isSensorActive(sensorId)) continue;

    // Compute per-sensor dt
    float dt = (currentTime - lastSensorTime[sensorId]) / 1000.0f;
    // The gyroscope integration assumes constant angular velocity over the integration period
    if (dt > 0.2f) dt = 0.2f; // Large time steps dt can introduce significant numerical errors, prevents orientation jumps
    if (dt < 0.001f) dt = 0.001f; // Ensures dt is never zero, which would cause division errors
    lastSensorTime[sensorId] = currentTime;

    float ax, ay, az, gx, gy, gz, mx, my, mz, temp;
    sensorManager->getRawAccel(sensorId, ax, ay, az);
    sensorManager->getRawGyro(sensorId, gx, gy, gz);
    sensorManager->getRawMag(sensorId, mx, my, mz);
    temp = sensorManager->getTemperature(sensorId);

    // Apply axis transformation (this was commented out before)
    // calibrationManager->transformSensorAxes(ax, ay, az, axisMapping, axisSigns);
    // calibrationManager->transformSensorAxes(gx, gy, gz, axisMapping, axisSigns);
    // calibrationManager->transformSensorAxes(mx, my, mz, axisMapping, axisSigns);

    // Calibration
    float cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz, cal_mx, cal_my, cal_mz;
    calibrationManager->calibrateAccelData(sensorId, ax, ay, az, temp, cal_ax, cal_ay, cal_az);
    calibrationManager->calibrateGyroData(sensorId, gx, gy, gz, temp, cal_gx, cal_gy, cal_gz);
    calibrationManager->calibrateMagData(sensorId, mx, my, mz, cal_mx, cal_my, cal_mz);

    // Filtered data (basic moving average + Kalman filtering)
    float fa_x = applyKalmanFilter(applyMovingAverage(cal_ax, accel_x_buffer[sensorId]), kalman_accel_x[sensorId]);
    float fa_y = applyKalmanFilter(applyMovingAverage(cal_ay, accel_y_buffer[sensorId]), kalman_accel_y[sensorId]);
    float fa_z = applyKalmanFilter(applyMovingAverage(cal_az, accel_z_buffer[sensorId]), kalman_accel_z[sensorId]);

    float fgx = applyKalmanFilter(cal_gx, kalman_gyro_x[sensorId]);
    float fgy = applyKalmanFilter(cal_gy, kalman_gyro_y[sensorId]);
    float fgz = applyKalmanFilter(cal_gz, kalman_gyro_z[sensorId]);

    float fmx = applyKalmanFilter(cal_mx, kalman_mag_x[sensorId]);
    float fmy = applyKalmanFilter(cal_my, kalman_mag_y[sensorId]);
    float fmz = applyKalmanFilter(cal_mz, kalman_mag_z[sensorId]);

    // Store in the filtered data containers
    filteredData[sensorId].accel[0] = fa_x;
    filteredData[sensorId].accel[1] = fa_y;
    filteredData[sensorId].accel[2] = fa_z;
    filteredData[sensorId].gyro[0] = fgx;
    filteredData[sensorId].gyro[1] = fgy;
    filteredData[sensorId].gyro[2] = fgz;
    filteredData[sensorId].mag[0] = fmx;
    filteredData[sensorId].mag[1] = fmy;
    filteredData[sensorId].mag[2] = fmz;

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
  const float DISTURBANCE_THRESHOLD = 0.20f; // 20% change
  float percentChange = fabsf(magMagnitude - lastMagMagnitude[sensorId]) / lastMagMagnitude[sensorId];
  
  magDisturbance[sensorId] = (percentChange > DISTURBANCE_THRESHOLD);
  
  // Gradually update reference magnitude with a low-pass filter
  // This allows adaptation to slowly changing environments
  if (!magDisturbance[sensorId]) {  
    lastMagMagnitude[sensorId] = 0.98f * lastMagMagnitude[sensorId] + 0.02f * magMagnitude;
  }
  // When a disturbance is detected, the current magnetic reading is considered unreliable or abnormal
  // By not updating the reference value during a disturbance, the system preserves the last known "good" reference value 
  // (allows for faster recovery when disturbance ends)
}

bool FilterManager::detectSpineMovement(int sensorId) {
  // Get current gyro values
  float gx = filteredData[sensorId].gyro[0];
  float gy = filteredData[sensorId].gyro[1];
  float gz = filteredData[sensorId].gyro[2];
  
  // Calculate gyro magnitude
  float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);

  
  return gyro_mag > MOVEMENT_THRESHOLD;
}

void FilterManager::updateEulerAngles(int sensorId, float dt) {
  if (dt <= 0.0f || isnan(dt)) return;

  // Get filtered sensor data
  float ax = filteredData[sensorId].accel[0];
  float ay = filteredData[sensorId].accel[1];
  float az = filteredData[sensorId].accel[2];
  float gx = filteredData[sensorId].gyro[0];
  float gy = filteredData[sensorId].gyro[1];
  float gz = filteredData[sensorId].gyro[2];
  
  // Sanity check inputs
  if (isnan(ax) || isnan(ay) || isnan(az)) return;
  if (isnan(gx) || isnan(gy) || isnan(gz)) return;

  // Check accelerometer reliability by verifying its magnitude is close to gravity (9.81 m/s²)
  // A reliable accelerometer should measure approximately 1g when stationary
  float accel_mag = sqrt(ax*ax + ay*ay + az*az);
  bool reliable_accel = (accel_mag > 9.5f && accel_mag < 11.5f);
  if (!reliable_accel) {  // Accelerometer data is unreliable (may be experiencing external acceleration)
    // Fallback to gyro-only integration for this update cycle
    // This will accumulate drift over time, but avoids corruption from accelerometer spikes!
    euler_angles[sensorId].roll += gx * dt * RAD_TO_DEG;
    euler_angles[sensorId].pitch += gy * dt * RAD_TO_DEG;
    euler_angles[sensorId].yaw += gz * dt * RAD_TO_DEG;
    euler_angles[sensorId].yaw = fmod((euler_angles[sensorId].yaw + 360.0f), 360.0f); // Normalize yaw to -180 to 180 degree range
    if (euler_angles[sensorId].yaw > 180.0f) {
        euler_angles[sensorId].yaw -= 360.0f;
    }
    return;
  }
  
  // Calculate accelerometer-based angles (relies on gravity)
  float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
  float accel_roll = atan2(-ay, az) * RAD_TO_DEG;
  
  // Check if we're in a static position by examining gyro magnitude
  float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);
  
  bool is_static = (gyro_mag < MOVEMENT_THRESHOLD);  // 0.03 rad/s threshold (~1 deg/s)

  float alpha;
  if (is_static) {
    // When static, trust accelerometer more (very small weight assigned to gyro integration)
    alpha = 0.02f;  // 2% gyro, 98% accelerometer
    
    // When static, update gyro bias estimates using current measurements
    CalibrationData* calData = calibrationManager->getCalibrationData(sensorId);
    if (calData) {
      // Low-pass filter for gyro bias estimation
      // When the IMU is stationary, gyroscope should read zero - any readings represent additional bias
      const float BIAS_SMOOTHING_FACTOR = 0.995f;
      const float BIAS_LEARNING_RATE = 1.0f - BIAS_SMOOTHING_FACTOR;
      
      // Update each axis independently
      float new_bias_x = BIAS_SMOOTHING_FACTOR * calData->gyro_offset[0] + BIAS_LEARNING_RATE * gx;
      float new_bias_y = BIAS_SMOOTHING_FACTOR * calData->gyro_offset[1] + BIAS_LEARNING_RATE * gy;
      float new_bias_z = BIAS_SMOOTHING_FACTOR * calData->gyro_offset[2] + BIAS_LEARNING_RATE * gz;
      
      // Prevent extreme bias values that might indicate sensor error rather than drift
      const float MAX_BIAS_CHANGE = 0.01f;  // in rad/s
      if (fabs(new_bias_x - calData->gyro_offset[0]) < MAX_BIAS_CHANGE &&
          fabs(new_bias_y - calData->gyro_offset[1]) < MAX_BIAS_CHANGE &&
          fabs(new_bias_z - calData->gyro_offset[2]) < MAX_BIAS_CHANGE) {
        // Only update bias if changes are within reasonable limits
        calibrationManager->setGyroOffset(sensorId, new_bias_x, new_bias_y, new_bias_z);
      }
    }
  } 
  else {
    // When moving, trust gyro more (large weight to gyro integration)
    alpha = 0.97f;  // 97% gyro, 3% accelerometer
  }
  
  // Integrate angular velocity (gyroscope data) to update orientation
  float gyro_roll = euler_angles[sensorId].roll + gx * dt * RAD_TO_DEG;   // Euler angles were initialized with non-zero value to prevent a lagging system  
  float gyro_pitch = euler_angles[sensorId].pitch + gy * dt * RAD_TO_DEG;
  float gyro_yaw = euler_angles[sensorId].yaw + gz * dt * RAD_TO_DEG;

  // Use tilt-compensated magnetometer to correct gyroscope yaw drift
  // Gyroscopes accumulate error over time (drift), while magnetometers provide absolute heading reference
  // Note: Gyroscopes already benefit from multiple calibration methods for enhanced accuracy:
  // 1. Factory calibration via the DMP (Digital Motion Processor) on the IMU chip
  // 2. Initial zero-bias calibration performed during system startup
  // 3. Continuous auto-calibration during stationary periods (described above; 548-574)
  // 4. Temperature compensation to account for thermal drift (forseen in code, but not yet implemented)
  float mx = filteredData[sensorId].mag[0];
  float my = filteredData[sensorId].mag[1];
  float mz = filteredData[sensorId].mag[2];

  // Normalize magnetometer vector to isolate field direction and eliminate magnitude variations
  float mag_norm = sqrt(mx*mx + my*my + mz*mz);
  if (mag_norm > 0.001f) {
    // Use single division and multiply for better efficiency
    float inv_mag = 1.0f / mag_norm;
    mx *= inv_mag;
    my *= inv_mag;
    mz *= inv_mag;
  } else {
    // Default to theoretical north if magnetometer reading is too weak/invalid
    mx = 1.0f;
    my = 0.0f;
    mz = 0.0f;
  }

  // Compute tilt-compensated yaw
  float pitch_rad = euler_angles[sensorId].pitch * DEG_TO_RAD;
  float roll_rad  = euler_angles[sensorId].roll  * DEG_TO_RAD;

  float cos_pitch = cos(pitch_rad);
  float sin_pitch = sin(pitch_rad);
  float cos_roll  = cos(roll_rad);
  float sin_roll  = sin(roll_rad);

  float mx_h = mx * cos_pitch + mz * sin_pitch;
  float my_h = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

  float mag_yaw = atan2(-my_h, mx_h) * RAD_TO_DEG + MAGNETIC_DECLINATION;
  mag_yaw = fmod((mag_yaw + 360.0f), 360.0f);  // Normalize
  if (mag_yaw > 180.0f) {
    mag_yaw -= 360.0f;
  }

  // Fuse gyro-integrated and mag yaw
  const float yaw_alpha = 0.98f;  // 98% gyro, 2% mag
  euler_angles[sensorId].yaw = yaw_alpha * gyro_yaw + (1.0f - yaw_alpha) * mag_yaw;

  // Normalize fused yaw to -180 to 180 range
  euler_angles[sensorId].yaw = fmod((euler_angles[sensorId].yaw + 360.0f), 360.0f);
  if (euler_angles[sensorId].yaw > 180.0f) {
      euler_angles[sensorId].yaw -= 360.0f;
  }
  
 // Finally, apply complementary filter to roll and pitch
  euler_angles[sensorId].roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
  euler_angles[sensorId].pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;

  // Serial.print("Static: ");
  // Serial.print(is_static ? "TRUE" : "FALSE");
  
  // Sanity check outputs
  if (isnan(euler_angles[sensorId].roll)) euler_angles[sensorId].roll = accel_roll;
  if (isnan(euler_angles[sensorId].pitch)) euler_angles[sensorId].pitch = accel_pitch;
  if (isnan(euler_angles[sensorId].yaw)) euler_angles[sensorId].yaw = 0.0f;
}

void FilterManager::updateQuaternionOrientation(int sensorId, float dt) {
  if (!sensorManager->isSensorActive(sensorId)) return;
  if (dt <= 0.0f || dt > 1.0f) return; 
  // Guard: dt too large => might be a glitch

  // 1) Retrieve filtered sensor data
  float gx = filteredData[sensorId].gyro[0];  // in rad/s
  float gy = filteredData[sensorId].gyro[1];
  float gz = filteredData[sensorId].gyro[2];
  
  float ax = filteredData[sensorId].accel[0];
  float ay = filteredData[sensorId].accel[1];
  float az = filteredData[sensorId].accel[2];
  
  float mx = filteredData[sensorId].mag[0];
  float my = filteredData[sensorId].mag[1];
  float mz = filteredData[sensorId].mag[2];

  // 2) Normalization checks
  float normA = sqrtf(ax*ax + ay*ay + az*az);
  if (normA > 0.001f) {
    ax /= normA;
    ay /= normA;
    az /= normA;
  } else {
    // If no valid accel data, fallback to gyro-only integration
    ax = ay = 0.0f; 
    az = 1.0f; // artificial gravity
  }

  float normM = sqrtf(mx*mx + my*my + mz*mz);
  if (normM > 0.001f) {
    mx /= normM;
    my /= normM;
    mz /= normM;
  } else {
    // If no valid mag data, set to a default "north" vector
    mx = 1.0f; my = 0.0f; mz = 0.0f;
  }
  
  // 3) Current quaternion estimate
  float qw = quaternions[sensorId].w;
  float qx = quaternions[sensorId].x;
  float qy = quaternions[sensorId].y;
  float qz = quaternions[sensorId].z;

  // 4) Gyro integration (i.e., dq/dt = 0.5 * q ⊗ w)
  // Where w = (0, gx, gy, gz), and ⊗ is quaternion multiplication
  float halfDT = 0.5f * dt;
  
  // Predict new orientation from gyro alone
  float qw_dot = -qx*gx - qy*gy - qz*gz;  // qdot.w
  float qx_dot =  qw*gx + qy*gz - qz*gy;  // qdot.x
  float qy_dot =  qw*gy - qx*gz + qz*gx;  // qdot.y
  float qz_dot =  qw*gz + qx*gy - qy*gx;  // qdot.z
  
  // Integrate
  qw += qw_dot * halfDT;
  qx += qx_dot * halfDT;
  qy += qy_dot * halfDT;
  qz += qz_dot * halfDT;

  // 5) Gradient descent correction from accel & mag
  // Steps:
  //   A) Estimated direction of gravity & magnetic field from current q
  //   B) Compute the error between measured direction (ax, ay, az), (mx, my, mz) 
  //      and the estimated direction.  
  //   C) Apply derivative of the error to adjust quaternion.

  // Reference vectors in Earth frame: 
  //   Gravity should be (0, 0, 1)
  //   Magnetic field is (Bx, 0, Bz) after tilt compensation (Madgwick lumps that into a 2D param)
  
  // Convert quaternion to reference direction of gravity & magnetometer
  //   (this is effectively rotating the “Earth frame” vector into sensor frame)
  float vx = 2.0f*(qx*qz - qw*qy);
  float vy = 2.0f*(qw*qx + qy*qz);
  float vz = qw*qw - qx*qx - qy*qy + qz*qz; // for gravity

  // For magnetometer, the algorithm uses an intermediate approach:
  //   Reorients the Earth magnetic field (which should be mostly in XZ-plane for local field)
  //   There's a more complete form in the official Madgwick code. We'll do simplified.

  // Let’s define reference direction of Earth's magnetic field in sensor frame:
  float wx = 2.0f*(mx*(0.5f - qy*qy - qz*qz) + my*(qx*qy - qw*qz) + mz*(qx*qz + qw*qy));
  float wy = 2.0f*(mx*(qx*qy + qw*qz)       + my*(0.5f - qx*qx - qz*qz) + mz*(qy*qz - qw*qx));
  float wz = 2.0f*(mx*(qx*qz - qw*qy)       + my*(qy*qz + qw*qx)        + mz*(0.5f - qx*qx - qy*qy));

  // The “desired” direction of gravity is (ax, ay, az) 
  // The “desired” direction of mag is (mx, my, mz)
  // => Error in sensor frame = cross(estimated, measured)
  
  // Gravity error
  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  // Magnetic error
  float ex_m = (my * wz - mz * wy);
  float ey_m = (mz * wx - mx * wz);
  float ez_m = (mx * wy - my * wx);

  // Combine them
  ex += ex_m;
  ey += ey_m;
  ez += ez_m;

  // 6) Apply a gain, “beta,” to scale the correction 
  float beta = 0.05f;  // typical range: 0.01 -> 0.1
  // If you see slow convergence or big noise, raise or lower this accordingly.

  // Adjust the quaternion derivative
  qw_dot = -beta * ex;
  qx_dot = -beta * ey;
  qy_dot = -beta * ez;
  qz_dot = 0.0f; // Some versions skip the last component or do a different approach. 
                 // The official Madgwick uses a 2D mag approach. This is a simplified version.

  // Integrate the correction
  qw += qw_dot * dt;
  qx += qx_dot * dt;
  qy += qy_dot * dt;
  qz += qz_dot * dt;

  // 7) Normalize the updated quaternion
  float normQ = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
  if (normQ > 0.0f) {
    qw /= normQ;
    qx /= normQ;
    qy /= normQ;
    qz /= normQ;
  } else {
    // fallback if zero
    qw = 1.0f; qx = qy = qz = 0.0f;
  }

  // 8) Store updated quaternion
  quaternions[sensorId].w = qw;
  quaternions[sensorId].x = qx;
  quaternions[sensorId].y = qy;
  quaternions[sensorId].z = qz;

  // 9) Convert to Euler angles for convenience
  quaternionToEuler(quaternions[sensorId], euler_angles[sensorId]);
}

void FilterManager::configureAxisMapping(int xMap, int yMap, int zMap, int xSign, int ySign, int zSign) {
  // Set axis mapping (which input axis maps to which output axis)
  // 0 = X, 1 = Y, 2 = Z
  axisMapping[0] = (xMap >= 0 && xMap <= 2) ? xMap : 0;
  axisMapping[1] = (yMap >= 0 && yMap <= 2) ? yMap : 1;
  axisMapping[2] = (zMap >= 0 && zMap <= 2) ? zMap : 2;
  
  // Set axis signs (1 or -1 to invert axis)  
  axisSigns[0] = (xSign == 1 || xSign == -1) ? xSign : 1;
  axisSigns[1] = (ySign == 1 || ySign == -1) ? ySign : 1;
  axisSigns[2] = (zSign == 1 || zSign == -1) ? zSign : 1;
  
  Serial.println("Axis mapping configured:");
  Serial.print("X maps to axis "); Serial.print(axisMapping[0]); Serial.print(" with sign "); Serial.println(axisSigns[0]);
  Serial.print("Y maps to axis "); Serial.print(axisMapping[1]); Serial.print(" with sign "); Serial.println(axisSigns[1]);
  Serial.print("Z maps to axis "); Serial.print(axisMapping[2]); Serial.print(" with sign "); Serial.println(axisSigns[2]);
}

void FilterManager::applySpinalConstraints(int sensorId) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    return;
  }
  
  // Get current angles
  float axial_rotation, flexion_extension, lateral_bending;
  
    axial_rotation = euler_angles[sensorId].roll;
    flexion_extension = euler_angles[sensorId].pitch;
    lateral_bending = euler_angles[sensorId].yaw;
  
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
    eulerToQuaternion(euler_angles[sensorId], quaternions[sensorId]);
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

