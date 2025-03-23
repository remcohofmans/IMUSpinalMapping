/*
 * FilterManager.cpp
 * Enhanced filtering and sensor fusion for spinal IMU monitoring
 */

#include "FilterManager.h"
#include "CalibrationManager.h"  
#include <math.h>

FilterManager::FilterManager() : 
  adaptiveFilteringEnabled(true),
  anatomicalConstraintsEnabled(false),
  useQuaternions(false) {
  
  // Initialize buffers to zero and tracking flags to false
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
    kalman_accel_x[i] = {0, 1, 0.0001f, 0.001f, 0};
    kalman_accel_y[i] = {0, 1, 0.0001f, 0.001f, 0};
    kalman_accel_z[i] = {0, 1, 0.0001f, 0.001f, 0};

    // Gyroscope (spine-optimized)
    kalman_gyro_x[i] = {0, 1, 0.0001f, 0.001f, 0};  // Axial rotation
    kalman_gyro_y[i] = {0, 1, 0.001f, 0.001f, 0};   // Flexion/Extension
    kalman_gyro_z[i] = {0, 1, 0.001f, 0.001f, 0};   // Lateral Bending

    // Magnetometer
    kalman_mag_x[i] = {0, 1, 0.0001f, 0.001f, 0};
    kalman_mag_y[i] = {0, 1, 0.0001f, 0.001f, 0};
    kalman_mag_z[i] = {0, 1, 0.0001f, 0.001f, 0};

    // Initialize Euler angles from first sensor reading    
    // First declare floats and populate with raw sensor data
    float ax, ay, az, mx, my, mz;

    sensorManager->getRawAccel(i, ax, ay, az);
    sensorManager->getRawMag(i, mx, my, mz);
    float temp = sensorManager->getTemperature(i);

    // Declare floats to store the calibrated data
    float c_ax, c_ay, c_az, c_mx, c_my, c_mz;

    calibrationManager->calibrateAccelData(i, ax, ay, az, temp, c_ax, c_ay, c_az);
    calibrationManager->calibrateMagData(i, mx, my, mz, c_mx, c_my, c_mz);

    // Apply axis mapping before calculating angles
    // calibrationManager->transformSensorAxes(ax, ay, az, axisMapping, axisSigns);
    // calibrationManager->transformSensorAxes(mx, my, mz, axisMapping, axisSigns);

    // Normalize accel
    float accel_mag = sqrt(c_ax * c_ax + c_ay * c_ay + c_az * c_az);
    if (accel_mag > 0.001f) {
      c_ax /= accel_mag;
      c_ay /= accel_mag;
      c_az /= accel_mag;
    }

    // Calculate initial pitch and roll using the aircraft formulas
    EulerAngles initial;

    // Calculate pitch using square root method to decouple from roll angle
    // This prevents pitch from being affected by device rotation around X-axis
    // When device is rolled 90°, Z approaches 0 while Y becomes large
    // Using sqrt(ay²+az²) ensures stable pitch calculation in all orientations
    initial.pitch = atan2(-c_ax, sqrt(c_ay * c_ay + c_az * c_az)) * 180.0f / M_PI;

    // Calculate roll as rotation between Y and Z axes
    // Note: This calculation becomes unstable when pitch approaches ±90°!
    // due to gimbal lock inherent in Euler angle representations
    initial.roll = atan2(c_ay, c_az) * 180.0f / M_PI;

    // Tilt-compensated magnetometer yaw
    float pitch_rad = initial.pitch * M_PI / 180.0f;
    float roll_rad = initial.roll * M_PI / 180.0f;

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
    }

    // Apply tilt compensation to project magnetic field vector onto horizontal plane
    // First, rotate around X-axis (roll) and then around Y-axis (pitch)
    // This transforms magnetometer readings as if the device were perfectly horizontal
    float mx_h = c_mx * cos_pitch + c_my * sin_roll * sin_pitch + c_mz * cos_roll * sin_pitch;
    float my_h = c_my * cos_roll - c_mz * sin_roll;

    // Calculate heading (yaw) using the horizontally projected magnetic field components
    // The magnetic declination adjusts for difference between magnetic and true geographic north
    initial.yaw = atan2(my_h, mx_h) * 180.0f / M_PI + MAGNETIC_DECLINATION;
    initial.yaw = fmod((initial.yaw + 360.0f), 360.0f); // Yaw angle between 0 and 360 degrees -> standard compass convention

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

float FilterManager::applyMovingAverage(float new_value, float buffer[], int &buffer_idx) {
  // Add new value to buffer
  buffer[buffer_idx] = new_value;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < 3; i++) {
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

void FilterManager::normalizeQuaternion(Quaternion &q) {
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

void FilterManager::eulerToQuaternion(const EulerAngles &euler, Quaternion &q) {
  // Convert Euler angles to quaternion
  // Using Z-Y-X convention (yaw, pitch, roll) which corresponds to
  // Lateral Bending, Flexion/Extension, Axial Rotation in the spine model
  
  // Convert degrees to radians
  float roll_rad = euler.roll * M_PI / 180.0f;  // Axial Rotation
  float pitch_rad = euler.pitch * M_PI / 180.0f; // Flexion/Extension
  float yaw_rad = euler.yaw * M_PI / 180.0f;    // Lateral Bending
  
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
    if (dt > 0.2f) dt = 0.2f;
    if (dt < 0.001f) dt = 0.001f;
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
    float fa_x = applyKalmanFilter(applyMovingAverage(cal_ax, accel_x_buffer[sensorId], buffer_index[sensorId]), kalman_accel_x[sensorId]);
    float fa_y = applyKalmanFilter(applyMovingAverage(cal_ay, accel_y_buffer[sensorId], buffer_index[sensorId]), kalman_accel_y[sensorId]);
    float fa_z = applyKalmanFilter(applyMovingAverage(cal_az, accel_z_buffer[sensorId], buffer_index[sensorId]), kalman_accel_z[sensorId]);

    float fgx = applyKalmanFilter(cal_gx, kalman_gyro_x[sensorId]);
    float fgy = applyKalmanFilter(cal_gy, kalman_gyro_y[sensorId]);
    float fgz = applyKalmanFilter(cal_gz, kalman_gyro_z[sensorId]);

    float fmx = applyKalmanFilter(applyMovingAverage(cal_mx, mag_x_buffer[sensorId], buffer_index[sensorId]), kalman_mag_x[sensorId]);
    float fmy = applyKalmanFilter(applyMovingAverage(cal_my, mag_y_buffer[sensorId], buffer_index[sensorId]), kalman_mag_y[sensorId]);
    float fmz = applyKalmanFilter(applyMovingAverage(cal_mz, mag_z_buffer[sensorId], buffer_index[sensorId]), kalman_mag_z[sensorId]);

    filteredData[sensorId].accel[0] = fa_x;
    filteredData[sensorId].accel[1] = fa_y;
    filteredData[sensorId].accel[2] = fa_z;
    filteredData[sensorId].gyro[0] = fgx;
    filteredData[sensorId].gyro[1] = fgy;
    filteredData[sensorId].gyro[2] = fgz;
    filteredData[sensorId].mag[0] = fmx;
    filteredData[sensorId].mag[1] = fmy;
    filteredData[sensorId].mag[2] = fmz;

    // Sanity check accel magnitude to avoid NaN
    float accel_mag = sqrt(fa_x*fa_x + fa_y*fa_y + fa_z*fa_z);
    if (accel_mag < 1e-3f) {
      Serial.println("Warning: accel magnitude too low, skipping orientation update.");
      continue;
    }

    // Catch and fix NaN Euler angles
    if (isnan(euler_angles[sensorId].roll) || isnan(euler_angles[sensorId].pitch) || isnan(euler_angles[sensorId].yaw)) {
      Serial.println("NaN detected in Euler angles. Resetting orientation.");
      euler_angles[sensorId] = {0, 0, 0};
    }

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
  // When a disturbance is detected, the current magnetic reading is considered unreliable or abnormal
  // By not updating the reference value during a disturbance, the system preserves the last known "good" reference value 
  // (allows for faster recovery when disturbance ends)
}

void FilterManager::updateEulerAngles(int sensorId, float dt) {
  if (dt <= 0.0f || isnan(dt)) return;

  float ax = filteredData[sensorId].accel[0];
  float ay = filteredData[sensorId].accel[1];
  float az = filteredData[sensorId].accel[2];

  float gx = filteredData[sensorId].gyro[0];
  float gy = filteredData[sensorId].gyro[1];
  float gz = filteredData[sensorId].gyro[2];

  float mx = filteredData[sensorId].mag[0];
  float my = filteredData[sensorId].mag[1];
  float mz = filteredData[sensorId].mag[2];

  // NEW DEBUG CODE - Add this
  if (sensorId == 1) {
    Serial.print("Filtered accel: x="); Serial.print(ax, 6);
    Serial.print(", y="); Serial.print(ay, 6);
    Serial.print(", z="); Serial.print(az, 6);
    Serial.println();
    
    Serial.print("Filtered gyro: x="); Serial.print(gx, 6);
    Serial.print(", y="); Serial.print(gy, 6);
    Serial.print(", z="); Serial.print(gz, 6);
    Serial.println();
    
    Serial.print("Filtered mag: x="); Serial.print(mx, 6);
    Serial.print(", y="); Serial.print(my, 6);
    Serial.print(", z="); Serial.print(mz, 6);
    Serial.println();
  }

  if (isnan(ax) || isnan(ay) || isnan(az) || isnan(mx) || isnan(my) || isnan(mz)) {
    Serial.println("Invalid sensor data — Euler angles not updated!");
    return;
  }

  float accel_mag = sqrt(ax * ax + ay * ay + az * az);
  bool reliable_accel = (accel_mag > 9.5f && accel_mag < 10.1f);

  // Pitch calculation uses square root approach to decouple it from roll (Z-Y-X parenting/ rotation sequence)
  // This ensures accurate pitch measurement regardless of roll angle
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

  // Roll is calculated as rotation around X axis using Y and Z components
  float roll = atan2(ay, az) * 180.0f / M_PI;

  float mag_norm = sqrt(mx * mx + my * my + mz * mz);
  if (mag_norm <= 0.0001f) {
    Serial.println("Mag norm too low — skipping yaw computation");
    return;
  }

  mx /= mag_norm;
  my /= mag_norm;
  mz /= mag_norm;

  float pitch_rad = pitch * M_PI / 180.0f;
  float roll_rad = roll * M_PI / 180.0f;
  float cos_pitch = cos(pitch_rad);
  float sin_pitch = sin(pitch_rad);
  float cos_roll = cos(roll_rad);
  float sin_roll = sin(roll_rad);

  float mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
  float my_h = my * cos_roll - mz * sin_roll;

  float yaw = atan2(my_h, mx_h) * 180.0f / M_PI + MAGNETIC_DECLINATION;

  if (yaw < 0) yaw += 360.0f;
  if (yaw > 360.0f) yaw -= 360.0f;

  if (isnan(roll) || isnan(pitch) || isnan(yaw)) {
    Serial.println("Euler result is NaN — resetting to 0");
    roll = pitch = yaw = 0;
  }

  float alpha_roll = ALPHA;
  float alpha_pitch = reliable_accel ? 0.90f : 0.98f;
  float alpha_yaw = (magDisturbance[sensorId]) ? 0.995f : 0.90f;

  // euler_angles[sensorId].roll = alpha_roll * (euler_angles[sensorId].roll + gx * dt * 180.0f / M_PI) + (1 - alpha_roll) * roll;
  // euler_angles[sensorId].pitch = alpha_pitch * (euler_angles[sensorId].pitch + gy * dt * 180.0f / M_PI) + (1 - alpha_pitch) * pitch;
  // euler_angles[sensorId].yaw = alpha_yaw * (euler_angles[sensorId].yaw + gz * dt * 180.0f / M_PI) + (1 - alpha_yaw) * yaw;
  euler_angles[sensorId].roll = roll;
  euler_angles[sensorId].pitch = pitch;
  euler_angles[sensorId].yaw = yaw;

    // NEW DEBUG CODE - Add this at the end of the function before the closing bracket
  if (sensorId == 1) {
    Serial.print("FINAL Euler angles: roll="); Serial.print(euler_angles[sensorId].roll, 6);
    Serial.print(", pitch="); Serial.print(euler_angles[sensorId].pitch, 6);
    Serial.print(", yaw="); Serial.print(euler_angles[sensorId].yaw, 6);
    Serial.println();
  }
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
  if (accel_mag > 0.001f) {
    accel_x /= accel_mag;
    accel_y /= accel_mag;
    accel_z /= accel_mag;
  }
  
  // Normalize magnetometer data
  float mag_mag = sqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
  if (mag_mag > 0.001f) {
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
  matrixToQuaternion(R, q_obs);
  
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
  normalizeQuaternion(q_gyro);
  
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
  normalizeQuaternion(quaternions[sensorId]);
  
  // Update Euler angles from quaternion for convenience
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

  // NEW DEBUG CODE - Add this
  if (sensorId == 1) {
    Serial.println("DEBUG getOrientation for Sensor 1");
    Serial.print("Returning: roll="); Serial.print(roll, 6);
    Serial.print(", pitch="); Serial.print(pitch, 6);
    Serial.print(", yaw="); Serial.print(yaw, 6);
    Serial.println();
  }
}

