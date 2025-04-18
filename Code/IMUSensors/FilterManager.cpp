/*
 * FilterManager.cpp
 * Enhanced filtering and sensor fusion for spinal IMU monitoring
 */

#include "FilterManager.h"
#include "CalibrationManager.h"  
#include <math.h>
#include <cppQueue.h>


FilterManager::FilterManager() : 
  adaptiveFilteringEnabled(false),
  anatomicalConstraintsEnabled(false),
  useQuaternions(false) {
  
  // Initialize buffers to zero and tracking flags to false
  for (int i = 0; i < NO_OF_UNITS; i++) {

    // Initialize MA buffer status flags to false
    accel_buffer_initialized[i] = false;
    gyro_buffer_initialized[i] = false;
    mag_buffer_initialized[i] = false;
    
    // // Initialize quaternion to identity
    // quaternions[i].w = 1.0f;
    // quaternions[i].x = 0.0f;
    // quaternions[i].y = 0.0f;
    // quaternions[i].z = 0.0f;

    // Create a Mahony filter for each sensor
    mahonyFilters[i] = new AHRS_mahony_filter();
    
    // Initialize magnetic disturbance detection flags to false
    magDisturbance[i] = false;
    lastMagMagnitude[i] = 0.0f;
  }
}

void FilterManager::initialize(SensorManager* sensorMgr, CalibrationManager* calMgr) {
  sensorManager = sensorMgr;
  calibrationManager = calMgr;

  // Initialize Mahony filters
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (sensorManager->isSensorActive(i)) {
      mahonyFilters[i]->begin(FILTER_UPDATE_RATE_HZ); // 100Hz update rate
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

void FilterManager::eulerToQuaternion(const RotationData &euler, Quaternion &q) {
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

void FilterManager::quaternionToEuler(const Quaternion &q, RotationData &euler) {
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

  for (int sensorId = 0; sensorId < NO_OF_UNITS; sensorId++) {
    if (!sensorManager->isSensorActive(sensorId)) continue;

    float ax, ay, az, gx, gy, gz, mx, my, mz, temp;
    sensorManager->getRawAccel(sensorId, ax, ay, az);
    sensorManager->getRawGyro(sensorId, gx, gy, gz);
    sensorManager->getRawMag(sensorId, mx, my, mz);

    // Apply axis transformation
    calibrationManager->transformSensorAxes(ax, ay, az, axisMapping, axisSigns);
    calibrationManager->transformSensorAxes(gx, gy, gz, axisMapping, axisSigns);
    calibrationManager->transformSensorAxes(mx, my, mz, axisMapping, axisSigns);

    // Calibration
    float cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz, cal_mx, cal_my, cal_mz;
    calibrationManager->calibrateAccelData(sensorId, ax, ay, az, temp, cal_ax, cal_ay, cal_az);
    calibrationManager->calibrateGyroData(sensorId, gx, gy, gz, temp, cal_gx, cal_gy, cal_gz);
    calibrationManager->calibrateMagData(sensorId, mx, my, mz, cal_mx, cal_my, cal_mz);

    mahonyFilters[sensorId]->update(cal_gx, cal_gy, cal_gz,
        filteredData[sensorId].accel[0], filteredData[sensorId].accel[1], filteredData[sensorId].accel[2],
        filteredData[sensorId].mag[0], filteredData[sensorId].mag[1], filteredData[sensorId].mag[2]);

    // Filtered data (basic moving average + Kalman filtering)
    float fa_x = cal_ax;
    float fa_y = cal_ay;
    float fa_z = cal_az;

    float fgx = cal_gx;
    float fgy = cal_gy;
    float fgz = cal_gz;

    float fmx = cal_mx;
    float fmy = cal_my;
    float fmz = cal_mz;

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

    updateAngles(sensorId);

    // Apply anatomical constraints if enabled
    if (anatomicalConstraintsEnabled) {
      applySpinalConstraints(sensorId);
    }
  }
}

void CalibrationManager::transformSensorAxes(float &x, float &y, float &z, int axisMapping[3], int axisSigns[3]) {

  // Store original values
  float original[3] = { x, y, z };

  // Remap axes according to mapping and signs
  x = original[axisMapping[0]] * axisSigns[0];
  y = original[axisMapping[1]] * axisSigns[1];
  z = original[axisMapping[2]] * axisSigns[2];

  // Serial.print("Output values axes: x="); Serial.print(x, 6);
  // Serial.print(", y="); Serial.print(y, 6);
  // Serial.print(", z="); Serial.println(z, 6);
  // Serial.println("=====================================");
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

void FilterManager::updateAngles(int sensorId) {

  // Get filtered sensor data
  float ax = filteredData[sensorId].accel[0];
  float ay = filteredData[sensorId].accel[1];
  float az = filteredData[sensorId].accel[2];
  float gx = filteredData[sensorId].gyro[0];
  float gy = filteredData[sensorId].gyro[1];
  float gz = filteredData[sensorId].gyro[2];
  float mx = filteredData[sensorId].mag[0];
  float my = filteredData[sensorId].mag[1];
  float mz = filteredData[sensorId].mag[2];
  
  // Sanity check inputs
  if (isnan(ax) || isnan(ay) || isnan(az)) return;
  if (isnan(gx) || isnan(gy) || isnan(gz)) return;

  // Convert gyro values from rad/s to degrees/s to make compatible with Mahony filter
  float gx_dps = gx * SENSORS_RADS_TO_DPS;
  float gy_dps = gy * SENSORS_RADS_TO_DPS;
  float gz_dps = gz * SENSORS_RADS_TO_DPS;
  
  // Update Mahony filter
  mahonyFilters[sensorId]->update(gx_dps, gy_dps, gz_dps, ax, ay, az, mx, my, mz);
  
  // Get values from Mahony filter
  rotations[sensorId].roll = mahonyFilters[sensorId]->getRoll();
  rotations[sensorId].pitch = mahonyFilters[sensorId]->getPitch();
  rotations[sensorId].yaw = mahonyFilters[sensorId]->getYaw();
  
  // Normalize yaw to -180 to 180 range
  rotations[sensorId].yaw = fmod((rotations[sensorId].yaw + 360.0f), 360.0f);
  if (rotations[sensorId].yaw > 180.0f) {
    rotations[sensorId].yaw -= 360.0f;
  }
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
  
    axial_rotation = rotations[sensorId].roll;
    flexion_extension = rotations[sensorId].pitch;
    lateral_bending = rotations[sensorId].yaw;
  
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
    rotations[sensorId].roll = axial_rotation;
    rotations[sensorId].pitch = flexion_extension;
    rotations[sensorId].yaw = lateral_bending;
    
    // Convert back to quaternion
    eulerToQuaternion(rotations[sensorId], quaternions[sensorId]);
  } else {
    rotations[sensorId].roll = axial_rotation;
    rotations[sensorId].pitch = flexion_extension;
    rotations[sensorId].yaw = lateral_bending;
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
    roll = pitch = yaw = 0;
    return;
  }
  
  roll = mahonyFilters[sensorId]->getRoll();
  pitch = mahonyFilters[sensorId]->getPitch();
  yaw = mahonyFilters[sensorId]->getYaw();
}

void FilterManager::getQuaternion(int sensorId, float &w, float &x, float &y, float &z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    w = x = y = z = 0;
    return;
  }
  mahonyFilters[sensorId]->getQuaternion(&w, &x, &y, &z);
}

