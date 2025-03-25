/*
 * FilterManager.h - Enhanced
 * Advanced filtering and sensor fusion for spinal IMU monitoring
 */

#ifndef FILTER_MANAGER_H
#define FILTER_MANAGER_H

#include "SensorManager.h"
#include "SensorQueue.h"


// Forward declaration to avoid circular dependency
class CalibrationManager;

// Kalman filter state structure
struct KalmanState {
  float x;      // State
  float p;      // Estimation error covariance
  float q;      // Process noise covariance
  float r;      // Measurement noise covariance
  float k;      // Kalman gain
};

// Quaternion structure
struct Quaternion {
  float w;      // Scalar part
  float x;      // Vector part (x)
  float y;      // Vector part (y)
  float z;      // Vector part (z)
};

// Euler angles structure
struct EulerAngles {
  float roll;   // Rotation around X axis (Axial Rotation)
  float pitch;  // Rotation around Y axis (Flexion/Extension)
  float yaw;    // Rotation around Z axis (Lateral Bending)
};

class FilterManager {
public:
  FilterManager();
  
  // Initialize with sensor manager
  void initialize(SensorManager* sensorMgr, CalibrationManager* calMgr);
  
  // Configure filtering options
  void configureFiltering(bool enableAdaptiveFiltering, bool enableAnatomicalConstraints, bool useQuaternionMode);
  
  // Process and filter sensors
  void processAllSensors();

  // Individual filter implementations
  float applyMovingAverage(float new_value, SensorQueue& q);
  float applyKalmanFilter(float measurement, KalmanState &state);
  
  // Apply complementary filter for orientation
  void updateComplementaryFilter(int sensorId, float accel_x, float accel_y, float accel_z,
                              float gyro_x, float gyro_y, float gyro_z,
                              float mag_x, float mag_y, float mag_z);

  // Detect magnetic disturbances
  void detectMagneticDisturbance(int sensorId, float mag_x, float mag_y, float mag_z);
  
  // Euler and quaternion update methods
  void updateEulerAngles(int sensorId, float dt);
  void updateQuaternionOrientation(int sensorId, float dt);

  // Configure axis mapping
  void configureAxisMapping(int xMap, int yMap, int zMap, int xSign, int ySign, int zSign);

  // Calculate rotation matrix from current orientation
  void calculateRotationMatrix(int sensorId, float rotMatrix[3][3]);

  // Extract Euler angles from a rotation matrix
  void extractEulerAnglesFromMatrix(float rotMatrix[3][3], float &roll, float &pitch, float &yaw);
  
  // Get filtered data values
  void getFilteredAccel(int sensorId, float &x, float &y, float &z);
  void getFilteredGyro(int sensorId, float &x, float &y, float &z);
  void getFilteredMag(int sensorId, float &x, float &y, float &z);
  void getOrientation(int sensorId, float &roll, float &pitch, float &yaw);
  void getQuaternion(int sensorId, float &w, float &x, float &y, float &z);
  
  // Motion state tracking
  bool inMotion[NO_OF_UNITS] = {false};

  // Analyze gyro data to detect movement
  bool detectSpineMovement(int sensorId);
    
  // Quaternion operations
  void normalizeQuaternion(Quaternion &q);
  void matrixToQuaternion(float R[3][3], Quaternion &q);
  void quaternionToEuler(const Quaternion &q, EulerAngles &euler);
  void eulerToQuaternion(const EulerAngles &euler, Quaternion &q);
  void slerp(const Quaternion &q1, const Quaternion &q2, float t, Quaternion &result);

private:
  SensorManager* sensorManager;
  CalibrationManager* calibrationManager;
  
  // Filter configuration
  bool adaptiveFilteringEnabled;
  bool anatomicalConstraintsEnabled;
  bool useQuaternions;
  
  // Magnetic declination adjustment (in degrees)
  static constexpr float MAGNETIC_DECLINATION = 1.5f; // Constant for Belgium
  
  // Filter coefficient (0.98 = 98% gyro, 2% accel)
  static constexpr float ALPHA = 0.98f;

  static unsigned long lastDebugTime;
  
  // Moving average filter buffer
  static const int FILTER_SAMPLES = 10;
  // Weights for each sample in filter buffer (newest to oldest)
  const float weights[FILTER_SAMPLES] = {0.25, 0.15, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.05};

  // Threshold for detecting movement
  static constexpr float MOVEMENT_THRESHOLD = 0.06f; // rad/s

  // Buffering queue objects
  SensorQueue accel_x_buffer[NO_OF_UNITS];
  SensorQueue accel_y_buffer[NO_OF_UNITS];
  SensorQueue accel_z_buffer[NO_OF_UNITS];

  SensorQueue gyro_x_buffer[NO_OF_UNITS];
  SensorQueue gyro_y_buffer[NO_OF_UNITS];
  SensorQueue gyro_z_buffer[NO_OF_UNITS];

  SensorQueue mag_x_buffer[NO_OF_UNITS];
  SensorQueue mag_y_buffer[NO_OF_UNITS];
  SensorQueue mag_z_buffer[NO_OF_UNITS];
  
  // Buffer initialization flags
  bool accel_buffer_initialized[NO_OF_UNITS];
  bool gyro_buffer_initialized[NO_OF_UNITS];
  bool mag_buffer_initialized[NO_OF_UNITS];

  // Axis remapping configuration
  int axisMapping[3] = {0, 1, 2};  // Default mapping: X->X, Y->Y, Z->Z
  int axisSigns[3] = {1, 1, 1};    // Default signs: all positive
  
  // Kalman filter states
  KalmanState kalman_accel_x[NO_OF_UNITS];
  KalmanState kalman_accel_y[NO_OF_UNITS];
  KalmanState kalman_accel_z[NO_OF_UNITS];
  KalmanState kalman_gyro_x[NO_OF_UNITS];
  KalmanState kalman_gyro_y[NO_OF_UNITS];
  KalmanState kalman_gyro_z[NO_OF_UNITS];
  KalmanState kalman_mag_x[NO_OF_UNITS];
  KalmanState kalman_mag_y[NO_OF_UNITS];
  KalmanState kalman_mag_z[NO_OF_UNITS];
  
  // For backward compatibility (original implementation)
  float comp_angle_x[NO_OF_UNITS];  // Roll
  float comp_angle_y[NO_OF_UNITS];  // Pitch
  float comp_angle_z[NO_OF_UNITS];  // Yaw
  unsigned long last_time;
  
  // Enhanced orientation representation
  EulerAngles euler_angles[NO_OF_UNITS];
  Quaternion quaternions[NO_OF_UNITS];
    
  // Magnetic disturbance detection
  bool magDisturbance[NO_OF_UNITS];
  float lastMagMagnitude[NO_OF_UNITS];
  
  // Timing for enhanced algorithm
  unsigned long lastSensorTime[NO_OF_UNITS] = {0};
  
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