/*
 * OutputManager.cpp
 * Implementation of the OutputManager class
 */

#include "OutputManager.h"
#include <math.h>

OutputManager::OutputManager() 
  : sensorManager(nullptr),
    calibrationManager(nullptr),
    filterManager(nullptr),
    lastPrintTime(0),
    outputRate(50) // Default to 50ms/0.05s output rate
{
}

void OutputManager::initialize(SensorManager* sensorMgr, CalibrationManager* calMgr, FilterManager* filterMgr) {
  sensorManager = sensorMgr;
  calibrationManager = calMgr;
  filterManager = filterMgr;
  lastPrintTime = millis();
}

void OutputManager::setOutputRate(unsigned long rate_ms) {
  outputRate = rate_ms;
}

void OutputManager::printSensorData() {
  unsigned long now = millis();
  
  // Only print at the specified output rate
  if (now - lastPrintTime >= outputRate) {
    lastPrintTime = now;
    
    for (int sensorId = 0; sensorId < NO_OF_UNITS; sensorId++) {
      if (sensorManager->isSensorActive(sensorId)) {
        printSensorDataForUnit(sensorId);
      }
    }
  }
}

void OutputManager::printSensorDataForUnit(int sensorId) {
  // Get raw sensor data
  float raw_accel_x, raw_accel_y, raw_accel_z;
  float raw_gyro_x, raw_gyro_y, raw_gyro_z;
  float raw_mag_x, raw_mag_y, raw_mag_z;
  
  sensorManager->getRawAccel(sensorId, raw_accel_x, raw_accel_y, raw_accel_z);
  sensorManager->getRawGyro(sensorId, raw_gyro_x, raw_gyro_y, raw_gyro_z);
  sensorManager->getRawMag(sensorId, raw_mag_x, raw_mag_y, raw_mag_z);
  float temp = sensorManager->getTemperature(sensorId);
  
  // Get calibrated and filtered sensor data
  float cal_accel_x, cal_accel_y, cal_accel_z;
  float cal_gyro_x, cal_gyro_y, cal_gyro_z;
  float cal_mag_x, cal_mag_y, cal_mag_z;
  
  // Apply calibration
  calibrationManager->calibrateAccelData(sensorId, raw_accel_x, raw_accel_y, raw_accel_z, 
                                      temp, cal_accel_x, cal_accel_y, cal_accel_z);
                       
  calibrationManager->calibrateGyroData(sensorId, raw_gyro_x, raw_gyro_y, raw_gyro_z, 
                                     temp, cal_gyro_x, cal_gyro_y, cal_gyro_z);
                    
  calibrationManager->calibrateMagData(sensorId, raw_mag_x, raw_mag_y, raw_mag_z, 
                                   cal_mag_x, cal_mag_y, cal_mag_z);
  
  // Get filtered data
  float filtered_accel_x, filtered_accel_y, filtered_accel_z;
  float filtered_gyro_x, filtered_gyro_y, filtered_gyro_z;
  float filtered_mag_x, filtered_mag_y, filtered_mag_z;
  
  filterManager->getFilteredAccel(sensorId, filtered_accel_x, filtered_accel_y, filtered_accel_z);
  filterManager->getFilteredGyro(sensorId, filtered_gyro_x, filtered_gyro_y, filtered_gyro_z);
  filterManager->getFilteredMag(sensorId, filtered_mag_x, filtered_mag_y, filtered_mag_z);
  
  // Get orientation
  float roll, pitch, yaw;
  filterManager->getOrientation(sensorId, roll, pitch, yaw);
  
  // Print data
  Serial.println("\n===========================");
  Serial.println("Sensor Unit: " + String(sensorId + 1));
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");
  
  // Accelerometer data
  Serial.println("\nAccelerometer (m/s²):");
  Serial.print("  Raw: X=");
  Serial.print(raw_accel_x, 3);
  Serial.print(" Y=");
  Serial.print(raw_accel_y, 3);
  Serial.print(" Z=");
  Serial.println(raw_accel_z, 3);
  
  Serial.print("  Calibrated: X=");
  Serial.print(cal_accel_x, 3);
  Serial.print(" Y=");
  Serial.print(cal_accel_y, 3);
  Serial.print(" Z=");
  Serial.println(cal_accel_z, 3);
  
  Serial.print("  Filtered: X=");
  Serial.print(filtered_accel_x, 3);
  Serial.print(" Y=");
  Serial.print(filtered_accel_y, 3);
  Serial.print(" Z=");
  Serial.println(filtered_accel_z, 3);
  
  // Gyroscope data
  Serial.println("\nGyroscope (rad/s):");
  Serial.print("  Raw: X=");
  Serial.print(raw_gyro_x, 4);
  Serial.print(" Y=");
  Serial.print(raw_gyro_y, 4);
  Serial.print(" Z=");
  Serial.println(raw_gyro_z, 4);
  
  Serial.print("  Calibrated: X=");
  Serial.print(cal_gyro_x, 4);
  Serial.print(" Y=");
  Serial.print(cal_gyro_y, 4);
  Serial.print(" Z=");
  Serial.println(cal_gyro_z, 4);
  
  Serial.print("  Filtered: X=");
  Serial.print(filtered_gyro_x, 4);
  Serial.print(" Y=");
  Serial.print(filtered_gyro_y, 4);
  Serial.print(" Z=");
  Serial.println(filtered_gyro_z, 4);
  
  // Magnetometer data
  Serial.println("\nMagnetometer (uT):");
  Serial.print("  Raw: X=");
  Serial.print(raw_mag_x, 2);
  Serial.print(" Y=");
  Serial.print(raw_mag_y, 2);
  Serial.print(" Z=");
  Serial.println(raw_mag_z, 2);
  
  Serial.print("  Calibrated: X=");
  Serial.print(cal_mag_x, 2);
  Serial.print(" Y=");
  Serial.print(cal_mag_y, 2);
  Serial.print(" Z=");
  Serial.println(cal_mag_z, 2);
  
  Serial.print("  Filtered: X=");
  Serial.print(filtered_mag_x, 2);
  Serial.print(" Y=");
  Serial.print(filtered_mag_y, 2);
  Serial.print(" Z=");
  Serial.println(filtered_mag_z, 2);
  
  // Orientation from complementary filter
  Serial.println("\nOrientation (degrees):");
  Serial.print("  Roll (X): ");
  Serial.print(roll, 1);
  Serial.print(" Pitch (Y): ");
  Serial.print(pitch, 1);
  Serial.print(" Yaw (Z): ");
  Serial.println(yaw, 1);
  
  // Calculate magnitude to verify calibration
  float raw_accel_mag = sqrt(sq(raw_accel_x) + sq(raw_accel_y) + sq(raw_accel_z));
  float cal_accel_mag = sqrt(sq(cal_accel_x) + sq(cal_accel_y) + sq(cal_accel_z));
  
  Serial.println("\nAccel Magnitude (should be ~9.8 m/s² when stationary):");
  Serial.print("  Raw: ");
  Serial.print(raw_accel_mag, 3);
  Serial.print(" Calibrated: ");
  Serial.println(cal_accel_mag, 3);
  
  float raw_mag_mag = sqrt(sq(raw_mag_x) + sq(raw_mag_y) + sq(raw_mag_z));
  float cal_mag_mag = sqrt(sq(cal_mag_x) + sq(cal_mag_y) + sq(cal_mag_z));
  
  Serial.println("Mag Magnitude (should be constant regardless of orientation):");
  Serial.print("  Raw: ");
  Serial.print(raw_mag_mag, 2);
  Serial.print(" Calibrated: ");
  Serial.println(cal_mag_mag, 2);
}
