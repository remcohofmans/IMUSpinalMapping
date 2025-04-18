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
    outputRate(100) // Default to 50ms/0.05s output rate
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
  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);


  float qw, qx, qy, qz;
  filterManager->getQuaternion(sensorId, qw, qx, qy, qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 3);
  Serial.print(", ");
  Serial.print(qx, 3);
  Serial.print(", ");
  Serial.print(qy, 3);
  Serial.print(", ");
  Serial.println(qz, 3);
}
