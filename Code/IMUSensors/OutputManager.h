/*
 * OutputManager.h
 * Handles output formatting and display of sensor data
 */

#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include "SensorManager.h"
#include "CalibrationManager.h"
#include "FilterManager.h"

class OutputManager {
public:
  OutputManager();
  
  // Initialize with references to other managers
  void initialize(SensorManager* sensorMgr, CalibrationManager* calMgr, FilterManager* filterMgr);
  
  // Print formatted sensor data to Serial
  void printSensorData();
  
  // Set output data rate in ms (0 = as fast as possible)
  void setOutputRate(unsigned long rate_ms);

private:
  SensorManager* sensorManager;
  CalibrationManager* calibrationManager;
  FilterManager* filterManager;
  
  unsigned long lastPrintTime;
  unsigned long outputRate;
  
  // Print data for a specific sensor
  void printSensorDataForUnit(int sensorId);
};

#endif // OUTPUT_MANAGER_H
