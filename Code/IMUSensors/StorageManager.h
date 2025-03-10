/*
 * StorageManager.h
 * Handles saving and loading calibration data to/from EEPROM
 */

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <EEPROM.h>
#include "CalibrationManager.h"

class StorageManager {
public:
  StorageManager();
  
  // Initialize with calibration manager
  void initialize(CalibrationManager* calMgr);
  
  // Save calibration data to EEPROM
  void saveCalibrationToEEPROM();
  
  // Load calibration data from EEPROM
  bool loadCalibrationFromEEPROM();

private:
  CalibrationManager* calibrationManager;
  
  // EEPROM storage addresses
  static const int EEPROM_ADDR_CALIBRATION = 0;
  static const uint32_t EEPROM_MAGIC_NUMBER = 12345;  // To verify data is valid
};

#endif // STORAGE_MANAGER_H
