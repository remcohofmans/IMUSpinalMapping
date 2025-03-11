/*
 * StorageManager.cpp
 * Implementation of the StorageManager class
 */

#include "StorageManager.h"

// Define static constants
const int StorageManager::EEPROM_ADDR_CALIBRATION;
const uint32_t StorageManager::EEPROM_MAGIC_NUMBER;

StorageManager::StorageManager() : calibrationManager(nullptr) {
}

void StorageManager::initialize(CalibrationManager* calMgr) {
  calibrationManager = calMgr;
}

void StorageManager::saveCalibrationToEEPROM() {
  if (!calibrationManager) {
    Serial.println("Error: Cannot save calibration - CalibrationManager not initialized");
    return;
  }
  
  int address = EEPROM_ADDR_CALIBRATION;
  
  // Write magic number to indicate valid data
  EEPROM.put(address, EEPROM_MAGIC_NUMBER);
  address += sizeof(uint32_t);
  
  // Write calibration data for all units
  for (int i = 0; i < NO_OF_UNITS; i++) {
    CalibrationData* data = calibrationManager->getCalibrationData(i);
    if (data) {
      EEPROM.put(address, *data);
      address += sizeof(CalibrationData);
    }
  }
  
  EEPROM.commit();
  Serial.println("Calibration data saved to EEPROM");
}

bool StorageManager::loadCalibrationFromEEPROM() {
  if (!calibrationManager) {
    Serial.println("Error: Cannot load calibration - CalibrationManager not initialized");
    return false;
  }
  
  int address = EEPROM_ADDR_CALIBRATION;
  
  // Check magic number
  uint32_t magic;
  EEPROM.get(address, magic);
  if (magic != EEPROM_MAGIC_NUMBER) {
    return false;
  }
  address += sizeof(uint32_t);
  
  // Read calibration data for all units
  for (int i = 0; i < NO_OF_UNITS; i++) {
    CalibrationData data;
    EEPROM.get(address, data);  // Reads calibration data from EEPROM at address 'address' into the data object
    calibrationManager->setCalibrationData(i, data);
    address += sizeof(CalibrationData);
  }
  
  return true;
}