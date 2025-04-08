/*
 * StorageManager.cpp
 * Implementation of the StorageManager class
 */

#include "StorageManager.h"

// Define static constants
const int StorageManager::EEPROM_ADDR_CALIBRATION;
const uint32_t StorageManager::EEPROM_MAGIC_NUMBER;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal[NO_OF_UNITS];
#else
  Adafruit_Sensor_Calibration_SDFat cal[NO_OF_UNITS];
#endif

StorageManager::StorageManager() : calibrationManager(nullptr) {
  for(int i = 0; i < NO_OF_UNITS; i++) {
    if (!cal[i].begin()) {
      Serial.println("Failed to initialize calibration helper");
    }
  }
}

void StorageManager::initialize(CalibrationManager* calMgr) {
  calibrationManager = calMgr;

  // Calculate required size: magic number + calibration data for all units
  size_t requiredSize = sizeof(uint32_t) + (NO_OF_UNITS * sizeof(CalibrationData));
  
  // Initialize EEPROM (virtualized SRAM) with required size
  EEPROM.begin(requiredSize);

  Serial.print("EEPROM initialized with size: ");
  Serial.println(requiredSize);
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
    CalibrationData *data = calibrationManager->getCalibrationData(i);
    if (data) {
      EEPROM.put(address, *data);  // Attention: only stores data in a RAM buffer
      address += sizeof(CalibrationData);
    }
  }
  
  EEPROM.commit();  // Required to actually transfer buffered data from RAM to flash memory
  Serial.println("Calibration data saved to EEPROM");
}

bool StorageManager::loadCalibrationFromEEPROM() {
  if (!calibrationManager) {
    Serial.println("Error: Cannot load calibration data - CalibrationManager not initialized");
    return false;
  }

  return cal[0].loadCalibration();
  
  // int address = EEPROM_ADDR_CALIBRATION;
  
  // // Check magic number
  // uint32_t magic;
  // EEPROM.get(address, magic);
  // if (magic != EEPROM_MAGIC_NUMBER) {
  //   Serial.println("Error: Invalid calibration data in EEPROM.");
  //   return false;
  // }
  // address += sizeof(uint32_t);
  
  // // Read calibration data for all units
  // for (int i = 0; i < NO_OF_UNITS; i++) {
  //   CalibrationData data;
  //   EEPROM.get(address, data);  // Reads calibration data from EEPROM at address 'address' into the data object
  //   calibrationManager->setCalibrationData(i, data);
  //   address += sizeof(CalibrationData);
    
  //   Serial.print("Loaded Calibration Data for Sensor ");
  //   Serial.println(i);
  // }
  
  return true;
}