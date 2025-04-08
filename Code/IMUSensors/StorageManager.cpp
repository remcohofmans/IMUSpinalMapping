/*
 * StorageManager.cpp
 * Implementation of the StorageManager class
 */

#include "StorageManager.h"


#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal[NO_OF_UNITS];
#else
  Adafruit_Sensor_Calibration_SDFat cal[NO_OF_UNITS];
#endif

StorageManager::StorageManager() : calibrationManager(nullptr), EEPROM_ADDR_CALIBRATION(0) {
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
  
  // Initialize EEPROM (virtualized) with required size
  EEPROM.begin(requiredSize);

  Serial.print("EEPROM initialized with size: ");
  Serial.println(requiredSize);
}

// CRC16 calculation function
uint16_t StorageManager::crc16_update(uint16_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

bool StorageManager::saveCalibration() {
  if (!calibrationManager) {
    Serial.println("Error: Cannot save calibration - CalibrationManager not initialized");
    return false;
  }
  Serial.println("Saving calibration to EEPROM...");

  // Request all calibration data from CalibrationManager
  CalibrationData* allCalData = calibrationManager->getAllCalibrationData();
  
  if (!allCalData) {
    Serial.println("Error: Failed to get calibration data");
    return false;
  }

  // Calculate size excluding CRC
  size_t payloadSize = (NO_OF_UNITS * sizeof(CalibrationData));
  size_t headerSize = 2;
  size_t CRCSize = 2;
  size_t totalSize = headerSize + payloadSize + CRCSize;
  // Define the static member
  const size_t EEPROM_SIZE = totalSize;

  // Create a buffer for the data
  uint8_t* buffer = new uint8_t[totalSize];
  if (!buffer) {
    Serial.println("Error: Failed to allocate buffer for EEPROM data");
    return false;
  }
  
  // Clear buffer
  memset(buffer, 0, totalSize);

  // Write magic header
  buffer[0] = 0x75;
  buffer[1] = 0x54;

  // Copy calibration data
  memcpy(buffer + headerSize, allCalData, payloadSize);

  // Calculate CRC
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < (headerSize + payloadSize); i++) {
    crc = crc16_update(crc, buffer[i]);
  }

  Serial.print("CRC: 0x");
  Serial.println(crc, HEX);

  // Add CRC to the end
  buffer[headerSize + payloadSize] = crc & 0xFF;
  buffer[headerSize + payloadSize + 1] = crc >> 8;

  // Write to EEPROM
  for (size_t i = 0; i < totalSize; i++) {
    EEPROM.write(EEPROM_ADDR_CALIBRATION + i, buffer[i]);
  }

  // Commit changes (required for ESP32/ESP8266)
  bool success = EEPROM.commit();
  
  // Clean up
  delete[] buffer;
  
  if (success) {
    Serial.println("Calibration data saved to EEPROM successfully");
  } else {
    Serial.println("Error: Failed to commit data to EEPROM");
  }
  
  return success;
}

bool StorageManager::loadCalibration() {
  if (!calibrationManager) {
    Serial.println("Error: Cannot load calibration data - CalibrationManager not initialized");
    return false;
  }

  Serial.println("Loading calibration from EEPROM...");

  // Calculate total size
  size_t dataSize = 2 + (NO_OF_UNITS * sizeof(CalibrationData)); // Magic bytes + actual data
  size_t totalSize = dataSize + 2; // Data + CRC
  
  // Create buffer for EEPROM data
  uint8_t* buffer = new uint8_t[totalSize];
  if (!buffer) {
    Serial.println("Error: Failed to allocate buffer for EEPROM data");
    return false;
  }

  // Read from EEPROM
  for (size_t i = 0; i < totalSize; i++) {
    buffer[i] = EEPROM.read(EEPROM_ADDR_CALIBRATION + i);
  }

  // Check magic header
  if (buffer[0] != 0x75 || buffer[1] != 0x54) {
    Serial.println("Error: Invalid magic header in EEPROM data");
    delete[] buffer;
    return false;
  }

  // Verify CRC
  uint16_t storedCrc = (buffer[dataSize + 1] << 8) | buffer[dataSize];
  uint16_t calculatedCrc = 0xFFFF;
  
  for (size_t i = 0; i < dataSize; i++) {
    calculatedCrc = crc16_update(calculatedCrc, buffer[i]);
  }

  if (calculatedCrc != storedCrc) {
    Serial.print("Error: CRC mismatch. Stored: 0x");
    Serial.print(storedCrc, HEX);
    Serial.print(", Calculated: 0x");
    Serial.println(calculatedCrc, HEX);
    delete[] buffer;
    return false;
  }

  // Extract calibration data
  CalibrationData* calData = new CalibrationData[NO_OF_UNITS];
  memcpy(calData, buffer + 2, NO_OF_UNITS * sizeof(CalibrationData));

  // Set calibration data for each active sensor
  for (int i = 0; i < NO_OF_UNITS; i++) {
    calibrationManager->setCalibrationData(i, calData[i]);
  }

  // Clean up
  delete[] buffer;
  delete[] calData;

  Serial.println("Calibration data loaded from EEPROM successfully");
  return true;
}

bool StorageManager::printSavedCalibration(void) {
  if (!calibrationManager) {
    Serial.println("Error: Cannot print calibration - CalibrationManager not initialized");
    return false;
  }
  
  // For this function, just print the calibration data
  calibrationManager->printCalibrationData();
  return true;
}