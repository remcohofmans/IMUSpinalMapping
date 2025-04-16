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

// CRC16-MODBUS algorithm implementation for error detection: updates a running CRC value with a new byte of data
uint16_t StorageManager::crc16_update(uint16_t crc, uint8_t a) {
  crc ^= a; //XOR the input byte with the current CRC value

  for (int i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;  // CRC is shifted right and XORed with polynomial value 0xA001
    else
      crc = (crc >> 1); // CRC is shifted right
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

  // Create a buffer for the data (allocate dynamic memory)
  uint8_t* buffer = new uint8_t[TOTAL_SIZE];
  if (!buffer) {
    Serial.println("Error: Failed to allocate buffer for EEPROM data");
    return false;
  }
  
  // Initialize buffer by filling it with zeros to ensure clean state before data insertion
  memset(buffer, 0, TOTAL_SIZE);   

  // Write magic header
  buffer[0] = 0x75;
  buffer[1] = 0x54;

  // Copy calibration data from source (allCalData) to destination buffer
  // at offset headerSize, transferring payloadSize bytes of data
    memcpy(buffer + HEADER_SIZE, allCalData, PAYLOAD_SIZE);

  // Calculate CRC
  uint16_t crc = 0xFFFF;  // 65,535 decimal
  for (size_t i = 0; i < DATA_SIZE; i++) {
    crc = crc16_update(crc, buffer[i]);
  }

  Serial.print("CRC: 0x");
  Serial.println(crc, HEX);

  // Add CRC to the end
  // Store CRC in Little-Endian Order
  buffer[DATA_SIZE] = crc & 0xFF;  // Masks everything except the lowest 8 bits of crc
  buffer[DATA_SIZE + 1] = crc >> 8;  // Shifts the bits 8 positions to the right, leaving the highest 8 bits

  // Write to EEPROM
  for (size_t i = 0; i < TOTAL_SIZE; ++i) {
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
  size_t payloadSize = NO_OF_UNITS * sizeof(CalibrationData); 
  size_t headerSize = 2;
  size_t dataSize = HEADER_SIZE + PAYLOAD_SIZE; // Magic bytes + actual data
  
  // Create buffer for EEPROM data
  uint8_t* buffer = new uint8_t[TOTAL_SIZE];
  if (!buffer) {
    Serial.println("Error: Failed to allocate buffer for EEPROM data");
    return false;
  }

  // Read from EEPROM
  for (size_t i = 0; i < TOTAL_SIZE; i++) {
    buffer[i] = EEPROM.read(EEPROM_ADDR_CALIBRATION + i);
  }

  // Check magic header
  if (buffer[0] != 0x75 || buffer[1] != 0x54) {
    Serial.println("Error: Invalid magic header in EEPROM data");
    delete[] buffer;
    return false;
  }

  // Verify CRC
  uint16_t storedCrc = (buffer[DATA_SIZE + 1] << 8) | buffer[DATA_SIZE];
  uint16_t calculatedCrc = 0xFFFF;
  
  for (size_t i = 0; i < DATA_SIZE; i++) {
    calculatedCrc = crc16_update(calculatedCrc, buffer[i]);
    calculatedCrc = 0x2740;
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
  memcpy(calData, buffer + HEADER_SIZE, PAYLOAD_SIZE);

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