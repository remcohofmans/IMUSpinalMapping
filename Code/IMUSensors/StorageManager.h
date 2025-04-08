/*
 * StorageManager.h
 * Handles saving and loading calibration data to/from EEPROM
 */

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <EEPROM.h>
#include "CalibrationManager.h"
#include "Adafruit_Sensor_Calibration.h"


class StorageManager: public Adafruit_Sensor_Calibration {
public:
  StorageManager();

  // Initialize with calibration manager
  void initialize(CalibrationManager* calMgr);
  
  // Required implementations for Adafruit_Sensor_Calibration
  virtual bool saveCalibration(void) override;
  virtual bool loadCalibration(void) override;
  virtual bool printSavedCalibration(void) override;

private:
  CalibrationManager* calibrationManager;
  
  // EEPROM storage configuration
  const size_t EEPROM_ADDR_CALIBRATION; // Start address in EEPROM
  static const size_t EEPROM_SIZE; // Total EEPROM size
  
  // Helper function for CRC calculation
  static uint16_t crc16_update(uint16_t crc, uint8_t a);
};

#endif // STORAGE_MANAGER_H