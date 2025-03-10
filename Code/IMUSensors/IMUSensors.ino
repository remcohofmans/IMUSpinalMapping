/*
 * IMUSensors.ino - Main program file
 * Advanced ICM-20948 demo with comprehensive calibration and filtering
 */

#include "SensorManager.h"
#include "CalibrationManager.h"
#include "FilterManager.h"
#include "StorageManager.h"
#include "OutputManager.h"
 
// Create global manager instances
SensorManager sensorManager;
CalibrationManager calibrationManager;
FilterManager filterManager;
StorageManager storageManager;
OutputManager outputManager;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("ICM-20948 Advanced Calibration and Filtering Demo");
  
  // Initialize Wire library
  Wire.begin();
  
  // Initialize sensor hardware
  if (!sensorManager.initialize()) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    while (1) {
      delay(1000);  // Persistent halt with occasional LED blink possible
    }
  }
  
  // Initialize all the managers with references to each other
  filterManager.initialize(&sensorManager);
  calibrationManager.initialize(&sensorManager, &filterManager);
  storageManager.initialize(&calibrationManager);
  outputManager.initialize(&sensorManager, &calibrationManager, &filterManager);
  
  // Try to load calibration data from EEPROM
  if (!storageManager.loadCalibrationFromEEPROM()) {
    Serial.println("No valid calibration data found in EEPROM");
  } else {
    Serial.println("Calibration data loaded from EEPROM");
    calibrationManager.printCalibrationData();
  }
  
  // Ask user if they want to perform calibration
  Serial.println("\nDo you want to perform sensor calibration?");
  Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");
  
  while (!Serial.available()) {
    // Wait for user input
  }
  
  char response = Serial.read();
  while (Serial.available()) Serial.read();  // Clear input buffer
  
  if (response == 'Y' || response == 'y') {
    calibrationManager.performFullCalibration();
    storageManager.saveCalibrationToEEPROM();
  } else {
    Serial.println("Using existing calibration values");
  }
  
  // Initialize timing for filters
  filterManager.resetTimers();
}

void loop() {
  // Read raw sensor data for all active sensors
  sensorManager.readAllSensors();
  
  // Process sensor data through filters
  filterManager.processAllSensors();
  
  // Print processed data periodically
  outputManager.printSensorData();
  
  // Small delay to not overwhelm the serial output
  delay(10);
}
