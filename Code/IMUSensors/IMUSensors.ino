/*
 * IMUSensors.ino - Main program file
 * Advanced ICM-20948 demo with comprehensive calibration and filtering
 * with 3D visualization web interface
 */

#include "SensorManager.h"
#include "CalibrationManager.h"
#include "FilterManager.h"
#include "StorageManager.h"
#include "OutputManager.h"
#include "WebServer.h"

// WiFi credentials
const char* ssid = "iPhone van Remco";
const char* password = "555333222";
const char* username = nullptr;

// Create global manager instances
SensorManager sensorManager;
CalibrationManager calibrationManager;
FilterManager filterManager;
StorageManager storageManager;
OutputManager outputManager;
WebServer webServer(&sensorManager, &filterManager);

// Timing variables
unsigned long lastWebUpdateTime = 0;
const unsigned long WEB_UPDATE_INTERVAL = 50; // 50ms -> 20 Hz update rate

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("ICM-20948 Advanced Calibration and Filtering Demo with Web Visualization");
  
  // Initialize Wire library
  Wire.begin();
  
  // Initialize sensor hardware
  if (!sensorManager.initialize()) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    while (1) {
      delay(1000);  // Halt for 1000ms
    }
  }
  
  // Initialize all the managers with references to each other
  filterManager.initialize(&sensorManager);
  calibrationManager.initialize(&sensorManager, &filterManager);
  storageManager.initialize(&calibrationManager);
  outputManager.initialize(&sensorManager, &calibrationManager, &filterManager);

  // Initialize LittleFS - we'll load the HTML file that was uploaded separately
  if(!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  Serial.println("LittleFS File List:");
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print(" (");
    Serial.print(file.size());
    Serial.println(" bytes)");
    file = root.openNextFile();
  }
  
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
    
  // Initialize and start web server
  if (webServer.initialize(ssid, password, username)) {
    webServer.start();
  } else {
    Serial.println("WARNING: Web server initialization failed");
    Serial.println("Continuing with Serial output only");
  }
  
  lastWebUpdateTime = millis();
}

void loop() {
  // Read raw sensor data for all active sensors
  sensorManager.readAllSensors();
  
  // Process sensor data through filters
  filterManager.processAllSensors();
  
  // Print processed data periodically to Serial
  outputManager.printSensorData();
  
  // Update web clients with orientation data
  unsigned long currentTime = millis();
  if (currentTime - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
    webServer.updateOrientationData();
    lastWebUpdateTime = currentTime;
  }
  
  // Small delay to not overwhelm the system
  delay(5);
}