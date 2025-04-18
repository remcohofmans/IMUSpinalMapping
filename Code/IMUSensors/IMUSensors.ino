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

#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor_Calibration.h>
#include "AHRS_mahony_filter.h"
#include <Wire.h>

// WiFi credentials
const char* ssid = "telenet-27CE5";
const char* password = "jEy6kRyfzBnY";
const char* username = nullptr;

// Create global manager instances
SensorManager sensorManager;
CalibrationManager calibrationManager;
FilterManager filterManager;
StorageManager storageManager;
OutputManager outputManager;
WebServer webServer(&sensorManager, &filterManager);

// Timing variables
#define PRINT_EVERY_N_UPDATES 1
uint32_t timestamp;

// flag to toggle on/off calibration
bool calibrateSensors = true;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize sensors
  if (!sensorManager.initialize()) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    Serial.print("Try to restart or check the connections to the multiplexer");
    while (1) {
      delay(1000);
      Serial.print(".");
    }
  }

  // Initialize system managers
  calibrationManager.initialize(&sensorManager);
  storageManager.initialize(&calibrationManager);
  filterManager.initialize(&sensorManager, &calibrationManager);
  outputManager.initialize(&sensorManager, &calibrationManager, &filterManager);

  timestamp = millis();

  // Mount filesystem
  if(!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // List files
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

  // Initialize and start web server to serve data over HTTP and WebSockets to the browser client
  if (webServer.initialize(ssid, password, "esp32-imu")) {
    webServer.start();
  } else {
    Serial.println("WARNING: Web server initialization failed.");
    Serial.println("Continuing with Serial output only");
  }

  // Always attempt to load calibration from EEPROM
  if (!storageManager.loadCalibration()) {
    // If loading fails, force full calibration regardless of calibrateSensors flag
    calibrationManager.performFullCalibration(true);
    Serial.println("No valid calibration data found in EEPROM");
    storageManager.saveCalibration();
  } else {
    // Calibration data exists in EEPROM
    Serial.println("Calibration data loaded from EEPROM");
    calibrationManager.printCalibrationData();

    if (calibrateSensors) {
      // Prompt user for manual calibration
      Serial.println("\nDo you want to perform sensor calibration?");
      Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");

      while(true){
        while (!Serial.available()) {}
        char response = Serial.read();
        while (Serial.available()) Serial.read(); // clear buffer

        if (response == 'Y' || response == 'y') {
          calibrationManager.performFullCalibration(false);
          storageManager.saveCalibration();
          break;
        } else if (response == 'N' || response == 'n') {
          Serial.println("Using existing calibration values");
          break;
        } else {
          Serial.print("Not a valid input, ");
          Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");
        }
      }
    } else {
      Serial.println("Using EEPROM calibration without re-calibration (calibrateSensors = false)");
    }
  }
  // For vertical orientation with X pointing down:
  // filterManager.configureAxisMapping(2, 1, 0, 1, 1, -1);    // This maps X->Z, Y->Y, Z->X with appropriate sign changes
    
  
}

void loop() {

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Serial.println(timestamp);

  // Read raw sensor data for all active sensors
  sensorManager.readAllSensors();

  // int timepassed = millis() - timestamp;
  // Serial.print("Time ellapsed after reading all sensors: ");
  // Serial.println(timepassed);
  
  // Process sensor data through filters
  filterManager.processAllSensors();

  // timepassed = millis() - timestamp;
  // Serial.print("Time ellapsed after processing sensors: ");
  // Serial.println(timepassed);
  
  // Print processed data periodically to Serial and WebSocket
  outputManager.printSensorData();

  // timepassed = millis() - timestamp;
  // Serial.print("Time ellapsed after print sensors data: ");
  // Serial.println(timepassed);
  
  // Update web clients with orientation data
  unsigned long currentTime = millis();
  webServer.updateOrientationData();
  
}