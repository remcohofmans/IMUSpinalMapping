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
const char* ssid = "telenet-6073619";
const char* password = "6Gby6hBrenwc";
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
const unsigned long WEB_UPDATE_INTERVAL = 1; // 1ms -> 1000Hz update rate

void setup(void) {
  Serial.begin(115200); // Initialize serial communication with a baud rate of 115200 bps
  while (!Serial)
    delay(10);

  // Initialize Wire library
  Wire.begin();
  
  // Set up communication channels between ESP32 and sensor hardware (I2C)
  if (!sensorManager.initialize()) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    while (1) {
      delay(1000);  // Halt for 1000ms
    }
  }
  
  // Initialize all the managers with references to each other
  calibrationManager.initialize(&sensorManager, &filterManager);
  storageManager.initialize(&calibrationManager);
  filterManager.initialize(&sensorManager, &calibrationManager);
  outputManager.initialize(&sensorManager, &calibrationManager, &filterManager);

  // Initialize LittleFS - load the HTML file after this step
  if(!LittleFS.begin(true)) { // If mounting fails, it will automatically format the flash storage and then mount it
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
  
  while (!Serial.available()) { // Check if data is available in the serial buffer
    // Wait for user input
  }
  
  char response = Serial.read();  // Read the user input (FIFO, 1 byte)
  while (Serial.available()) Serial.read();  // Clear input buffer
  
  if (response == 'Y' || response == 'y') {
    calibrationManager.performFullCalibration();
    storageManager.saveCalibrationToEEPROM();
  } else {
    Serial.println("Using existing calibration values");
  }
  
  // Initialize timing for filters
  filterManager.resetTimers();
    
  // Initialize and start web server to serve data over HTTP and WebSockets to the browser client
  if (webServer.initialize(ssid, password, "esp32-imu")) {
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
  
  // Print processed data periodically to Serial and WebSocket
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