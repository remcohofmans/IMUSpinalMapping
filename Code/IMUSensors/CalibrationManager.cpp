/*
 * CalibrationManager.cpp
 * Implementation of the CalibrationManager class
 */

#include "CalibrationManager.h"
#include <math.h>

CalibrationManager::CalibrationManager() {
  // Calibration data already initialized with defaults in the struct definition
}

void CalibrationManager::initialize(SensorManager* sensorMgr, FilterManager* filterMgr) {
  sensorManager = sensorMgr;
  filterManager = filterMgr;
}

void CalibrationManager::performFullCalibration() {
  calibrateGyros();
  calibrateAccelerometers();
  calibrateMagnetometers();
  calibrateTemperatures();
}

void CalibrationManager::calibrateGyros() {
  Serial.println("\n=== Gyroscope Calibration ===");
  Serial.println("Keep the sensor units completely STILL");
  Serial.println("Starting in 5 seconds...");
  
  for (int i = 5; i > 0; i--) {
    Serial.print(i); Serial.println("...");
    delay(1000);
  }
  
  Serial.println("Calibrating - DO NOT MOVE...");
  
  float gyro_x_sum[NO_OF_UNITS] = {0};
  float gyro_y_sum[NO_OF_UNITS] = {0};
  float gyro_z_sum[NO_OF_UNITS] = {0};
  const int samples = 500;  // More samples for better accuracy
  
  for (int i = 0; i < samples; i++) {
    sensorManager->readAllSensors();
    
    for (int j = 0; j < NO_OF_UNITS; j++) {
      if (!sensorManager->isSensorActive(j)) continue;
      
      float x, y, z;
      sensorManager->getRawGyro(j, x, y, z);
      
      gyro_x_sum[j] += x;
      gyro_y_sum[j] += y;
      gyro_z_sum[j] += z;
    }
    
    if (i % 50 == 0) Serial.print(".");
    delay(10);
  }
  
  // Calculate averages to get bias
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensorManager->isSensorActive(i)) continue;
    
    calibrationData[i].gyro_offset[0] = gyro_x_sum[i] / samples;
    calibrationData[i].gyro_offset[1] = gyro_y_sum[i] / samples;
    calibrationData[i].gyro_offset[2] = gyro_z_sum[i] / samples;
    
    Serial.println("\nGyroscope calibration complete!");
    Serial.println("Gyroscope offsets (rad/s) for Unit " + String(i + 1) + ":");
    Serial.print("X: "); Serial.print(calibrationData[i].gyro_offset[0], 6);
    Serial.print(" Y: "); Serial.print(calibrationData[i].gyro_offset[1], 6);
    Serial.print(" Z: "); Serial.print(calibrationData[i].gyro_offset[2], 6);
    Serial.println();
  }
}

void CalibrationManager::calibrateAccelerometers() {
  Serial.println("\n=== Accelerometer Calibration for Multiple Units ===");
  Serial.println("This requires positioning all sensor units in 6 different orientations");
  Serial.println("You'll be prompted to place all units on each face at the same time");
  
  // Create arrays to store readings for all 6 positions for each unit
  float accel_readings[NO_OF_UNITS][6][3];  // [unit][position][axis]
  const int samples_per_position = 100;
  
  for (int position = 0; position < 6; position++) {
    String directions[] = {
      "Z up (flat)",
      "Z down (upside down)",
      "X up (on long edge)",
      "X down (on opposite long edge)",
      "Y up (on short edge)",
      "Y down (on opposite short edge)"
    };
    
    Serial.print("\nPosition ALL sensors with ");
    Serial.println(directions[position]);
    Serial.println("Press any key when ready");
    
    while (!Serial.available()) {
      // Wait for user input
    }
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    Serial.println("Hold still... measuring");
    delay(2000);  // Give time to stabilize
    
    // Create arrays to sum readings for each unit
    float sum_x[NO_OF_UNITS] = {0};
    float sum_y[NO_OF_UNITS] = {0};
    float sum_z[NO_OF_UNITS] = {0};
    
    for (int i = 0; i < samples_per_position; i++) {
      sensorManager->readAllSensors();
      
      for (int unit = 0; unit < NO_OF_UNITS; unit++) {
        if (!sensorManager->isSensorActive(unit)) continue;
        
        float x, y, z;
        sensorManager->getRawAccel(unit, x, y, z);
        
        sum_x[unit] += x;
        sum_y[unit] += y;
        sum_z[unit] += z;
      }
      
      if (i % 20 == 0) Serial.print(".");
      delay(10);
    }
    
    // Store average for each unit
    for (int unit = 0; unit < NO_OF_UNITS; unit++) {
      if (!sensorManager->isSensorActive(unit)) continue;
      
      accel_readings[unit][position][0] = sum_x[unit] / samples_per_position;
      accel_readings[unit][position][1] = sum_y[unit] / samples_per_position;
      accel_readings[unit][position][2] = sum_z[unit] / samples_per_position;
    }
    
    Serial.println("\nRecorded position " + String(position + 1) + "/6 for all units");
  }
  
  // Process calibration data for each unit
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;
    
    Serial.println("\n=== Processing calibration for Unit " + String(unit + 1) + " ===");
    
    // For each axis, find min and max values
    float min_x = 9999, max_x = -9999;
    float min_y = 9999, max_y = -9999;
    float min_z = 9999, max_z = -9999;
    
    for (int i = 0; i < 6; i++) {
      if (accel_readings[unit][i][0] < min_x) min_x = accel_readings[unit][i][0];
      if (accel_readings[unit][i][0] > max_x) max_x = accel_readings[unit][i][0];
      
      if (accel_readings[unit][i][1] < min_y) min_y = accel_readings[unit][i][1];
      if (accel_readings[unit][i][1] > max_y) max_y = accel_readings[unit][i][1];
      
      if (accel_readings[unit][i][2] < min_z) min_z = accel_readings[unit][i][2];
      if (accel_readings[unit][i][2] > max_z) max_z = accel_readings[unit][i][2];
    }
    
    // Calculate offsets (bias) - the center of min and max
    calibrationData[unit].accel_offset[0] = (min_x + max_x) / 2.0;
    calibrationData[unit].accel_offset[1] = (min_y + max_y) / 2.0;
    calibrationData[unit].accel_offset[2] = (min_z + max_z) / 2.0;
    
    // Calculate scale factors - normalized to 9.8 m/s² (gravity)
    calibrationData[unit].accel_scale[0] = 19.6 / (max_x - min_x);
    calibrationData[unit].accel_scale[1] = 19.6 / (max_y - min_y);
    calibrationData[unit].accel_scale[2] = 19.6 / (max_z - min_z);
    
    Serial.println("Accelerometer calibration complete for Unit " + String(unit + 1));
    Serial.println("Accel offsets (m/s²):");
    Serial.print("X: "); Serial.print(calibrationData[unit].accel_offset[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].accel_offset[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].accel_offset[2], 4);
    Serial.println();
    
    Serial.println("Accel scale factors:");
    Serial.print("X: "); Serial.print(calibrationData[unit].accel_scale[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].accel_scale[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].accel_scale[2], 4);
    Serial.println();
  }
  
  Serial.println("\nAll accelerometers calibration complete!");
}

void CalibrationManager::calibrateMagnetometers() {
  Serial.println("\n=== Magnetometer Calibration for Multiple Units ===");
  Serial.println("We'll calibrate each sensor unit one by one.");
  Serial.println("For each unit, rotate it slowly in all directions to");
  Serial.println("sample the complete 3D magnetic field.");
  Serial.println("Try to cover all possible orientations.");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;
    
    Serial.println("\n=== Calibrating Unit " + String(unit + 1) + " Magnetometer ===");
    Serial.println("Hold Unit " + String(unit + 1) + " in your hand.");
    Serial.println("Press any key to start, then again to stop when done.");
    
    while (!Serial.available()) {
      // Wait for user input
    }
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    Serial.println("\nCalibrating Unit " + String(unit + 1) + " - ROTATE SENSOR IN FIGURE 8 PATTERN...");
    
    // Collect samples for ellipsoid fitting
    const int max_samples = 500;
    float mag_x[max_samples], mag_y[max_samples], mag_z[max_samples];
    int sample_count = 0;
    bool done = false;
    
    while (!done && sample_count < max_samples) {
      if (Serial.available()) {
        done = true;
        while (Serial.available()) Serial.read();  // Clear input buffer
      }
      
      sensorManager->readAllSensors();
      float x, y, z;
      sensorManager->getRawMag(unit, x, y, z);
      
      // Store sample if it's different from previous ones to avoid duplicates
      if (sample_count == 0 || 
          (x != mag_x[sample_count-1] || 
           y != mag_y[sample_count-1] || 
           z != mag_z[sample_count-1])) {
        
        mag_x[sample_count] = x;
        mag_y[sample_count] = y;
        mag_z[sample_count] = z;
        sample_count++;
        
        if (sample_count % 50 == 0) {
          Serial.print(".");
          if (sample_count % 250 == 0) {
            Serial.println();
          }
        }
      }
      delay(20);
    }
    
    Serial.println("\nCollected " + String(sample_count) + " samples for Unit " + String(unit + 1));
    
    // Find min and max for each axis
    float min_x = 9999, max_x = -9999;
    float min_y = 9999, max_y = -9999;
    float min_z = 9999, max_z = -9999;
    
    for (int i = 0; i < sample_count; i++) {
      if (mag_x[i] < min_x) min_x = mag_x[i];
      if (mag_x[i] > max_x) max_x = mag_x[i];
      
      if (mag_y[i] < min_y) min_y = mag_y[i];
      if (mag_y[i] > max_y) max_y = mag_y[i];
      
      if (mag_z[i] < min_z) min_z = mag_z[i];
      if (mag_z[i] > max_z) max_z = mag_z[i];
    }
    
    // Calculate hard iron offsets
    calibrationData[unit].mag_offset[0] = (min_x + max_x) / 2.0;
    calibrationData[unit].mag_offset[1] = (min_y + max_y) / 2.0;
    calibrationData[unit].mag_offset[2] = (min_z + max_z) / 2.0;
    
    // Calculate soft iron scale factors
    float avg_delta = ((max_x - min_x) + (max_y - min_y) + (max_z - min_z)) / 3.0;
    
    calibrationData[unit].mag_scale[0] = avg_delta / (max_x - min_x);
    calibrationData[unit].mag_scale[1] = avg_delta / (max_y - min_y);
    calibrationData[unit].mag_scale[2] = avg_delta / (max_z - min_z);
    
    Serial.println("\nMagnetometer calibration complete for Unit " + String(unit + 1));
    Serial.println("Hard iron offsets (uT):");
    Serial.print("X: "); Serial.print(calibrationData[unit].mag_offset[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].mag_offset[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].mag_offset[2], 4);
    Serial.println();
    
    Serial.println("Soft iron scale factors:");
    Serial.print("X: "); Serial.print(calibrationData[unit].mag_scale[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].mag_scale[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].mag_scale[2], 4);
    Serial.println();
  }
  
  Serial.println("\nAll units magnetometer calibration complete!");
}

void CalibrationManager::calibrateTemperatures() {
  Serial.println("\n=== Temperature Compensation Calibration for Multiple Units ===");
  Serial.println("This calibration collects reference temperature data for each sensor unit.");
  Serial.println("We'll collect data at room temperature now.");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;
    
    // Get reference temperature and readings for this unit
    float temp = sensorManager->getTemperature(unit);
    
    calibrationData[unit].temp_ref = temp;
    
    float x, y, z;
    sensorManager->getRawAccel(unit, x, y, z);
    
    Serial.println("Unit " + String(unit + 1) + " reference temperature: " + String(calibrationData[unit].temp_ref) + " °C");
    
    // Using default values for temperature coefficients
    for (int i = 0; i < 3; i++) {
      calibrationData[unit].temp_coef_accel[i] = 0.0;  // Default: no temperature compensation
      calibrationData[unit].temp_coef_gyro[i] = 0.0;   // Default: no temperature compensation
    }
    
    Serial.println("Temperature calibration completed for Unit " + String(unit + 1));
  }
  
  Serial.println("\nFor full temperature calibration, you would need to");
  Serial.println("collect data at different temperatures and calculate coefficients.");
  Serial.println("This is just a simplified version that records the reference values.");
  
  Serial.println("\nAll units temperature calibration complete.");
}

float CalibrationManager::compensateForTemperature(float value, float temp_coef, float temp, float temp_ref) {
  return value - (temp - temp_ref) * temp_coef;
}

void CalibrationManager::calibrateAccelData(int sensorId, float raw_x, float raw_y, float raw_z, 
                                          float temp, float &cal_x, float &cal_y, float &cal_z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    cal_x = raw_x;
    cal_y = raw_y;
    cal_z = raw_z;
    return;
  }
  
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, calibrationData[sensorId].temp_coef_accel[0], temp, calibrationData[sensorId].temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, calibrationData[sensorId].temp_coef_accel[1], temp, calibrationData[sensorId].temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, calibrationData[sensorId].temp_coef_accel[2], temp, calibrationData[sensorId].temp_ref);
  
  // Apply offset and scale calibration
  cal_x = (temp_comp_x - calibrationData[sensorId].accel_offset[0]) * calibrationData[sensorId].accel_scale[0];
  cal_y = (temp_comp_y - calibrationData[sensorId].accel_offset[1]) * calibrationData[sensorId].accel_scale[1];
  cal_z = (temp_comp_z - calibrationData[sensorId].accel_offset[2]) * calibrationData[sensorId].accel_scale[2];
}

void CalibrationManager::calibrateGyroData(int sensorId, float raw_x, float raw_y, float raw_z, 
                                         float temp, float &cal_x, float &cal_y, float &cal_z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    cal_x = raw_x;
    cal_y = raw_y;
    cal_z = raw_z;
    return;
  }
  
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, calibrationData[sensorId].temp_coef_gyro[0], temp, calibrationData[sensorId].temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, calibrationData[sensorId].temp_coef_gyro[1], temp, calibrationData[sensorId].temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, calibrationData[sensorId].temp_coef_gyro[2], temp, calibrationData[sensorId].temp_ref);
  
  // Apply offset and scale calibration
  cal_x = (temp_comp_x - calibrationData[sensorId].gyro_offset[0]) * calibrationData[sensorId].gyro_scale[0];
  cal_y = (temp_comp_y - calibrationData[sensorId].gyro_offset[1]) * calibrationData[sensorId].gyro_scale[1];
  cal_z = (temp_comp_z - calibrationData[sensorId].gyro_offset[2]) * calibrationData[sensorId].gyro_scale[2];
}

void CalibrationManager::calibrateMagData(int sensorId, float raw_x, float raw_y, float raw_z, 
                                        float &cal_x, float &cal_y, float &cal_z) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    cal_x = raw_x;
    cal_y = raw_y;
    cal_z = raw_z;
    return;
  }
  
  // Apply hard iron (offset) calibration
  float offset_cal_x = raw_x - calibrationData[sensorId].mag_offset[0];
  float offset_cal_y = raw_y - calibrationData[sensorId].mag_offset[1];
  float offset_cal_z = raw_z - calibrationData[sensorId].mag_offset[2];
  
  // Apply soft iron (scale) calibration
  cal_x = offset_cal_x * calibrationData[sensorId].mag_scale[0];
  cal_y = offset_cal_y * calibrationData[sensorId].mag_scale[1];
  cal_z = offset_cal_z * calibrationData[sensorId].mag_scale[2];
}

void CalibrationManager::printCalibrationData() {
  Serial.println("\n=== Current Calibration Data for All Units ===");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;
    
    Serial.println("\n--- Unit " + String(unit + 1) + " ---");
    
    Serial.println("Accelerometer offsets (m/s²):");
    Serial.print("X: "); Serial.print(calibrationData[unit].accel_offset[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].accel_offset[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].accel_offset[2], 4);
    Serial.println();
    
    Serial.println("Accelerometer scale factors:");
    Serial.print("X: "); Serial.print(calibrationData[unit].accel_scale[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].accel_scale[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].accel_scale[2], 4);
    Serial.println();
    
    Serial.println("Gyroscope offsets (rad/s):");
    Serial.print("X: "); Serial.print(calibrationData[unit].gyro_offset[0], 6);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].gyro_offset[1], 6);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].gyro_offset[2], 6);
    Serial.println();
    
    Serial.println("Magnetometer hard iron offsets (uT):");
    Serial.print("X: "); Serial.print(calibrationData[unit].mag_offset[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].mag_offset[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].mag_offset[2], 4);
    Serial.println();
    
    Serial.println("Magnetometer soft iron scale factors:");
    Serial.print("X: "); Serial.print(calibrationData[unit].mag_scale[0], 4);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].mag_scale[1], 4);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].mag_scale[2], 4);
    Serial.println();
    
    Serial.println("Temperature reference: " + String(calibrationData[unit].temp_ref) + " °C");
    
    // Optional: print temperature coefficients
    Serial.println("\nTemperature Coefficients:");
    Serial.println("Accelerometer:");
    Serial.print("X: "); Serial.print(calibrationData[unit].temp_coef_accel[0], 6);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].temp_coef_accel[1], 6);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].temp_coef_accel[2], 6);
    Serial.println();
    
    Serial.println("Gyroscope:");
    Serial.print("X: "); Serial.print(calibrationData[unit].temp_coef_gyro[0], 6);
    Serial.print(" Y: "); Serial.print(calibrationData[unit].temp_coef_gyro[1], 6);
    Serial.print(" Z: "); Serial.print(calibrationData[unit].temp_coef_gyro[2], 6);
    Serial.println();
  }
}

CalibrationData* CalibrationManager::getCalibrationData(int sensorId) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS) {
    return nullptr;
  }
  return &calibrationData[sensorId];
}

void CalibrationManager::setCalibrationData(int sensorId, const CalibrationData& data) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS) {
    return;
  }
  calibrationData[sensorId] = data;
}

size_t CalibrationManager::getCalibrationDataSize() const {
  return sizeof(CalibrationData) * NO_OF_UNITS;
}

CalibrationData* CalibrationManager::getAllCalibrationData() {
  return calibrationData;
}