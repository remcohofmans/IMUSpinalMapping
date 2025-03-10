// Advanced ICM-20948 demo with comprehensive calibration and filtering
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>

#define NO_OF_UNITS 2

Adafruit_ICM20948 icm[NO_OF_UNITS];
// Flags to track which sensors are active
bool sensor_active[NO_OF_UNITS] = {0};

// Calibration offsets and scale factors
// Offset = bias correction
// Scale = sensitivity correction
struct CalibrationData {
  // Accelerometer
  float accel_offset[3] = {0, 0, 0};  // X, Y, Z
  float accel_scale[3] = {1, 1, 1};   // X, Y, Z
  
  // Gyroscope
  float gyro_offset[3] = {0, 0, 0};   // X, Y, Z
  float gyro_scale[3] = {1, 1, 1};    // X, Y, Z
  
  // Magnetometer (includes hard and soft iron correction)

  // Hard iron distortion is caused by permanent magnets or magnetized materials that create a constant magnetic field in the vicinity of your sensor.
  // Without any interference, magnetometer readings plotted in 3D would form a sphere centered at the origin. Hard iron effects shift this sphere away from the origin.
  // --> subtract a fixed offset vector from all readings
  // Soft iron distortion is caused by materials that distort the Earth's magnetic field but don't generate their own magnetic field.
  // Soft iron effects transform the ideal measurement sphere into an ellipsoid (stretched or squashed sphere).
  // The ellipsoid remains centered at the origin after hard iron correction, but it's no longer spherical.

  float mag_offset[3] = {0, 0, 0};    // X, Y, Z (hard iron)
  float mag_scale[3] = {1, 1, 1};     // X, Y, Z (soft iron)
  float mag_transform[3][3] = {       // Soft iron transform matrix (scale factors)
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // Temperature compensation
  float temp_ref = 25.0;  // Reference temperature
  float temp_coef_accel[3] = {0, 0, 0};  // Temperature coefficients for accel
  float temp_coef_gyro[3] = {0, 0, 0};   // Temperature coefficients for gyro
};

CalibrationData cal[NO_OF_UNITS];

// Moving average filter buffers
#define FILTER_SAMPLES 10
float accel_x_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float accel_y_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float accel_z_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float gyro_x_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float gyro_y_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float gyro_z_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float mag_x_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float mag_y_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
float mag_z_buffer[NO_OF_UNITS][FILTER_SAMPLES] = {{0}};
int buffer_index[NO_OF_UNITS] = {0};

// Complementary filter variables
float comp_angle_x[NO_OF_UNITS] = {0}, comp_angle_y[NO_OF_UNITS] = {0}, comp_angle_z[NO_OF_UNITS] = {0};
unsigned long last_time[NO_OF_UNITS] = {0};
float alpha = 0.98;  // Filter coefficient (0.98 = 98% gyro, 2% accel)

// Kalman filter variables
struct KalmanState {
  float x;      // State
  float p;      // Estimation error covariance
  float q;      // Process noise covariance
  float r;      // Measurement noise covariance
  float k;      // Kalman gain
};

KalmanState kalman_accel_x[NO_OF_UNITS];
KalmanState kalman_accel_y[NO_OF_UNITS];
KalmanState kalman_accel_z[NO_OF_UNITS];
KalmanState kalman_gyro_x[NO_OF_UNITS];
KalmanState kalman_gyro_y[NO_OF_UNITS];
KalmanState kalman_gyro_z[NO_OF_UNITS];

// EEPROM storage addresses
#define EEPROM_ADDR_CALIBRATION 0
#define EEPROM_MAGIC_NUMBER 12345  // To verify data is valid

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("ICM-20948 Advanced Calibration and Filtering Demo");
  
  // Initialize Wire library
  Wire.begin();
  
  // Initialize sensor_active array
  for (int i = 0; i < NO_OF_UNITS; i++) {
    sensor_active[i] = false;
  }
  
  // Try to initialize the IMUs with specific addresses
  int active_sensors = 0;
  
  // Addresses to try: 0x68 and 0x69
  uint8_t addresses[] = {0x68, 0x69};
  
  for (int i = 0; i < NO_OF_UNITS; i++) {
    // Try to initialize each sensor with its specific address
    if (icm[i].begin_I2C(addresses[i])) {
      sensor_active[i] = true;
      active_sensors++;
      Serial.print("Sensor Unit "); 
      Serial.print(i + 1);
      Serial.print(" initialized successfully at address 0x");
      Serial.println(addresses[i], HEX);
    } else {
      Serial.print("Failed to find ICM-20948 chip for Unit ");
      Serial.print(i + 1);
      Serial.print(" at address 0x");
      Serial.println(addresses[i], HEX);
    }
  }
  
  // Halt if no sensors found
  if (active_sensors == 0) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    while (1) {
      delay(1000);  // Persistent halt with occasional LED blink possible
    }
  }
  
  Serial.print("Total active sensors: ");
  Serial.println(active_sensors);
  
  // Initialize Kalman filter states for all sensors
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensor_active[i]) continue;  // Skip inactive sensors
    
    // Accelerometer Kalman filters
    kalman_accel_x[i].x = 0;
    kalman_accel_x[i].p = 1;
    kalman_accel_x[i].q = 0.01;
    kalman_accel_x[i].r = 0.1;
    
    kalman_accel_y[i].x = 0;
    kalman_accel_y[i].p = 1;
    kalman_accel_y[i].q = 0.01;
    kalman_accel_y[i].r = 0.1;
    
    kalman_accel_z[i].x = 0;
    kalman_accel_z[i].p = 1;
    kalman_accel_z[i].q = 0.01;
    kalman_accel_z[i].r = 0.1;
    
    // Gyroscope Kalman filters
    kalman_gyro_x[i].x = 0;
    kalman_gyro_x[i].p = 1;
    kalman_gyro_x[i].q = 0.001;
    kalman_gyro_x[i].r = 0.03;

    kalman_gyro_y[i].x = 0;
    kalman_gyro_y[i].p = 1;
    kalman_gyro_y[i].q = 0.001;
    kalman_gyro_y[i].r = 0.03;

    kalman_gyro_z[i].x = 0;
    kalman_gyro_z[i].p = 1;
    kalman_gyro_z[i].q = 0.001;
    kalman_gyro_z[i].r = 0.03;
  }
  
  // Configure sensor settings for best calibration results
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensor_active[i]) continue;  // Skip inactive sensors
    
    icm[i].setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm[i].setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
    icm[i].setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  }
  
  Serial.println("Sensors configured for calibration");
  
  // Try to load calibration data from EEPROM
  if (!loadCalibrationFromEEPROM()) {
    Serial.println("No valid calibration data found in EEPROM");
  } else {
    Serial.println("Calibration data loaded from EEPROM");
    printCalibrationData();
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
    performFullCalibration();
    saveCalibrationToEEPROM();
  } else {
    Serial.println("Using existing calibration values");
  }
  
  // Initialize timing for complementary filter
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensor_active[i]) continue;  // Skip inactive sensors
    last_time[i] = millis();
  }
}

void performFullCalibration() {
  calibrateGyros();
  calibrateAccelerometers();
  calibrateMagnetometers();
  calibrateTemperatures();
}

// Gyroscope calibration - just capture offsets while sensor is still
void calibrateGyros() {
  Serial.println("\n=== Gyroscope Calibration ===");
  Serial.println("Keep the sensor units completely STILL");
  Serial.println("Starting in 5 seconds...");
  
  for (int i = 5; i > 0; i--) {
    Serial.print(i); Serial.println("...");
    delay(1000);
  }
  
  Serial.println("Calibrating - DO NOT MOVE...");
  
  float gyro_x_sum[NO_OF_UNITS] = {0}, gyro_y_sum[NO_OF_UNITS] = {0}, gyro_z_sum[NO_OF_UNITS] = {0};
  const int samples = 500;  // More samples for better accuracy
  
  for (int i = 0; i < samples; i++) {
    for(int j = 0; i < NO_OF_UNITS; i++) {
      sensors_event_t accel, gyro, mag, temp;
      icm[j].getEvent(&accel, &gyro, &temp, &mag);
      
      gyro_x_sum[j] += gyro.gyro.x;
      gyro_y_sum[j] += gyro.gyro.y;
      gyro_z_sum[j] += gyro.gyro.z;
    }
    if (i % 50 == 0) Serial.print(".");
    delay(10);
  }
  
  // Calculate averages to get bias
  for(int i = 0; i < NO_OF_UNITS; i++) {
    cal[i].gyro_offset[0] = gyro_x_sum[i] / samples;
    cal[i].gyro_offset[1] = gyro_y_sum[i] / samples;
    cal[i].gyro_offset[2] = gyro_z_sum[i] / samples;
    
    Serial.println("\nGyroscope calibration complete!");
    Serial.println("Gyroscope offsets (rad/s):");
    Serial.print("X: "); Serial.print(cal[i].gyro_offset[0], 6);
    Serial.print(" Y: "); Serial.print(cal[i].gyro_offset[1], 6);
    Serial.print(" Z: "); Serial.print(cal[i].gyro_offset[2], 6);
    Serial.println();
  }
}

// Accelerometer calibration - needs 6-position calibration for multiple units
void calibrateAccelerometers() {
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
      for (int unit = 0; unit < NO_OF_UNITS; unit++) {
        if (sensor_active[unit]) {  // Only process active sensors
          sensors_event_t accel, gyro, mag, temp;
          icm[unit].getEvent(&accel, &gyro, &temp, &mag);
          
          sum_x[unit] += accel.acceleration.x;
          sum_y[unit] += accel.acceleration.y;
          sum_z[unit] += accel.acceleration.z;
        }
      }
      
      if (i % 20 == 0) Serial.print(".");
      delay(10);
    }
    
    // Store average for each unit
    for (int unit = 0; unit < NO_OF_UNITS; unit++) {
      if (sensor_active[unit]) {
        accel_readings[unit][position][0] = sum_x[unit] / samples_per_position;
        accel_readings[unit][position][1] = sum_y[unit] / samples_per_position;
        accel_readings[unit][position][2] = sum_z[unit] / samples_per_position;
      }
    }
    
    Serial.println("\nRecorded position " + String(position + 1) + "/6 for all units");
  }
  
  // Process calibration data for each unit
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (sensor_active[unit]) {
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
      cal[unit].accel_offset[0] = (min_x + max_x) / 2.0;
      cal[unit].accel_offset[1] = (min_y + max_y) / 2.0;
      cal[unit].accel_offset[2] = (min_z + max_z) / 2.0;
      
      // Calculate scale factors - normalized to 9.8 m/s² (gravity)
      cal[unit].accel_scale[0] = 19.6 / (max_x - min_x);
      cal[unit].accel_scale[1] = 19.6 / (max_y - min_y);
      cal[unit].accel_scale[2] = 19.6 / (max_z - min_z);
      
      Serial.println("Accelerometer calibration complete for Unit " + String(unit + 1));
      Serial.println("Accel offsets (m/s²):");
      Serial.print("X: "); Serial.print(cal[unit].accel_offset[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].accel_offset[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].accel_offset[2], 4);
      Serial.println();
      
      Serial.println("Accel scale factors:");
      Serial.print("X: "); Serial.print(cal[unit].accel_scale[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].accel_scale[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].accel_scale[2], 4);
      Serial.println();
    }
  }
  
  Serial.println("\nAll accelerometers calibration complete!");
}

// Magnetometer calibration (hard iron and soft iron) for multiple units
void calibrateMagnetometers() {
  Serial.println("\n=== Magnetometer Calibration for Multiple Units ===");
  Serial.println("We'll calibrate each sensor unit one by one.");
  Serial.println("For each unit, rotate it slowly in all directions to");
  Serial.println("sample the complete 3D magnetic field.");
  Serial.println("Try to cover all possible orientations.");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (sensor_active[unit]) {
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
        
        sensors_event_t accel, gyro, mag, temp;
        icm[unit].getEvent(&accel, &gyro, &temp, &mag);
        
        // Store sample if it's different from previous ones to avoid duplicates
        if (sample_count == 0 || 
            (mag.magnetic.x != mag_x[sample_count-1] || 
             mag.magnetic.y != mag_y[sample_count-1] || 
             mag.magnetic.z != mag_z[sample_count-1])) {
          
          mag_x[sample_count] = mag.magnetic.x;
          mag_y[sample_count] = mag.magnetic.y;
          mag_z[sample_count] = mag.magnetic.z;
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
      cal[unit].mag_offset[0] = (min_x + max_x) / 2.0;
      cal[unit].mag_offset[1] = (min_y + max_y) / 2.0;
      cal[unit].mag_offset[2] = (min_z + max_z) / 2.0;
      
      // Calculate soft iron scale factors
      float avg_delta = ((max_x - min_x) + (max_y - min_y) + (max_z - min_z)) / 3.0;
      
      cal[unit].mag_scale[0] = avg_delta / (max_x - min_x);
      cal[unit].mag_scale[1] = avg_delta / (max_y - min_y);
      cal[unit].mag_scale[2] = avg_delta / (max_z - min_z);
      
      // For a full soft iron calibration, we would calculate a 3x3 transformation matrix
      // This is simplified to the use of scale factors
      
      Serial.println("\nMagnetometer calibration complete for Unit " + String(unit + 1));
      Serial.println("Hard iron offsets (uT):");
      Serial.print("X: "); Serial.print(cal[unit].mag_offset[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].mag_offset[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].mag_offset[2], 4);
      Serial.println();
      
      Serial.println("Soft iron scale factors:");
      Serial.print("X: "); Serial.print(cal[unit].mag_scale[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].mag_scale[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].mag_scale[2], 4);
      Serial.println();
    }
  }
  
  Serial.println("\nAll units magnetometer calibration complete!");
}

// Temperature calibration for multiple units
void calibrateTemperatures() {
  Serial.println("\n=== Temperature Compensation Calibration for Multiple Units ===");
  Serial.println("This calibration collects reference temperature data for each sensor unit.");
  Serial.println("We'll collect data at room temperature now.");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (sensor_active[unit]) {
      // Get reference temperature and readings for this unit
      sensors_event_t accel, gyro, mag, temp;
      icm[unit].getEvent(&accel, &gyro, &temp, &mag);
      
      cal[unit].temp_ref = temp.temperature;
      
      float accel_ref[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
      float gyro_ref[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
      
      Serial.println("Unit " + String(unit + 1) + " reference temperature: " + String(cal[unit].temp_ref) + " °C");
      
      // Using default values for temperature coefficients
      for (int i = 0; i < 3; i++) {
        cal[unit].temp_coef_accel[i] = 0.0;  // Default: no temperature compensation
        cal[unit].temp_coef_gyro[i] = 0.0;   // Default: no temperature compensation
      }
      
      Serial.println("Temperature calibration completed for Unit " + String(unit + 1));
    }
  }
  
  Serial.println("\nFor full temperature calibration, you would need to");
  Serial.println("collect data at different temperatures and calculate coefficients.");
  Serial.println("This is just a simplified version that records the reference values.");
  
  // For a real calibration, undertake the following actions:
  // 1. Change sensor temperature
  // 2. Collect data at each temperature
  // 3. Calculate temperature coefficients (change per degree C)
  
  Serial.println("\nAll units temperature calibration complete.");
}

// Moving average filter
float applyMovingAverage(float new_value, float buffer[], int &buffer_idx) {
  // Add new value to buffer
  buffer[buffer_idx] = new_value;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  
  return sum / FILTER_SAMPLES;
}

// Simple Kalman filter implementation
float applyKalmanFilter(float measurement, KalmanState &state) {
  // Prediction
  // x = x (no state transition model for constant value)
  // p = p + q
  state.p = state.p + state.q;
  
  // Update
  state.k = state.p / (state.p + state.r);
  state.x = state.x + state.k * (measurement - state.x);
  state.p = (1 - state.k) * state.p;
  
  return state.x;
}

// Apply temperature compensation
float compensateForTemperature(float value, float temp_coef, float temp, float temp_ref) {
  return value - (temp - temp_ref) * temp_coef;
}

// Apply all calibrations to raw accelerometer data
void calibrateAccelData(int sensor_id, float raw_x, float raw_y, float raw_z, float temp, float &cal_x, float &cal_y, float &cal_z) {
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, cal[sensor_id].temp_coef_accel[0], temp, cal[sensor_id].temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, cal[sensor_id].temp_coef_accel[1], temp, cal[sensor_id].temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, cal[sensor_id].temp_coef_accel[2], temp, cal[sensor_id].temp_ref);
  
  // Apply offset and scale calibration
  float offset_cal_x = (temp_comp_x - cal[sensor_id].accel_offset[0]) * cal[sensor_id].accel_scale[0];
  float offset_cal_y = (temp_comp_y - cal[sensor_id].accel_offset[1]) * cal[sensor_id].accel_scale[1];
  float offset_cal_z = (temp_comp_z - cal[sensor_id].accel_offset[2]) * cal[sensor_id].accel_scale[2];
  
  // Apply moving average filter
  float filtered_x = applyMovingAverage(offset_cal_x, accel_x_buffer[sensor_id], buffer_index[sensor_id]);
  float filtered_y = applyMovingAverage(offset_cal_y, accel_y_buffer[sensor_id], buffer_index[sensor_id]);
  float filtered_z = applyMovingAverage(offset_cal_z, accel_z_buffer[sensor_id], buffer_index[sensor_id]);
  
  // Apply Kalman filter
  cal_x = applyKalmanFilter(filtered_x, kalman_accel_x[sensor_id]);
  cal_y = applyKalmanFilter(filtered_y, kalman_accel_y[sensor_id]);
  cal_z = applyKalmanFilter(filtered_z, kalman_accel_z[sensor_id]);
}

// Apply all calibrations to raw gyroscope data
void calibrateGyroData(int sensor_id, float raw_x, float raw_y, float raw_z, float temp, float &cal_x, float &cal_y, float &cal_z) {
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, cal[sensor_id].temp_coef_gyro[0], temp, cal[sensor_id].temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, cal[sensor_id].temp_coef_gyro[1], temp, cal[sensor_id].temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, cal[sensor_id].temp_coef_gyro[2], temp, cal[sensor_id].temp_ref);
  
  // Apply offset and scale calibration
  float offset_cal_x = (temp_comp_x - cal[sensor_id].gyro_offset[0]) * cal[sensor_id].gyro_scale[0];
  float offset_cal_y = (temp_comp_y - cal[sensor_id].gyro_offset[1]) * cal[sensor_id].gyro_scale[1];
  float offset_cal_z = (temp_comp_z - cal[sensor_id].gyro_offset[2]) * cal[sensor_id].gyro_scale[2];
  
  // Apply moving average filter
  float filtered_x = applyMovingAverage(offset_cal_x, gyro_x_buffer[sensor_id], buffer_index[sensor_id]);
  float filtered_y = applyMovingAverage(offset_cal_y, gyro_y_buffer[sensor_id], buffer_index[sensor_id]);
  float filtered_z = applyMovingAverage(offset_cal_z, gyro_z_buffer[sensor_id], buffer_index[sensor_id]);
  
  // Apply Kalman filter
  cal_x = applyKalmanFilter(filtered_x, kalman_gyro_x[sensor_id]);
  cal_y = applyKalmanFilter(filtered_y, kalman_gyro_y[sensor_id]);
  cal_z = applyKalmanFilter(filtered_z, kalman_gyro_z[sensor_id]);
}

// Apply all calibrations to raw magnetometer data
void calibrateMagData(int sensor_id, float raw_x, float raw_y, float raw_z, float &cal_x, float &cal_y, float &cal_z) {
  // Apply hard iron (offset) calibration
  float offset_cal_x = raw_x - cal[sensor_id].mag_offset[0];
  float offset_cal_y = raw_y - cal[sensor_id].mag_offset[1];
  float offset_cal_z = raw_z - cal[sensor_id].mag_offset[2];
  
  // Apply soft iron (scale) calibration
  float scale_cal_x = offset_cal_x * cal[sensor_id].mag_scale[0];
  float scale_cal_y = offset_cal_y * cal[sensor_id].mag_scale[1];
  float scale_cal_z = offset_cal_z * cal[sensor_id].mag_scale[2];
  
  // Apply moving average filter
  cal_x = applyMovingAverage(scale_cal_x, mag_x_buffer[sensor_id], buffer_index[sensor_id]);
  cal_y = applyMovingAverage(scale_cal_y, mag_y_buffer[sensor_id], buffer_index[sensor_id]);
  cal_z = applyMovingAverage(scale_cal_z, mag_z_buffer[sensor_id], buffer_index[sensor_id]);
  
  // Note: Not applying Kalman to mag data as it's usually slower changing
}

// Update complementary filter for multiple units
void updateComplementaryFilter(int sensor_id, float accel_x, float accel_y, float accel_z,
                              float gyro_x, float gyro_y, float gyro_z,
                              float mag_x, float mag_y, float mag_z) {
  // Calculate dt
  unsigned long now = millis();
  float dt = (now - last_time[sensor_id]) / 1000.0;
  last_time[sensor_id] = now;
  
  // Calculate angles from accelerometer (roll & pitch only, not yaw)
  float accel_angle_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / PI;
  float accel_angle_y = atan2(-accel_x, accel_z) * 180.0 / PI;
  
  // Calculate yaw from magnetometer (tilt-compensated compass)
  float mag_angle_z = 0;
  
  // Tilt compensation for magnetometer
  float cosRoll = cos(comp_angle_x[sensor_id] * PI / 180.0);
  float sinRoll = sin(comp_angle_x[sensor_id] * PI / 180.0);
  float cosPitch = cos(comp_angle_y[sensor_id] * PI / 180.0);
  float sinPitch = sin(comp_angle_y[sensor_id] * PI / 180.0);
  
  // Tilt compensated magnetic field X
  float tiltCompensatedX = mag_x * cosPitch + mag_y * sinRoll * sinPitch + mag_z * cosPitch * sinPitch;
  // Tilt compensated magnetic field Y
  float tiltCompensatedY = mag_y * cosRoll - mag_z * sinRoll;
  // Magnetic yaw
  mag_angle_z = atan2(-tiltCompensatedY, tiltCompensatedX) * 180.0 / PI;
  
  // Complementary filter for roll and pitch
  comp_angle_x[sensor_id] = alpha * (comp_angle_x[sensor_id] + gyro_x * dt * 180.0 / PI) + (1.0 - alpha) * accel_angle_x;
  comp_angle_y[sensor_id] = alpha * (comp_angle_y[sensor_id] + gyro_y * dt * 180.0 / PI) + (1.0 - alpha) * accel_angle_y;
  
  // Complementary filter for yaw using magnetometer
  float gyro_angle_z = comp_angle_z[sensor_id] + gyro_z * dt * 180.0 / PI;
  
  // Normalize mag_angle_z to be within ±180° of the current gyro-based yaw
  while (mag_angle_z - gyro_angle_z > 180) mag_angle_z -= 360;
  while (mag_angle_z - gyro_angle_z < -180) mag_angle_z += 360;
  
  // Yaw can't be corrected with accelerometer, so just integrate gyro
  comp_angle_z[sensor_id] = alpha * gyro_angle_z + (1.0 - alpha) * mag_angle_z;
}

// Save calibration data to EEPROM
void saveCalibrationToEEPROM() {
  int address = EEPROM_ADDR_CALIBRATION;
  
  // Write magic number to indicate valid data
  EEPROM.writeUInt(address, EEPROM_MAGIC_NUMBER);
  address += sizeof(uint32_t);
  
  // Write calibration data
  uint8_t* data = (uint8_t*)&cal;
  for (size_t i = 0; i < sizeof(CalibrationData); i++) {
    EEPROM.write(address++, data[i]);
  }
  
  EEPROM.commit();
  Serial.println("Calibration data saved to EEPROM");
}

// Load calibration data from EEPROM
bool loadCalibrationFromEEPROM() {
  int address = EEPROM_ADDR_CALIBRATION;
  
  // Check magic number
  uint32_t magic = EEPROM.readUInt(address);
  if (magic != EEPROM_MAGIC_NUMBER) {
    return false;
  }
  address += sizeof(uint32_t);
  
  // Read calibration data
  uint8_t* data = (uint8_t*)&cal;
  for (size_t i = 0; i < sizeof(CalibrationData); i++) {
    data[i] = EEPROM.read(address++);
  }
  
  return true;
}

/// Print calibration data for multiple units
void printCalibrationData() {
  Serial.println("\n=== Current Calibration Data for All Units ===");
  
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (sensor_active[unit]) {
      Serial.println("\n--- Unit " + String(unit + 1) + " ---");
      
      Serial.println("Accelerometer offsets (m/s²):");
      Serial.print("X: "); Serial.print(cal[unit].accel_offset[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].accel_offset[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].accel_offset[2], 4);
      Serial.println();
      
      Serial.println("Accelerometer scale factors:");
      Serial.print("X: "); Serial.print(cal[unit].accel_scale[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].accel_scale[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].accel_scale[2], 4);
      Serial.println();
      
      Serial.println("Gyroscope offsets (rad/s):");
      Serial.print("X: "); Serial.print(cal[unit].gyro_offset[0], 6);
      Serial.print(" Y: "); Serial.print(cal[unit].gyro_offset[1], 6);
      Serial.print(" Z: "); Serial.print(cal[unit].gyro_offset[2], 6);
      Serial.println();
      
      Serial.println("Magnetometer hard iron offsets (uT):");
      Serial.print("X: "); Serial.print(cal[unit].mag_offset[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].mag_offset[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].mag_offset[2], 4);
      Serial.println();
      
      Serial.println("Magnetometer soft iron scale factors:");
      Serial.print("X: "); Serial.print(cal[unit].mag_scale[0], 4);
      Serial.print(" Y: "); Serial.print(cal[unit].mag_scale[1], 4);
      Serial.print(" Z: "); Serial.print(cal[unit].mag_scale[2], 4);
      Serial.println();
      
      Serial.println("Temperature reference: " + String(cal[unit].temp_ref) + " °C");
      
      // Optional: print temperature coefficients
      Serial.println("\nTemperature Coefficients:");
      Serial.println("Accelerometer:");
      Serial.print("X: "); Serial.print(cal[unit].temp_coef_accel[0], 6);
      Serial.print(" Y: "); Serial.print(cal[unit].temp_coef_accel[1], 6);
      Serial.print(" Z: "); Serial.print(cal[unit].temp_coef_accel[2], 6);
      Serial.println();
      
      Serial.println("Gyroscope:");
      Serial.print("X: "); Serial.print(cal[unit].temp_coef_gyro[0], 6);
      Serial.print(" Y: "); Serial.print(cal[unit].temp_coef_gyro[1], 6);
      Serial.print(" Z: "); Serial.print(cal[unit].temp_coef_gyro[2], 6);
      Serial.println();
    }
  }
}

void loop() {
  // Iterate through all sensor units
  for (int sensor_id = 0; sensor_id < NO_OF_UNITS; sensor_id++) {

    // Skip inactive sensors
    if (!sensor_active[sensor_id]) continue;

    // Get raw sensor readings for this specific sensor
    sensors_event_t accel, gyro, mag, temp;
    icm[sensor_id].getEvent(&accel, &gyro, &temp, &mag);

    // Update buffer index for moving average filter
    buffer_index[sensor_id] = (buffer_index[sensor_id] + 1) % FILTER_SAMPLES;
  
    // Apply calibration and filtering
    float cal_accel_x, cal_accel_y, cal_accel_z;
    float cal_gyro_x, cal_gyro_y, cal_gyro_z;
    float cal_mag_x, cal_mag_y, cal_mag_z;

    calibrateAccelData(sensor_id, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                     temp.temperature, cal_accel_x, cal_accel_y, cal_accel_z);
                      
    calibrateGyroData(sensor_id, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
                     temp.temperature, cal_gyro_x, cal_gyro_y, cal_gyro_z);
                     
    calibrateMagData(sensor_id, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, 
                    cal_mag_x, cal_mag_y, cal_mag_z);
    
    // Update complementary filter for orientation
    updateComplementaryFilter(sensor_id, 
                             cal_accel_x, cal_accel_y, cal_accel_z, 
                             cal_gyro_x, cal_gyro_y, cal_gyro_z,
                             cal_mag_x, cal_mag_y, cal_mag_z);
  }
  
  // Print data for all active sensors every 250ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    
    for (int sensor_id = 0; sensor_id < NO_OF_UNITS; sensor_id++) {
      if (!sensor_active[sensor_id]) continue;

      // Get raw sensor readings for this specific sensor
      sensors_event_t accel, gyro, mag, temp;
      icm[sensor_id].getEvent(&accel, &gyro, &temp, &mag);

      // Recalculate calibrated data for printing
      float cal_accel_x, cal_accel_y, cal_accel_z;
      float cal_gyro_x, cal_gyro_y, cal_gyro_z;
      float cal_mag_x, cal_mag_y, cal_mag_z;

      calibrateAccelData(sensor_id, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                       temp.temperature, cal_accel_x, cal_accel_y, cal_accel_z);
                        
      calibrateGyroData(sensor_id, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
                       temp.temperature, cal_gyro_x, cal_gyro_y, cal_gyro_z);
                       
      calibrateMagData(sensor_id, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, 
                      cal_mag_x, cal_mag_y, cal_mag_z);
    
      Serial.println("\n===========================");
      Serial.println("Sensor Unit: " + String(sensor_id + 1));
      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" °C");
      
      // Accelerometer data
      Serial.println("\nAccelerometer (m/s²):");
      Serial.print("  Raw: X=");
      Serial.print(accel.acceleration.x, 3);
      Serial.print(" Y=");
      Serial.print(accel.acceleration.y, 3);
      Serial.print(" Z=");
      Serial.println(accel.acceleration.z, 3);
      
      Serial.print("  Calibrated: X=");
      Serial.print(cal_accel_x, 3);
      Serial.print(" Y=");
      Serial.print(cal_accel_y, 3);
      Serial.print(" Z=");
      Serial.println(cal_accel_z, 3);
      
      // Gyroscope data
      Serial.println("\nGyroscope (rad/s):");
      Serial.print("  Raw: X=");
      Serial.print(gyro.gyro.x, 4);
      Serial.print(" Y=");
      Serial.print(gyro.gyro.y, 4);
      Serial.print(" Z=");
      Serial.println(gyro.gyro.z, 4);
      
      Serial.print("  Calibrated: X=");
      Serial.print(cal_gyro_x, 4);
      Serial.print(" Y=");
      Serial.print(cal_gyro_y, 4);
      Serial.print(" Z=");
      Serial.println(cal_gyro_z, 4);
      
      // Magnetometer data
      Serial.println("\nMagnetometer (uT):");
      Serial.print("  Raw: X=");
      Serial.print(mag.magnetic.x, 2);
      Serial.print(" Y=");
      Serial.print(mag.magnetic.y, 2);
      Serial.print(" Z=");
      Serial.println(mag.magnetic.z, 2);
      
      Serial.print("  Calibrated: X=");
      Serial.print(cal_mag_x, 2);
      Serial.print(" Y=");
      Serial.print(cal_mag_y, 2);
      Serial.print(" Z=");
      Serial.println(cal_mag_z, 2);
      
      // Orientation from complementary filter
      Serial.println("\nOrientation (degrees):");
      Serial.print("  Roll (X): ");
      Serial.print(comp_angle_x[sensor_id], 1);
      Serial.print(" Pitch (Y): ");
      Serial.print(comp_angle_y[sensor_id], 1);
      Serial.print(" Yaw (Z): ");
      Serial.println(comp_angle_z[sensor_id], 1);
      
      // Calculate magnitude to verify calibration
      float raw_accel_mag = sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.y) + sq(accel.acceleration.z));
      float cal_accel_mag = sqrt(sq(cal_accel_x) + sq(cal_accel_y) + sq(cal_accel_z));
      
      Serial.println("\nAccel Magnitude (should be ~9.8 m/s² when stationary):");
      Serial.print("  Raw: ");
      Serial.print(raw_accel_mag, 3);
      Serial.print(" Calibrated: ");
      Serial.println(cal_accel_mag, 3);
      
      float raw_mag_mag = sqrt(sq(mag.magnetic.x) + sq(mag.magnetic.y) + sq(mag.magnetic.z));
      float cal_mag_mag = sqrt(sq(cal_mag_x) + sq(cal_mag_y) + sq(cal_mag_z));
      
      Serial.println("Mag Magnitude (should be constant regardless of orientation):");
      Serial.print("  Raw: ");
      Serial.print(raw_mag_mag, 2);
      Serial.print(" Calibrated: ");
      Serial.println(cal_mag_mag, 2);
    }
  }
  
  // Delay to not overwhelm the serial output
  delay(10);
}
