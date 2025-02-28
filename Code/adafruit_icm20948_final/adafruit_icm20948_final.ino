// Advanced ICM-20948 demo with comprehensive calibration and filtering
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>

Adafruit_ICM20948 icm;

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

CalibrationData cal;

// Moving average filter buffers
#define FILTER_SAMPLES 10
float accel_x_buffer[FILTER_SAMPLES] = {0};
float accel_y_buffer[FILTER_SAMPLES] = {0};
float accel_z_buffer[FILTER_SAMPLES] = {0};
float gyro_x_buffer[FILTER_SAMPLES] = {0};
float gyro_y_buffer[FILTER_SAMPLES] = {0};
float gyro_z_buffer[FILTER_SAMPLES] = {0};
float mag_x_buffer[FILTER_SAMPLES] = {0};
float mag_y_buffer[FILTER_SAMPLES] = {0};
float mag_z_buffer[FILTER_SAMPLES] = {0};
int buffer_index = 0;

// Complementary filter variables
float comp_angle_x = 0, comp_angle_y = 0, comp_angle_z = 0;
unsigned long last_time = 0;
float alpha = 0.98;  // Filter coefficient (0.98 = 98% gyro, 2% accel)

// Kalman filter variables
struct KalmanState {
  float x;      // State
  float p;      // Estimation error covariance
  float q;      // Process noise covariance
  float r;      // Measurement noise covariance
  float k;      // Kalman gain
};

KalmanState kalman_accel_x = {0, 1, 0.01, 0.1};
KalmanState kalman_accel_y = {0, 1, 0.01, 0.1};
KalmanState kalman_accel_z = {0, 1, 0.01, 0.1};
KalmanState kalman_gyro_x = {0, 1, 0.001, 0.03};
KalmanState kalman_gyro_y = {0, 1, 0.001, 0.03};
KalmanState kalman_gyro_z = {0, 1, 0.001, 0.03};

// EEPROM storage addresses
#define EEPROM_ADDR_CALIBRATION 0
#define EEPROM_MAGIC_NUMBER 12345  // To verify data is valid

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("ICM-20948 Advanced Calibration and Filtering Demo");
  
  // Initialize EEPROM for ESP32
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialize EEPROM");
  }
  
  // Try to initialize the sensor
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM-20948 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("ICM-20948 Found!");
  
  // Configure sensor settings for best calibration results
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  Serial.println("Sensor configured for calibration");
  
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
  last_time = millis();
}

void performFullCalibration() {
  calibrateGyro();
  calibrateAccelerometer();
  calibrateMagnetometer();
  calibrateTemperature();
}

// Gyroscope calibration - just capture offsets while sensor is still
void calibrateGyro() {
  Serial.println("\n=== Gyroscope Calibration ===");
  Serial.println("Keep the sensor completely STILL");
  Serial.println("Starting in 5 seconds...");
  
  for (int i = 5; i > 0; i--) {
    Serial.print(i); Serial.println("...");
    delay(1000);
  }
  
  Serial.println("Calibrating - DO NOT MOVE...");
  
  float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  const int samples = 500;  // More samples for better accuracy
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    
    gyro_x_sum += gyro.gyro.x;
    gyro_y_sum += gyro.gyro.y;
    gyro_z_sum += gyro.gyro.z;
    
    if (i % 50 == 0) Serial.print(".");
    delay(10);
  }
  
  // Calculate averages to get bias
  cal.gyro_offset[0] = gyro_x_sum / samples;
  cal.gyro_offset[1] = gyro_y_sum / samples;
  cal.gyro_offset[2] = gyro_z_sum / samples;
  
  Serial.println("\nGyroscope calibration complete!");
  Serial.println("Gyroscope offsets (rad/s):");
  Serial.print("X: "); Serial.print(cal.gyro_offset[0], 6);
  Serial.print(" Y: "); Serial.print(cal.gyro_offset[1], 6);
  Serial.print(" Z: "); Serial.print(cal.gyro_offset[2], 6);
  Serial.println();
}

// Accelerometer calibration - needs 6-position calibration
void calibrateAccelerometer() {
  Serial.println("\n=== Accelerometer Calibration ===");
  Serial.println("This requires positioning the sensor in 6 different orientations");
  Serial.println("You'll be prompted to place the sensor on each face");
  
  float accel_readings[6][3];  // Store readings for all 6 positions
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
    
    Serial.print("\nPosition sensor with ");
    Serial.println(directions[position]);
    Serial.println("Press any key when ready");
    
    while (!Serial.available()) {
      // Wait for user input
    }
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    Serial.println("Hold still... measuring");
    delay(2000);  // Give time to stabilize
    
    // Take measurements
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (int i = 0; i < samples_per_position; i++) {
      sensors_event_t accel, gyro, mag, temp;
      icm.getEvent(&accel, &gyro, &temp, &mag);
      
      sum_x += accel.acceleration.x;
      sum_y += accel.acceleration.y;
      sum_z += accel.acceleration.z;
      
      if (i % 20 == 0) Serial.print(".");
      delay(10);
    }
    
    // Store average
    accel_readings[position][0] = sum_x / samples_per_position;
    accel_readings[position][1] = sum_y / samples_per_position;
    accel_readings[position][2] = sum_z / samples_per_position;
    
    Serial.println("\nRecorded position " + String(position + 1) + "/6");
  }
  
  // Process 6-position calibration data
  // This is a simplified approach - a more complex approach would for example use least squares fitting
  
  // For each axis, find min and max values
  float min_x = 9999, max_x = -9999;
  float min_y = 9999, max_y = -9999;
  float min_z = 9999, max_z = -9999;
  
  for (int i = 0; i < 6; i++) {
    if (accel_readings[i][0] < min_x) min_x = accel_readings[i][0];
    if (accel_readings[i][0] > max_x) max_x = accel_readings[i][0];
    
    if (accel_readings[i][1] < min_y) min_y = accel_readings[i][1];
    if (accel_readings[i][1] > max_y) max_y = accel_readings[i][1];
    
    if (accel_readings[i][2] < min_z) min_z = accel_readings[i][2];
    if (accel_readings[i][2] > max_z) max_z = accel_readings[i][2];
  }
  
  // Calculate offsets (bias) - the center of min and max
  cal.accel_offset[0] = (min_x + max_x) / 2.0;
  cal.accel_offset[1] = (min_y + max_y) / 2.0;
  cal.accel_offset[2] = (min_z + max_z) / 2.0;
  
  // Calculate scale factors. These compensates for manufacturing variations in the sensitivity of the sensor - normalized to 9.8 m/s² (gravity).
  // Applying a correction factor ensures a measured physical acceleration produces the correct measured value along each axis. If not handled properly, angles will need correction. 
  cal.accel_scale[0] = 19.6 / (max_x - min_x);
  cal.accel_scale[1] = 19.6 / (max_y - min_y);
  cal.accel_scale[2] = 19.6 / (max_z - min_z);
  
  Serial.println("\nAccelerometer calibration complete!");
  Serial.println("Accel offsets (m/s²):");
  Serial.print("X: "); Serial.print(cal.accel_offset[0], 4);
  Serial.print(" Y: "); Serial.print(cal.accel_offset[1], 4);
  Serial.print(" Z: "); Serial.print(cal.accel_offset[2], 4);
  Serial.println();
  
  Serial.println("Accel scale factors:");
  Serial.print("X: "); Serial.print(cal.accel_scale[0], 4);
  Serial.print(" Y: "); Serial.print(cal.accel_scale[1], 4);
  Serial.print(" Z: "); Serial.print(cal.accel_scale[2], 4);
  Serial.println();
}

// Magnetometer calibration (hard iron and soft iron)
void calibrateMagnetometer() {
  Serial.println("\n=== Magnetometer Calibration ===");
  Serial.println("Slowly rotate the sensor in all directions to");
  Serial.println("sample the complete 3D magnetic field.");
  Serial.println("Try to cover all possible orientations.");
  Serial.println("Press any key to start, then again to stop");
  
  while (!Serial.available()) {
    // Wait for user input
  }
  while (Serial.available()) Serial.read();  // Clear input buffer
  
  Serial.println("\nCalibrating - ROTATE SENSOR IN FIGURE 8 PATTERN...");
  
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
    icm.getEvent(&accel, &gyro, &temp, &mag);
    
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
  
  Serial.println("\nCollected " + String(sample_count) + " samples.");
  
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
  cal.mag_offset[0] = (min_x + max_x) / 2.0;
  cal.mag_offset[1] = (min_y + max_y) / 2.0;
  cal.mag_offset[2] = (min_z + max_z) / 2.0;
  
  // Calculate soft iron scale factors
  float avg_delta = ((max_x - min_x) + (max_y - min_y) + (max_z - min_z)) / 3.0;
  
  cal.mag_scale[0] = avg_delta / (max_x - min_x);
  cal.mag_scale[1] = avg_delta / (max_y - min_y);
  cal.mag_scale[2] = avg_delta / (max_z - min_z);
  
  // For a full soft iron calibration, we would calculate a 3x3 transformation matrix
  // This is simplified to the use of scale factors
  
  Serial.println("\nMagnetometer calibration complete!");
  Serial.println("Hard iron offsets (uT):");
  Serial.print("X: "); Serial.print(cal.mag_offset[0], 4);
  Serial.print(" Y: "); Serial.print(cal.mag_offset[1], 4);
  Serial.print(" Z: "); Serial.print(cal.mag_offset[2], 4);
  Serial.println();
  
  Serial.println("Soft iron scale factors:");
  Serial.print("X: "); Serial.print(cal.mag_scale[0], 4);
  Serial.print(" Y: "); Serial.print(cal.mag_scale[1], 4);
  Serial.print(" Z: "); Serial.print(cal.mag_scale[2], 4);
  Serial.println();
}

// Temperature calibration
void calibrateTemperature() {
  Serial.println("\n=== Temperature Compensation Calibration ===");
  Serial.println("This calibration requires changing the temperature of the sensor.");
  Serial.println("We'll collect data at room temperature now.");
  
  // Get reference temperature and readings
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  cal.temp_ref = temp.temperature;
  
  float accel_ref[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  float gyro_ref[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
  
  Serial.println("Reference temperature: " + String(cal.temp_ref) + " °C");
  Serial.println("\nFor full temperature calibration, you would need to");
  Serial.println("collect data at different temperatures and calculate coefficients.");
  Serial.println("This is just a simplified version that records the reference values.");
  
  // For a real calibration, undertake the following actions:
  // 1. Change sensor temperature
  // 2. Collect data at each temperature
  // 3. Calculate temperature coefficients (change per degree C)
  
  // Using default values
  for (int i = 0; i < 3; i++) {
    cal.temp_coef_accel[i] = 0.0;  // Default: no temperature compensation
    cal.temp_coef_gyro[i] = 0.0;   // Default: no temperature compensation
  }
  
  Serial.println("\nTemperature calibration complete.");
}

// Moving average filter
float applyMovingAverage(float new_value, float buffer[]) {
  // Add new value to buffer
  buffer[buffer_index] = new_value;
  
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
void calibrateAccelData(float raw_x, float raw_y, float raw_z, float temp, float &cal_x, float &cal_y, float &cal_z) {
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, cal.temp_coef_accel[0], temp, cal.temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, cal.temp_coef_accel[1], temp, cal.temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, cal.temp_coef_accel[2], temp, cal.temp_ref);
  
  // Apply offset and scale calibration
  float offset_cal_x = (temp_comp_x - cal.accel_offset[0]) * cal.accel_scale[0];
  float offset_cal_y = (temp_comp_y - cal.accel_offset[1]) * cal.accel_scale[1];
  float offset_cal_z = (temp_comp_z - cal.accel_offset[2]) * cal.accel_scale[2];
  
  // Apply moving average filter
  float filtered_x = applyMovingAverage(offset_cal_x, accel_x_buffer);
  float filtered_y = applyMovingAverage(offset_cal_y, accel_y_buffer);
  float filtered_z = applyMovingAverage(offset_cal_z, accel_z_buffer);
  
  // Apply Kalman filter
  cal_x = applyKalmanFilter(filtered_x, kalman_accel_x);
  cal_y = applyKalmanFilter(filtered_y, kalman_accel_y);
  cal_z = applyKalmanFilter(filtered_z, kalman_accel_z);
}

// Apply all calibrations to raw gyroscope data
void calibrateGyroData(float raw_x, float raw_y, float raw_z, float temp, float &cal_x, float &cal_y, float &cal_z) {
  // Apply temperature compensation
  float temp_comp_x = compensateForTemperature(raw_x, cal.temp_coef_gyro[0], temp, cal.temp_ref);
  float temp_comp_y = compensateForTemperature(raw_y, cal.temp_coef_gyro[1], temp, cal.temp_ref);
  float temp_comp_z = compensateForTemperature(raw_z, cal.temp_coef_gyro[2], temp, cal.temp_ref);
  
  // Apply offset and scale calibration
  float offset_cal_x = (temp_comp_x - cal.gyro_offset[0]) * cal.gyro_scale[0];
  float offset_cal_y = (temp_comp_y - cal.gyro_offset[1]) * cal.gyro_scale[1];
  float offset_cal_z = (temp_comp_z - cal.gyro_offset[2]) * cal.gyro_scale[2];
  
  // Apply moving average filter
  float filtered_x = applyMovingAverage(offset_cal_x, gyro_x_buffer);
  float filtered_y = applyMovingAverage(offset_cal_y, gyro_y_buffer);
  float filtered_z = applyMovingAverage(offset_cal_z, gyro_z_buffer);
  
  // Apply Kalman filter
  cal_x = applyKalmanFilter(filtered_x, kalman_gyro_x);
  cal_y = applyKalmanFilter(filtered_y, kalman_gyro_y);
  cal_z = applyKalmanFilter(filtered_z, kalman_gyro_z);
}

// Apply all calibrations to raw magnetometer data
void calibrateMagData(float raw_x, float raw_y, float raw_z, float &cal_x, float &cal_y, float &cal_z) {
  // Apply hard iron (offset) calibration
  float offset_cal_x = raw_x - cal.mag_offset[0];
  float offset_cal_y = raw_y - cal.mag_offset[1];
  float offset_cal_z = raw_z - cal.mag_offset[2];
  
  // Apply soft iron (scale) calibration
  float scale_cal_x = offset_cal_x * cal.mag_scale[0];
  float scale_cal_y = offset_cal_y * cal.mag_scale[1];
  float scale_cal_z = offset_cal_z * cal.mag_scale[2];
  
  // Apply moving average filter
  cal_x = applyMovingAverage(scale_cal_x, mag_x_buffer);
  cal_y = applyMovingAverage(scale_cal_y, mag_y_buffer);
  cal_z = applyMovingAverage(scale_cal_z, mag_z_buffer);
  
  // Note: Not applying Kalman to mag data as it's usually slower changing
}

// Update complementary filter
void updateComplementaryFilter(float accel_x, float accel_y, float accel_z, 
                              float gyro_x, float gyro_y, float gyro_z) {
  // Calculate dt
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;
  
  // Calculate angles from accelerometer (roll & pitch only, not yaw)
  float accel_angle_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / PI;
  float accel_angle_y = atan2(-accel_x, accel_z) * 180.0 / PI;
  
  // Complementary filter
  comp_angle_x = alpha * (comp_angle_x + gyro_x * dt) + (1.0 - alpha) * accel_angle_x;
  comp_angle_y = alpha * (comp_angle_y + gyro_y * dt) + (1.0 - alpha) * accel_angle_y;
  
  // Yaw can't be corrected with accelerometer, so just integrate gyro
  comp_angle_z += gyro_z * dt;
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

// Print calibration data
void printCalibrationData() {
  Serial.println("\n=== Current Calibration Data ===");
  
  Serial.println("Accelerometer offsets (m/s²):");
  Serial.print("X: "); Serial.print(cal.accel_offset[0], 4);
  Serial.print(" Y: "); Serial.print(cal.accel_offset[1], 4);
  Serial.print(" Z: "); Serial.print(cal.accel_offset[2], 4);
  Serial.println();
  
  Serial.println("Accelerometer scale factors:");
  Serial.print("X: "); Serial.print(cal.accel_scale[0], 4);
  Serial.print(" Y: "); Serial.print(cal.accel_scale[1], 4);
  Serial.print(" Z: "); Serial.print(cal.accel_scale[2], 4);
  Serial.println();
  
  Serial.println("Gyroscope offsets (rad/s):");
  Serial.print("X: "); Serial.print(cal.gyro_offset[0], 6);
  Serial.print(" Y: "); Serial.print(cal.gyro_offset[1], 6);
  Serial.print(" Z: "); Serial.print(cal.gyro_offset[2], 6);
  Serial.println();
  
  Serial.println("Magnetometer hard iron offsets (uT):");
  Serial.print("X: "); Serial.print(cal.mag_offset[0], 4);
  Serial.print(" Y: "); Serial.print(cal.mag_offset[1], 4);
  Serial.print(" Z: "); Serial.print(cal.mag_offset[2], 4);
  Serial.println();
  
  Serial.println("Magnetometer soft iron scale factors:");
  Serial.print("X: "); Serial.print(cal.mag_scale[0], 4);
  Serial.print(" Y: "); Serial.print(cal.mag_scale[1], 4);
  Serial.print(" Z: "); Serial.print(cal.mag_scale[2], 4);
  Serial.println();
  
  Serial.println("Temperature reference: " + String(cal.temp_ref) + " °C");
}

void loop() {
  // Get raw sensor readings
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Update buffer index for moving average filter
  buffer_index = (buffer_index + 1) % FILTER_SAMPLES;
  
// Apply calibration and filtering
  float cal_accel_x, cal_accel_y, cal_accel_z;
  float cal_gyro_x, cal_gyro_y, cal_gyro_z;
  float cal_mag_x, cal_mag_y, cal_mag_z;
  
  calibrateAccelData(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                    temp.temperature, cal_accel_x, cal_accel_y, cal_accel_z);
                    
  calibrateGyroData(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
                   temp.temperature, cal_gyro_x, cal_gyro_y, cal_gyro_z);
                   
  calibrateMagData(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, 
                  cal_mag_x, cal_mag_y, cal_mag_z);
  
  // Update complementary filter for orientation
  updateComplementaryFilter(cal_accel_x, cal_accel_y, cal_accel_z, 
                          cal_gyro_x, cal_gyro_y, cal_gyro_z);
  
  // Print data every 250ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    
    Serial.println("\n===========================");
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
    Serial.print(comp_angle_x, 1);
    Serial.print(" Pitch (Y): ");
    Serial.print(comp_angle_y, 1);
    Serial.print(" Yaw (Z): ");
    Serial.println(comp_angle_z, 1);
    
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
  
  // Delay to not overwhelm the serial output
  delay(10);
}
