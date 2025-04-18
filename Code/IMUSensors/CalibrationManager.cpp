#ifndef CALIBRATION_MANAGER_CPP
#define CALIBRATION_MANAGER_CPP

#include "CalibrationManager.h"
#include "FilterManager.h"
#include <math.h>

CalibrationManager::CalibrationManager() {
  // Calibration data already initialized with defaults in the struct definition
}

void CalibrationManager::initialize(SensorManager *sensorMgr) {
  sensorManager = sensorMgr;
}

void CalibrationManager::performFullCalibration(bool calibrateAll) {

  // Configure sensors for calibration
  sensorManager->configureForCalibration();

  // Perform calibration
  calibrateGyros();
  calibrateAccelerometers();
  calibrateMagnetometers();
  calibrateTemperatures();

  // Configure sensors back for normal operation
  sensorManager->configureForNormalOperation();
}

void CalibrationManager::calibrateGyros() {

  // Ask user if they want to perform calibration
  Serial.println("\nDo you want to perform a gyro calibration on all sensors?");
  Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");

  while(true){
    while (!Serial.available()) {} // Check if data is available in the serial buffer
    // Wait for user input
  
    char response = Serial.read();  // Read the user input (FIFO, 1 byte)
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    if (response == 'Y' || response == 'y') {
      break;
    } else if(response == 'N' || response == 'n') {
      for(int unit = 0; unit < NO_OF_UNITS; unit++){
        Serial.println("Gyroscope offsets (rad/s):");
        Serial.print("X: ");
        Serial.print(calibrationData[unit].gyro_offset[0], 6);
        Serial.print(" Y: ");
        Serial.print(calibrationData[unit].gyro_offset[1], 6);
        Serial.print(" Z: ");
        Serial.print(calibrationData[unit].gyro_offset[2], 6);
        Serial.println();

        Serial.println("Gyroscope scale factors:");
        Serial.print("X: ");
        Serial.print(calibrationData[unit].gyro_scale[0], 6);
        Serial.print(" Y: ");
        Serial.print(calibrationData[unit].gyro_scale[1], 6);
        Serial.print(" Z: ");
        Serial.print(calibrationData[unit].gyro_scale[2], 6);
        Serial.println();
      }
      return;
    } else {
      Serial.print("not a valid input, ");
      Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");
    }
  } 

  Serial.println("\n=== Gyroscope Calibration ===");
  Serial.println("Keep the sensor units completely STILL.");
  Serial.println("Starting in 5 seconds...");

  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  Serial.println("Calibrating - DO NOT MOVE...");

  float gyro_x_sum[NO_OF_UNITS] = { 0 };
  float gyro_y_sum[NO_OF_UNITS] = { 0 };
  float gyro_z_sum[NO_OF_UNITS] = { 0 };
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

    calibrationData[i].gyro_scale[0] = 1.0f;
    calibrationData[i].gyro_scale[1] = 1.0f;
    calibrationData[i].gyro_scale[2] = 1.0f;

    Serial.println("Gyroscope scale factors for Unit " + String(i + 1) + ":");
    Serial.print("X: ");
    Serial.print(calibrationData[i].gyro_scale[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[i].gyro_scale[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[i].gyro_scale[2], 6);
    Serial.println();

    Serial.println("Gyroscope offsets (rad/s) for Unit " + String(i + 1) + ":");
    Serial.print("X: ");
    Serial.print(calibrationData[i].gyro_offset[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[i].gyro_offset[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[i].gyro_offset[2], 6);
    Serial.println();
  }

  Serial.println("\nAll gyroscopes have been calibrated!");
}

void CalibrationManager::calibrateAccelerometers() {

  // Ask user if they want to perform calibration
  Serial.println("\nDo you want to perform an accelerometer calibration on all sensors?");
  Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");

  while(true){
    while (!Serial.available()) {} // Check if data is available in the serial buffer
    // Wait for user input
  
    char response = Serial.read();  // Read the user input (FIFO, 1 byte)
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    if (response == 'Y' || response == 'y') {
      break;
    } else if(response == 'N' || response == 'n') {
      for(int unit = 0; unit < NO_OF_UNITS; unit++){
        Serial.println("Accelerometer offsets (m/s²):");
        Serial.print("X: ");
        Serial.print(calibrationData[unit].accel_offset[0], 4);
        Serial.print(" Y: ");
        Serial.print(calibrationData[unit].accel_offset[1], 4);
        Serial.print(" Z: ");
        Serial.print(calibrationData[unit].accel_offset[2], 4);
        Serial.println();

        Serial.println("Accelerometer scale factors:");
        Serial.print("X: ");
        Serial.print(calibrationData[unit].accel_scale[0], 4);
        Serial.print(" Y: ");
        Serial.print(calibrationData[unit].accel_scale[1], 4);
        Serial.print(" Z: ");
        Serial.print(calibrationData[unit].accel_scale[2], 4);
        Serial.println();
      }
      return;
    } else {
      Serial.print("not a valid input, ");
      Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");
    }
  } 

  Serial.println("\n=== Accelerometer Calibration ===");
  Serial.println("This requires positioning all sensor units in 6 different orientations.");
  Serial.println("You'll be prompted to place all units on each face at the same time.");

  // Create arrays to store readings for all 6 positions for each unit
  float accel_readings[NO_OF_UNITS][6][3];  // [unit][position][axis]
  const int samples_per_position = 200;

  for (int position = 0; position < 6; position++) {
    String directions[] = {
      "Z up (upside down)",
      "Z down (flat)",
      "X up (on short edge)",
      "X down (on opposite short edge)",
      "Y up (on long edge)",
      "Y down (on opposite long edge)"
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
    float accel_x_sum[NO_OF_UNITS] = { 0 };
    float accel_y_sum[NO_OF_UNITS] = { 0 };
    float accel_z_sum[NO_OF_UNITS] = { 0 };

    for (int i = 0; i < samples_per_position; i++) {
      sensorManager->readAllSensors();

      for (int j = 0; j < NO_OF_UNITS; j++) {
        if (!sensorManager->isSensorActive(j)) continue;

        float x, y, z;
        sensorManager->getRawAccel(j, x, y, z);

        accel_x_sum[j] += x;
        accel_y_sum[j] += y;
        accel_z_sum[j] += z;
      }

      if (i % 20 == 0) Serial.print(".");
      delay(10);
    }

    // Store average for each unit
    for (int unit = 0; unit < NO_OF_UNITS; unit++) {
      if (!sensorManager->isSensorActive(unit)) continue;

      accel_readings[unit][position][0] = accel_x_sum[unit] / samples_per_position;
      accel_readings[unit][position][1] = accel_y_sum[unit] / samples_per_position;
      accel_readings[unit][position][2] = accel_z_sum[unit] / samples_per_position;
    }

    Serial.println("\nRecorded position " + String(position + 1) + "/6 for all units");
  }

  // Process calibration data for each unit -> TODO: Implement fail-safe way to account for new active sensor (locks?)
  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;

    Serial.println("\n=== Processing calibration data for Unit " + String(unit + 1) + " ===");

    // For each axis, find min and max values
    float min_x = 9999, max_x = -9999;
    float min_y = 9999, max_y = -9999;
    float min_z = 9999, max_z = -9999;

    for (int pos = 0; pos < 6; pos++) {
      if (accel_readings[unit][pos][0] < min_x) min_x = accel_readings[unit][pos][0];
      if (accel_readings[unit][pos][0] > max_x) max_x = accel_readings[unit][pos][0];

      if (accel_readings[unit][pos][1] < min_y) min_y = accel_readings[unit][pos][1];
      if (accel_readings[unit][pos][1] > max_y) max_y = accel_readings[unit][pos][1];

      if (accel_readings[unit][pos][2] < min_z) min_z = accel_readings[unit][pos][2];
      if (accel_readings[unit][pos][2] > max_z) max_z = accel_readings[unit][pos][2];
    }

    // Calculate offsets (bias) for every unit - the center of min and max
    calibrationData[unit].accel_offset[0] = (min_x + max_x) / 2.0;
    calibrationData[unit].accel_offset[1] = (min_y + max_y) / 2.0;
    calibrationData[unit].accel_offset[2] = (min_z + max_z) / 2.0;

    // Calculate scale factors - normalized to 9.81 m/s² (gravity)
    calibrationData[unit].accel_scale[0] = 19.62 / (max_x - min_x);
    calibrationData[unit].accel_scale[1] = 19.62 / (max_y - min_y);
    calibrationData[unit].accel_scale[2] = 19.62 / (max_z - min_z);

    Serial.println("Accelerometer calibration complete for Unit " + String(unit + 1));
    Serial.println("Accel offsets (m/s²):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].accel_offset[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].accel_offset[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].accel_offset[2], 4);
    Serial.println();

    Serial.println("Accel scale factors:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].accel_scale[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].accel_scale[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].accel_scale[2], 4);
    Serial.println();
  }

  Serial.println("\nAll accelerometers have been calibrated!");
}

void CalibrationManager::calibrateMagnetometers() {

  Serial.println("\n=== Magnetometer Calibration ===");
  Serial.println("We'll calibrate each sensor unit one by one.");
  Serial.println("For each unit, rotate it slowly in all directions to sample the complete 3D magnetic field.");
  Serial.println("Try to cover all possible orientations in a figure-8 pattern.");

  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    bool calibrate; 

    Serial.println("Magnetometer data for unit: " + String(unit + 1));
    Serial.println("Magnetometer hard iron offsets (uT):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_offset[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_offset[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_offset[2], 4);
    Serial.println();

    Serial.println("Magnetometer soft iron scale factors:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_scale[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_scale[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_scale[2], 4);
    Serial.println();

    Serial.println("Magnetometer soft iron correction matrix:");
    for (int i = 0; i < 3; i++) {
      Serial.print("[ ");
      for (int j = 0; j < 3; j++) {
        Serial.print(calibrationData[unit].soft_iron_matrix[i][j], 2);
        Serial.print(" ");
      }
      Serial.println("]");
    }

    // Ask user if they want to perform calibration
    Serial.println("\nDo you want to perform a magnetometer calibration of sensor unit: " + String(unit + 1) + " ?");
    Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");

    while(true){
      while (!Serial.available()) {} // Check if data is available in the serial buffer
      // Wait for user input
    
      char response = Serial.read();  // Read the user input (FIFO, 1 byte)
      while (Serial.available()) Serial.read();  // Clear input buffer
      
      if (response == 'Y' || response == 'y') {
        calibrate = true;
        break;
      } else if(response == 'N' || response == 'n') {
        calibrate = false;
        break;
      } else {
        Serial.print("not a valid input, ");
        Serial.println("Type 'Y' for Yes or 'N' for No and press Enter");
      }
    } 

    // if the sensor is inactive or we don't want to calibrate it skip the calibration
    if (!sensorManager->isSensorActive(unit) || !calibrate) continue;

    Serial.println("\n=== Calibrating Unit " + String(unit + 1) + " Magnetometer ===");
    Serial.println("Hold Unit " + String(unit + 1) + " in your hand.");
    Serial.println("Press any key to start, then again to stop when done.");

    while (!Serial.available()) {
      // Wait for user input
    }
    while (Serial.available()) Serial.read();  // Clear input buffer

    Serial.println("\nCalibrating Unit " + String(unit + 1) + " - ROTATE SENSOR IN FIGURE 8 PATTERN...");

    // Collect samples for ellipsoid fitting
    // If these vars would be put on stack, they would total up to 12K bytes, plus whatever else already on the stack frame
    // The ESP32 task stack is with its 8K bytes limited in size (less than what would be required)
    const int max_samples = 2500;

    float *mag_x = new float[max_samples];
    float *mag_y = new float[max_samples];
    float *mag_z = new float[max_samples];
    int sample_count = 0;
    bool done = false;

    Serial.println("X,Y,Z");

    while (!done && sample_count < max_samples) {
      if (Serial.available()) {
        done = true;
        while (Serial.available()) Serial.read();  // Clear input buffer
        break;
      }

      sensorManager->readAllSensors();
      float x, y, z;
      sensorManager->getRawMag(unit, x, y, z);

      // Store sample if it's different from previous ones to avoid duplicates
      if (sample_count == 0 || (x != mag_x[sample_count - 1] || y != mag_y[sample_count - 1] || z != mag_z[sample_count - 1])) {

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
      delay(10);
    }
    Serial.println("\nCollected " + String(sample_count) + " samples for Unit " + String(unit + 1));

    // Find min and max for each axis
    float min_x = mag_x[0], max_x = mag_x[0];
    float min_y = mag_y[0], max_y = mag_y[0];
    float min_z = mag_z[0], max_z = mag_z[0];

    // Calculate mean for each axis
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < sample_count; i++) {
      // Update min/max
      if (mag_x[i] < min_x) min_x = mag_x[i];
      if (mag_x[i] > max_x) max_x = mag_x[i];

      if (mag_y[i] < min_y) min_y = mag_y[i];
      if (mag_y[i] > max_y) max_y = mag_y[i];

      if (mag_z[i] < min_z) min_z = mag_z[i];
      if (mag_z[i] > max_z) max_z = mag_z[i];

      // Update sums
      sum_x += mag_x[i];
      sum_y += mag_y[i];
      sum_z += mag_z[i];
    }

    float mean_x = sum_x / sample_count;
    float mean_y = sum_y / sample_count;
    float mean_z = sum_z / sample_count;

    // Use both methods and compare
    float center_x_minmax = (min_x + max_x) / 2.0f;
    float center_y_minmax = (min_y + max_y) / 2.0f;
    float center_z_minmax = (min_z + max_z) / 2.0f;

    // Calculate distances
    float mean_dist = sqrt(pow(center_x_minmax - mean_x, 2) + pow(center_y_minmax - mean_y, 2) + pow(center_z_minmax - mean_z, 2));

    Serial.print("Min/Max vs Mean difference: ");
    Serial.println(mean_dist, 4);

    // Choose offset method based on quality of data
    if (mean_dist < 0.5f) {
      // If methods are close, use minmax for better centering
      calibrationData[unit].mag_offset[0] = center_x_minmax;
      calibrationData[unit].mag_offset[1] = center_y_minmax;
      calibrationData[unit].mag_offset[2] = center_z_minmax;
    } else {
      // Otherwise use mean (likely better with noisy data)
      calibrationData[unit].mag_offset[0] = mean_x;
      calibrationData[unit].mag_offset[1] = mean_y;
      calibrationData[unit].mag_offset[2] = mean_z;
    }

    // Calculate hard iron offsets (center of ellipsoid)
    // calibrationData[unit].mag_offset[0] = (min_x + max_x) / 2.0f;
    // calibrationData[unit].mag_offset[1] = (min_y + max_y) / 2.0f;
    // calibrationData[unit].mag_offset[2] = (min_z + max_z) / 2.0f;
    calibrationData[unit].mag_offset[0] = mean_x;
    calibrationData[unit].mag_offset[1] = mean_y;
    calibrationData[unit].mag_offset[2] = mean_z;

    // Create centered data for covariance calculation
    float *centered_x = new float[sample_count];
    float *centered_y = new float[sample_count];
    float *centered_z = new float[sample_count];

    for (int i = 0; i < sample_count; i++) {
      centered_x[i] = mag_x[i] - calibrationData[unit].mag_offset[0];
      centered_y[i] = mag_y[i] - calibrationData[unit].mag_offset[1];
      centered_z[i] = mag_z[i] - calibrationData[unit].mag_offset[2];
    }

    // Calculate the covariance matrix
    float cov[3][3] = { { 0 } };

    for (int i = 0; i < sample_count; i++) {
      cov[0][0] += centered_x[i] * centered_x[i];
      cov[0][1] += centered_x[i] * centered_y[i];
      cov[0][2] += centered_x[i] * centered_z[i];
      cov[1][1] += centered_y[i] * centered_y[i];
      cov[1][2] += centered_y[i] * centered_z[i];
      cov[2][2] += centered_z[i] * centered_z[i];
    }

    delete[] mag_x;
    delete[] mag_y;
    delete[] mag_z;
    delete[] centered_x;
    delete[] centered_y;
    delete[] centered_z;

    // Matrix is symmetric
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];

    // Normalize by sample count
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cov[i][j] /= sample_count;
      }
    }

    // Perform eigenvalue decomposition
    float eigenvalues[3];
    float eigenvectors[3][3];
    eigenDecomposition(cov, eigenvalues, eigenvectors);

    // Ensure positive eigenvalues (numerical stability)
    const float MIN_EIGENVALUE = 1e-6;
    for (int i = 0; i < 3; i++) {
      if (eigenvalues[i] < MIN_EIGENVALUE) eigenvalues[i] = MIN_EIGENVALUE;
    }

    // Calculate average radius for normalization
    float avg_radius = sqrt((eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) / 3.0);

    // Create diagonal scaling matrix D (each element scales one axis of the ellipsoid)
    float D[3][3] = { { 0 } };
    D[0][0] = avg_radius / sqrt(eigenvalues[0]);
    D[1][1] = avg_radius / sqrt(eigenvalues[1]);
    D[2][2] = avg_radius / sqrt(eigenvalues[2]);

    // Calculate V * D (eigenvectors * scaling)
    float VD[3][3] = { { 0 } };
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          VD[i][j] += eigenvectors[i][k] * D[k][j];
        }
      }
    }

    // Calculate V * D * V^T (final soft iron correction matrix)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        calibrationData[unit].soft_iron_matrix[i][j] = 0;
        for (int k = 0; k < 3; k++) {
          calibrationData[unit].soft_iron_matrix[i][j] += VD[i][k] * eigenvectors[j][k];  // V^T[k][j] = V[j][k]
        }
      }
    }

    // Store the diagonal values in the mag_scale array for compatibility with older code following a more simplified approach
    calibrationData[unit].mag_scale[0] = calibrationData[unit].soft_iron_matrix[0][0];
    calibrationData[unit].mag_scale[1] = calibrationData[unit].soft_iron_matrix[1][1];
    calibrationData[unit].mag_scale[2] = calibrationData[unit].soft_iron_matrix[2][2];

    Serial.println("\nMagnetometer calibration complete for Unit " + String(unit + 1));
    Serial.println("Hard iron offsets (uT):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_offset[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_offset[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_offset[2], 4);
    Serial.println();

    Serial.println("Eigenvalues:");
    Serial.print("λ1: ");
    Serial.print(eigenvalues[0], 4);
    Serial.print(" λ2: ");
    Serial.print(eigenvalues[1], 4);
    Serial.print(" λ3: ");
    Serial.print(eigenvalues[2], 4);
    Serial.println();

    Serial.println("Soft iron correction matrix:");
    for (int i = 0; i < 3; i++) {
      Serial.print("[ ");
      for (int j = 0; j < 3; j++) {
        Serial.print(calibrationData[unit].soft_iron_matrix[i][j], 4);
        Serial.print(" ");
      }
      Serial.println("]");
    }

    Serial.println("Diagonal scale factors (for compatibility):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_scale[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_scale[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_scale[2], 4);
    Serial.println();
  }

  Serial.println("\nAll magnetometers have been calibrated!");
  Serial.println("Note: Magnetic declination is not accounted for. For accurate heading, we later apply the local magnetic declination correction.");
}

void CalibrationManager::eigenDecomposition(float cov[3][3], float eigenvalues[3], float eigenvectors[3][3]) {
  // Jacobi eigenvalue algorithm for 3x3 matrices
  const int MAX_ITER = 50;
  const float EPSILON = 1e-10;

  // Initialize eigenvectors to identity matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eigenvectors[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }

  // Copy covariance matrix for computation
  float a[3][3];
  memcpy(a, cov, sizeof(a));

  // Jacobi iteration
  for (int iter = 0; iter < MAX_ITER; iter++) {
    // Find the largest off-diagonal element
    float max_val = 0.0f;
    int p = 0, q = 1;

    for (int i = 0; i < 3; i++) {
      for (int j = i + 1; j < 3; j++) {
        if (fabs(a[i][j]) > max_val) {
          max_val = fabs(a[i][j]);
          p = i;
          q = j;
        }
      }
    }

    // Check for convergence
    if (max_val < EPSILON) {
      break;
    }

    // Compute rotation parameters
    float theta = 0.5f * atan2(2.0f * a[p][q], a[p][p] - a[q][q]);
    float c = cos(theta);
    float s = sin(theta);

    // Apply rotation to entire matrix
    for (int i = 0; i < 3; i++) {
      if (i != p && i != q) {
        // Save original values before modification
        float a_ip = a[i][p];
        float a_iq = a[i][q];

        // Update row elements
        a[i][p] = a_ip * c + a_iq * s;
        a[i][q] = -a_ip * s + a_iq * c;

        // Update corresponding column elements (symmetric matrix)
        a[p][i] = a[i][p];
        a[q][i] = a[i][q];
      }
    }

    // Update the diagonal and target off-diagonal elements
    float a_pp = a[p][p];
    float a_qq = a[q][q];
    float a_pq = a[p][q];

    a[p][p] = a_pp * c * c + a_qq * s * s + 2.0f * a_pq * c * s;
    a[q][q] = a_pp * s * s + a_qq * c * c - 2.0f * a_pq * c * s;
    a[p][q] = 0.0f;  // Explicitly zero out
    a[q][p] = 0.0f;  // Explicitly zero out

    // Update eigenvectors
    for (int i = 0; i < 3; i++) {
      float v_ip = eigenvectors[i][p];
      float v_iq = eigenvectors[i][q];
      eigenvectors[i][p] = v_ip * c + v_iq * s;
      eigenvectors[i][q] = -v_ip * s + v_iq * c;
    }
  }

  // Extract eigenvalues (diagonal elements)
  eigenvalues[0] = a[0][0];
  eigenvalues[1] = a[1][1];
  eigenvalues[2] = a[2][2];
}

void CalibrationManager::calibrateTemperatures() {
  Serial.println("\n=== Temperature Compensation Calibration ===");
  Serial.println("This calibration collects reference temperature data for each sensor unit.");
  Serial.println("We'll collect data at room temperature now.");

  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;

    // Get reference temperature
    float temp = sensorManager->getTemperature(unit);

    calibrationData[unit].temp_ref = temp;

    Serial.println("Unit " + String(unit + 1) + " reference temperature: " + String(calibrationData[unit].temp_ref) + " °C");

    // Using default values for temperature coefficients
    for (int i = 0; i < 3; i++) {
      calibrationData[unit].temp_coef_accel[i] = 0.0;  // Set to 0: no temperature compensation
      calibrationData[unit].temp_coef_gyro[i] = 0.0;
    }

    Serial.println("Temperature calibration completed for Unit " + String(unit + 1));
  }

  Serial.println("\nFor full temperature calibration, you would need to collect data at different temperatures and calculate coefficients.");
  Serial.println("This is just a simplified version that records the reference values.");

  Serial.println("\nAll temperature sensors have been calibrated.");
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

  // Apply offset and scale calibration
  cal_x = (raw_x - calibrationData[sensorId].accel_offset[0]) * calibrationData[sensorId].accel_scale[0];
  cal_y = (raw_y - calibrationData[sensorId].accel_offset[1]) * calibrationData[sensorId].accel_scale[1];
  cal_z = (raw_z - calibrationData[sensorId].accel_offset[2]) * calibrationData[sensorId].accel_scale[2];

  //cal.calibrate(accel);
}

void CalibrationManager::calibrateGyroData(int sensorId, float raw_x, float raw_y, float raw_z,
                                           float temp, float &cal_x, float &cal_y, float &cal_z) {

  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    cal_x = raw_x;
    cal_y = raw_y;
    cal_z = raw_z;
    return;
  }

  // Calculate with offsets
  float offset_x = raw_x - calibrationData[sensorId].gyro_offset[0];
  float offset_y = raw_y - calibrationData[sensorId].gyro_offset[1];
  float offset_z = raw_z - calibrationData[sensorId].gyro_offset[2];

  // Apply scale
  cal_x = offset_x * calibrationData[sensorId].gyro_scale[0];
  cal_y = offset_y * calibrationData[sensorId].gyro_scale[1];
  cal_z = offset_z * calibrationData[sensorId].gyro_scale[2];

  //cal.calibrate(gyro);
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

  // Apply soft iron correction matrix
  cal_x = calibrationData[sensorId].soft_iron_matrix[0][0] * offset_cal_x + calibrationData[sensorId].soft_iron_matrix[0][1] * offset_cal_y + calibrationData[sensorId].soft_iron_matrix[0][2] * offset_cal_z;

  cal_y = calibrationData[sensorId].soft_iron_matrix[1][0] * offset_cal_x + calibrationData[sensorId].soft_iron_matrix[1][1] * offset_cal_y + calibrationData[sensorId].soft_iron_matrix[1][2] * offset_cal_z;

  cal_z = calibrationData[sensorId].soft_iron_matrix[2][0] * offset_cal_x + calibrationData[sensorId].soft_iron_matrix[2][1] * offset_cal_y + calibrationData[sensorId].soft_iron_matrix[2][2] * offset_cal_z;

  // Add safety check at the end
  if (isnan(cal_x) || isnan(cal_y) || isnan(cal_z)) {
    cal_x = raw_x;
    cal_y = raw_y;
    cal_z = raw_z;
  }

  //cal.calibrate(mag);
}

void CalibrationManager::printCalibrationData() {
  Serial.println("\n=== Current Calibration Data for All Units ===");

  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) continue;

    Serial.println("\n--- Unit " + String(unit + 1) + " ---");

    Serial.println("Accelerometer offsets (m/s²):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].accel_offset[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].accel_offset[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].accel_offset[2], 4);
    Serial.println();

    Serial.println("Accelerometer scale factors:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].accel_scale[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].accel_scale[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].accel_scale[2], 4);
    Serial.println();

    Serial.println("Gyroscope offsets (rad/s):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].gyro_offset[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].gyro_offset[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].gyro_offset[2], 6);
    Serial.println();

    Serial.println("Gyroscope scale factors:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].gyro_scale[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].gyro_scale[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].gyro_scale[2], 6);
    Serial.println();

    Serial.println("Magnetometer hard iron offsets (uT):");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_offset[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_offset[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_offset[2], 4);
    Serial.println();

    Serial.println("Magnetometer soft iron scale factors:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].mag_scale[0], 4);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].mag_scale[1], 4);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].mag_scale[2], 4);
    Serial.println();

    Serial.println("Magnetometer soft iron correction matrix:");
    for (int i = 0; i < 3; i++) {
      Serial.print("[ ");
      for (int j = 0; j < 3; j++) {
        Serial.print(calibrationData[unit].soft_iron_matrix[i][j], 2);
        Serial.print(" ");
      }
      Serial.println("]");
    }

    Serial.println();

    Serial.println("Temperature reference: " + String(calibrationData[unit].temp_ref) + " °C");

    // Optional: print temperature coefficients
    Serial.println("\nTemperature Coefficients:");
    Serial.println("Accelerometer:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].temp_coef_accel[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].temp_coef_accel[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].temp_coef_accel[2], 6);
    Serial.println();

    Serial.println("Gyroscope:");
    Serial.print("X: ");
    Serial.print(calibrationData[unit].temp_coef_gyro[0], 6);
    Serial.print(" Y: ");
    Serial.print(calibrationData[unit].temp_coef_gyro[1], 6);
    Serial.print(" Z: ");
    Serial.print(calibrationData[unit].temp_coef_gyro[2], 6);
    Serial.println();
  }

  for (int unit = 0; unit < NO_OF_UNITS; unit++) {
    if (!sensorManager->isSensorActive(unit)) {
      // Still print placeholders so the plotter keeps columns aligned, or skip entirely
      // up to you. Here we’ll skip sensors that aren’t active:
      continue;
    }

    // Accelerometer offsets
    Serial.print(calibrationData[unit].accel_offset[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].accel_offset[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].accel_offset[2]);
    Serial.print(",");

    // Accelerometer scale factors
    Serial.print(calibrationData[unit].accel_scale[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].accel_scale[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].accel_scale[2]);
    Serial.print(",");

    // Gyroscope offsets
    Serial.print(calibrationData[unit].gyro_offset[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].gyro_offset[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].gyro_offset[2]);
    Serial.print(",");

    // Gyroscope scale factors
    Serial.print(calibrationData[unit].gyro_scale[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].gyro_scale[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].gyro_scale[2]);
    Serial.print(",");

    // Magnetometer hard iron offsets
    Serial.print(calibrationData[unit].mag_offset[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].mag_offset[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].mag_offset[2]);
    Serial.print(",");

    // Magnetometer soft iron scale factors
    Serial.print(calibrationData[unit].mag_scale[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].mag_scale[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].mag_scale[2]);
    Serial.print(",");

    // Temperature reference
    Serial.print(calibrationData[unit].temp_ref);
    Serial.print(",");

    // Temperature Coefficients - accelerometer
    Serial.print(calibrationData[unit].temp_coef_accel[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].temp_coef_accel[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].temp_coef_accel[2]);
    Serial.print(",");

    // Temperature Coefficients - gyroscope
    Serial.print(calibrationData[unit].temp_coef_gyro[0]);
    Serial.print(",");
    Serial.print(calibrationData[unit].temp_coef_gyro[1]);
    Serial.print(",");
    Serial.print(calibrationData[unit].temp_coef_gyro[2]);

    // End the line for this unit
    Serial.println();
  }
}

CalibrationData *CalibrationManager::getCalibrationData(int sensorId) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) return nullptr;

  return &calibrationData[sensorId];
}

void CalibrationManager::setGyroOffset(int sensorId, float offsetX, float offsetY, float offsetZ) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) {
    Serial.println(F("Invalid sensorId in setGyroOffset()"));
    return;
  }

  calibrationData[sensorId].gyro_offset[0] = offsetX;
  calibrationData[sensorId].gyro_offset[1] = offsetY;
  calibrationData[sensorId].gyro_offset[2] = offsetZ;
}

void CalibrationManager::setCalibrationData(int sensorId, const CalibrationData &data) {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorManager->isSensorActive(sensorId)) return;
  calibrationData[sensorId] = data;
}

size_t CalibrationManager::getCalibrationDataSize() const {
  return sizeof(CalibrationData) * NO_OF_UNITS;
}

CalibrationData *CalibrationManager::getAllCalibrationData() {
  return calibrationData;
}

#endif  // CALIBRATION_MANAGER_C