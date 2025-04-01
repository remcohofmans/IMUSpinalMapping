![Lateral extension](https://github.com/user-attachments/assets/afadcc30-d6f2-4d2e-b9c2-1e757073446c)


[ESP32 + 2x ICM-20948 breakout boards](https://github.com/user-attachments/assets/38adc208-5c5b-4c4c-b61a-d0a6b7f35e00)

# IMU Sensor Management System Documentation

## System Overview

This system is designed to manage, calibrate, and process data from multiple IMUs (Inertial Measurement Unit), specifically the ICM-20948 which combines accelerometer, gyroscope, and magnetometer in a single package. The architecture enables multiple sensor units to be connected, calibrated, and monitored, while generated data is accessible through both serial output and a (preliminary) web interface (to be extended).

# Axis Definition  

**Viewing from the back of a person:**  
- **X-axis (Green):** Positive direction → Down  
- **Y-axis (Red):** Positive direction → Left  
- **Z-axis (Blue):** Positive direction → Out of the back  

### Rotational Movements  
- **Roll (X-axis rotation, Green):** Axial Rotation  
- **Pitch (Y-axis rotation, Red):** Flexion/Extension  
- **Yaw (Z-axis rotation, Blue):** Lateral Bending  


![Axis definition](https://github.com/user-attachments/assets/61c982bd-b6a4-4698-bf09-3cdd19daa8f5)


### Key Features

- Support for multiple IMU sensor units (currently configured for 2 units)
- Comprehensive calibration procedures for all sensor types
- Advanced filtering using Kalman and complementary filters
- Temperature compensation for improved accuracy
- Real-time orientation tracking using both Euler angles and quaternions
- Web interface hosted on ESP32 with data streaming via WebSocket protocol
- EEPROM storage for calibration data persistence

## System Architecture

The system is built using a modular architecture with several manager classes, each responsible for a specific aspect of the system:

```
┌─────────────────┐     ┌───────────────────┐     ┌───────────────────┐
│  SensorManager  │◄────┤ CalibrationManager│◄────┤   FilterManager   │
└────────┬────────┘     └─────────┬─────────┘     └────────┬──────────┘
         │                        │                        │
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────┐     ┌───────────────────┐     ┌───────────────────┐
│  OutputManager  │     │  StorageManager   │     │    WebServer      │
└─────────────────┘     └───────────────────┘     └───────────────────┘
```

### Components

1. **SensorManager**: Handles communication with the physical ICM-20948 sensors, initializes them, and reads raw data.
2. **CalibrationManager**: Provides procedures for calibrating the sensors and applies calibration corrections to raw data pulled in by the SensorManager.
3. **FilterManager**: Implements digital filters (moving average, Kalman) and sensor fusion algorithms (complementary filter) to improve data quality.
4. **OutputManager**: Formats and outputs the sensor data to the serial interface.
5. **StorageManager**: Saves and loads calibration data to/from EEPROM.
6. **WebServer**: Provides a web interface and WebSocket connection for real-time data visualization.

## Hardware Requirements

- ESP32 microcontroller
- 2x ICM-20948 IMU sensors (with I2C addresses 0x68 and 0x69) -> extendable to 5
- Wiring for I2C communication
- Power supply

## Key Data Structures

### Sensor Data

Each sensor has 9 DOF and provides the following measurements:
- Accelerometer (3 axes: X, Y, Z) in m/s²
- Gyroscope (3 axes: X, Y, Z) in rad/s
- Magnetometer (3 axes: X, Y, Z) in μT
- Temperature in °C -> irrelevant within the scope of this project

### Calibration Data

For each sensor unit, the system stores:
- Accelerometer offsets and scale factors
- Gyroscope offsets and scale factors
- Magnetometer hard iron offsets and soft iron transformation matrix
- Temperature reference and temperature coefficients

### Orientation Data

The system calculates and provides orientation data in two formats:

- Euler angles (roll, pitch, yaw) in degrees
- Quaternions (w, x, y, z) for more robust 3D orientation representation (deals with Gimbal lock)

## Calibration Procedures

The system includes detailed calibration procedures for each sensor type:

### Gyroscope Calibration

1. Prompts the user to keep the sensor units completely still
2. Collects 500 samples from each sensor
3. Calculates the average (bias) for each axis
4. Stores these as gyro_offset values

### Accelerometer Calibration

1. Places sensor units in 6 different orientations (all major axes facing up and down)
3. Collects 200 samples for each orientation
4. Finds min/max values for each axis
5. Calculates offsets (center of range) and scale factors normalized to gravity (9.81 m/s²)

### Magnetometer Calibration

1. Rotates each sensor unit in a figure-8 pattern to cover all orientations
2. Collects up to 500 samples
3. Calculates hard iron offsets (center of ellipsoid)
4. Calculates soft iron transformation matrix using eigendecomposition
5. Corrects for magnetic field distortions

### Temperature Calibration

1. Records reference temperature for each sensor
2. Sets up temperature coefficients for compensation

## Filtering Algorithms

### Moving Average Filter

Implemented for all sensor data with a configurable window size (currently 10 samples), to smoothen out the readings. The filter works by:

1. Adding new values to a circular buffer
2. Calculating the average of all values in the buffer
3. Advancing the buffer index with wraparound

### Kalman Filter

Applied to accelerometer and gyroscope data with separate filter states for each axis:
- State (x): Current estimated value
- Error covariance (p)
- Process noise (q)
- Measurement noise (r)
- Gain (k)

The algorithm estimates and predicts system states amid uncertainty, guided by knowledge of physical limitations in the incoming data.

### Adaptive Complementary Filter

Used for orientation estimation with DYNAMIC weighting:
- Adjusts filter coefficients based on sensor reliability
- Reduces accelerometer influence during high acceleration events
- Reduces magnetometer influence during magnetic disturbances
- Features magnetic disturbance detection and compensation

### Quaternion Implementation

The system offers quaternion-based orientation tracking for improved stability:

- Avoids gimbal lock issues that affect Euler angles
- Provides smoother transitions between orientations
- Implements SLERP (Spherical Linear Interpolation) for quaternion blending
- Transforms between quaternions and Euler angles as needed

### Regional Settings

Magnetic Declination: Set to 1.5° East for Belgium to align magnetic north with true north

## Web Interface

The system provides a web interface with:

- Real-time visualization of sensor orientation
- JSON-formatted data streaming via WebSockets
- Support for multiple connected clients
- Responsive design for desktop and mobile devices

## Usage Instructions

### Initial Setup

1. Flash the web interface files to the ESP32's LittleFS:

   Install the ESP32 LittleFS filesystem uploader plugin for Arduino IDE
   Create a "data" folder in your Arduino project directory
   Place all web interface files (HTML, CSS, JavaScript) inside the data folder
   Select Tools > ESP32 Sketch Data Upload to flash the files to LittleFS

2. Upload the main firmware to the ESP32

3. Connect the IMU sensors to the ESP32 via I2C:
   - Sensor 1 at address 0x68 (connect SDO to GND)
   - Sensor 2 at address 0x69 (DEFAULT addr: SDO connected to 3V)

3. Power on the system and check the serial output for successful initialization.

## Calibration Procedures

The system includes detailed calibration procedures for each sensor type:

### Gyroscope Calibration

1. Prompts the user to keep the sensor units completely still
2. Collects 500 samples from each sensor
3. Calculates the average (bias) for each axis
4. Stores these as gyro_offset values

#### Mathematical Basis

The gyroscope calibration addresses bias error, which causes drift in orientation estimates:

1. **Static Bias Determination**
   - When the sensor is stationary, any non-zero readings represent bias
   - This bias is calculated by averaging multiple samples:
     ```
     gyro_offset[axis] = sum_of_readings[axis] / number_of_samples
     ```

2. **Application**
   - During normal operation, the bias is subtracted from raw readings:
     ```
     calibrated_gyro[axis] = raw_gyro[axis] - gyro_offset[axis]
     ```

3. **Temperature Effects**
   - Gyroscope bias varies with temperature
   - Temperature coefficients (temp_coef_gyro) allow for dynamic bias correction:
     ```
     temp_adjusted_gyro = raw_gyro - (gyro_offset + temp_coef_gyro * (current_temp - ref_temp))
     ```

Properly calibrated gyroscopes are essential for accurate integration of angular velocity into orientation estimates, minimizing drift over time.

### Accelerometer Calibration

1. Places sensor units in 6 different orientations (all major axes facing up and down)
2. Collects 200 samples for each orientation
3. Finds min/max values for each axis
4. Calculates offsets (center of range) and scale factors normalized to gravity (9.81 m/s²)

#### Mathematical Basis

The accelerometer calibration addresses both bias (offset) and scale factor errors:

1. **Six-Position Static Test**
   - By placing the sensor in six orthogonal orientations, each axis experiences both +1g and -1g
   - This provides the needed information to calculate both offset and scale:
     ```
     accel_offset[axis] = (min_reading[axis] + max_reading[axis]) / 2
     accel_scale[axis] = 19.62 / (max_reading[axis] - min_reading[axis])
     ```
     - The value 19.62 represents 2g in m/s² (2 × 9.81), as each axis experiences both +g and -g

2. **Interpretation of Scale Factors**
   - Scale factor < 1: the axis readings are stretched and need compression
   - Scale factor > 1: the axis readings are compressed and need stretching

3. **Application**
   - During normal operation, both corrections are applied:
     ```
     calibrated_accel[axis] = (raw_accel[axis] - accel_offset[axis]) * accel_scale[axis]
     ```

4. **Temperature Compensation**
   - Similar to gyroscopes, temperature effects are addressed with coefficients:
     ```
     temp_adjusted_accel = raw_accel - (accel_offset + temp_coef_accel * (current_temp - ref_temp))
     ```

Well-calibrated accelerometers provide accurate gravity vector measurements, essential for determining pitch and roll angles in static conditions.

### Magnetometer Calibration: Ellipsoid Fitting

![Raw Magnetometer Data](https://github.com/user-attachments/assets/42d491fd-5cba-4915-9606-83719153071a)

![Calibrated Magnetometer Data](https://github.com/user-attachments/assets/13f9c078-76d4-43d6-aa13-e1ac7e4ffef2)

#### Overview
The magnetometer calibration uses an advanced ellipsoid fitting technique to correct for both hard and soft iron distortions. This ensures accurate heading measurements regardless of sensor orientation.

#### The Calibration Process

1. **Hard Iron Correction**
   - Hard iron distortions shift the center of the measurement sphere
   - These are corrected by subtracting offset values (calibrationData[unit].mag_offset)
   - Calculated as the center of the ellipsoid formed by samples

2. **Soft Iron Correction**
   - Soft iron distortions warp the perfect sphere into an ellipsoid
   - These are corrected using a transformation matrix (calibrationData[unit].soft_iron_matrix)
   - Implemented through eigendecomposition and matrix operations

#### Mathematical Implementation

The soft iron correction matrix is calculated through the following steps:

1. **Covariance Matrix Calculation**
   - Build a covariance matrix from centered magnetometer samples

2. **Eigendecomposition**
   - Decompose the covariance matrix into eigenvalues and eigenvectors
   - Eigenvalues represent the squared lengths of the ellipsoid's semi-axes
   - Eigenvectors represent the directions of these axes

3. **Scaling Matrix Creation**
   - Calculate the average radius as a target sphere size
   - Create a diagonal scaling matrix to normalize each axis

4. **Final Transformation Matrix**
   - Combine these components as V * D * V^T
   - V represents eigenvectors (orientation of distortion)
   - D represents scaling factors (magnitude of distortion)
   - V^T represents the transpose of V (return to original coordinate system)

#### Matrix Multiplication Explained

The soft iron correction matrix (V * D * V^T) applies a coordinate transformation that:
1. Transforms from sensor space to eigenvector space (V^T)
2. Applies scaling to normalize the ellipsoid into a sphere (D)
3. Transforms back to sensor space (V)

This sequence ensures that the distorted ellipsoid is transformed into a sphere while preserving the orientation of the magnetic field vector.

#### Application

When applied to raw magnetometer readings, this transformation:
1. Centers the readings (hard iron correction)
2. Transforms the ellipsoid into a sphere (soft iron correction)

The result is a properly calibrated magnetometer that provides accurate heading information regardless of sensor orientation or nearby ferromagnetic materials.

```cpp
// Application of calibration
// First apply hard iron correction
float offset_cal_x = raw_mag_x - mag_offset[0];
float offset_cal_y = raw_mag_y - mag_offset[1];
float offset_cal_z = raw_mag_z - mag_offset[2];

// Then apply soft iron correction matrix
cal_mag_x = soft_iron_matrix[0][0] * offset_cal_x + 
            soft_iron_matrix[0][1] * offset_cal_y + 
            soft_iron_matrix[0][2] * offset_cal_z;
            
cal_mag_y = soft_iron_matrix[1][0] * offset_cal_x + 
            soft_iron_matrix[1][1] * offset_cal_y + 
            soft_iron_matrix[1][2] * offset_cal_z;
            
cal_mag_z = soft_iron_matrix[2][0] * offset_cal_x + 
            soft_iron_matrix[2][1] * offset_cal_y + 
            soft_iron_matrix[2][2] * offset_cal_z;
```

### Temperature Calibration

1. Records reference temperature for each sensor
2. Sets up temperature coefficients for compensation
3. Applies temperature-based corrections to sensor readings:
   ```cpp
   float compensateForTemperature(float value, float temp_coef, float temp, float temp_ref) {
     return value - (temp - temp_ref) * temp_coef;
   }
   ```

For complete temperature calibration, data should be collected at multiple temperatures to determine accurate coefficients for each axis.
### Saving Calibration

After calibration, use the StorageManager to save the calibration data to EEPROM (virtualized):
```cpp
storageManager.saveCalibrationToEEPROM();
```

### Viewing Data

1. Serial Output:
   - Connect to the serial monitor at the appropriate baud rate
   - Data will be output at the configured rate

2. Web Interface:
   - Connect your device to the ESP32's WiFi network
   - Navigate to the ESP32's IP address in a web browser (check console log after connecting to network)
   - View real-time orientation data

## Technical Details

### Sensor Specifications

ICM-20948 IMU:
- Accelerometer ranges: ±2g, ±4g, ±8g, ±16g
- Gyroscope ranges: ±250, ±500, ±1000, ±2000 degrees per second
- Magnetometer: ±4900μT

### System Configuration

Configurable parameters:
- `NO_OF_UNITS`: Number of sensor units (default: 2)
- `FILTER_SAMPLES`: Window size for moving average filter (default: 10)
- `ALPHA`: Complementary filter coefficient (default: 0.98)
- `MAGNETIC_DECLINATION`: Local magnetic declination (default: 1.5° for Belgium)

## Error Handling

The system includes error checking for:
- Sensor initialization failures
- Invalid sensor IDs
- Inactive sensors
- EEPROM data validation

## Development and Expansion

### Adding More Sensors

To support more than 2 sensor units:
1. Modify the `NO_OF_UNITS` constant in SensorManager.h
2. Use a multiplexer, e.g. DollaTek TCA9548A

### Tuning Filters

Filter parameters can be adjusted in FilterManager.cpp:
- Kalman filter process noise (q) and measurement noise (r)
- Complementary filter coefficient (ALPHA)

### Enhanced Temperature Compensation

The current implementation includes a basic temperature compensation framework. This can be extended by:
1. Collecting data at multiple temperatures
2. Calculating more accurate temperature coefficients for each axis
3. Implementing them in the `temp_coef_accel` and `temp_coef_gyro` arrays

## Troubleshooting

### Common Issues

1. **Sensors not detected**
   - Check I2C connections
   - Verify correct I2C addresses (One of the board's address set to 0x68?)
   - Check power supply

2. **Poor calibration results**
   - Follow calibration instructions carefully
   - Keep sensor stable during gyro calibration
   - Ensure full rotation during magnetometer calibration

3. **Orientation drift**
   - Recalibrate the sensors
   - Adjust filter parameters
   - Check for magnetic interference

4. **Web interface not working**
   - verify connection to local network
   - Check that the web server started successfully (to be sure, press RST button on ESP32)
   - Ensure WebSocket connection is established (check browser console for errors)
