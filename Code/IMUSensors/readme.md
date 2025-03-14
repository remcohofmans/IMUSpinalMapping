![Lateral extension](https://github.com/user-attachments/assets/afadcc30-d6f2-4d2e-b9c2-1e757073446c)


[ESP32 + 2x ICM-20948 breakout boards](https://github.com/user-attachments/assets/38adc208-5c5b-4c4c-b61a-d0a6b7f35e00)

# IMU Sensor Management System Documentation

## System Overview

This system is designed to manage, calibrate, and process data from multiple IMUs (Inertial Measurement Unit), specifically the ICM-20948 which combines accelerometer, gyroscope, and magnetometer in a single package. The architecture enables multiple sensor units to be connected, calibrated, and monitored, while generated data is accessible through both serial output and a (preliminary) web interface (to be extended).

# axis definition

Viewing from the back of a person:
- x is positive down
- y is positive to the left
- z is positive going out of the back

![axis definition](https://github.com/user-attachments/assets/61c982bd-b6a4-4698-bf09-3cdd19daa8f5)


### Key Features

- Support for <em> IMU sensor units</em> (currently configured for 2 units)
- Comprehensive calibration procedures for all sensor types
- Advanced filtering using Kalman and complementary filters
- Temperature compensation for improved accuracy
- Real-time orientation tracking (roll, pitch, yaw), currently limited to Euler angles
- Web interface hosted on ESP32 with data streaming via WebSocket protocol
- EEPROM storage for calibration data persistence and enhanced user experience

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
- Magnetometer hard iron offsets and soft iron scale factors
- Temperature reference and temperature coefficients

### Orientation Data

The system calculates and provides orientation data:
- Roll (rotation around X-axis) in degrees
- Pitch (rotation around Y-axis) in degrees
- Yaw (rotation around Z-axis) in degrees

## Calibration Procedures

The system includes detailed calibration procedures for each sensor type:

### Gyroscope Calibration

1. Prompts the user to keep the sensor units completely still
2. Collects 500 samples from each sensor
3. Calculates the average (bias) for each axis
4. Stores these as gyro_offset values

### Accelerometer Calibration

1. Places sensor units in 6 different orientations (all major axes facing up and down)
2. Prompts the user once again to keep still
3. Collects 100 samples for each orientation
4. Finds min/max values for each axis
5. Calculates offsets (center of range) and scale factors

### Magnetometer Calibration

1. Rotates each sensor unit in a figure-8 pattern to cover all orientations
2. Collects up to 500 samples
3. Calculates hard iron offsets (center of range) -> shift the center of the magnetometer's measurement sphere -> requires simple additive corrections
4. Calculates soft iron scale factors 
   -> create an ellipsoidal rather than (ideal) spherical measurement space -> stretch or compress the measurements along each axis to restore the spherical shape of the measurement space

### Temperature Calibration

1. Records reference temperature for each sensor
2. Sets up temperature coefficients for compensation

## Filtering Algorithms

### Moving Average Filter

Implemented for all sensor data with a configurable window size (currently 10 samples), to smoothen out the readings.

### Kalman Filter

Applied to accelerometer and gyroscope data with separate filter states for each axis:
- State (x): Current estimated value
- Error covariance (p)
- Process noise (q)
- Measurement noise (r)
- Gain (k)

### Complementary Filter

Used for orientation estimation, combining:
- Accelerometer data (for roll and pitch)
- Magnetometer data (for yaw)
- Gyroscope data (for rate of change)

The filter uses a configurable alpha value (ALPHA = 0.98) to determine how much weight to give to gyroscope vs. accelerometer/magnetometer data. As the accelero picks up and hence is contaminated by a significant amount of noise, its weight is limited.

## Web Interface

The system sets up an ESP32 as an access point (AP) with SSID "ESP32-IMU-Sensor" and provides:
- Web server on port 80
- WebSocket server for real-time data streaming
- JSON-formatted orientation data for all active sensors

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

### Calibration

Run the `performFullCalibration()` method to calibrate all sensors:
1. Follow the serial prompts for each calibration step
2. Keep the sensors still during gyro calibration
3. Position the sensors as instructed during accelerometer calibration
4. Rotate the sensors as instructed during magnetometer calibration

### Saving Calibration

After calibration, use the StorageManager to save the calibration data to EEPROM (virtualized):
```cpp
storageManager.saveCalibrationToEEPROM();
```

### Viewing Data

1. Serial Output:
   - Connect to the serial monitor at the appropriate baud rate
   - Data will be output at the configured rate (default 5s)

2. Web Interface:
   - Connect your device to the "ESP32-IMU-Sensor" WiFi access point using the password 'password123'
   - Navigate to 192.168.4.1 in a web browser
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

Option: Restrict movement in 3D space.

### Enhanced Temperature Compensation

The current implementation includes a basic temperature compensation framework. This can be extended by:
1. Collecting data at multiple temperatures
2. Calculating more accurate temperature coefficients
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
   - Verify connection to the ESP32 access point
   - Check that the web server started successfully (to be sure, press RST button on ESP32)
   - Ensure WebSocket connection is established (check browser console for errors)
