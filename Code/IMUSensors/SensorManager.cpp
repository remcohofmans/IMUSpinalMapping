/*
 * SensorManager.cpp
 * Implementation of the SensorManager class
 */

#include "SensorManager.h"

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM* calibrations[NO_OF_UNITS];
#else
  Adafruit_Sensor_Calibration_SDFat* calibrations[NO_OF_UNITS];
#endif

const SensorConfig SensorManager::sensorConfigs[NO_OF_UNITS] = {
  {0, 0x68},
  {0, 0x69},
  {1, 0x68},
  {1, 0x69}
  // {3, 0x69}
};

SensorManager::SensorManager() : activeCount(0) {
  // Initialize sensorActive array
  for (int i = 0; i < NO_OF_UNITS; i++) {
    sensorActive[i] = false;
  }
}

void SensorManager::tcaSelect(uint8_t i) {
  if (i > 7) return;  // TCA9548A has 8 channels (0-7)
  //Serial.print("tca selected port: ");
  //Serial.println(i);
  Wire.beginTransmission(TCAADDR);  
  delay(10);
  Wire.write(1 << i);  // Select the desired channel by sending a byte with the bit corresponding to that channel set to 1
  Wire.endTransmission();
}

bool SensorManager::initialize() {

  for (int i = 0; i < NO_OF_UNITS && i < (sizeof(sensorConfigs) / sizeof(sensorConfigs[0])); i++) {
    Serial.print("Trying sensor ");
    Serial.print(i + 1); 
    Serial.print(" at channel ");
    Serial.print(sensorConfigs[i].channel);
    Serial.print(", address 0x");
    Serial.println(sensorConfigs[i].address, HEX);

    tcaSelect(sensorConfigs[i].channel);
    delay(10);

    if (icm[i].begin_I2C(sensorConfigs[i].address)) {
      sensorActive[i] = true;
      activeCount++;
      Serial.println(" -> SUCCESS");
    } else {
      sensorActive[i] = false;  // Re-assure sensor is deemed inactive
      Serial.println(" -> FAILED");
    }

    accelerometers[i] = icm[i].getAccelerometerSensor();
    gyroscopes[i] = icm[i].getGyroSensor();
    magnetometers[i] = icm[i].getMagnetometerSensor();
  }

  if (activeCount == 0) {
    Serial.println("CRITICAL ERROR: No sensors found!");
    return false;
  }

  return true;
}

void SensorManager::readAllSensors() {
  for (int i = 0; i < NO_OF_UNITS; i++) {
    tcaSelect(sensorConfigs[i].channel);
    if (!sensorActive[i]) continue;

    sensors_event_t accel, gyro, mag, temp;
    icm[i].getEvent(&accel, &gyro, &temp, &mag);
    
    // Store raw values
    sensorData[i].accel[0] = accel.acceleration.x;
    sensorData[i].accel[1] = accel.acceleration.y;
    sensorData[i].accel[2] = accel.acceleration.z;
    
    sensorData[i].gyro[0] = gyro.gyro.x;
    sensorData[i].gyro[1] = gyro.gyro.y;
    sensorData[i].gyro[2] = gyro.gyro.z;
    
    sensorData[i].mag[0] = mag.magnetic.x;
    sensorData[i].mag[1] = -mag.magnetic.y;
    sensorData[i].mag[2] = -mag.magnetic.z;
    
    sensorData[i].temp = temp.temperature;

    // Store timestamp of this reading
    sensorData[i].timestamp = millis();
  }
}

bool SensorManager::isSensorActive(int sensorId) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS) {
    return false;
  }
  return sensorActive[sensorId];
}

int SensorManager::getActiveCount() const {
  return activeCount;
}

void SensorManager::getRawAccel(int sensorId, float &x, float &y, float &z) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorActive[sensorId]) {
    x = y = z = 0;
    return;
  }

  x = sensorData[sensorId].accel[0];
  y = sensorData[sensorId].accel[1];
  z = sensorData[sensorId].accel[2];
}

void SensorManager::getRawGyro(int sensorId, float &x, float &y, float &z) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorActive[sensorId]) {
    x = y = z = 0;
    return;
  }
  
  x = sensorData[sensorId].gyro[0];
  y = sensorData[sensorId].gyro[1];
  z = sensorData[sensorId].gyro[2];
}

void SensorManager::getRawMag(int sensorId, float &x, float &y, float &z) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorActive[sensorId]) {
    x = y = z = 0;
    return;
  }
  
  x = sensorData[sensorId].mag[0];
  y = sensorData[sensorId].mag[1];
  z = sensorData[sensorId].mag[2];
}

float SensorManager::getTemperature(int sensorId) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorActive[sensorId]) {
    return 0;
  }
  
  return sensorData[sensorId].temp;
}

unsigned long SensorManager::getReadingTimestamp(int sensorId) const {
  if (sensorId < 0 || sensorId >= NO_OF_UNITS || !sensorActive[sensorId]) {
    return 0;
  }
  
  return sensorData[sensorId].timestamp;
}

void SensorManager::configureForCalibration() {
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensorActive[i]) continue;
    
    icm[i].setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm[i].setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
    icm[i].setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  }
  
  Serial.println("Sensors configured for calibration");
}

void SensorManager::configureForNormalOperation() {
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (!sensorActive[i]) continue;
    
    icm[i].setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm[i].setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);
    icm[i].setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  }
  
  Serial.println("Sensors configured for normal operation");
}
