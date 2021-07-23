#include "RTIMUFXOS8700FXAS21002C.h"
#include "RTIMUSettings.h"

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

RTIMUFXOS8700FXAS21002C::RTIMUFXOS8700FXAS21002C(RTIMUSettings *settings) : RTIMU(settings)
{
  memset(&accel_raw, 0, sizeof(fxos8700RawData_t));
  memset(&mag_raw, 0, sizeof(fxos8700RawData_t));
  memset(&gyro_raw, 0, sizeof(gyroRawData_t));

}

RTIMUFXOS8700FXAS21002C::~RTIMUFXOS8700FXAS21002C()
{


  m_sampleRate = 100;


}

bool RTIMUFXOS8700FXAS21002C::write8(int slave, int reg, unsigned char val)
{
  return m_settings->HALWrite(slave, reg, 1, &val, "Failed to Write");
}

unsigned char RTIMUFXOS8700FXAS21002C::read8(int slave, int reg)
{
  uint8_t data = 0;
  if (m_settings->HALRead(slave, reg, 1, &data, "Failed to read"))
  {
    return false;
  }

  return true;
}

bool RTIMUFXOS8700FXAS21002C::IMUInit()
{
  uint8_t fxos8700_id = 0;
  uint8_t fxas21002c_id = 0;

  m_imuData.fusionPoseValid = false;
  m_imuData.fusionQPoseValid = false;
  m_imuData.gyroValid = true;
  m_imuData.accelValid = true;
  m_imuData.compassValid = true;
  m_imuData.pressureValid = false;
  m_imuData.temperatureValid = false;
  m_imuData.humidityValid = false;

  //  configure IMU

  m_fxos8700cAddr = FXOS8700_ADDRESS;
  m_fxas21002cAddr = FXAS21002C_ADDRESS;

  _range = ACCEL_RANGE_4G;
  _gyrorange = GYRO_RANGE_2000DPS;

  setCalibrationData();

  //  enable the I2C bus

  if (!m_settings->HALOpen())
    return false;

  if (!m_settings->HALRead(m_fxos8700cAddr, FXOS8700_REGISTER_WHO_AM_I, 1, &fxos8700_id, "Failed to read FXOS8700 id"))
  {
    return false;
  }

  if (fxos8700_id != FXOS8700_ID)
  {
    HAL_ERROR1("Incorrect FX8700 IMU id %d", fxos8700_id);
    return false;
  }

  if (!m_settings->HALRead(m_fxas21002cAddr, FXAS21002C_REGISTER_WHO_AM_I, 1, &fxas21002c_id, "Failed to read FXOS8700 id"))
  {
    return false;
  }

  if (fxas21002c_id != FXAS21002C_ID)
  {
    HAL_ERROR1("Incorrect FXAS21002C IMU id %d", fxos8700_id);
    return false;
  }

  /* Set to standby mode (required to make changes to this register) */
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_CTRL_REG1, 0);

  /* Configure the accelerometer */
  switch (_range) {
  case (ACCEL_RANGE_2G):
    write8(m_fxos8700cAddr, FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
    break;
  case (ACCEL_RANGE_4G):
    write8(m_fxos8700cAddr, FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01);
    break;
  case (ACCEL_RANGE_8G):
    write8(m_fxos8700cAddr, FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02);
    break;
  }
  /* High resolution */
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_CTRL_REG2, 0x02);
  /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_CTRL_REG1, 0x15);

  /* Configure the magnetometer */
  /* Hybrid Mode, Over Sampling Rate = 16 */
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
  /* Jump to reg 0x33 after reading 0x06 */
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_MCTRL_REG2, 0x20);
  write8(m_fxos8700cAddr, FXOS8700_REGISTER_CTRL_REG1, 1);

  uint8_t ctrlReg0 = 0x03;

  switch (_gyrorange) {
  case GYRO_RANGE_250DPS:
    ctrlReg0 = 0x03;
    break;
  case GYRO_RANGE_500DPS:
    ctrlReg0 = 0x02;
    break;
  case GYRO_RANGE_1000DPS:
    ctrlReg0 = 0x01;
    break;
  case GYRO_RANGE_2000DPS:
    ctrlReg0 = 0x00;
    break;
  }

  /* Reset then switch to active mode with 100Hz output */
  write8(m_fxas21002cAddr, FXAS21002C_REGISTER_CTRL_REG1, 0x00);     // Standby
  write8(m_fxas21002cAddr, FXAS21002C_REGISTER_CTRL_REG1, (1 << 6)); // Reset
  write8(m_fxas21002cAddr, FXAS21002C_REGISTER_CTRL_REG0, ctrlReg0); // Set sensitivity
  write8(m_fxas21002cAddr, FXAS21002C_REGISTER_CTRL_REG1, 0x0E);     // Active
  return true;
}


int RTIMUFXOS8700FXAS21002C::IMUGetPollInterval()
{
  return 1;
}
bool RTIMUFXOS8700FXAS21002C::IMURead()
{
 
  uint32_t const timestamp = RTMath::currentUSecsSinceEpoch();
  uint8_t data[13];
  uint8_t bVal = FXOS8700_REGISTER_STATUS | 0x80;
  m_settings->HALWrite(m_fxas21002cAddr, FXOS8700_REGISTER_STATUS, 1, &bVal, "");
  if (!m_settings->HALRead(m_fxos8700cAddr, FXOS8700_REGISTER_STATUS, 7, data, "Failed to read FXOS8700 id"))
  {
    return false;
  }

  for (int i = 0; i < 13; i++)
  {
    //HAL_INFO1("val: %02x ", data[i]);

  }
  //HAL_INFO("\n");
  
  accel_raw.x = data[1] << 8 | data[2];
  accel_raw.y = data[3] << 8 | data[4];
  accel_raw.z = data[5] << 8 | data[6];
  
  /* Convert accel values to m/s^2 */
  switch (_range) {
  case (ACCEL_RANGE_2G):
    accel_raw.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
    accel_raw.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
    accel_raw.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
    break;
  case (ACCEL_RANGE_4G):
    accel_raw.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
    accel_raw.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
    accel_raw.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
    break;
  case (ACCEL_RANGE_8G):
    accel_raw.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
    accel_raw.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
    accel_raw.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
    break;
  }


  m_imuData.accel.setX(accel_raw.x);
  m_imuData.accel.setY(accel_raw.y);
  m_imuData.accel.setZ(accel_raw.z);

 if (!m_settings->HALRead(m_fxos8700cAddr, FXOS8700_REGISTER_STATUS+0x32, 7, data, "Failed to read FXOS8700 id"))
  {
    return false;
  }

  mag_raw.x = (int16_t)((data[1] << 8) | data[2]);
  mag_raw.y = (int16_t)((data[3] << 8) | data[4]);
  mag_raw.z = (int16_t)((data[5] << 8) | data[6]);


  /* Convert mag values to uTesla */
  //mag_raw.x *= MAG_UT_LSB;
  //mag_raw.y *= MAG_UT_LSB;
  //mag_raw.z *= MAG_UT_LSB;
  //write8(m_fxos8700cAddr, FXOS8700_REGISTER_STATUS, 0x00);

  m_imuData.compass.setX(mag_raw.x);
  m_imuData.compass.setY(mag_raw.y);
  m_imuData.compass.setZ(mag_raw.z);


  m_imuData.timestamp = timestamp;

if (!m_settings->HALRead(m_fxas21002cAddr, FXAS21002C_REGISTER_STATUS, 7, data, "Failed to read FXOS8700 id"))
  {
    return false;
  }

 /* Shift values to create properly formed integer */
  gyro_raw.x = (int16_t)((data[1] << 8) | data[2]);
  gyro_raw.y = (int16_t)((data[3] << 8) | data[4]);
  gyro_raw.z = (int16_t)((data[5] << 8) | data[6]);

  /* Compensate values depending on the resolution */
  switch (_gyrorange) {
  case GYRO_RANGE_250DPS:
    gyro_raw.x *= GYRO_SENSITIVITY_250DPS;
    gyro_raw.y *= GYRO_SENSITIVITY_250DPS;
    gyro_raw.z *= GYRO_SENSITIVITY_250DPS;
    break;
  case GYRO_RANGE_500DPS:
    gyro_raw.x *= GYRO_SENSITIVITY_500DPS;
    gyro_raw.y *= GYRO_SENSITIVITY_500DPS;
    gyro_raw.z *= GYRO_SENSITIVITY_500DPS;
    break;
  case GYRO_RANGE_1000DPS:
    gyro_raw.x *= GYRO_SENSITIVITY_1000DPS;
    gyro_raw.y *= GYRO_SENSITIVITY_1000DPS;
    gyro_raw.z *= GYRO_SENSITIVITY_1000DPS;
    break;
  case GYRO_RANGE_2000DPS:
    gyro_raw.x *= GYRO_SENSITIVITY_2000DPS;
    gyro_raw.y *= GYRO_SENSITIVITY_2000DPS;
    gyro_raw.z *= GYRO_SENSITIVITY_2000DPS;
    break;
  }

  /* Convert values to rad/s */
  gyro_raw.x *= SENSORS_DPS_TO_RADS;
  gyro_raw.y *= SENSORS_DPS_TO_RADS;
  gyro_raw.z *= SENSORS_DPS_TO_RADS;


  m_imuData.gyro.setX(gyro_raw.x);
  m_imuData.gyro.setY(gyro_raw.y);
  m_imuData.gyro.setZ(gyro_raw.z);

  handleGyroBias();
  calibrateAverageCompass();
  calibrateAccel();

  //  now update the filter

  updateFusion();

  //HAL_INFO("Read Value\n");
  return true;
}
