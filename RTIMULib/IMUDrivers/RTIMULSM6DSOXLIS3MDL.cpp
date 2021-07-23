#include "RTIMULSM6DSOXLIS3MDL.h"
#include "RTIMUSettings.h"
#include "Adafruit_BusIO_Register.cpp"
RTIMULSM6DSOXLIS3MDL::RTIMULSM6DSOXLIS3MDL(RTIMUSettings* settings) : RTIMU(settings)
{
m_sampleRate = 100;
}

RTIMULSM6DSOXLIS3MDL::~RTIMULSM6DSOXLIS3MDL()
{

}


const char * RTIMULSM6DSOXLIS3MDL::IMUName()
{
	return "LSM6DSOX + LIS3MDL";
}

int RTIMULSM6DSOXLIS3MDL::IMUType()
{
	return RTIMU_TYPE_LSM6DSOXLIS3MDL;
}

bool RTIMULSM6DSOXLIS3MDL::IMUInit()
{

	
   uint8_t lsm6dsox_id = 0;
  uint8_t lis3mdl_id = 0;

  m_imuData.fusionPoseValid = false;
  m_imuData.fusionQPoseValid = false;
  m_imuData.gyroValid = true;
  m_imuData.accelValid = true;
  m_imuData.compassValid = true;
  m_imuData.pressureValid = false;
  m_imuData.temperatureValid = true;
  m_imuData.humidityValid = false;

  m_LSM6DSOXAddr = LSM6DS_I2CADDR_DEFAULT;
  m_LIS3DMLAddr = LIS3MDL_I2CADDR_DEFAULT;

    //  enable the I2C bus

    if (!m_settings->HALOpen())
      return false;

    if (!m_settings->HALRead(m_LSM6DSOXAddr, LSM6DS_WHOAMI, 1, &lsm6dsox_id, "Failed to read LSM6DS id"))
    {
      return false;
    }

    if (lsm6dsox_id != LSM6DSOX_CHIP_ID)
    {
      HAL_ERROR1("Incorrect FX8700 IMU id %d", lsm6dsox_id);
      return false;
    }

    if (!m_settings->HALRead(m_LIS3DMLAddr, LIS3MDL_REG_WHO_AM_I, 1, &lis3mdl_id, "Failed to read LIS3MDL id"))
    {
      return false;
    }

    if (lis3mdl_id != LIS3MDL_CHIP_ID)
    {
      HAL_ERROR1("Incorrect LIS3MDL_CHIP_ID IMU id %d", lis3mdl_id);
      return false;
    }


	BlockDataUpdate();
	DisableISC();

	 setAccelDataRate(LSM6DS_RATE_104_HZ);
  setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  // Enable gyro with 104 Hz data rate, 2000 dps
  setGyroDataRate(LSM6DS_RATE_104_HZ);
  setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);


setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);

  // 155Hz default rate
  setDataRate(LIS3MDL_DATARATE_155_HZ);

  // lowest range
  setRange(LIS3MDL_RANGE_4_GAUSS);

  setOperationMode(LIS3MDL_CONTINUOUSMODE);
  //sleep(10);

    //write8(m_LSM6DSOXAddr, )

    return true;
}

void RTIMULSM6DSOXLIS3MDL::BlockDataUpdate()
{
	Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DSOX_CTRL3_C);
	Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);
	bdu.write(true);
}

void RTIMULSM6DSOXLIS3MDL::DisableISC()
{
	// Disable I3C
	Adafruit_BusIO_Register ctrl_9 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DSOX_CTRL9_XL);
	Adafruit_BusIO_RegisterBits i3c_disable_bit =
		Adafruit_BusIO_RegisterBits(&ctrl_9, 1, 1);

	i3c_disable_bit.write(true);
}

int RTIMULSM6DSOXLIS3MDL::IMUGetPollInterval()
{
  return (400 / m_sampleRate);
}

bool RTIMULSM6DSOXLIS3MDL::IMURead()
{
	HAL_INFO("Read");
	_read();
	// Timestamp the data
    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    m_imuData.accel.setX(accX); m_imuData.accel.setY(accY); m_imuData.accel.setZ(accZ);
    m_imuData.gyro.setX(gyroX); m_imuData.gyro.setY(gyroY); m_imuData.gyro.setZ(gyroZ);
    m_imuData.compass.setX(x_gauss); m_imuData.compass.setY(y_gauss); m_imuData.compass.setZ(z_gauss);
    m_imuData.temperature= temperature;
  
    handleGyroBias();
  calibrateAverageCompass();
  calibrateAccel();

  //  now update the filter

  updateFusion();
    return true;
}

void RTIMULSM6DSOXLIS3MDL::reset(void)
{
	Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL3_C);

	Adafruit_BusIO_RegisterBits sw_reset =
		Adafruit_BusIO_RegisterBits(&ctrl3, 1, 0);

	// Adafruit_BusIO_RegisterBits boot = Adafruit_BusIO_RegisterBits(&ctrl3, 1,
	// 7);

	sw_reset.write(true);

	while (sw_reset.read()) {
		usleep(1);
	}
}

lsm6ds_data_rate_t RTIMULSM6DSOXLIS3MDL::getAccelDataRate(void)
{
	Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL1_XL);

	Adafruit_BusIO_RegisterBits accel_data_rate =
		Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

	return (lsm6ds_data_rate_t)accel_data_rate.read();
}

void RTIMULSM6DSOXLIS3MDL::setAccelDataRate(lsm6ds_data_rate_t data_rate)
{
	Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL1_XL);

	Adafruit_BusIO_RegisterBits accel_data_rate =
		Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

	accel_data_rate.write(data_rate);
}

lsm6ds_accel_range_t RTIMULSM6DSOXLIS3MDL::getAccelRange(void)
{
	Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL1_XL);

	Adafruit_BusIO_RegisterBits accel_range =
		Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

	return (lsm6ds_accel_range_t)accel_range.read();
}

void RTIMULSM6DSOXLIS3MDL::setAccelRange(lsm6ds_accel_range_t new_range)
{
	Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL1_XL);

	Adafruit_BusIO_RegisterBits accel_range =
		Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

	accel_range.write(new_range);
	usleep(20);
}

lsm6ds_data_rate_t RTIMULSM6DSOXLIS3MDL::getGyroDataRate(void)
{
	Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL2_G);

	Adafruit_BusIO_RegisterBits gyro_data_rate =
		Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

	return (lsm6ds_data_rate_t)gyro_data_rate.read();
}

void RTIMULSM6DSOXLIS3MDL::setGyroDataRate(lsm6ds_data_rate_t data_rate)
{
	Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL2_G);

	Adafruit_BusIO_RegisterBits gyro_data_rate =
		Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

	gyro_data_rate.write(data_rate);
}

lsm6ds_gyro_range_t RTIMULSM6DSOXLIS3MDL::getGyroRange(void)
{
	Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL2_G);

	Adafruit_BusIO_RegisterBits gyro_range =
		Adafruit_BusIO_RegisterBits(&ctrl2, 4, 0);

	return (lsm6ds_gyro_range_t)gyro_range.read();
}

void RTIMULSM6DSOXLIS3MDL::setGyroRange(lsm6ds_gyro_range_t new_range)
{
	Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL2_G);

	Adafruit_BusIO_RegisterBits gyro_range =
		Adafruit_BusIO_RegisterBits(&ctrl2, 4, 0);

	gyro_range.write(new_range);
	usleep(20);
}

void RTIMULSM6DSOXLIS3MDL::highPassFilter(bool filter_enabled, lsm6ds_hp_filter_t filter)
{
	Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
		m_settings, ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_CTRL8_XL);
	Adafruit_BusIO_RegisterBits HPF_en =
		Adafruit_BusIO_RegisterBits(&ctrl8, 1, 2);
	Adafruit_BusIO_RegisterBits HPF_filter =
		Adafruit_BusIO_RegisterBits(&ctrl8, 2, 5);
	HPF_en.write(filter_enabled);
	HPF_filter.write(filter);
}

void RTIMULSM6DSOXLIS3MDL::enableI2CMasterPullups(bool enable_pullups)
{
	Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
		m_settings,ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DSOX_FUNC_CFG_ACCESS);
	Adafruit_BusIO_RegisterBits master_cfg_enable_bit =
		Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);

	Adafruit_BusIO_Register master_config = Adafruit_BusIO_Register(
		m_settings,ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DSOX_MASTER_CONFIG);
	Adafruit_BusIO_RegisterBits i2c_master_pu_en =
		Adafruit_BusIO_RegisterBits(&master_config, 1, 3);

	master_cfg_enable_bit.write(true);
	i2c_master_pu_en.write(enable_pullups);
	master_cfg_enable_bit.write(false);
}

void RTIMULSM6DSOXLIS3MDL::_read(void)
{
	// get raw readings
	Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
		m_settings,  ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_OUT_TEMP_L, 14);

	uint8_t buffer[14];
	data_reg.read(buffer, 14);

	rawTemp = buffer[1] << 8 | buffer[0];
	temperature = (rawTemp / 256.0) + 25.0;

	rawGyroX = buffer[3] << 8 | buffer[2];
	rawGyroY = buffer[5] << 8 | buffer[4];
	rawGyroZ = buffer[7] << 8 | buffer[6];

	rawAccX = buffer[9] << 8 | buffer[8];
	rawAccY = buffer[11] << 8 | buffer[10];
	rawAccZ = buffer[13] << 8 | buffer[12];

	lsm6ds_gyro_range_t gyro_range = getGyroRange();
	float gyro_scale = 1; // range is in milli-dps per bit!
	if (gyro_range == ISM330DHCX_GYRO_RANGE_4000_DPS)
		gyro_scale = 140.0;
	if (gyro_range == LSM6DS_GYRO_RANGE_2000_DPS)
		gyro_scale = 70.0;
	if (gyro_range == LSM6DS_GYRO_RANGE_1000_DPS)
		gyro_scale = 35.0;
	if (gyro_range == LSM6DS_GYRO_RANGE_500_DPS)
		gyro_scale = 17.50;
	if (gyro_range == LSM6DS_GYRO_RANGE_250_DPS)
		gyro_scale = 8.75;
	if (gyro_range == LSM6DS_GYRO_RANGE_125_DPS)
		gyro_scale = 4.375;

	gyroX = rawGyroX * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
	gyroY = rawGyroY * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
	gyroZ = rawGyroZ * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;

	lsm6ds_accel_range_t accel_range = getAccelRange();
	float accel_scale = 1; // range is in milli-g per bit!
	if (accel_range == LSM6DS_ACCEL_RANGE_16_G)
		accel_scale = 0.488;
	if (accel_range == LSM6DS_ACCEL_RANGE_8_G)
		accel_scale = 0.244;
	if (accel_range == LSM6DS_ACCEL_RANGE_4_G)
		accel_scale = 0.122;
	if (accel_range == LSM6DS_ACCEL_RANGE_2_G)
		accel_scale = 0.061;

	accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
	accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
	accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;

	 Adafruit_BusIO_Register XYZDataReg = Adafruit_BusIO_Register(
      m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC, m_LIS3DMLAddr, LIS3MDL_REG_OUT_X_L, 6);
  
  XYZDataReg.read(buffer, 6);
  x = buffer[0];
  x |= buffer[1] << 8;
  y = buffer[2];
  y |= buffer[3] << 8;
  z = buffer[4];
  z |= buffer[5] << 8;

  lis3mdl_range_t range = getRange();
  float scale = 1; // LSB per gauss
  if (range == LIS3MDL_RANGE_16_GAUSS)
    scale = 1711;
  if (range == LIS3MDL_RANGE_12_GAUSS)
    scale = 2281;
  if (range == LIS3MDL_RANGE_8_GAUSS)
    scale = 3421;
  if (range == LIS3MDL_RANGE_4_GAUSS)
    scale = 6842;

  x_gauss = (float)x / scale;
  y_gauss = (float)y / scale;
  z_gauss = (float)z / scale;
}

void RTIMULSM6DSOXLIS3MDL::configIntOutputs(bool active_low, bool open_drain)
{
	Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
		m_settings,  ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr,LSM6DS_CTRL3_C);
	Adafruit_BusIO_RegisterBits ppod_bits =
		Adafruit_BusIO_RegisterBits(&ctrl3, 2, 4);

	ppod_bits.write((active_low << 1) | open_drain);
}

void RTIMULSM6DSOXLIS3MDL::configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl, bool step_detect, bool wakeup)
{
	Adafruit_BusIO_Register int1_ctrl = Adafruit_BusIO_Register(
		m_settings,  ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_INT1_CTRL);

	int1_ctrl.write((step_detect << 7) | (drdy_temp << 2) | (drdy_g << 1) |
		drdy_xl);

	Adafruit_BusIO_Register md1cfg = Adafruit_BusIO_Register(
		m_settings,  ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_MD1_CFG);

	Adafruit_BusIO_RegisterBits wu = Adafruit_BusIO_RegisterBits(&md1cfg, 1, 5);
	wu.write(wakeup);
}

void RTIMULSM6DSOXLIS3MDL::configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl)
{
	Adafruit_BusIO_Register int2_ctrl = Adafruit_BusIO_Register(
		m_settings,  ADDRBIT8_HIGH_TOREAD, m_LSM6DSOXAddr, LSM6DS_INT2_CTRL);

	Adafruit_BusIO_RegisterBits int2_drdy_bits =
		Adafruit_BusIO_RegisterBits(&int2_ctrl, 3, 0);

	int2_drdy_bits.write((drdy_temp << 2) | (drdy_g << 1) | drdy_xl);
}

void RTIMULSM6DSOXLIS3MDL::setPerformanceMode(lis3mdl_performancemode_t mode)
{
	// write xy
	Adafruit_BusIO_Register CTRL_REG1 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG1, 1);
	Adafruit_BusIO_RegisterBits performancemodebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG1, 2, 5);
	performancemodebits.write((uint8_t)mode);

	// write z
	Adafruit_BusIO_Register CTRL_REG4 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG4, 1);
	Adafruit_BusIO_RegisterBits performancemodezbits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG4, 2, 2);
	performancemodezbits.write((uint8_t)mode);
}

lis3mdl_performancemode_t RTIMULSM6DSOXLIS3MDL::getPerformanceMode(void)
{
	Adafruit_BusIO_Register CTRL_REG1 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG1, 1);
	Adafruit_BusIO_RegisterBits performancemodebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG1, 2, 5);
	return (lis3mdl_performancemode_t)performancemodebits.read();
}

void RTIMULSM6DSOXLIS3MDL::setDataRate(lis3mdl_dataRate_t dataRate)
{
	if (dataRate == LIS3MDL_DATARATE_155_HZ) {
		// set OP to UHP
		setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
	}
	if (dataRate == LIS3MDL_DATARATE_300_HZ) {
		// set OP to HP
		setPerformanceMode(LIS3MDL_HIGHMODE);
	}
	if (dataRate == LIS3MDL_DATARATE_560_HZ) {
		// set OP to MP
		setPerformanceMode(LIS3MDL_MEDIUMMODE);
	}
	if (dataRate == LIS3MDL_DATARATE_1000_HZ) {
		// set OP to LP
		setPerformanceMode(LIS3MDL_LOWPOWERMODE);
	}
	usleep(10);
	Adafruit_BusIO_Register CTRL_REG1 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG1, 1);
	Adafruit_BusIO_RegisterBits dataratebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG1, 4, 1); // includes FAST_ODR
	dataratebits.write((uint8_t)dataRate);
}

lis3mdl_dataRate_t RTIMULSM6DSOXLIS3MDL::getDataRate(void)
{
	Adafruit_BusIO_Register CTRL_REG1 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG1, 1);
	Adafruit_BusIO_RegisterBits dataratebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG1, 4, 1); // includes FAST_ODR
	return (lis3mdl_dataRate_t)dataratebits.read();
}

void RTIMULSM6DSOXLIS3MDL::setOperationMode(lis3mdl_operationmode_t mode)
{
	// write x and y
	Adafruit_BusIO_Register CTRL_REG3 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG3, 1);
	Adafruit_BusIO_RegisterBits opmodebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG3, 2, 0);
	opmodebits.write((uint8_t)mode);
}

lis3mdl_operationmode_t RTIMULSM6DSOXLIS3MDL::getOperationMode(void)
{
	Adafruit_BusIO_Register CTRL_REG3 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG3, 1);
	Adafruit_BusIO_RegisterBits opmodebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG3, 2, 0);
	return (lis3mdl_operationmode_t)opmodebits.read();
}

void RTIMULSM6DSOXLIS3MDL::setRange(lis3mdl_range_t range)
{
	Adafruit_BusIO_Register CTRL_REG2 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG2, 1);
	Adafruit_BusIO_RegisterBits rangebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG2, 2, 5);
	rangebits.write((uint8_t)range);
}

lis3mdl_range_t RTIMULSM6DSOXLIS3MDL::getRange(void)
{
	Adafruit_BusIO_Register CTRL_REG2 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG2, 1);
	Adafruit_BusIO_RegisterBits rangebits =
		Adafruit_BusIO_RegisterBits(&CTRL_REG2, 2, 5);
	return (lis3mdl_range_t)rangebits.read();
}

void RTIMULSM6DSOXLIS3MDL::setIntThreshold(uint16_t value)
{
	value &= 0x7FFF; // high bit must be 0!
	Adafruit_BusIO_Register INT_THS =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_INT_THS_L, 2);
	INT_THS.write(value);
}

uint16_t RTIMULSM6DSOXLIS3MDL::getIntThreshold(void)
{
	Adafruit_BusIO_Register INT_THS =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_INT_THS_L, 2);
	return INT_THS.read();
}

void RTIMULSM6DSOXLIS3MDL::configInterrupt(bool enableX, bool enableY, bool enableZ, bool polarity, bool latch, bool enableInt)
{
	uint8_t value = 0x08; // set default bits, see table 36
	value |= enableX << 7;
	value |= enableY << 6;
	value |= enableZ << 5;
	value |= polarity << 2;
	value |= latch << 1;
	value |= enableInt;

	Adafruit_BusIO_Register INT_CFG = Adafruit_BusIO_Register(
		m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC, m_LIS3DMLAddr ,LIS3MDL_REG_INT_CFG, 1);
	INT_CFG.write(value);
}

void RTIMULSM6DSOXLIS3MDL::selfTest(bool flag)
{
	Adafruit_BusIO_Register CTRL_REG1 =
		Adafruit_BusIO_Register(m_settings, AD8_HIGH_TOREAD_AD7_HIGH_TOINC,
			m_LIS3DMLAddr ,LIS3MDL_REG_CTRL_REG1, 1);

	Adafruit_BusIO_RegisterBits stbit =
		Adafruit_BusIO_RegisterBits(&CTRL_REG1, 1, 0);

	stbit.write(flag);
}

float RTIMULSM6DSOXLIS3MDL::magneticFieldSampleRate(void)
{
	switch (this->getDataRate()) {
	case LIS3MDL_DATARATE_0_625_HZ:
		return 0.625f;
	case LIS3MDL_DATARATE_1_25_HZ:
		return 1.25f;
	case LIS3MDL_DATARATE_2_5_HZ:
		return 2.5f;
	case LIS3MDL_DATARATE_5_HZ:
		return 5.0f;
	case LIS3MDL_DATARATE_10_HZ:
		return 10.0f;
	case LIS3MDL_DATARATE_20_HZ:
		return 20.0f;
	case LIS3MDL_DATARATE_40_HZ:
		return 40.0f;
	case LIS3MDL_DATARATE_80_HZ:
		return 80.0f;
	case LIS3MDL_DATARATE_155_HZ:
		return 155.0f;
	case LIS3MDL_DATARATE_300_HZ:
		return 300.0f;
	case LIS3MDL_DATARATE_560_HZ:
		return 560.0f;
	case LIS3MDL_DATARATE_1000_HZ:
		return 1000.0f;
	}

	return 0;
}

bool RTIMULSM6DSOXLIS3MDL::write8(int slave, int reg, unsigned char val)
{
  return m_settings->HALWrite(slave, reg, 1, &val, "Failed to Write");
}

unsigned char RTIMULSM6DSOXLIS3MDL::read8(int slave, int reg)
{
  uint8_t data = 0;
  if (m_settings->HALRead(slave, reg, 1, &data, "Failed to read"))
  {
    return false;
  }

  return data;
}


