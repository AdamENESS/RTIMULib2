#ifndef __LSM6DSOX_LIS3MDL__
#define __LSM6DSOX_LIS3MDL__

#include "RTIMU.h"
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX                                             \
  (60.0F) /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN                                             \
  (30.0F) /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA                                           \
  (1013.25F) /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS                                                    \
  (0.017453293F) /**< Degrees/s to rad/s multiplier                            \
                  */
#define SENSORS_RADS_TO_DPS                                                    \
  (57.29577793F) /**< Rad/s to degrees/s  multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA                                            \
  (100) /**< Gauss to micro-Tesla multiplier */



class RTIMULSM6DSOXLIS3MDL : public RTIMU
{
public:
    RTIMULSM6DSOXLIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DSOXLIS3MDL();

    virtual const char *IMUName();
    virtual int IMUType();
    virtual bool IMUInit();

	void BlockDataUpdate();

	void DisableISC();

	virtual int IMUGetPollInterval();
    virtual bool IMURead();
    void reset(void);

	lsm6ds_data_rate_t getAccelDataRate(void);

	/**************************************************************************/
	/*!
		@brief Sets the accelerometer data rate.
		@param  data_rate
				The the accelerometer data rate. Must be a `lsm6ds_data_rate_t`.
	*/
	void setAccelDataRate(lsm6ds_data_rate_t data_rate);

	/**************************************************************************/
	/*!
		@brief Gets the accelerometer measurement range.
		@returns The the accelerometer measurement range.
	*/
	lsm6ds_accel_range_t getAccelRange(void);
	/**************************************************************************/
	/*!
		@brief Sets the accelerometer measurement range.
		@param new_range The `lsm6ds_accel_range_t` range to set.
	*/
	void setAccelRange(lsm6ds_accel_range_t new_range);

	/**************************************************************************/
	/*!
		@brief Gets the gyro data rate.
		@returns The the gyro data rate.
	*/
	lsm6ds_data_rate_t getGyroDataRate(void);

	/**************************************************************************/
	/*!
		@brief Sets the gyro data rate.
		@param  data_rate
				The the gyro data rate. Must be a `lsm6ds_data_rate_t`.
	*/
	void setGyroDataRate(lsm6ds_data_rate_t data_rate);

	/**************************************************************************/
	/*!
		@brief Gets the gyro range.
		@returns The the gyro range.
	*/
	lsm6ds_gyro_range_t getGyroRange(void);

	/**************************************************************************/
	/*!
		@brief Sets the gyro range.
		@param new_range The `lsm6ds_gyro_range_t` to set.
	*/
	void setGyroRange(lsm6ds_gyro_range_t new_range);

	/**************************************************************************/
	/*!
		@brief Enables the high pass filter and/or slope filter
		@param filter_enabled Whether to enable the slope filter (see datasheet)
		@param filter The lsm6ds_hp_filter_t that sets the data rate divisor
	*/
	/**************************************************************************/
	void highPassFilter(bool filter_enabled,
		lsm6ds_hp_filter_t filter);


	void enableI2CMasterPullups(bool enable_pullups);

	void _read(void);
	void configIntOutputs(bool active_low, bool open_drain);
	void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
		bool step_detect, bool wakeup);
	void configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);


	void setPerformanceMode(lis3mdl_performancemode_t mode);

	/**************************************************************************/
	/*!
		@brief Get the performance mode
		@returns Enumerated lis3mdl_performancemode_t, LIS3MDL_LOWPOWERMODE,
		LIS3MDL_MEDIUMMODE, LIS3MDL_HIGHMODE or LIS3MDL_ULTRAHIGHMODE
	*/
	/**************************************************************************/
	lis3mdl_performancemode_t getPerformanceMode(void);

	/**************************************************************************/
	/*!
		@brief  Sets the data rate for the LIS3MDL (controls power consumption)
		from 0.625 Hz to 80Hz
		@param dataRate Enumerated lis3mdl_dataRate_t
	*/
	/**************************************************************************/
	void setDataRate(lis3mdl_dataRate_t dataRate);

	/**************************************************************************/
	/*!
		@brief  Gets the data rate for the LIS3MDL (controls power consumption)
		@return Enumerated lis3mdl_dataRate_t from 0.625 Hz to 80Hz
	*/
	/**************************************************************************/
	lis3mdl_dataRate_t getDataRate(void);

	/**************************************************************************/
	/*!
		@brief Set the operation mode, LIS3MDL_CONTINUOUSMODE,
		LIS3MDL_SINGLEMODE or LIS3MDL_POWERDOWNMODE
		@param mode Enumerated lis3mdl_operationmode_t
	*/
	/**************************************************************************/
	void setOperationMode(lis3mdl_operationmode_t mode);

	/**************************************************************************/
	/*!
		@brief Get the operation mode
		@returns Enumerated lis3mdl_operationmode_t, LIS3MDL_CONTINUOUSMODE,
		LIS3MDL_SINGLEMODE or LIS3MDL_POWERDOWNMODE
	*/
	/**************************************************************************/
	lis3mdl_operationmode_t getOperationMode(void);

	/**************************************************************************/
	/*!
		@brief Set the resolution range: +-4 gauss, 8 gauss, 12 gauss, or 16 gauss.
		@param range Enumerated lis3mdl_range_t
	*/
	/**************************************************************************/
	void setRange(lis3mdl_range_t range);

	/**************************************************************************/
	/*!
		@brief Read the resolution range: +-4 gauss, 8 gauss, 12 gauss, or 16 gauss.
		@returns Enumerated lis3mdl_range_t
	*/
	/**************************************************************************/
	lis3mdl_range_t getRange(void);

	/**************************************************************************/
	/*!
		@brief Set the interrupt threshold value
		@param value 16-bit unsigned raw value
	*/
	/**************************************************************************/
	void setIntThreshold(uint16_t value);

	/**************************************************************************/
	/*!
		@brief Get the interrupt threshold value
		@returns 16-bit unsigned raw value
	*/
	/**************************************************************************/
	uint16_t getIntThreshold(void);

	/**************************************************************************/
	/*!
		@brief Configure INT_CFG
		@param enableX Enable interrupt generation on X-axis
		@param enableY Enable interrupt generation on Y-axis
		@param enableZ Enable interrupt generation on Z-axis
		@param polarity Sets the polarity of the INT output logic
		@param latch If true (latched) the INT pin remains in the same state
		until INT_SRC is read.
		@param enableInt Interrupt enable on INT pin
	*/
	/**************************************************************************/
	void configInterrupt(bool enableX, bool enableY, bool enableZ,
		bool polarity, bool latch,
		bool enableInt);

	/**************************************************************************/
	/*!
		@brief Enable or disable self-test
		@param flag If true, enable self-test
	*/
	/**************************************************************************/
	void selfTest(bool flag);

	/**************************************************************************/
	/*!
		@brief Get the magnetic data rate.
		@returns The data rate in float
	*/
	float magneticFieldSampleRate(void);
protected:
  bool write8(int slave, int register, unsigned char val);
  unsigned char read8(int slave, int register);
private:

   unsigned char m_LSM6DSOXAddr;
   unsigned char m_LIS3DMLAddr;   

    int16_t rawAccX, ///< Last reading's raw accelerometer X axis
      rawAccY,     ///< Last reading's raw accelerometer Y axis
      rawAccZ,     ///< Last reading's raw accelerometer Z axis
      rawTemp,     ///< Last reading's raw temperature reading
      rawGyroX,    ///< Last reading's raw gyro X axis
      rawGyroY,    ///< Last reading's raw gyro Y axis
      rawGyroZ;    ///< Last reading's raw gyro Z axis   


      float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ;         ///< Last reading's gyro Z axis in rad/s



   int16_t x,     ///< The last read X mag in raw units
      y,         ///< The last read Y mag in raw units
      z;         ///< The last read Z mag in raw units
  float x_gauss, ///< The last read X mag in 'gauss'
      y_gauss,   ///< The last read Y mag in 'gauss'
      z_gauss;   ///< The last read Z mag in 'gauss'
   /*! Raw accelerometer values from last sucsessful sensor read */
  //fxos8700RawData_t accel_raw;
  /*! Raw magnetometer values from last successful sensor read */
  //fxos8700RawData_t mag_raw;
  //gyroRawData_t gyro_raw;

  //fxos8700AccelRange_t _range;
  //gyroRange_t _gyrorange;
};

#endif 

