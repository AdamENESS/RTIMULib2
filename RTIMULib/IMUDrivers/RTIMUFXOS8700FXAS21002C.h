#ifndef __FXOS8700_FXAS21002CH__
#define __FXOS8700_FXAS21002CH__

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



class RTIMUFXOS8700FXAS21002C : public RTIMU
{
public:
    RTIMUFXOS8700FXAS21002C(RTIMUSettings *settings);
    ~RTIMUFXOS8700FXAS21002C();

    virtual const char *IMUName() { return "FXOS8700 + FXAS21002C"; }
    virtual int IMUType() { return RTIMU_TYPE_FXOS8700FXAS21002C; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();
protected:
  bool write8(int slave, int register, unsigned char val);
  unsigned char read8(int slave, int register);
private:

   unsigned char m_fxos8700cAddr;
   unsigned char m_fxas21002cAddr;      
   /*! Raw accelerometer values from last sucsessful sensor read */
  fxos8700RawData_t accel_raw;
  /*! Raw magnetometer values from last successful sensor read */
  fxos8700RawData_t mag_raw;
  gyroRawData_t gyro_raw;

  fxos8700AccelRange_t _range;
  gyroRange_t _gyrorange;
};

#endif 