
// Include Wire Library for I2C
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

enum MPU6050_filter {_256Hz, _188Hz, _98Hz, _42Hz, _20Hz, _10Hz, _5Hz};
enum MPU6050_gyro {_250dps, _500dps, _1000dps, _2000dps};
enum MPU6050_accel {_2g, _4g, _8g, _16Hz};

class MPU6050
{
public:

  MPU6050 (bool AD0_value);

  bool testConnection ();

  bool start ();

  bool goToSleep ();

  bool stopSleeping ();

  bool setFilterVal (MPU6050_filter filter_val);

  bool setGyroRange (MPU6050_gyro range);

  bool setAccelRange (MPU6050_accel range);

  bool dataAvailable (bool& data_available);

  bool getLastGyroData (float& gx, float& gy, float& gz, bool calibrate = true);

  bool getRawGyroData (int& gx, int& gy, int& gz, bool calibrate = true);

  bool getRawAccelData (int& ax, int& ay, int& az, bool calibrate = true);

  bool getLastAccelData (float& ax, float& ay, float& az, bool ms2, bool calibrate = true);

  bool resetFIFO ();

  bool getFIFOCount (unsigned& qty);

  // process all the data from the FIFO
  bool processFIFOMotion ();

  bool processMotion (float g_calibration_factor);

  void getAngles (float& ang_x, float& ang_y, float& ang_z);

  bool getG (float& g);

  // angles obtained from the accelerometer values
  bool getAccelAngles(float& ang_x, float& ang_y, float& ang_z, bool raw = false);

  bool gVectorAxisProjections (float& x_axis, float& y_axis, float& z_axis);

private:

  bool writeRegister (byte address, byte data);

  bool readData (byte start_address, byte bytes, byte* data);

  float convertGyroToDPS (long gyro);

  float contAccelToMS2 (long accel);

  bool readFIFOData (unsigned samples, long& sum_x, long& sum_y, long& sum_z);
  
  bool initAngles ();

  bool calibrate (unsigned samples);

  float gravityAngleTanCorrection (float proj_axis_1, float proj_axis_2);

  float addAngles (float angle_1, float angle_2);
  
  // I2c object for communication with the sensor
  Adafruit_I2CDevice i2c_com;

  // general config. variables
  bool AD0_val;
  MPU6050_filter filter;
  MPU6050_accel accel_range;
  MPU6050_gyro gyro_range;
  unsigned samples_to_calibrate = 1000;
  const float g_value;

  // motion processing variables
  float last_gx, last_gy, last_gz;
  float angle_x, angle_y, angle_z;
  float init_accel_x_angle, init_accel_y_angle, init_accel_z_angle;
  unsigned long last_FIFO_read;
  int x_gyro_cal, y_gyro_cal, z_gyro_cal;     // calibration values for gyro
  int x_accel_cal, y_accel_cal, z_accel_cal;  // calibration values for accel
  const unsigned samples_to_avg_gyro;
  unsigned long last_gyro_read_time;
};
