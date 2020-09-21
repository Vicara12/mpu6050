
#include "mpu6050.h"

MPU6050::MPU6050 (bool AD0_value) : AD0_val(AD0_value),
                                    filter(_256Hz),
                                    accel_range(_2g),
                                    gyro_range(_250dps),
                                    g_value(9.8),
                                    samples_to_avg_gyro(5),
                                    i2c_com(0x68 + AD0_value) {}

bool MPU6050::testConnection ()
{
  return i2c_com.detected();
}

bool MPU6050::start ()
{ 
  //if (not writeRegister(0x6B, 0x00)) Serial.println("wrong");
/*
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
*/
  byte add = 0x6B, dat = 0x00;

  i2c_com.begin();
  i2c_com.write(&add, 1, false);
  i2c_com.write(&dat, 1, true);

  byte data;
  readData(0x6B, 1, &data);
  Serial.println(data, HEX);
  
  bool correct_start =
  
  // init sample rate div to 0 (max sample rate)
  writeRegister(0x19, 0x00) and
  // activate FIFO for gyroscope data
  writeRegister(0x23, 0x70) and
  // clear config setup register (disable temp)
  writeRegister(0x6B, 0x44) and

  // setup the interrupt register
  writeRegister(0x37, 0x10) and
  // set interrupt by data ready
  writeRegister(0x38, 0x01) and

  stopSleeping() and

  // start motion processing
  initAngles() and
  calibrate(samples_to_calibrate) and
  resetFIFO() and

  getLastGyroData(last_gx, last_gy, last_gz);
  last_gyro_read_time = micros();

  // enable FIFO
  return correct_start and writeRegister(0x6A, 0x40);
}

bool MPU6050::goToSleep ()
{
  byte prev_data;
  if (not readData(0x6B, 1, &prev_data)) return false;
  prev_data = (prev_data | 0x40);
  return writeRegister(0x6B, prev_data);
}

bool MPU6050::stopSleeping ()
{
  byte prev_data;
  if (not readData(0x6B, 1, &prev_data)) return false;
  prev_data = (prev_data & 0xBF);
  return writeRegister(0x6B, prev_data);
}

bool MPU6050::setFilterVal (MPU6050_filter filter_val)
{
  int val;

  if      (filter_val == _256Hz) val = 0;
  else if (filter_val == _188Hz) val = 1;
  else if (filter_val == _98Hz)  val = 2;
  else if (filter_val == _42Hz)  val = 3;
  else if (filter_val == _20Hz)  val = 4;
  else if (filter_val == _10Hz)  val = 5;
  else                           val = 6;

  byte data;
  if (not readData(0x1A, 1, &data)) return false;
  data = (data & 0xF8) | (val & 0x07);
  if (not writeRegister(0x1A, data)) return false;

  filter = filter_val;
  return true;
}

bool MPU6050::setAccelRange (MPU6050_accel range)
{
  byte value;

  if (range == _2g)      value = 0;
  else if (range == _4g) value = 1;
  else if (range == _8g) value = 2;
  else                   value = 3;

  byte reg_value;
  if (not readData(0x1C, 1, &reg_value)) return false;
  reg_value = (reg_value & 0xE0) | (value << 3);
  if (not writeRegister(0x1C, reg_value)) return false;

  accel_range = range;
  return true;
}

bool MPU6050::setGyroRange (MPU6050_gyro range)
{
  byte value;

  if      (range == _250dps)  value = 0;
  else if (range == _500dps)  value = 1;
  else if (range == _1000dps) value = 2;
  else                        value = 3;

  byte reg_value;
  if (not readData(0x1B, 1, &reg_value)) return false;
  reg_value = (reg_value & 0xE0) | (value << 3);
  if (not writeRegister(0x1B, reg_value)) return false;

  gyro_range = range;
  return true;
}

bool MPU6050::dataAvailable (bool &data_available)
{
  byte reg;
  if (not readData(0x3A, 1, &reg)) return false;
  data_available = bool(reg & 0x01);
  return true;
}

bool MPU6050::getRawGyroData (int& gx, int& gy, int& gz, bool calibrate)
{
  byte data[6];

  if (not readData(0x43, 6, data)) return false;

  gx = (data[0] << 8 | data[1]) - (calibrate ? x_gyro_cal : 0);
  gy = (data[2] << 8 | data[3]) - (calibrate ? y_gyro_cal : 0);
  gz = (data[4] << 8 | data[5]) - (calibrate ? z_gyro_cal : 0);

  return true;
}

bool MPU6050::getLastGyroData (float& gx, float& gy, float& gz, bool calibrate)
{
  int raw_x, raw_y, raw_z;

  if (not getRawGyroData(raw_x, raw_y, raw_z, calibrate)) return false;

  gx = convertGyroToDPS(raw_x);
  gy = convertGyroToDPS(raw_y);
  gz = convertGyroToDPS(raw_z);

  return true;
}

bool MPU6050::getRawAccelData (int& ax, int& ay, int& az, bool calibrate)
{
  byte data[6];

  if (not readData(0x3B, 6, data)) return false;

  ax = (data[0] << 8 | data[1]);
  ay = (data[2] << 8 | data[3]);
  az = (data[4] << 8 | data[5]);

  return true;
}

bool MPU6050::getLastAccelData (float& ax, float& ay, float& az, bool ms2, bool calibrate)
{
  int raw_x, raw_y, raw_z;

  if (not getRawAccelData(raw_x, raw_y, raw_z)) return false;

  ax = contAccelToMS2(raw_x) * (ms2 ? g_value : 1);
  ay = contAccelToMS2(raw_y) * (ms2 ? g_value : 1);
  az = contAccelToMS2(raw_z) * (ms2 ? g_value : 1);

  return true;
}

bool MPU6050::resetFIFO ()
{
  last_FIFO_read = micros();
  byte reg;
  if (not readData(0x6A, 1, &reg)) return false;
  return writeRegister(0x6A, reg | 0x04);
}

bool MPU6050::getFIFOCount (unsigned& qty)
{
  byte data[2];
  if (not readData(0x72, 2, data)) return false;
  qty = data[0] << 8 | data[1];
  return true;
}

bool MPU6050::processFIFOMotion ()
{
  // save last time FIFO was read
  unsigned long previous_FIFO_read = last_FIFO_read;
  // actualize fifo read timing
  last_FIFO_read = micros();
  unsigned qty;
  if (not getFIFOCount(qty)) return false;
  // qty of samples to be read
  qty /= 6;

  long sum_x, sum_y, sum_z;
  if (not readFIFOData(qty, sum_x, sum_y, sum_z)) return false;

  float time_between_readings = (float(last_FIFO_read - previous_FIFO_read) / 1000000.0);

  angle_x += convertGyroToDPS(sum_x - qty * x_gyro_cal) * time_between_readings;
  angle_y += convertGyroToDPS(sum_y - qty * y_gyro_cal) * time_between_readings;
  angle_z += convertGyroToDPS(sum_z - qty * z_gyro_cal) * time_between_readings;

  return true;
}

bool MPU6050::processMotion (float g_calibration_factor)
{
  float gx, gy, gz;
  unsigned long previous_read_time = last_gyro_read_time;

  if (not getLastGyroData(gx, gy, gz)) return false;
  last_gyro_read_time = micros();

  last_gx = (last_gx * (samples_to_avg_gyro - 1) + gx) / samples_to_avg_gyro;
  last_gy = (last_gy * (samples_to_avg_gyro - 1) + gy) / samples_to_avg_gyro;
  last_gz = (last_gz * (samples_to_avg_gyro - 1) + gz) / samples_to_avg_gyro;

  float delta_t = (last_gyro_read_time - previous_read_time) / 1000000.0;

  angle_x += last_gx * delta_t;
  angle_y += last_gy * delta_t;
  angle_z += last_gz * delta_t;

  // if gravity compensation is activated, use gravity to calibrate the angles
  if (g_calibration_factor != 0)
  {
    float g_ang_x, g_ang_y, g_ang_z, x_proj, y_proj, z_proj;
    if (not getAccelAngles(g_ang_x, g_ang_y, g_ang_z)) return false;
    if (not gVectorAxisProjections(x_proj, y_proj, z_proj)) return false;

    // pi/180 = 0.01745329252
    float x_calibration = g_calibration_factor * abs(x_proj);
    float y_calibration = g_calibration_factor * abs(y_proj);
    float z_calibration = g_calibration_factor * abs(z_proj);

    angle_x = addAngles((1.0 - x_calibration) * angle_x,
                        x_calibration * (g_ang_x - init_accel_x_angle));
    angle_y = addAngles((1.0 - y_calibration) * angle_y,
                        y_calibration * (g_ang_y - init_accel_y_angle));
    angle_z = addAngles((1.0 - z_calibration) * angle_z,
                        z_calibration * (g_ang_z - init_accel_z_angle));
  }

  return true;
}

void MPU6050::getAngles (float& ang_x, float& ang_y, float& ang_z)
{
  ang_x = angle_x;
  ang_y = angle_y;
  ang_z = angle_z;
}

bool MPU6050::getG (float& g)
{
  float gx, gy, gz;

  if (not getLastAccelData(gx, gy, gz, true)) return false;

  g = sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2));
  return true;
}

bool MPU6050::getAccelAngles(float& ang_x, float& ang_y, float& ang_z, bool raw)
{
  float gx, gy, gz;

  if (not getLastAccelData(gx, gy, gz, not raw)) return false;

  // convert radians to degrees: 57.29578 = 180/pi
  // as tan(45dg) == tan(135dg), we must take into consideration where the angle is
  ang_x = atan(gy / gz) * 57.29578 + gravityAngleTanCorrection(gy, gz);
  ang_y = atan(gz / gx) * 57.29578 + gravityAngleTanCorrection(gz, gx);
  ang_z = atan(gx / gy) * 57.29578 + gravityAngleTanCorrection(gx, gy);

  return true;
}

bool MPU6050::gVectorAxisProjections (float& x_axis, float& y_axis, float& z_axis)
{
  float gx, gy, gz, g;

  if (not getLastAccelData(gx, gy, gz, true)) return false;

  g = sqrt(pow(gx,2) + pow(gy,2) + pow(gz,2));

  x_axis = gx/g;
  y_axis = gy/g;
  z_axis = gz/g;

  return true;
}

bool MPU6050::writeRegister (byte address, byte data)
{
  /*
  return i2c_com.begin() and
  i2c_com.write(&address, 1, false) and
  i2c_com.write(&data, 1, true);
  */
  Wire.beginTransmission(0x68 + AD0_val);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
  return true;
}

bool MPU6050::readData (byte start_address, byte bytes, byte* data)
{
  return i2c_com.begin() and
  //Send the requested starting register
  i2c_com.write(&start_address, 1, false) and
  //Request data from the MPU-6050
  i2c_com.read(data, bytes, true);
}

float MPU6050::convertGyroToDPS (long gyro)
{
  if      (gyro_range == _250dps)  return float(gyro) / 131.0;
  else if (gyro_range == _500dps)  return float(gyro) / 65.5;
  else if (gyro_range == _1000dps) return float(gyro) / 32.8;
  else                             return float(gyro) / 16.4;
}

float MPU6050::contAccelToMS2 (long accel)
{
  if      (accel_range == _2g) return float(accel) / 16384.0;
  else if (accel_range == _4g) return float(accel) / 8192.0;
  else if (accel_range == _8g) return float(accel) / 4096.0;
  else                         return float(accel) / 2048;
}

bool MPU6050::readFIFOData (unsigned samples, long& sum_x, long& sum_y, long& sum_z)
{
  sum_x = sum_y = sum_z = 0;
  
  i2c_com.begin();

  byte FIFO_data_address = 0x74;

  for (int sample = 0; sample < samples; sample++)
  {
    byte data_bytes[6];

    for (int sample_byte = 0; sample_byte < 6; sample_byte++)
    {
      if (not i2c_com.write_then_read(&FIFO_data_address, 1,
                                      data_bytes + sample_byte, 1,
                                      sample == samples - 1 and sample_byte == 5))
        return false;
    }

    sum_x += data_bytes[0] << 8 | data_bytes[1];
    sum_y += data_bytes[2] << 8 | data_bytes[3];
    sum_z += data_bytes[4] << 8 | data_bytes[5];
  }

  return true;
}

bool MPU6050::initAngles ()
{
  angle_x = angle_y = angle_z = 0;

  return getAccelAngles(init_accel_x_angle, init_accel_y_angle, init_accel_z_angle);
}

bool MPU6050::calibrate (unsigned samples)
{
  long x_sum = 0, y_sum = 0, z_sum = 0;

  for (int i = 0; i < samples; i++)
  {
    int this_x, this_y, this_z;
    bool data_av;
    bool read_correctly;
    unsigned long loop_start = millis();

    // add some blocking control (max loop time is 100ms)
    do {
      read_correctly = dataAvailable(data_av);
      //if (not data_av and read_correctly) Serial.println("xd");
    } while ((not read_correctly or not data_av) and millis() - loop_start < 1000);

    if (millis() - loop_start >= 100)
      return false;

    loop_start = millis();
    while (not getRawGyroData(this_x, this_y, this_z, false) and millis() - loop_start < 100);

    if (millis() - loop_start >= 100)
      return false;

    x_sum += this_x;
    y_sum += this_y;
    z_sum += this_z;

    delay(1);
  }

  x_gyro_cal = x_sum / samples;
  y_gyro_cal = y_sum / samples;
  z_gyro_cal = z_sum / samples;

  return true;
}

float MPU6050::gravityAngleTanCorrection (float proj_axis_1, float proj_axis_2)
{
  if (proj_axis_1 > 0 and proj_axis_2 < 0) return 180;
  else if (proj_axis_1 < 0 and proj_axis_2 < 0) return -180;
  else return 0;
}

float MPU6050::addAngles (float angle_1, float angle_2)
{
  float result = angle_1 + angle_2;

  if (result > 180)
    result -= 360;
  else if (result <= -180)
    result += 360;

  return result;
}
