#include "MPU6050.h"
#include "MK60_i2c.h"


void MPU6050_read()
{
  X_6050=i2c_read_reg(I2C0,MPU6050_Addr,GYRO_XOUT_H);
  Y_6050=i2c_read_reg(I2C0,MPU6050_Addr,GYRO_YOUT_H);
  Z_6050=i2c_read_reg(I2C0,MPU6050_Addr,GYRO_ZOUT_H);
}
