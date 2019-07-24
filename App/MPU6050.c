#include "include.h"
#include "SOFT_IIC.h"



#define	SMPLRT_DIV		0x19	//0x00
#define	CONFIG			0x1A	//0x00
#define	GYRO_CONFIG		0x1B	//0x18
#define	ACCEL_CONFIG	        0x1C	//0x18
#define	ACCEL_XOUT_H	        0x3B
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//0x68

#define USER_CTRL               0x6a
#define I2C_MST_CTRL            0x24
#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27
                                    
#define INT_PIN_CFG             0x37


#define MAG_ADDRESS             0x1E
#define MAG_DATA_REGISTER       0x03
#define	MPU6050_Addr            (0xD0>>1)	



float MPU6050_a_x,MPU6050_a_y,MPU6050_a_z;
float MPU6050_g_x=0.0,MPU6050_g_y,MPU6050_g_z;





uint8_t MPU6050_readbyte(uint8_t address)
{
  uint8_t data;
  data=simiic_read_reg(MPU6050_Addr, address, IIC);
  return data;
}

void MPU6050_writebyte(uint8_t address,uint8_t data)
{
   simiic_write_reg(MPU6050_Addr,address, data);
}


uint8_t debug_6050;
void MPU6050_init()
{
  debug_6050=MPU6050_readbyte(WHO_AM_I);
  while(MPU6050_readbyte(WHO_AM_I)!=0x68);
  MPU6050_writebyte(PWR_MGMT_1, 0x00);	//解除休眠状态
  MPU6050_writebyte(SMPLRT_DIV, 0x00);          //0x00
  MPU6050_writebyte(CONFIG, 0x00);      //0x00
  MPU6050_writebyte(GYRO_CONFIG, 0x18);//0x18 +-2000dps  full:+-32767
  MPU6050_writebyte(ACCEL_CONFIG, 0x18);//0x18
}

//////////所有加速度计和陀螺仪读值////////
void MPU6050_XYZ()
{
  uint8_t result[14];//[14];
  int16_t temp;
  simiic_read_regs(MPU6050_Addr,ACCEL_XOUT_H,result,14, IIC);
    
  temp=(int16_t)(result[0]<<8)|result[1];
  MPU6050_a_x=(float)temp;
  temp=(int16_t)(result[2]<<8)|result[3];
  MPU6050_a_y=(float)temp;
  temp=(int16_t)(result[4]<<8)|result[5];
  MPU6050_a_z=(float)temp;
    //6
    //7   温度
  temp=(int16_t)(result[8]<<8)|result[9];
  MPU6050_g_x=(float)temp*(2000.0f/32767.0f);
  temp=(int16_t)(result[10]<<8)|result[11];
  MPU6050_g_y=(float)temp*(2000.0f/32767.0f);
  temp=(int16_t)(result[12]<<8)|result[13];
  MPU6050_g_z=(float)temp*(2000.0f/32767.0f);
}

//////////陀螺仪X轴读值/////////
void MPU6050_GYRO_X()
{
  uint8_t result[2];//[14];
  int16_t temp;
  simiic_read_regs(MPU6050_Addr,GYRO_XOUT_H,result,2, IIC);
  
  temp=(int16_t)(result[0]<<8)|result[1];
  MPU6050_g_x=-(float)temp*(2000.0f/32767.0f)-7.56478905E-1;
  
}
////////陀螺仪Z轴读值//////////
void MPU6050_GYRO_Z()
{
  uint8_t result[2];//[14];
  int16_t temp;
  simiic_read_regs(MPU6050_Addr,GYRO_ZOUT_H,result,2, IIC);
  
  temp=(int16_t)(result[0]<<8)|result[1];
  MPU6050_g_z=-(float)temp*(2000.0f/32767.0f)-5.2115762233E-1;
  
}

float gyro_zero;
void gyro_z_0()
{
  uint16_t i;
  for(i=gyro_zero=0;i<2000;i++)
  {
    MPU6050_GYRO_Z();
    DELAY_MS(5);
    gyro_zero+=MPU6050_g_z;
  }
  gyro_zero/=2000;
    
}
