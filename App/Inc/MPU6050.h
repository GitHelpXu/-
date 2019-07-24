#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "common.h"

#define	MPU6050_Addr            (0xD0>>1)
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
extern uint8 X_6050;
extern uint8 Y_6050;
extern uint8 Z_6050;

extern void MPU6050_read();

#endif