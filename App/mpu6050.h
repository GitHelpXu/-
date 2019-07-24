#ifndef MPU6050_H_
#define MPU6050_H_





extern float MPU6050_a_x,MPU6050_a_y,MPU6050_a_z;
extern float MPU6050_g_x,MPU6050_g_y,MPU6050_g_z;

extern void MPU6050_init(void);
extern void MPU6050_XYZ(void);
extern void MPU6050_GYRO_X(void);
extern void MPU6050_GYRO_Z(void);
#endif
