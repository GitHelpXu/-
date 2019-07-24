#ifndef _ANGLE_H_
#define _ANGLE_H_

typedef struct
{
  float pitch;
  float roll;
  float yaw;
  float pitch_d;
  float roll_d;
  float yaw_d;
}angle;
/*
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
*/
extern angle filted;
extern angle raw;
extern float gz;
void angle_init();
void IMU_update();
void angle_get();
void angle_resume();
#endif