#ifndef _MORTOR_H_
#define _MORTOR_H_

#include "common.h"
#include "MK60_FTM.h"






#define pwm_startup 130
#define speed_feed_K 123.3333333
#define speed_KP 1
#define pwm_up 500//400
#define pwm_cut 540//250

#define speed_goal_B 1.4
#define speed_range 0.3





















/////////////电机模块宏定义//////////
//电机端口定义
#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH5//主板1-驱动1//左轮
#define MOTOR4_PWM  FTM_CH6//主板2-驱动3//右轮
#define MOTOR_HZ    (8*1000)
//电机PID定义量
#define speed_s 50
#define speed_c 40
#define speed_ss 55
#define Mortor_kP 0.9
#define Mortor_kI 0.1
#define Mortor_kD 0.02


//////////////////////速度/////////////////////////////////
//#define speed_1 280
//#define speed_2 220
//#define speed_3 200
//#define speed_4 180


//#define speed_1 400
//#define speed_2 260
//#define speed_3 240
//#define speed_4 240

#define speed_1 320
#define speed_2 280
#define speed_3 260
#define speed_4 240

















extern int Mortor_Error_now;
extern int Mortor_Error_last;
extern int Mortor_Error_prev;

//////////////////电机变量定义//////
////速度采集
extern float speedL;
extern float speedR;
extern float speed_now;
extern float speed_err;
extern float pwm_KP_output;
extern float pwm_feed_output;
extern float pwm_output;
//期望速度
extern int16 speed_goal;
//电机误差定义//

//电机PD使用量定义//
extern float Mortor_out_P;
extern float Mortor_out_I;
extern float Mortor_out_D;
extern float Mortor_ADD;
extern float Mortor_add;

extern float Mortor_PID();
extern void Motor();
extern void speed_control();
extern void feedforward_calculate(float goal);
extern void speed_calculate(float goal);
extern void speed_calculate_B(float goal);
extern void speed_calculate_special(float goal);
//extern void feedforward_calculate_l(float goal);
//extern void speed_calculate_l(float goal);

#endif
