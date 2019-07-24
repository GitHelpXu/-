#ifndef _FLAG_H_
#define _FLAG_H_



#include "common.h"


////////////////////////////////////////////////////////////////////////////避障宏定义/////////////////////////////////////////////////////////////
#define cross_kP 0.55
#define cross_kD 0.15
/////////////////////轻////////////////
//#define d1 0.0           //控制距离
//#define d2 0.4           //控制距离
//#define d3 1.0           //控制距离 
//#define d4 1.2           //控制距离 
//#define d5 1.4           //控制距离 
//#define turn_a1 55       //控制角度 
//#define turn_a2 -50      //控制角度 
//#define turn_a3 -50      //控制角度
//#define turn_a4 45       //控制角度
//////////////重////////////
#define d1 0.0068            //控制距离
#define d2 0.5644         //控制距离
#define d3 1.292     //控制距离                  
#define d4 1.428   //控制距离                  
//#define d5 1.4+0.1+0.2         //控制距离                  
/////////////#define turn_a1  55//55           LEFT        
/////////////#define turn_a2  65            
/////////////#define turn_a3  20//            
/////////////#define turn_a4  /*45*/35        

#define turn_a1  55//55                
#define turn_a2  65           
#define turn_a3  20//            
#define turn_a4  /*45*/35        




///////////////////////////////////////////////////环岛宏定义///////////////////
#define circle_kP 0.55
#define circle_kD 0.15


////////////坡道宏定义////////////////   
#define angle_IX1 20
#define angle_IX2 -10
#define angle_IX3 -10
#define angle_IX4 10




#define ULS_T 60




typedef struct
{
  uint8 uls;
  uint8 ramp;
  uint8 circle;
  uint8 circle_L;
  uint8 circle_R;
  uint8 circle_M;
  uint8 circle_S3010;
  uint8 circle_protect;
  uint8 cross;
  uint8 cross_M;
  uint8 cross_S3010;
  uint8 reed;
}Flag;

extern Flag signal;
//////////////避障模块/////////////////

/////////路障舵机PD///////
extern int16_t count_number;

extern float angle_cross;
extern float angle_error_now_cross;
extern float angle_error_last_cross;
extern float angle_error_prev_cross;
extern float angle_out_P_cross;
extern float angle_out_D_cross;
extern float angle_add_cross;
extern float S3010_ADD_cross;
///////超声波/////////
extern float ULS_time;
extern float ULS_distance;
extern uint8 uls_number;
/////////////////////////////////环岛模块/////////////////


extern float circle_angle_goal;
/////////环岛舵机PD////////
extern float angle_circle;
extern float angle_error_now_circle;
extern float angle_error_last_circle;
extern float angle_error_prev_circle;
extern float angle_out_P_circle;
extern float angle_out_D_circle;
extern float angle_add_circle;
extern float S3010_ADD_circle;

extern uint8 flag_circle_counter;
extern float angle_circle;
 
extern float distance;
extern float protect_distance;

extern int16_t  brake_number;

extern uint8 circle_number;

////////////////////////////避障/////////////////////
//void uls(uint8 flag);
//void porte_handler(void);
float cross_PD(float goal);
void uls();
void flag_uls_F();
void flag_cross_F();
void cross_control(uint8 flag);
////////////////////////////环岛/////////////
void flag_circle_F();
float cirlce_PD(float goal);
void circle_control(uint8 flag);













#endif
