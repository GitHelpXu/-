#ifndef _CAMEAR_LCD_H_
#define _CAMEAR_LCD_H_


#include "include.h"
#include "common.h"

//////////////////////////Camear&LCD宏定义/////////////////////
//找像素
#define findpix(x,y)        ((imgbuff[(y)*10+(x)/8]>>(7-(x)%8))&0x01)//预编译函数，提取像素点坐标
//写像素
#define writepix(x,y,data)  (data?(imgbuff[(y)*10+(x)/8]|=(uint8_t)0x01<<(7-(x)%8)):(imgbuff[(y)*10+(x)/8]&=~((uint8_t)0x01<<(7-(x)%8))))
#define LCD_W 80
#define LCD_H 60
#define count 5

////////////////////////Camear&LCD变量定义/////////////////
////定义存储接收图像的数组
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W];
//暂存前一次中点的列

//左右黑点列
extern int8_t left_dot,right_dot;
//图像左上角的位置

//数字显示区域

//存放中线、左右黑线
//////////////////////////////////////////////////////////////////////////变量体///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Camear&LCD变量定义/////////////////
////定义存储接收图像的数组
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W];
////暂存前一次中点的列
extern uint8_t mid_dot;
extern uint8_t Mid_dot;
////左右黑点列
extern int8_t left_dot,right_dot;
////图像左上角的位置
extern Site_t site;
////数字显示区域
extern Site_t site_1;
extern Site_t site_2;
//存放中线、左右黑线
extern Site_t center_line[60];
extern Site_t left_line[60];
extern Site_t right_line[60];
//舵机参考的位置
extern Site_t CarPoint;
//电机减速参考位置
extern Site_t MotorPoint;
//摄像头获得图片大小
extern Size_t imgsize;
//显示区域图像大小
extern Size_t size;
////最小二乘法变量定义/
extern float A;
extern float B;
extern int sum_X;
extern int sum_Y;
extern float sum_FZ;
extern float sum_FM;
extern int x_MIX;
extern int y_MIX;
extern float aver_X;
extern float aver_Y;
//最小二乘法补线
extern int8 K_l;
extern int8 K_r;
extern int8 B_l;
extern Site_t next_l;
extern int8 B_r;
extern Site_t next_r;
//避障区域标志位
extern uint8 flag_cross;
//调试数组
extern uint8 Z_O[60][80];
///赛道宽度
extern uint8 width[60];
//避障跳变行标志位
extern uint8 flag_saltus;

extern int16_t find_center_dot(uint8_t rol,uint8_t mid_dot);//基础找中点
extern void find_center_line();///基础找中线
extern void fill_l();/////左补线
extern void fill_r();////右补线
extern void ff_line();



extern void ten_fill_L();//线性十字补线
extern void ten_fill_R();//线性十字补线
extern void Z_OF();//////调试函数
//extern uint8 flag_cross_F();////路障标志位判定
//extern uint8 flag_saltus_F();////路障跳变行判定



#endif