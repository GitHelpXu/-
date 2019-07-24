#ifndef _CAMEAR_LCD_H_
#define _CAMEAR_LCD_H_


#include "include.h"
#include "common.h"

//////////////////////////Camear&LCD�궨��/////////////////////
//������
#define findpix(x,y)        ((imgbuff[(y)*10+(x)/8]>>(7-(x)%8))&0x01)//Ԥ���뺯������ȡ���ص�����
//д����
#define writepix(x,y,data)  (data?(imgbuff[(y)*10+(x)/8]|=(uint8_t)0x01<<(7-(x)%8)):(imgbuff[(y)*10+(x)/8]&=~((uint8_t)0x01<<(7-(x)%8))))
#define LCD_W 80
#define LCD_H 60
#define count 5

////////////////////////Camear&LCD��������/////////////////
////����洢����ͼ�������
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W];
//�ݴ�ǰһ���е����

//���Һڵ���
extern int8_t left_dot,right_dot;
//ͼ�����Ͻǵ�λ��

//������ʾ����

//������ߡ����Һ���
//////////////////////////////////////////////////////////////////////////������///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Camear&LCD��������/////////////////
////����洢����ͼ�������
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W];
////�ݴ�ǰһ���е����
extern uint8_t mid_dot;
extern uint8_t Mid_dot;
////���Һڵ���
extern int8_t left_dot,right_dot;
////ͼ�����Ͻǵ�λ��
extern Site_t site;
////������ʾ����
extern Site_t site_1;
extern Site_t site_2;
//������ߡ����Һ���
extern Site_t center_line[60];
extern Site_t left_line[60];
extern Site_t right_line[60];
//����ο���λ��
extern Site_t CarPoint;
//������ٲο�λ��
extern Site_t MotorPoint;
//����ͷ���ͼƬ��С
extern Size_t imgsize;
//��ʾ����ͼ���С
extern Size_t size;
////��С���˷���������/
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
//��С���˷�����
extern int8 K_l;
extern int8 K_r;
extern int8 B_l;
extern Site_t next_l;
extern int8 B_r;
extern Site_t next_r;
//���������־λ
extern uint8 flag_cross;
//��������
extern uint8 Z_O[60][80];
///�������
extern uint8 width[60];
//���������б�־λ
extern uint8 flag_saltus;

extern int16_t find_center_dot(uint8_t rol,uint8_t mid_dot);//�������е�
extern void find_center_line();///����������
extern void fill_l();/////����
extern void fill_r();////�Ҳ���
extern void ff_line();



extern void ten_fill_L();//����ʮ�ֲ���
extern void ten_fill_R();//����ʮ�ֲ���
extern void Z_OF();//////���Ժ���
//extern uint8 flag_cross_F();////·�ϱ�־λ�ж�
//extern uint8 flag_saltus_F();////·���������ж�



#endif