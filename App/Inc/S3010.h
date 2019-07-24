#ifndef _S3010_H_
#define _s3010_H_

#include "MK60_FTM.h"
#include "common.h"


#define S3010_FTM   FTM3
#define S3010_CH    FTM_CH0
#define S3010_HZ    (100)
////���PD������
//CamearPD�궨��
#define S3010_kP_C 0.5
#define S3010_kD_C 0.15
//���PID�궨��
#define S3010_kP_E /*0.45*//*0.65*/                     0.45
#define S3010_kI_E 0.001
#define S3010_kD_E /*0.55*//*0.02*//*0.01*/             0.01

//////////////////////�����������////////////////////
//�������//

//�������ͷ
extern float S3010_out_PC;
extern float S3010_out_IC;
extern float S3010_out_DC;
extern float S3010_ADDC;
extern float S3010_addC;
//������
extern float S3010_out_PE;
extern float S3010_out_IE;
extern float S3010_out_DE;
extern float S3010_ADDE;
extern float S3010_addE;
//�����ǰ����
extern float S3010_out_PM;
extern float S3010_out_IM;
extern float S3010_out_DM;
extern float S3010_ADDM;
extern float S3010_addM;
////////����//////
extern float S3010_out_PE_circle;
extern float S3010_out_IE_circle;
extern float S3010_out_DE_circle;
extern float S3010_ADDE_circle;
extern float S3010_addE_circle;
//�������ͷ���
extern int16 VALUEC;
//�����Ŵ��
extern int16 VALUEE;
//�����ǰ����
extern int16 VALUEM;

extern int16 VALUEE_circle;
//������
extern int16 VALUE;
//�����ֵ����//
extern int16 S3010_Error_nowC;
extern int16 S3010_Error_lastC;
extern int16 S3010_Error_prevC;
extern int16 S3010_Error_nowE;
extern int16 S3010_Error_lastE;
extern int16 S3010_Error_prevE;
extern int16 S3010_Error_nowM;
extern int16 S3010_Error_lastM;
extern int16 S3010_Error_prevM;
extern int16 S3010_Error_nowE_circle;
extern int16 S3010_Error_lastE_circle;
extern int16 S3010_Error_prevE_circle;
extern uint16 S3010_value;

extern int VALUE_cross;

extern int VALUE_circle;


extern int16 VALUEE_lock;


extern float angle_straight;


extern float S3010_Camear_PD();
extern float S3010_Camear_MotorPoint_PD();



///extern MPU6050_Z_straight();
extern float S3010_ELE_PD();
extern void PRE();
extern void S3010_control();
extern void S3010_control_special();
//extern void S3010_clear_F(uint8 flag);


#endif