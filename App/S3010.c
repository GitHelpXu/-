#include "include.h"
#include "Camear_Lcd.h"
#include "S3010.h"
#include "ELE.h"
#include "flag.h"
#include "mpu6050.h"


float S3010_out_PC;
float S3010_out_IC;
float S3010_out_DC;
float S3010_ADDC;
float S3010_addC;
//舵机电磁
float S3010_out_PE;
float S3010_out_IE;
float S3010_out_DE;
float S3010_ADDE;
float S3010_addE;
//电机提前减速
float S3010_out_PM;
float S3010_out_IM;
float S3010_out_DM;
float S3010_ADDM;
float S3010_addM;
////////环岛//////
float S3010_out_PE_circle;
float S3010_out_IE_circle;
float S3010_out_DE_circle;
float S3010_ADDE_circle;
float S3010_addE_circle;

//舵机摄像头打脚
int16 VALUEC;
//舵机电磁打脚
int16 VALUEE;
//舵机提前减速
int16 VALUEM;
////环岛///////
int16 VALUEE_circle;
//////打脚锁定///////
int16 VALUEE_lock;

//最终打脚//
int16 VALUE;
int16 S3010_Error_nowC=0;
int16 S3010_Error_lastC=0;
int16 S3010_Error_prevC=0;
int16 S3010_Error_nowE=0;
int16 S3010_Error_lastE=0;
int16 S3010_Error_prevE=0;
int16 S3010_Error_nowM=0;
int16 S3010_Error_lastM=0;
int16 S3010_Error_prevM=0;


int16 S3010_Error_nowE_circle=0;
int16 S3010_Error_lastE_circle=0;
int16 S3010_Error_prevE_circle=0;
uint16 S3010_value=1525;


int VALUE_cross;
int VALUE_circle;


float asgle_straight;
/////////摄像头PD/////////////
float S3010_Camear_PD()
{
  CarPoint.x=/*center_line[40].x*/(center_line[39].x+center_line[40].x+center_line[41].x+center_line[42].x+center_line[43].x)/5;
  CarPoint.y=40;

  S3010_Error_nowC=CarPoint.x-/*center_line[40].x*/40;
  S3010_out_PC+=S3010_kP_C*(S3010_Error_nowC-S3010_Error_lastC);
  S3010_out_DC+=S3010_kD_C*(S3010_Error_nowC-2*S3010_Error_lastC+S3010_Error_prevC);
  S3010_addC=S3010_out_PC+S3010_out_DC;
  S3010_Error_prevC=S3010_Error_lastC;
  S3010_Error_lastC=S3010_Error_nowC;
  return S3010_addC;
}

///////////减速提前控制//////////////
float S3010_Camear_MotorPoint_PD()
{


  MotorPoint.x=/*center_line[40].x*/(center_line[15].x+center_line[16].x+center_line[17].x+center_line[18].x+center_line[19].x)/5;
  MotorPoint.y=15;

  S3010_Error_nowM=MotorPoint.x-/*center_line[40].x*/40;
  S3010_out_PM+=S3010_kP_C*(S3010_Error_nowM-S3010_Error_lastM);
  S3010_out_DM+=S3010_kD_C*(S3010_Error_nowM-2*S3010_Error_lastM+S3010_Error_prevM);
  S3010_addM=S3010_out_PM+S3010_out_DM;
  S3010_Error_prevM=S3010_Error_lastM;
  S3010_Error_lastM=S3010_Error_nowM;
  return S3010_addM;
}

///////////////电磁PD控制//////////////

//void MPU6050_Z_straight(){
//        MPU6050_GYRO_Z();
//        angle_straight=MPU6050_g_z*control_dt/1000.0f;     
//}




float S3010_ELE_PD(){
           //MPU6050_Z_straight();
           S3010_Error_nowE=par-0;
           S3010_out_PE+=S3010_kP_E*(S3010_Error_nowE-S3010_Error_lastE);
           S3010_out_DE+=S3010_kD_E*(S3010_Error_nowE-2*S3010_Error_lastE+S3010_Error_prevE);
           S3010_addE=S3010_out_PE+S3010_out_DE;
           S3010_Error_prevE=S3010_Error_lastE;
           S3010_Error_lastE=S3010_Error_nowE;
           return S3010_addE;
}

float S3010_ELE_circle_PD(){
              S3010_Error_nowE_circle=-ver;
              S3010_out_PE_circle+=S3010_kP_E*(S3010_Error_nowE_circle-S3010_Error_lastE_circle);
  /*S3010_out_IE+=S3010_kI_E*S3010_Error_nowE;*/
              S3010_out_DE_circle+=S3010_kD_E*(S3010_Error_nowE_circle-2*S3010_Error_lastE_circle+S3010_Error_prevE_circle);
              S3010_addE_circle=S3010_out_PE_circle+/*S3010_out_IE+*/S3010_out_DE_circle;
              S3010_Error_prevE_circle=S3010_Error_lastE_circle;
              S3010_Error_lastE_circle=S3010_Error_nowE_circle;
              return S3010_addE_circle;
}



void PRE(){
     if(VALUEC<=1365) VALUEC=1365;
     if(VALUEC>=1685) VALUEC=1685;
     
     if(VALUEE<=1365) VALUEE=1365;
     if(VALUEE>=1685) VALUEE=1685;
    
     if(VALUEM<=1365) VALUEM=1365;
     if(VALUEM>=1685) VALUEM=1685;
  
     if(VALUE_cross<=1365) VALUE_cross=1365;
     if(VALUE_cross>=1685) VALUE_cross=1685;
     
     if(VALUE<=1365) VALUE=1365;
     if(VALUE>=1660) VALUE=1685;
     
     
     if(VALUEE_circle<=1365) VALUEE_circle=1365;
     if(VALUEE_circle>=1685) VALUEE_circle=1685;
     
     if(VALUE_circle<=1400) VALUE_circle=1400;
     if(VALUE_circle>=1650) VALUE_circle=1650;
  
  
}

void S3010_control(){
//          float gain;
          S3010_ADDE=S3010_ELE_PD();
          //    S3010_ADDE_circle=S3010_ELE_circle_PD();
 //         gain=linechart_for_steer_gain(fabs(par));
  //        VALUEE=(int)(S3010_value-gain*S3010_ADDE);

          
          if(fabs(par)<3)
          VALUEE=(int)(S3010_value-9*3.2*S3010_ADDE);
          else  VALUEE=(int)(S3010_value-9*fabs(par)/1.2*S3010_ADDE);
          
          //    VALUEE_circle=(int)(S3010_value-50*S3010_ADDE_circle);
          PRE();
          if(signal.circle_S3010==0&&signal.cross_S3010==0){
          ftm_pwm_duty(S3010_FTM, S3010_CH,VALUEE);
                 VALUEE_lock=VALUEE;//环岛锁定打脚
             }
  //   if(signal.circle_M==1)
  //        ftm_pwm_duty(S3010_FTM, S3010_CH,VALUEE_circle);
  ////    
        
        
}









//void S3010_control()
//{
//  if(signal.cross==1||signal.circle==1||signal.ramp==1);
//  else
//  {
//    S3010_ADDE=S3010_ELE_PD();
//    VALUE=(int)(S3010_value-9*3.2*S3010_ADDE);
//    PRE();
//    ftm_pwm_duty(S3010_FTM,S3010_CH,VALUE);
//  }
//}
//void S3010_control_special()
//{
//   S3010_ADDE=S3010_ELE_PD();
//   VALUE=(int)(S3010_value-9*3.2*S3010_ADDE);
//   PRE();
//   ftm_pwm_duty(S3010_FTM,S3010_CH,VALUE);
//}

//void S3010_clear_F(uint8 flag)
//{
//  if(flag==1)
//  {
//    S3010_Error_nowE=0;
//    S3010_Error_lastE=0;
//    S3010_Error_prevE=0;
//    signal.S3010_clear=0;
//  }
//}
