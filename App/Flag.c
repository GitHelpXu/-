#include "include.h"
#include "flag.h"
#include "S3010.h"
#include "Mortor.h"
#include "Camear_Lcd.h"
#include "ELE.h"
#include "angle.h"
#include "mpu6050.h"

int16_t count_number;



Flag signal={0,0,0,0,0,0,0,0,0,0,0,0};






uint8 uls_number=0;

uint8 circle_number=0;






//////////////避障模块/////////////////

/////////舵机PD///////
float angle_cross;
float angle_error_now_cross=0.0;
float angle_error_last_cross=0.0;
float angle_error_prev_cross=0.0;
float angle_out_P_cross=0.0;
float angle_out_D_cross=0.0;
float angle_add_cross=0.0;
float S3010_ADD_cross;
///////超声波/////////
float ULS_time=0.0;
float ULS_distance=0.0;




float distance=0.0;
float protect_distance=0.0;










/////////////////////环岛模块////////////////

float circle_angle_goal;
///////////舵机PD////////////
float angle_circle;
float angle_error_now_circle=0.0;
float angle_error_last_circle=0.0;
float angle_error_prev_circle=0.0;
float angle_out_P_circle=0.0;
float angle_out_D_circle=0.0;
float angle_add_circle=0.0;
float S3010_ADD_circle;



uint8 flag_circle_counter=0;
float angle_circle;










/************************避障摄像头辅助****************************/
//////////超声波启动标志位函数////
//void flag_uls_F(){
//           uint8 i,j,m,n,p,q,l,flag_B,flag_W,flag_BW,flag_saltus_;
//           uint8 counter_B=0,counter_W=0,counter_BW=0;
//           int8 temp=-1;
//           for(i=20;i<24;i++)
//             for(j=38;j<43;j++){
//               if(findpix(j,i)==1)
//                 counter_B++;
//             }
//           if(counter_B==20)
//             flag_B=1;
//           else flag_B=0;
//           for(m=30;m<34;m++)
//             for(n=38;n<43;n++){
//               if(findpix(n,m)==0)
//                 counter_W++;
//             }
//           if(counter_W==20)
//             flag_W=1;
//           else flag_W=0;
//           flag_BW=flag_B&&flag_W;
//           for(l=59;l>=4;l--){
//             if(width[l]-width[l-1]>20&&width[l-2]==0&&width[l-3]==0&&width[l-4]==0&&center_line[l].x>25&&center_line[l].x<55){
//               temp=l-1;
//               break;
//             }
//           }
//           if(temp==-1)
//             flag_saltus=0;
//           if(temp>=15)
//             flag_saltus=1;
//           else flag_saltus=0;
//           signal.uls=flag_BW&&flag_saltus;
//}
/////////////超声波工作函数/////////
//void uls(uint8 flag){
//           if(flag==1){
//             gpio_set(PTE11,1);
//             pit_delay_us(PIT2,12);
//             gpio_set(PTE11,0);
//
//           }
//}
////////////////避障转角PD函数/////////
//float cross_PD(float goal){
//             angle_error_now=goal-angle_cross;
//             angle_out_P+=cross_kP*(angle_error_now-angle_error_last);
//             angle_out_D+=cross_kD*(angle_error_now-2*angle_error_last+angle_error_prev);
//             angle_add=angle_out_P+angle_out_D;
//             angle_error_prev=angle_error_last;
//             angle_error_last=angle_error_now;
//             return angle_add;
//}
/////////////避障标志位函数////////
//void flag_cross_F(){
//       uls(signal.uls);
//       ULS_distance=340*ULS_time/1000.0/2;
//       if(ULS_distance>10&&ULS_distance<55)
//         signal.cross=1;
//}
//////////////避障控制函数//////////
/////////右转/////
//void cross_control(uint8 flag){
//        if(flag==1){
//          while(distance<d1){
//            speed_calculate(1.2)
//          }
//          signal.cross_M=1;
//          signal.cross_S3010=1;
//          while(distance>=d1&&distance<d2){
//            S3010_ADD_cross=cross_PD(/*50*/turn_a1);
//            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
//            PRE();
//            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
//            speed_calculate(1.4);
//          }
//          while(distance>=d2&&distance<d3){
//            S3010_ADD_cross=cross_PD(/*-45*/turn_a2);
//            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
//            PRE();
//            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
//            speed_calculate(1.4);
//          }
//          while(distance>=d3&&distance<d4){
//            S3010_ADD_cross=cross_PD(/*-30*/turn_a3);
//            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
//            PRE();
//            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
//            speed_calculate(1.4);
//          }
//          signal.cross_S3010=0;
//          signal.cross=0;
//          signal.cross_M=0;
//          ULS_time=0;
//          ULS_distance=0.0;
//          distance=0.0;
//          angle_cross=0.0;
//          uls_number++;
//        }
//}
//
//
//
//
//
//////////////左转////////
////void cross_control(uint8 flag){
////        if(flag==1){
////          while(distance<d1){
////            speed_calculate(1.2)
////          }
////          signal.cross_M=1;
////          signal.cross_S3010=1;
////          while(distance>=d1&&distance<d2){
////            S3010_ADD_cross=cross_PD(/*50*/-turn_a1);
////            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
////            PRE();
////            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
////            speed_calculate(1.4);
////          }
////          while(distance>=d2&&distance<d3){
////            S3010_ADD_cross=cross_PD(/*-45*/-turn_a2);
////            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
////            PRE();
////            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
////            speed_calculate(1.4);
////          }
////          while(distance>=d3&&distance<d4){
////            S3010_ADD_cross=cross_PD(/*-30*/-turn_a3);
////            VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
////            PRE();
////            ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
////            speed_calculate(1.4);
////          }
////          signal.cross_S3010=0;
////          signal.cross=0;
////          signal.cross_M=0;
////          ULS_time=0;
////          ULS_distance=0.0;
////          distance=0.0;
////          angle_cross=0.0;
////          uls_number++;
////        }
////}
//    
    
    
    
    
    
    
////////////////避障左转  
#define d1_L 0.0068            //控制距离
#define d2_L 0.76             //控制距离
#define d3_L 1.7
#define ele_L 2400
              
                           
#define turn_a1_L  55
#define turn_a2_L  70           
       











////////////避障右转
#define d1_R 0.0068             //控制距离
#define d2_R 0.86             //控制距离
#define d3_R 1.8             //控制距离  
#define ele_R 2400              //电磁检测值

#define turn_a1_R  55           //外打角         
#define turn_a2_R  65           //内打角   







/****************************避障无辅助************************/

///////////超声波工作函数/////////
void uls(){
  static uint16_t counter=0;
  if(++counter>=ULS_T/5) counter=0;
  if(counter==0) gpio_set(PTE11,1);
  if(counter==1) gpio_set(PTE11,0);             
}
//////////////避障转角PD函数/////////
float cross_PD(float goal){
         angle_error_now_cross=goal-angle_cross;
         angle_out_P_cross+=cross_kP*(angle_error_now_cross-angle_error_last_cross);
         angle_out_D_cross+=cross_kD*(angle_error_now_cross-2*angle_error_last_cross+angle_error_prev_cross);
         angle_add_cross=angle_out_P_cross+angle_out_D_cross;
         angle_error_prev_cross=angle_error_last_cross;
         angle_error_last_cross=angle_error_now_cross;
         return angle_add_cross;
}
///////////避障标志位函数////////
void flag_cross_F(){

        if(ULS_distance>10&&ULS_distance<65)
          signal.cross=1;
        else signal.cross=0;
}
////////////避障控制函数//////////
/////////右转/////////
void cross_control(uint8 flag){
             if(flag==1){
               while(distance<d1_R){
                 speed_calculate_B(1.2);
               }
               signal.cross_M=1;
               signal.cross_S3010=1;
               while(distance>=d1_R&&distance<d2_R){
                 S3010_ADD_cross=cross_PD(/*50*/turn_a1_R);
                 VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
                 PRE();
                 ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
                 speed_calculate_special(1.4);
               }
               angle_cross=0.0;
               angle_error_now_cross=angle_error_last_cross=angle_error_prev_cross=0;
               while(distance>=d2_R&&distance<d3_R){
                 S3010_ADD_cross=cross_PD(/*-45*/turn_a2_R);
                 VALUE_cross=(int)(S3010_value+S3010_ADD_cross*5);
                 PRE();
                 ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
                 speed_calculate_special(1.4);
               }
               angle_cross=0.0;
               angle_error_now_cross=angle_error_last_cross=angle_error_prev_cross=0;
               while(var[1]<=ele_R){
                 ftm_pwm_duty(S3010_FTM, S3010_CH,S3010_value);
               speed_calculate_special(1.2);
               }
               signal.cross_S3010=0;
               signal.cross=0;
               signal.cross_M=0;
               ULS_time=0;
               ULS_distance=0.0;
               distance=0.0;
               uls_number++;
             }   
}




//////////////左转////////
//void cross_control(uint8 flag){
//             if(flag==1){
//               while(distance<d1_L){
//                 speed_calculate_B(1.2);
//               }
//               signal.cross_M=1;
//               signal.cross_S3010=1;
//               while(distance>=d1_L&&distance<d2_L){
//                 S3010_ADD_cross=cross_PD(/*50*/turn_a1_L);
//                 VALUE_cross=(int)(S3010_value+S3010_ADD_cross*5);
//                 PRE();
//                 ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
//                 speed_calculate_special(1.4);
//               }
//               angle_cross=0.0;
//               angle_error_now_cross=angle_error_last_cross=angle_error_prev_cross=0;
//               while(distance>=d2_L&&distance<d3_L){
//                 S3010_ADD_cross=cross_PD(/*-45*/turn_a2_L);
//                 VALUE_cross=(int)(S3010_value-S3010_ADD_cross*5);
//                 PRE();
//                 ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_cross);
//                 speed_calculate_special(1.4);
//               }
//               angle_cross=0.0;
//               angle_error_now_cross=angle_error_last_cross=angle_error_prev_cross=0;
//               while(var[3]<=ele_L){
//                 ftm_pwm_duty(S3010_FTM, S3010_CH,1520);
//               speed_calculate_special(1.2);
//               }
//               signal.cross_S3010=0;
//               signal.cross=0;
//               signal.cross_M=0;
//               ULS_time=0;
//               ULS_distance=0.0;
//               distance=0.0;
//               uls_number++;
//             }   
//}














/////////////入环PD/////////////
float cirlce_PD(float goal){
          angle_error_now_circle=goal-angle_circle;
          angle_out_P_circle+=circle_kP*(angle_error_now_circle-angle_error_last_circle);
          angle_out_D_circle+=circle_kD*(angle_error_now_circle-2*angle_error_last_circle+angle_error_prev_circle);
          angle_add_circle=angle_out_P_circle+angle_out_D_circle;
          angle_error_prev_circle=angle_error_last_circle;
          angle_error_last_circle=angle_error_now_circle;
          return angle_add_circle;
}



#define  distance_circle_L 0.56              //1m-3m可用数据  //0.54  0.58 
#define  angle_circle_in_L 40                //1m-3m可用数据   
#define  angle_circle_goal_L 35              //1m-3m可用数据   
#define  angle_circle_lock_L 280             //1m-3m可用数据   
#define  steer_lock_L 1685                   //1m-3m可用数据   
#define  angle_circle_out_L 335              //1m-3m可用数据   



#define  distance_circle_R 0.51   //1.5m-3m可用数据 //             0.54  //1m可用数据
#define  angle_circle_in_R 40     //1.5m-3m可用数据 //             40    //1m可用数据
#define  angle_circle_goal_R 35   //1.5m-3m可用数据 //             35    //1m可用数据
#define  angle_circle_lock_R 280  //1.5m-3m可用数据 //             280   //1m可用数据
#define  steer_lock_R 1365        //1.5m-3m可用数据 //             1365  //1m可用数据
#define  angle_circle_out_R 335   //1.5m-3m可用数据 //             335   //1m可用数据

                                              


//////////////////////////////////////////////////////环岛//////////////////////////////////////////////////
////////////环岛标志位函数//////////
uint8 circle_L=0,circle_R=0;
void flag_circle_F(){
  if(((var[0]>=2500&&var[3]>=2800)||(var[3]>=2500&&var[0]>=2800))&&(fabs(var[1]-var[2])>1200)){
           signal.circle=1;
           if(var[0]>=2500&&var[3]>=2800) 
           signal.circle_R=1;
         if(var[3]>=2500&&var[0]>=2800) 
           signal.circle_L=1;
  }
  else signal.circle=0;       
}






///////////环岛控制函数//////////////
uint8_t once_flag = 0;
int16_t TIMER,LAS_TIMER;
void circle_control(uint8 flag){
                if(flag==1){
                  //flag_circle_counter++;
                // if(flag_circle_counter%2==1){/////开始控制
                   if(signal.circle_R==1){////右入环岛
                     /************阶段一*********///极减速
                    while(distance<=distance_circle_R)
                      speed_calculate_B(1.2);
                    /************中间过渡一**********/
                    signal.circle_M=1;
                    signal.circle_S3010=1;
                    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0.);
                    /************阶段二*************///舵机打入环角
                    while(fabs(angle_circle)<angle_circle_in_R)
                    {
                      S3010_ADD_circle=cirlce_PD(angle_circle_goal_R); 
                      VALUE_circle=(int)(1425-1.4*(160-S3010_ADD_circle*5));
                      PRE();
                      ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_circle);
                      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,220);
                      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,220);
                    }
                    /*************中间过渡二*******/
                    signal.circle_R=0;
                    angle_error_now_circle=0.0;
                    angle_error_last_circle=0.0;
                    angle_error_prev_circle=0.0;
                    signal.circle_S3010=0;
                    /***********阶段三**********///环内正常行驶
                    while(fabs(angle_circle)>=angle_circle_in_R&&fabs(angle_circle)<angle_circle_lock_R){
                      speed_calculate_special(1.4);
                    }
                    /**********中间过渡三*******/
                    signal.circle_S3010=1;
                    /**********阶段四***********///锁住出环角
                    while(fabs(angle_circle)>=angle_circle_lock_R&&fabs(angle_circle)<angle_circle_out_R){
                      ftm_pwm_duty(S3010_FTM, S3010_CH,steer_lock_R);
                      speed_calculate_special(1.4);
                    }
                    /**********阶段五*********///舵机回正
                    if(angle_circle>=angle_circle_out_R){
                      if(once_flag==0){ 
                        LAS_TIMER=count_number;
                        once_flag = 1;
                      }
                      while(count_number-LAS_TIMER<=60)
                      {
                        ftm_pwm_duty(S3010_FTM, S3010_CH,S3010_value);
                        speed_calculate_special(1.4);
                      }
                      once_flag = 0;
                      signal.circle_S3010=0;
                    }
                    /**********中间过渡四********/
                    signal.circle_protect=1;
                    signal.circle=0;
                    signal.circle_M=0;
                    angle_circle=0.0;
                    distance=0.0;
                    circle_number++;
                    /*****阶段六***********///防触发标志位
                    while(protect_distance<5);
                    signal.circle_protect=0;
                    protect_distance=0;
                   }
                   
                   
                   
                   
                   if(signal.circle_L==1){//左入环岛
                     /***********阶段一*********///极减速
                     while(distance<=distance_circle_L)
                    {
                      speed_calculate_B(1.2);
                    }
                    /************中间过渡一**********/
                    signal.circle_M=1;
                    signal.circle_S3010=1;
                    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0.);
                    /************阶段二****************///舵机打入环角
                    while(fabs(angle_circle)<angle_circle_in_L)
                    {
                      S3010_ADD_circle=cirlce_PD(angle_circle_goal_L);
                      VALUE_circle=(int)(1575+1.5*(160-S3010_ADD_circle*5));
                      PRE();
                      ftm_pwm_duty(S3010_FTM, S3010_CH,VALUE_circle);
                      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,220);
                      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,220);
                    }
                    /************中间过渡二**********/
                    signal.circle_L=0;
                    signal.circle_R=0;
                    angle_error_now_circle=0.0;
                    angle_error_last_circle=0.0;
                    angle_error_prev_circle=0.0;
                    signal.circle_S3010=0;
                    /************阶段三**********///环内正常行驶
                    while(fabs(angle_circle)>=angle_circle_in_L&&fabs(angle_circle)<angle_circle_lock_L){
                      speed_calculate_special(1.4);
                    }
                    /************中间过渡三**********/
                    signal.circle_S3010=1;
                    /***********阶段四***********///锁住出环角
                    while(fabs(angle_circle)>=angle_circle_lock_L&&fabs(angle_circle)<angle_circle_out_L){
                      ftm_pwm_duty(S3010_FTM, S3010_CH,steer_lock_L);
                      speed_calculate_special(1.4);
                    }
                    /**********阶段五***********///舵机回正
                    if(angle_circle>=angle_circle_out_L){
                      if(once_flag==0){ 
                        LAS_TIMER=count_number;
                        once_flag = 1;
                      }
                      while(count_number-LAS_TIMER<=60)
                      {
                        ftm_pwm_duty(S3010_FTM, S3010_CH,S3010_value);
                        speed_calculate_special(1.4);
                      }
                      once_flag = 0;
                      signal.circle_S3010=0;
                    }
                    /************中间过渡四**********/
                    signal.circle_protect=1;
                    signal.circle=0;
                    signal.circle_M=0;
                    angle_circle=0.0;
                    distance=0.0;
                    circle_number++;
                    /***********阶段六***************///防出发标志位
                    while(protect_distance<5);
                    signal.circle_protect=0;
                    protect_distance=0;
                   }
                  }
                
}















//////////////////////坡道模块///////////
//void flag_ramp_F()
//{
//  if(filted.roll>=5)
//    signal.ramp=1;
//  else signal.ramp=0;
//}
//
//
//
//
//
//
//void ramp_control(uint8 flag)
//{
//  if(flag==1)
//  {
//    while(distance<1.4)
//    {
//      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,380);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,380);
//    }
//    while(distance>=1.4&&distance<1.8)
//    {
//      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,100);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,100);
//    }
//    while(distance>=1.8&&distance<2.4)
//    {
//      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,200);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,200);
//    }
//    signal.ramp=0;
//    distance=0.0;
//  }
//}

void flag_ramp_F()
{
  if(MPU6050_a_y>=450)//(filted.roll>=-25&&filted.roll<=-20)
    signal.ramp=1;
  else signal.ramp=0;
}


void ramp_control(uint8 flag)
{
  if(flag==1)
  {
    while(filted.roll>=-30&&filted.roll<=-20)
    {
   // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
   // ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
   // ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,300);
   // ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,300);
      speed_calculate(200);
    }
    
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
    
    
    signal.ramp=0;
  }
}









