/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
//功能头文件   
#include "ELE.h"
#include "Camear_Lcd.h"
#include "S3010.h"
#include "Mortor.h"
//#include "MPU6050.h"
#include "SOFT_IIC.h"
#include "mpu6050.h"
#include "angle.h"
#include "flag.h"








   
////////////坡道////////////////   
#define angle_IX1 20
#define angle_IX2 -10
#define angle_IX3 -10
#define angle_IX4 10
   
  



   
   
   








uint16 counter_handler;
////坡道标志位///
uint8 flag_ramp=0;

////功能选择
int choice=0;





















////////////中断宏定义///////////
#define control_dt 5.0
































//////////////////////////////////////////////////////////////////////////函数体////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////中断服务数声明///////
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void sendimg(void *imgaddr, uint32 imgsize);
void PIT0_IRQHandler();
void porte_handler(void);
void portc_handler(void);

//////////初始化函数////////////
void init(){
              IIC_init();
              MPU6050_init();
              angle_init();

//////////////舵机初始化
              ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,1500);
//////////////电磁模块初始化
              adc_init(AMP1);
              adc_init(AMP2);
              adc_init(AMP3);
              adc_init(AMP4);
//////////////电机初始化
              ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,0);
              ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,0);
              ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);
              ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);
//////////////正交解码初始化
              ftm_quad_init(FTM2);//左
              ftm_quad_init(FTM1);//右
//////////////超声波管脚初始化              
              gpio_init(PTE11,GPO,0);//发
              gpio_init(PTE12,GPI,0);//收
//////////////干黄管管脚初始化 
              gpio_init(PTC16,GPI,1);
//////////////外部中断
              port_init_NoALT(PTE12,IRQ_FALLING|IRQ_RISING);
              port_init_NoALT(PTC16,IRQ_FALLING|PF|ALT1);   
//////////////配置中断服务函数
              //摄像头中断
              set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
              set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
              //定时器中断
              pit_init_ms(PIT0,(uint16_t)control_dt);                 //初始化PIT0，定时时间为： 5ms
              set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
              enable_irq (PIT0_IRQn);
              //超声波中断
              set_vector_handler(PORTE_VECTORn,porte_handler);
              enable_irq(PORTE_IRQn);
              //干簧管中断
              set_vector_handler(PORTC_VECTORn,portc_handler);
              enable_irq(PORTC_IRQn);
//////////////设置中断优先级
            NVIC_SetPriorityGrouping(3);
            NVIC_SetPriority(PORTC_VECTORn,0);
            NVIC_SetPriority(PORTE_VECTORn,1);
            NVIC_SetPriority(PIT0_VECTORn,2);
          //NVIC_SetPriority(PORTA_VECTORn,3);
          //NVIC_SetPriority(DMA0_VECTORn,4);
              
}


















float roll_init,roll_last,roll_now,roll_err,roll_sum;
void MPU6050_X(){
        angle_get();
        //angle_resume();
        IMU_update();
        if(counter_handler<=1000){
          roll_sum+=filted.roll;
          roll_init=roll_sum/counter_handler;
          // roll_last=filted.roll;
        }
        else{
          roll_now=filted.roll;
          roll_err=roll_now-roll_init;
          // roll_last=roll_now;
        } 
}



void MPU6050_Z(){
        float angle_cross_temp=0.0,angle_circle_temp=0.0;
        MPU6050_GYRO_Z();
        //angle_get();
        if(signal.cross_M==1){
          angle_cross_temp=fabs(MPU6050_g_z)*control_dt/1000.0f;
          angle_cross+=angle_cross_temp*0.4931506849315068;
        }
        if(signal.circle_M==1){
          angle_circle_temp=fabs(MPU6050_g_z)*control_dt/1000.0f;
          angle_circle+=angle_circle_temp*0.4931506849315068*2;
        }
}












void porte_handler(void){
         uint8 n=12;
         if(PORTE_ISFR & (1<<n)){
           PORTE_ISFR=(1<<n);
    /////////////
           //if(counter_handler>100){
           if(gpio_get(PTE12)==1){
             pit_time_start(PIT2);
           }
           else{
             //ULS_time=pit_time_get_us(PIT2);
             ULS_distance=(uint32)(pit_time_get_us(PIT2)/10000.0f*340.0f/2);
           }
            
           
          //}
       }
}














uint32_t power_up_time_ms=0;
uint8 flag_end=0;
void portc_handler(void){
         uint8 n=16;
         if(PORTC_ISFR & (1<<n)){
           PORTC_ISFR=(1<<n);
           /////////////
          if(power_up_time_ms/1000.0f>10){
            flag_end=1;
            speed_calculate_B(0);
            speed_calculate_B(0);
          while(1){
            ELE();
            S3010_control();
            ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
            ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
            ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
            ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
          }
         }
       }
}








void cross_mod(){
       flag_cross_F();
       cross_control(signal.cross);
}


void circle_mod(){
      flag_circle_F();
      circle_control(signal.circle);
}


void ramp_mod(){
      flag_ramp_F();
      ramp_control(signal.ramp);
}






uint8 flag_end_speed=0;
void stop_mod()
{
  if(counter_handler>1000){
    if(varE[0]+varE[1]+varE[2]+varE[3]<=15){
      flag_end_speed=1;
      speed_calculate_B(0);
      speed_calculate_B(0);
      while(1)
      {
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);  //左前
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);  //右前
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);  //左后
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);  //右后
      }
    }
  }
}













uint32 ll,nn;
float check_distance=0.0;

//////////////////////////////////////////////////////////////////////////////////主函数///////////////////////////////////////////////////////////////////////////////////////////////


void  main(void){
  
  
      systick_delay_ms(300);
      init();

      size.H = 60;
      size.W = 80;

 
    
   

   while(1){
           if(uls_number==0)
             cross_mod();
           if(check_distance>=10)
              circle_mod();
             stop_mod();
    }
}



/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}



uint16 pwm_L,pwm_R;
float PWM_K;
float acc_car_z,acc_car_y,acc_init_z,acc_init_y,acc[1000];
/////自定义中断////
float x_l,x_n,x_ga;
//uint16 counter_handler;
uint16 it_time_us,it_time_us_max;
uint32 s0=0,s1=0,s2=0,s3=0;
void PIT0_IRQHandler()
{
    PIT_Flag_Clear(PIT0);
    power_up_time_ms+=5;
    pit_time_start(PIT1);
    if(uls_number==0)
      uls();
    count_number++;
///////////////////////////////////////////////////////////////////////
    speedL=ftm_quad_get(FTM1)*0.0265625;//*0.086/512/15*20;//获取FTM 正交解码 的脉冲数(负数表示反方向)//左
    ftm_quad_clean(FTM1);
    speedR=(-1)*ftm_quad_get(FTM2)*0.0265625;//右
    ftm_quad_clean(FTM2);
    speed_now=(speedL+speedR)/2.0f;;//(speedL+speedR)/2;
    counter_handler++;
    ELE();
    S3010_control();
    check_distance+=speed_now/0.0265625/2/512*0.2/15*20;

//   check_distance+=speedR*0.0066666666666;

    if(flag_end_speed==0)
      speed_control();
       if(signal.cross==1){//避障总标志位{
         distance+=speed_now/0.0265625/2/512*0.2/15*20;
         if(signal.cross_M==1)
           MPU6050_Z();
       }

         
       if(signal.circle==1){
         distance+=speed_now/0.0265625/2/512*0.2/15*20;
         if(signal.circle_M==1)
           MPU6050_Z();
       }
       
       if(signal.circle_protect==1)
         protect_distance+=speed_now/0.0265625/2/512*0.2/15*20;

      
    ////////避障update
////////////////////////   if(ULS_distance>10&&ULS_distance<65) signal.cross=1;
////////////////////////   if(signal.cross==1){
////////////////////////     MPU6050_Z();
////////////////////////     speed_calculate_B(1.2);
////////////////////////     s_ba_interface.distance=ULS_distance/100.0f;
////////////////////////     s_ba_interface.rotation_th=-MPU6050_g_z;
////////////////////////     s_ba_interface.speed_m_s=speed_now;
////////////////////////     block_avoid_handler_dt();
////////////////////////     par=(-s_ba_interface.turn_want)/7.0f*3;
////////////////////////     S3010_ADDE=S3010_ELE_PD();
////////////////////////     VALUEE=(int)(S3010_value-9*3.2*S3010_ADDE);
////////////////////////     PRE();
////////////////////////     ftm_pwm_duty(S3010_FTM, S3010_CH,VALUEE);
////////////////////////     if(s_ba_interface.exit==1){
////////////////////////       signal.cross=0;
////////////////////////     }
////////////////////////   }
 //   MPU6050_XYZ();

///////////////////////////////////////////////////////////////////////
    it_time_us=pit_time_get_us(PIT1);
    if(it_time_us>it_time_us_max) it_time_us_max=it_time_us;//max不大于4000*/
}


//发送图像到上位机显示
//不同的上位机，不同的命令，这里使用 yy_摄像头串口调试 软件
//如果使用其他上位机，则需要修改代码
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_摄像头串口调试 使用的命令

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //先发送命令

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
}
