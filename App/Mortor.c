#include "Mortor.h"
#include "S3010.h"
#include "MK60_FTM.h"
#include "flag.h"
#include "ELE.h"




int Mortor_Error_now=0;
int Mortor_Error_last=0;
int Mortor_Error_prev=0;

//////////////////电机变量定义//////
//差速
float pwm_motor_turn;

////速度采集
float speedL;
float speedR;
float speed_now;
float speed_err;
float pwm_KP_output;
float pwm_feed_output;
float pwm_output;


//期望速度
int16 speed_goal;
//电机误差定义//

//电机PD使用量定义//
float Mortor_out_P;
float Mortor_out_I;
float Mortor_out_D;
float Mortor_ADD;
float Mortor_add;




uint8 flag_mod;



/*************前馈控制阶段函数***********/
void feedforward_calculate(float goal){
         speed_err=goal-speed_now;
         if(speed_err>=0)
           pwm_KP_output=speed_KP*speed_err*284-160;
         else pwm_KP_output=speed_KP*(-1)*(fabs(speed_err)*284-160);
         pwm_feed_output=pwm_startup+speed_feed_K*goal;
         pwm_output=pwm_KP_output+pwm_feed_output;
         if(pwm_output>999) pwm_output=999;
         if(speed_now-goal<=0&&speed_now-goal>-speed_range){
           ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
           ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,(uint32_t)pwm_output);
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,(uint32_t)pwm_output);
         }
         if(speed_now-goal>=0&&speed_now-goal<speed_range){
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
           ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(uint32_t)pwm_output);
           ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(uint32_t)pwm_output);
         }
}

/************差速量**************/
void turn_conrol_for_motor(float par){
  if(fabs(ver)>=6)
  pwm_motor_turn=par*fabs(par)/1.2*0.0;
  else pwm_motor_turn=0.0;
}


/****************PWM直接输出*************/
void motor_pwm_output(float pwm_L,float pwm_R){//电机输出 参数是百分比-100到100
/***********限幅*********/
  if(pwm_L<-100) pwm_L=-100;
  if(pwm_L>100) pwm_L=100;
  if(pwm_R<-100) pwm_R=-100;
  if(pwm_R>100) pwm_R=100;
/*********左轮*********/  
  if(pwm_L>0){
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,(uint32_t)(pwm_L/100.0f*999.0f));
  }else{
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(uint32_t)(-pwm_L/100.0f*999.0f));
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
  }
/********右轮*********/  
  if(pwm_R>0){
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,(uint32_t)(pwm_R/100.0f*999.0f));
  }else{
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(uint32_t)(-pwm_R/100.0f*999.0f));
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
  }
}





//////////pwm计算//////////////
void speed_calculate(float goal){
  float pwm_L,pwm_R;//左轮 正3 反1   //右轮  正4  反2
  pwm_L=pwm_R=0;
/****************前馈前大占空比加减速***************/  
  if(speed_now-goal<=-speed_range){//正转加速
    if(par>=10) par=10;
    if(par<=-10) par=-10;
    pwm_L+=((fabs(par)*(-0.20)+3.0)*284-160)/1000.0f*100.0f;
    pwm_R+=((fabs(par)*(-0.20)+3.0)*284-160)/1000.0f*100.0f;
  }
  else if(speed_now-goal>=speed_range){//反转减速
    if(par>=10) par=10;
    if(par<=-10) par=-10;
    pwm_L+=-((fabs(par)*0.15+2.0)*284-160)/1000.0f*100.0f;
    pwm_R+=-((fabs(par)*0.15+2.0)*284-160)/1000.0f*100.0f;
  }
/***************前馈控制*****************/
  else if(fabs(speed_now-goal)<speed_range){
   // feedforward_calculate(goal); 
    
     speed_err=goal-speed_now;
     if(speed_err>=0) pwm_KP_output=speed_KP*speed_err*284-160;
     else pwm_KP_output=speed_KP*(-1)*(fabs(speed_err)*284-160);
     
     pwm_feed_output=pwm_startup+speed_feed_K*goal;
     pwm_output=pwm_KP_output+pwm_feed_output;
     
     if(speed_now-goal<=0&&speed_now-goal>-speed_range){//正转
       pwm_L+=pwm_output/1000.0f*100.0f;
       pwm_R+=pwm_output/1000.0f*100.0f;
     }
     if(speed_now-goal>=0&&speed_now-goal<speed_range){//反转
       pwm_L+=-pwm_output/1000.0f*100.0f;
       pwm_R+=-pwm_output/1000.0f*100.0f;
     }
  }
/************大占空比量+前馈量+差速量******************/  
  pwm_L+=pwm_motor_turn;
  pwm_R-=pwm_motor_turn;
  motor_pwm_output(pwm_L,pwm_R);
}


void speed_calculate_B(float goal){//////////////弯道速度//////////
   if(speed_now-goal<=-speed_range){
   
      ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
      ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,320);   //速度
      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,320);   //速度
    }
    if(speed_now-goal>=speed_range){
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,999);
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,999);
    }
        if(fabs(speed_now-goal)<speed_range)
          feedforward_calculate(goal); 
}




void speed_calculate_special(float goal)
{
    if(speed_now-goal<=-speed_range)
        {
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,340);
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,340);
        }
        if(speed_now-goal>=speed_range)
        {
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,340);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,340);
        }
        if(fabs(speed_now-goal)<speed_range)
          feedforward_calculate(goal); 
}






void speed_control()
{
  //if(circle_number!=2)
  //{
    if(signal.circle==1||signal.cross==1);
    else
    {
      turn_conrol_for_motor(par);
      if(fabs(ver)<5)
        speed_calculate(fabs(par)*(-0.2)+3.0);
      else speed_calculate_B(speed_goal_B);
    }

}



//void low_filter(float throughput){


//void speed_control()
//{
//  //if(circle_number!=2)
//  //{
//    if(signal.circle==1||signal.cross==1);
//    else
//    {
//      if(fabs(ver)<8)
//        speed_calculate_special(fabs(par)*(-0.14)+2.4);
//      else speed_calculate_B(speed_goal_B);
//    }
//}
