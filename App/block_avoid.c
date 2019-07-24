#include "include.h"



const static float dt=0.005;

#define OBSTACLE_THICKNESS       (0.2)//(0.32) //障碍物厚度 (用于第二阶段向前这么多距离）
#define OBSTACLE_LENGTH		 (0.4)//障碍物宽度
#define INFLATION_X		 (0.1)//横向膨胀长度，这个值越大距离障碍就会越远

#define RADIUS_REPAIR            (-0.04) //转向半径修正，让在跟障碍平行时与赛道平行前进

#define BACK_DISTANCE		 	 (0.3)//越过障碍之后返回赛道时向前走这么多米，这个值越大返回越平缓
#define BACK_X						 (0.3)//返回到赛道中心点向右这么多距离，之后修正角度

#define BACK_THETA					(90+0)//修正到这个角度之后返回正常控制，90°是正前方，0°是正右方

static s_block_avoid_data_t s_block_avoid_data;
s_ba_interface_t s_ba_interface;


static void block_avoid_data_update(s_block_avoid_data_t *ps_ba_dat){
	ps_ba_dat->speed_line=s_ba_interface.speed_m_s;//速度
	ps_ba_dat->theta+=s_ba_interface.rotation_th*dt;//度数
	
	
	//分解速度
	ps_ba_dat->speed_x=ps_ba_dat->speed_line*cosf(ps_ba_dat->theta*AtoR);
	ps_ba_dat->speed_y=ps_ba_dat->speed_line*sinf(ps_ba_dat->theta*AtoR);
	
	//位置估计
	ps_ba_dat->car_position.x+=ps_ba_dat->speed_x*dt;
	ps_ba_dat->car_position.y+=ps_ba_dat->speed_y*dt;
	
}

void block_avoid_handler_dt(){
	(void)(dt);//avoid warning
	if(s_block_avoid_data.step==BLOCK_AVOID_END){//进入避障
		s_block_avoid_data.step=BLOCK_AVOID_START;
		s_block_avoid_data.theta=90.0f;
		s_block_avoid_data.car_position.x=s_block_avoid_data.car_position.y=0;
		s_block_avoid_data.position_want.x=OBSTACLE_LENGTH/2+INFLATION_X;
		s_block_avoid_data.position_want.y=s_ba_interface.distance;
		s_block_avoid_data.step=BLOCK_AVOID_START;
	}else if(s_block_avoid_data.step==BLOCK_AVOID_START){
		block_avoid_data_update(&s_block_avoid_data);//更新数据
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//转向期望角度返回
		//判断是否达到障碍
		if((1||s_block_avoid_data.car_position.y>s_block_avoid_data.position_want.y)&&s_block_avoid_data.car_position.x>s_block_avoid_data.position_want.x){
			s_block_avoid_data.position_want.y=s_block_avoid_data.car_position.y+OBSTACLE_THICKNESS;
			s_block_avoid_data.position_want.x=s_block_avoid_data.car_position.x+RADIUS_REPAIR;
			s_block_avoid_data.step=BLOCK_AVOID_ARRIVE;
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_ARRIVE){//已经与障碍平行，再向前障碍厚度
		block_avoid_data_update(&s_block_avoid_data);//更新数据
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//转向期望角度返回
		
		//判断是否越过障碍厚度
		if(s_block_avoid_data.car_position.y>s_block_avoid_data.position_want.y){
			s_block_avoid_data.position_want.y=s_block_avoid_data.car_position.y+BACK_DISTANCE;
			s_block_avoid_data.position_want.x=BACK_X;
			s_block_avoid_data.step=BLOCK_AVOID_PASS;
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_PASS){
		block_avoid_data_update(&s_block_avoid_data);//更新数据
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//转向期望角度返回
		
		//判断是否返回赛道
		if(s_block_avoid_data.car_position.x<s_block_avoid_data.position_want.x){
			s_block_avoid_data.step=BLOCK_AVOID_THETA_REPAIR;//下一步修正角度
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_THETA_REPAIR){
		block_avoid_data_update(&s_block_avoid_data);//更新数据
		s_ba_interface.turn_want=BACK_THETA-s_block_avoid_data.theta;
                while(s_ba_interface.turn_want>180) s_ba_interface.turn_want-=360;
                while(s_ba_interface.turn_want<-180) s_ba_interface.turn_want+=360;
		if(fabsf(s_block_avoid_data.theta-BACK_THETA)<3){//近似达到角度
			s_block_avoid_data.step=BLOCK_AVOID_END;
                        s_ba_interface.exit=1;
		}
	}
}