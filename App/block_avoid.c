#include "include.h"



const static float dt=0.005;

#define OBSTACLE_THICKNESS       (0.2)//(0.32) //�ϰ����� (���ڵڶ��׶���ǰ��ô����룩
#define OBSTACLE_LENGTH		 (0.4)//�ϰ�����
#define INFLATION_X		 (0.1)//�������ͳ��ȣ����ֵԽ������ϰ��ͻ�ԽԶ

#define RADIUS_REPAIR            (-0.04) //ת��뾶���������ڸ��ϰ�ƽ��ʱ������ƽ��ǰ��

#define BACK_DISTANCE		 	 (0.3)//Խ���ϰ�֮�󷵻�����ʱ��ǰ����ô���ף����ֵԽ�󷵻�Խƽ��
#define BACK_X						 (0.3)//���ص��������ĵ�������ô����룬֮�������Ƕ�

#define BACK_THETA					(90+0)//����������Ƕ�֮�󷵻��������ƣ�90������ǰ����0�������ҷ�

static s_block_avoid_data_t s_block_avoid_data;
s_ba_interface_t s_ba_interface;


static void block_avoid_data_update(s_block_avoid_data_t *ps_ba_dat){
	ps_ba_dat->speed_line=s_ba_interface.speed_m_s;//�ٶ�
	ps_ba_dat->theta+=s_ba_interface.rotation_th*dt;//����
	
	
	//�ֽ��ٶ�
	ps_ba_dat->speed_x=ps_ba_dat->speed_line*cosf(ps_ba_dat->theta*AtoR);
	ps_ba_dat->speed_y=ps_ba_dat->speed_line*sinf(ps_ba_dat->theta*AtoR);
	
	//λ�ù���
	ps_ba_dat->car_position.x+=ps_ba_dat->speed_x*dt;
	ps_ba_dat->car_position.y+=ps_ba_dat->speed_y*dt;
	
}

void block_avoid_handler_dt(){
	(void)(dt);//avoid warning
	if(s_block_avoid_data.step==BLOCK_AVOID_END){//�������
		s_block_avoid_data.step=BLOCK_AVOID_START;
		s_block_avoid_data.theta=90.0f;
		s_block_avoid_data.car_position.x=s_block_avoid_data.car_position.y=0;
		s_block_avoid_data.position_want.x=OBSTACLE_LENGTH/2+INFLATION_X;
		s_block_avoid_data.position_want.y=s_ba_interface.distance;
		s_block_avoid_data.step=BLOCK_AVOID_START;
	}else if(s_block_avoid_data.step==BLOCK_AVOID_START){
		block_avoid_data_update(&s_block_avoid_data);//��������
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//ת�������Ƕȷ���
		//�ж��Ƿ�ﵽ�ϰ�
		if((1||s_block_avoid_data.car_position.y>s_block_avoid_data.position_want.y)&&s_block_avoid_data.car_position.x>s_block_avoid_data.position_want.x){
			s_block_avoid_data.position_want.y=s_block_avoid_data.car_position.y+OBSTACLE_THICKNESS;
			s_block_avoid_data.position_want.x=s_block_avoid_data.car_position.x+RADIUS_REPAIR;
			s_block_avoid_data.step=BLOCK_AVOID_ARRIVE;
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_ARRIVE){//�Ѿ����ϰ�ƽ�У�����ǰ�ϰ����
		block_avoid_data_update(&s_block_avoid_data);//��������
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//ת�������Ƕȷ���
		
		//�ж��Ƿ�Խ���ϰ����
		if(s_block_avoid_data.car_position.y>s_block_avoid_data.position_want.y){
			s_block_avoid_data.position_want.y=s_block_avoid_data.car_position.y+BACK_DISTANCE;
			s_block_avoid_data.position_want.x=BACK_X;
			s_block_avoid_data.step=BLOCK_AVOID_PASS;
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_PASS){
		block_avoid_data_update(&s_block_avoid_data);//��������
		s_ba_interface.turn_want=RtoA*atan2(s_block_avoid_data.position_want.y-s_block_avoid_data.car_position.y,
																	 s_block_avoid_data.position_want.x-s_block_avoid_data.car_position.x)-s_block_avoid_data.theta;//ת�������Ƕȷ���
		
		//�ж��Ƿ񷵻�����
		if(s_block_avoid_data.car_position.x<s_block_avoid_data.position_want.x){
			s_block_avoid_data.step=BLOCK_AVOID_THETA_REPAIR;//��һ�������Ƕ�
		}
	}
	else if(s_block_avoid_data.step==BLOCK_AVOID_THETA_REPAIR){
		block_avoid_data_update(&s_block_avoid_data);//��������
		s_ba_interface.turn_want=BACK_THETA-s_block_avoid_data.theta;
                while(s_ba_interface.turn_want>180) s_ba_interface.turn_want-=360;
                while(s_ba_interface.turn_want<-180) s_ba_interface.turn_want+=360;
		if(fabsf(s_block_avoid_data.theta-BACK_THETA)<3){//���ƴﵽ�Ƕ�
			s_block_avoid_data.step=BLOCK_AVOID_END;
                        s_ba_interface.exit=1;
		}
	}
}