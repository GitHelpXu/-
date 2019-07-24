#ifndef BLOCK_AVOID_H__
#define BLOCK_AVOID_H__

#define AtoR (3.1415926535/180.0f)
#define RtoA (1.0f/AtoR)

typedef enum{
	BLOCK_AVOID_START=1,//开始避障
	BLOCK_AVOID_ARRIVE=2,//达到障碍位置，与障碍平行
	BLOCK_AVOID_PASS=3,//超过障碍
	BLOCK_AVOID_THETA_REPAIR=4,//回到赛道上之后，修正回角度
	BLOCK_AVOID_END=0//避障结束
}e_ba_procedure_t;


typedef struct{
	float x;
	float y;
}s_ba_point_t;



typedef struct{
	s_ba_point_t car_position;
	s_ba_point_t position_want;
	
	e_ba_procedure_t step;
	
	float speed_x;//分解到x方向的速度
	float speed_y;//分解到y方向的速度
	float speed_line;//线速度
	
	float theta;//0是x轴方向  逆时针转角度加大
	
}s_block_avoid_data_t;


typedef struct{
	//输入参数
	float distance;//距离障碍的距离  米
	float speed_m_s;//速度 米每秒
	float rotation_th;//角速度 度每秒
	
	//输出参数
	float turn_want;//返回给控制器的转向期望
	uint8_t exit;//避障结束之后这个值置1
}s_ba_interface_t;


extern s_ba_interface_t s_ba_interface;

void block_avoid_handler_dt();
#endif
