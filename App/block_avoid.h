#ifndef BLOCK_AVOID_H__
#define BLOCK_AVOID_H__

#define AtoR (3.1415926535/180.0f)
#define RtoA (1.0f/AtoR)

typedef enum{
	BLOCK_AVOID_START=1,//��ʼ����
	BLOCK_AVOID_ARRIVE=2,//�ﵽ�ϰ�λ�ã����ϰ�ƽ��
	BLOCK_AVOID_PASS=3,//�����ϰ�
	BLOCK_AVOID_THETA_REPAIR=4,//�ص�������֮�������ؽǶ�
	BLOCK_AVOID_END=0//���Ͻ���
}e_ba_procedure_t;


typedef struct{
	float x;
	float y;
}s_ba_point_t;



typedef struct{
	s_ba_point_t car_position;
	s_ba_point_t position_want;
	
	e_ba_procedure_t step;
	
	float speed_x;//�ֽ⵽x������ٶ�
	float speed_y;//�ֽ⵽y������ٶ�
	float speed_line;//���ٶ�
	
	float theta;//0��x�᷽��  ��ʱ��ת�ǶȼӴ�
	
}s_block_avoid_data_t;


typedef struct{
	//�������
	float distance;//�����ϰ��ľ���  ��
	float speed_m_s;//�ٶ� ��ÿ��
	float rotation_th;//���ٶ� ��ÿ��
	
	//�������
	float turn_want;//���ظ���������ת������
	uint8_t exit;//���Ͻ���֮�����ֵ��1
}s_ba_interface_t;


extern s_ba_interface_t s_ba_interface;

void block_avoid_handler_dt();
#endif
