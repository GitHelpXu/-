//�Ͼ�����ѧ�Ͻ�ѧԺ  ����
//2019.7.8
#include "linechart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define LINECHART_MIN_DX (0.0) //x��С��ֵ����ֹ����inf���粻��Ҫ��������Ϊ0.0

typedef struct{
	float x;
	float y;
}s_steer_point_t;

//�������ݵ㣬    ���ݵ�ṹ������ָ��    ��ǰ�������ж��ٸ����ݵ�   �����С       x        y
void add_points(s_steer_point_t* points,uint16_t *len,uint16_t max_point_len,float x,float y){
	if(*len<max_point_len){
		points[*len].x=x;
		points[*len].y=y;
		*len=*len+1;
	}
}

uint8_t linechart_debug_err=0;
float linechart_for_steer_gain(float x){
	float y;
	
	int i,p1,p2;
	
#define POINTS_LEN (30)//�۵�buff�Ĵ�С������ܴ�����ô����۵㣬
	static s_steer_point_t points[POINTS_LEN];
	static uint16_t point_len=0;
	static uint8_t point_init=0;
	
	if(point_init==0){
		point_init=1;
		//������д��Ҫ������۵�
		//add_points ��x�����С�����˳��
		//���磺
		add_points(points, &point_len, POINTS_LEN, 0, 9*3.2);
                add_points(points, &point_len, POINTS_LEN, 4, 9*3.2);
		add_points(points, &point_len, POINTS_LEN, 5, 30);
                add_points(points, &point_len, POINTS_LEN, 6, 36);
                add_points(points, &point_len, POINTS_LEN, 7, 42);
                add_points(points, &point_len, POINTS_LEN, 8, 48);
                add_points(points, &point_len, POINTS_LEN, 9, 54);
		add_points(points, &point_len, POINTS_LEN, 10,60);
		//������Ϊ(-100,50)��(0,0)��(100,-100)������ֵ����-100��50֮�䣬������-120Ҳֻ�᷵��50������120Ҳֻ�᷵��-100
	}
	
	
	//Ѱ����������p1-p2
	i=p1=0;
	p2=point_len-1;
	for(i=0;i<point_len;i++){
		if(x>=points[i].x){
			if(p1<i) p1=i;
		}
	}
	
	for(i=point_len-1;i>=0;i--){
		if(x<points[i].x){
			if(p2>i) p2=i;
		}
		if(i==0) break;
	}
	printf("p1:%d,p2:%d\n", p1, p2);
	if(p2<p1){
		y=0;
		linechart_debug_err =1;//p2Ӧ�ô���p1
	}
	else if(p2==p1){
		if(fabsf(points[p2].y)>fabsf(points[p1].y)){
			y=points[p2].y;
		}
		else{
			y=points[p1].y;
		}
	}
	else if(p2>p1){
		if(fabsf(points[p1].x-points[p2].x)>LINECHART_MIN_DX){
			y=points[p1].y+(x-points[p1].x)/(points[p2].x-points[p1].x)*(points[p2].y-points[p1].y);
		}
		else{
			if(fabsf(points[p2].y)>fabsf(points[p1].y)){
				y=points[p2].y;
			}
			else{
				y=points[p1].y;
			}
		}
	}
	return y;
}




