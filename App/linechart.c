//南京理工大学紫金学院  王瑞
//2019.7.8
#include "linechart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define LINECHART_MIN_DX (0.0) //x最小差值，防止出现inf，如不需要建议设置为0.0

typedef struct{
	float x;
	float y;
}s_steer_point_t;

//增加数据点，    数据点结构体数组指针    当前数组内有多少个数据点   数组大小       x        y
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
	
#define POINTS_LEN (30)//折点buff的大小，最多能存入这么多个折点，
	static s_steer_point_t points[POINTS_LEN];
	static uint16_t point_len=0;
	static uint8_t point_init=0;
	
	if(point_init==0){
		point_init=1;
		//在这里写你要放入的折点
		//add_points 的x必须从小到大的顺序
		//例如：
		add_points(points, &point_len, POINTS_LEN, 0, 9*3.2);
                add_points(points, &point_len, POINTS_LEN, 4, 9*3.2);
		add_points(points, &point_len, POINTS_LEN, 5, 30);
                add_points(points, &point_len, POINTS_LEN, 6, 36);
                add_points(points, &point_len, POINTS_LEN, 7, 42);
                add_points(points, &point_len, POINTS_LEN, 8, 48);
                add_points(points, &point_len, POINTS_LEN, 9, 54);
		add_points(points, &point_len, POINTS_LEN, 10,60);
		//则折线为(-100,50)到(0,0)到(100,-100)，返回值将在-100到50之间，即输入-120也只会返回50，输入120也只会返回-100
	}
	
	
	//寻找所在区间p1-p2
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
		linechart_debug_err =1;//p2应该大于p1
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




