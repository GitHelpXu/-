#include "Camear_Lcd.h"
#include "include.h"
#include "S3010.h"


//////////////////////////////////////////////////////////////////////////������///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Camear&LCD��������/////////////////
////����洢����ͼ�������
uint8 imgbuff[CAMERA_SIZE];
uint8 img[CAMERA_H][CAMERA_W];
////�ݴ�ǰһ���е����
uint8_t mid_dot=40;
uint8_t Mid_dot=40;
////���Һڵ���
int8_t left_dot,right_dot;
////ͼ�����Ͻǵ�λ��
Site_t site={0,0};
////������ʾ����
Site_t site_1={10,80};
Site_t site_2={40,80};
//������ߡ����Һ���
Site_t center_line[60];
Site_t left_line[60];
Site_t right_line[60];
//����ο���λ��
Site_t CarPoint;
//������ٲο�λ��
Site_t MotorPoint;
//����ͷ���ͼƬ��С
Size_t imgsize  = {CAMERA_W, CAMERA_H};
//��ʾ����ͼ���С
Size_t size;
////��С���˷���������/
float A;
float B;
int sum_X;
int sum_Y;
float sum_FZ=0.0;
float sum_FM=0.0;
int x_MIX;
int y_MIX;
float aver_X;
float aver_Y;
int8 K_l;
int8 K_r;
int8 K_l;
int8 K_r;
int8 B_l;
Site_t next_l;
int8 B_r;
Site_t next_r;
uint8 flag_cross;
uint8 Z_O[60][80];
uint8 width[60];
uint8 flag_saltus;

//////////////////���е㺯��/////////////////////////////////
int16_t find_center_dot(uint8_t rol,uint8_t mid_dot)
{
    int8_t left=mid_dot,right=mid_dot;
    for(;left>=0;left--)
    {
      if(findpix(left,rol)==1) break;
    }
    for(;right<=79;right++)
    {
      if(findpix(right,rol)==1) break;
    }
    if(left<0){left=0;}
    if(right>79){right=79;}
    left_dot=left;
    right_dot=right;
    //if(left<=0&&right<=79) return -1;//���û�к���
    //if(right>=79&&left>=0) return -2;//�ұ�û�к���
    //if(right>=79&&left<=0) return -3;//���Ҿ�û�к���
    return (left+right)/2;
}


void find_center_line()
{
    uint8_t j;
    for(j=59;j>0;j--)
    {
        find_center_dot(j,Mid_dot);
        if(left_dot<=0) left_dot=0;
        if(right_dot>=79) right_dot=79;
        //center_line[10].x=(uint16)((right_dot+left_dot)/2*1.5);
        //center_line[10].y=(uint16)(j*1.5);
        center_line[j].x=(right_dot+left_dot)/2;
        center_line[j].y=j;
        Mid_dot=center_line[j].x;
        left_line[j].x=left_dot;
        left_line[j].y=j;
        right_line[j].x=right_dot;
        right_line[j].y=j;
        width[j]=right_line[j].x-left_line[j].x;
    }
}
void fill_l()
{
  int8 i;
  for(i=59;i>=2;i--)
  {
    K_l=left_line[i-1].x-left_line[i].x;
    B_l=left_line[i].x-K_l*(-1)*i;
    next_l.x=K_l*(-1)*(i-2)+B_l;
    if(fabs(next_l.x-left_line[i-2].x)>2)
      left_line[i-2].x=next_l.x;
  }
}
void fill_r()
{
  int8 i;
  for(i=59;i>=2;i--)
  {
    K_r=right_line[i-1].x-right_line[i].x;
    B_r=right_line[i].x-K_r*(-1)*i;
    next_r.x=K_r*(-1)*(i-2)+B_r;
    if(fabs(next_r.x-right_line[i-2].x)>2)
    right_line[i-2].x=next_r.x;
  }
}

void ff_line()
{
  find_center_line();
  fill_l();
  fill_r();
}

///////����ʮ�ֲ���///////
void ten_fill_L()//��
{
    uint8 i,j,m,n,tempx,tempy,flag_a,flag_b;
    Site_t point_a,point_b;
    float K,B;
    for(i=57;i>=2;i--)
    {
        tempx=left_line[i].x;
        tempy=i;
        if(tempx-1<0||tempx-2<0)
            continue;
            else if(findpix(tempx-1,tempy-1)==0&&
                    findpix(tempx-2,tempy-2)==0&&
                    findpix(tempx+1,tempy+1)==0&&
                    findpix(tempx+2,tempy+2)==0)
                    {
                        point_b=left_line[i];
                        flag_b=1;
                        break;
                    }
                    else flag_b=0;
    }
    if(flag_b==1)
        for(j=i;j>=2;j--)
        {
            tempx=left_line[j].x;
            tempy=j;
            if(tempx-1<0||tempx-2<0)
                continue;
                else if(findpix(tempx-1,tempy+1)==0&&
                        findpix(tempx-2,tempy+2)==0&&
                        findpix(tempx+1,tempy-1)==0&&
                        findpix(tempx+2,tempy-2)==0)
                        {
                            point_a=left_line[j];
                            flag_a=1;
                            break;
                        }
                        else flag_a=0;
        }
        if(flag_b==1&&flag_a==1)
        {
            K=(point_a.x-point_b.x)/(point_b.y-point_a.y);
            B=point_a.x+K*point_a.y;
            for(m=point_b.y-1;m>point_a.y;m--)
            {
                left_line[m].x=K*(-1)*m+B;
            }
        }
}

void ten_fill_R()//��
{
    uint8 i,j,m,n,tempx,tempy,flag_a,flag_b;
    Site_t point_a,point_b;
    float K,B;
    for(i=57;i>=2;i--)
    {
        tempx=right_line[i].x;
        tempy=i;
        if(tempx+1>79||tempx+2>79)
            continue;
            else if(findpix(tempx-1,tempy+1)==0&&
                    findpix(tempx-2,tempy+2)==0&&
                    findpix(tempx+1,tempy-1)==0&&
                    findpix(tempx+2,tempy-2)==0)
                    {
                        point_b=right_line[i];
                        flag_b=1;
                        break;
                    }
                    else flag_b=0;
    }
    if(flag_b==1)
        for(j=i;j>=2;j--)
        {
            tempx=right_line[j].x;
            tempy=j;
            if(tempx+1>79||tempx+2>79)
                continue;
                else if(findpix(tempx-1,tempy-1)==0&&
                        findpix(tempx-2,tempy-2)==0&&
                        findpix(tempx+1,tempy+1)==0&&
                        findpix(tempx+2,tempy+2)==0)
                        {
                            point_a=right_line[j];
                            flag_a=1;
                            break;
                        }
                        else flag_a=0;
        }
        if(flag_b==1&&flag_a==1)
        {
            K=(point_a.x-point_b.x)/(point_b.y-point_a.y);
            B=point_a.x+K*point_a.y;
            for(m=point_b.y-1;m>point_a.y;m--)
            {
                right_line[m].x=K*(-1)*m+B;
            }
        }
}





//////�ڰ׵��Ժ���///////
void Z_OF()
{
  uint8 i,j;
  for(i=0;i<60;i++)
    for(j=0;j<80;j++)
    {
      Z_O[i][j]=findpix(j,i);
    }
}

////////���������־λ�б���////////
//uint8 flag_cross_F()
//{
//    uint8 i,j,m,n,p,q,flag_B,flag_W,flag_BW,flag_cross;
//    uint8 counter_B=0,counter_W=0,counter_BW=0;
//    for(i=25;i<29;i++)
//        for(j=38;j<43;j++)
//    {
//        if(findpix(j,i)==1)
//            counter_B++;
//    }
//    if(counter_B==20)
//        flag_B=1;
//    else flag_B=0;
//    for(m=31;m<35;m++)
//        for(n=38;n<43;n++)
//    {
//        if(findpix(n,m)==0)
//            counter_W++;
//    }
//    if(counter_W==20)
//        flag_W=1;
//    else flag_W=0;
//    flag_cross=flag_B&&flag_W;
//    return flag_cross;
//}
//
//
//////////���������б�־�б���/////////
//uint8 flag_saltus_F()
//{
//  uint8 i,flag_saltus;
//  int8 temp=-1;
//  for(i=59;i>=4;i--)
//  {
//    if(width[i]-width[i-1]>20&&width[i-2]==0&&width[i-3]==0&&width[i-4]==0&&center_line[i].x>25&&center_line[i].x<55)
//    {
//      temp=i-1;
//      break;
//    }
//  }
//  if(temp==-1)
//    flag_saltus=0;
//  if(temp>=15)
//    flag_saltus=1;
//  else flag_saltus=0;
//  return flag_saltus;
//}





