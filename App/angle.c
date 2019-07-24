//MCU_WANG 2017.3@�Ͻ�ѧԺ��ⴴ��ʵ������
//#include "headfile.h"
#include "angle.h"
#include "MPU6050.h"
#include "common.h"
#include "include.h"

#define N_WINDOW (4)//���ڻ����˲�����
angle filted;
float window_gx[N_WINDOW],window_gy[N_WINDOW],window_gz[N_WINDOW];
float window_ax[N_WINDOW],window_ay[N_WINDOW],window_az[N_WINDOW];
float window_mx[N_WINDOW],window_my[N_WINDOW],window_mz[N_WINDOW];
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


void angle_init()
{
  unsigned char i;
  for(i=0;i<N_WINDOW;i++)
  {
    window_gx[i]=window_gy[i]=window_gz[i]=0;
    window_ax[i]=window_ay[i]=window_az[i]=0;
  }

}



float ax=0,ay=0,az=0;
float gx=0,gy=0,gz=0;
float mx=0,my=0,mz=0;
  
void IMU_update()
{
  MPU6050_XYZ();
// FXAS21002_XYZ();
// MMA8451_XYZ();
}

void angle_get()
{
  static unsigned char window_counter;
  unsigned char i;


  if(++window_counter==N_WINDOW) window_counter=0;
  window_gx[window_counter]=-MPU6050_g_x;
  window_gy[window_counter]=-MPU6050_g_y;
  window_gz[window_counter]=MPU6050_g_z;
  
  window_ax[window_counter]=MPU6050_a_x;
  window_ay[window_counter]=MPU6050_a_y;
  window_az[window_counter]=MPU6050_a_z;
  
  gx=gy=gz=ax=ay=az=mx=my=mz=0;
  
  for(i=0;i<N_WINDOW;i++)
  {
    gx+=window_gx[i];
    gy+=window_gy[i];
    gz+=window_gz[i];
  
    
    ax+=window_ax[i];
    ay+=window_ay[i];
    az+=window_az[i];
  }
  
  gx/=N_WINDOW;
  gy/=N_WINDOW;
  gz/=N_WINDOW;
  
  ax/=N_WINDOW;
  ay/=N_WINDOW;
  az/=N_WINDOW;
  if(ax==0&&ay==0&&az==0) ax=ay=az=1;
  gx*=0.00106526443600124782986111111111f;
  gy*=0.00106526443600124782986111111111f;
  gz*=0.00106526443600124782986111111111f;
  
  /*
  mx=HMC5883L_x;
  my=HMC5883L_y;
  mz=HMC5883L_z;
  if(mx==0&&my==0&&mz==0) mx=my=mz=1;
  */
  IMUupdate(gx,gy,gz,ax,ay,az);
  //AHRSupdate(gx, gy,  gz,  ax,  ay, az,mx, my,  mz) ;
}

void angle_resume()
{
  uint16_t i;
  IMU_update();
  MPU6050_g_x=MPU6050_g_y=MPU6050_g_z=0;
  for(i=0;i<1000;i++) angle_get();
}


float q0=1, q1=0, q2=0, q3=0;
float exInt, eyInt, ezInt;

//*****************************************6���ں�*****************************************************
#define Kp 2.0f //���ٶ�Ȩ�أ�Խ��������ٶȲ���ֵ����Խ��

#define Ki 0.001f //����������
#define halfT 0.0025f


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; 

         filted.pitch_d=-gy*57.295779514719953173585430909526f;
         filted.roll_d=-gx*57.295779514719953173585430909526f;
         filted.yaw_d=-gz*57.295779514719953173585430909526;
 
	//�Ӽ�ֵ��һ������normalise the measurements
	norm = sqrtf(ax*ax + ay*ay + az*az);      
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;      

	//�����ɿ�����ϵestimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//����ϵ�������������error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	//��������integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�����ǻ����ں� adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	//������Ԫ�� integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

	//��Ԫ����һ������normalise quaternion
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);	
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	//����ŷ����
	filted.pitch	=	-asinf(-2*q1*q3 + 2*q0*q2)*57.30f;//����90��
	filted.roll	=	-atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1-2*q2*q2 + 1)*57.30f;//����180��
	//filted.yaw=	filted.yaw-gz;
        
        //filted.yaw+=(filted.yaw_d/cosf(filted.roll*AtoR)/cosf(filted.pitch*AtoR))*halfT*2;
        while(filted.yaw>360) filted.yaw-=360;
        while(filted.yaw<0) filted.yaw+=360;
        
}


////*********************************************9���ں�******************************************************
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz,bx;// by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
   
   
        filted.pitch_d=-gy*57.295779514719953173585430909526f;
        filted.roll_d=-gx*57.295779514719953173585430909526f;
        filted.yaw_d=-gz*57.295779514719953173585430909526;
        
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   norm = sqrtf(ax*ax + ay*ay + az*az);       
   ax = ax / norm;
   ay = ay / norm;
   az = az / norm;
   norm = sqrtf(mx*mx + my*my + mz*mz);          
   mx = mx / norm;
   my = my / norm;
   mz = mz / norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��h-xyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��b-xyz���ο�ֵ������Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣���
�Ҷ���b-yָ������������by=ĳֵ��bx=0������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by����������޸�b-y��b-x�Ǹ���ָ��������*/
	bx = sqrtf((hx*hx) + (hy*hy));
//   by = sqrtf((hx*hx) + (hy*hy));
	bz = hz;        
    
//����Ĺ�ʽ�Ǵӵ�������ϵ���ɿ�����ϵ��ת��
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��b-xyz��ת����������w-xyz��
��Ϊbx/y=0�����������漰��bx/y�Ĳ��ֶ���ʡ���ˣ�������Լ������Ǹ���ָ���йء�
������������v-xyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��w-xyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����v-xyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//x���׼����
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
//y���׼����
//   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

//���ּ��㣬���Բ������ڵ�һ��

	exInt = exInt + ex*Ki; //* halfT;			  
	eyInt = eyInt + ey*Ki;// * halfT;
	ezInt = ezInt + ez*Ki;// * halfT;
			
//����������PI����
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
 

//������Ԫ��
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
//��Ԫ����һ��
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;       //w
	q1 = q1 / norm;       //x
	q2 = q2 / norm;       //y
	q3 = q3 / norm;       //z
			

	filted.yaw= atan2f(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.30f;  //����57.30��Ϊ�˽�����ת��Ϊ�Ƕ�
	filted.roll= -asinf(2*q2*q3 + 2*q0*q1) * 57.30f;																	 //����180��	
	filted.pitch= atan2f(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.30f;
	
	
}
