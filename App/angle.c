//MCU_WANG 2017.3@紫金学院电光创新实践基地
//#include "headfile.h"
#include "angle.h"
#include "MPU6050.h"
#include "common.h"
#include "include.h"

#define N_WINDOW (4)//窗口滑动滤波次数
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

//*****************************************6轴融合*****************************************************
#define Kp 2.0f //加速度权重，越大则向加速度测量值收敛越快

#define Ki 0.001f //误差积分增益
#define halfT 0.0025f


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; 

         filted.pitch_d=-gy*57.295779514719953173585430909526f;
         filted.roll_d=-gx*57.295779514719953173585430909526f;
         filted.yaw_d=-gz*57.295779514719953173585430909526;
 
	//加计值归一化处理normalise the measurements
	norm = sqrtf(ax*ax + ay*ay + az*az);      
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;      

	//建立飞控坐标系estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//坐标系和重力叉积运算error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	//比例运算integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//陀螺仪积分融合 adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	//计算四元数 integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

	//四元数归一化处理normalise quaternion
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);	
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	//计算欧拉角
	filted.pitch	=	-asinf(-2*q1*q3 + 2*q0*q2)*57.30f;//正负90度
	filted.roll	=	-atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1-2*q2*q2 + 1)*57.30f;//正负180度
	//filted.yaw=	filted.yaw-gz;
        
        //filted.yaw+=(filted.yaw_d/cosf(filted.roll*AtoR)/cosf(filted.pitch*AtoR))*halfT*2;
        while(filted.yaw>360) filted.yaw-=360;
        while(filted.yaw<0) filted.yaw+=360;
        
}


////*********************************************9轴融合******************************************************
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz,bx;// by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*方便之后的程序使用，减少计算时间*/
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
        
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   norm = sqrtf(ax*ax + ay*ay + az*az);       
   ax = ax / norm;
   ay = ay / norm;
   az = az / norm;
   norm = sqrtf(mx*mx + my*my + mz*mz);          
   mx = mx / norm;
   my = my / norm;
   mz = mz / norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量h-xyz（测量值），下面这个是从飞行器坐标系到地理坐标系的转换公式*/
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量b-xyz（参考值）。因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），
我定义b-y指向正北，所以by=某值，bx=0但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。下面可以修改b-y和b-x那个轴指向正北。*/
	bx = sqrtf((hx*hx) + (hy*hy));
//   by = sqrtf((hx*hx) + (hy*hy));
	bz = hz;        
    
//下面的公式是从地理坐标系到飞控坐标系的转化
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量b-xyz，转到机体上来w-xyz。
因为bx/y=0，所以所有涉及到bx/y的部分都被省略了，这根据自己定义那个轴指北有关。
类似上面重力v-xyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：w-xyz的公式，把by换成ay（0），把bz换成az（1），就变成了v-xyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//x轴对准北方
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
//y轴对准北方
//   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

//积分计算，乘以采样周期的一半

	exInt = exInt + ex*Ki; //* halfT;			  
	eyInt = eyInt + ey*Ki;// * halfT;
	ezInt = ezInt + ez*Ki;// * halfT;
			
//补偿陀螺仪PI计算
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
 

//更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
//四元数归一化
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;       //w
	q1 = q1 / norm;       //x
	q2 = q2 / norm;       //y
	q3 = q3 / norm;       //z
			

	filted.yaw= atan2f(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.30f;  //乘以57.30是为了将弧度转化为角度
	filted.roll= -asinf(2*q2*q3 + 2*q0*q1) * 57.30f;																	 //正负180度	
	filted.pitch= atan2f(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.30f;
	
	
}
