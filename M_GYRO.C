//###############################################
//
// Author  : Gale
// Date    : 2011-4
// Email   : galemx@126.com
// Location: FuZhou FuJian China
//
#include "KK_C.H"
#include "F_DELAY.H"
#include "F_EEROM.H"

//###############################################
// Global Vars define
// 全局变量定义

//--------------------------------------
//##Gain 感度
uchar GainRol,GainPit,GainYaw;			//0~127

//--------------------------------------
//##Gyro signal 陀螺仪信号
#define GYROCOMPS	128			//Gyro compensation scale
uchar GyroBaseCnt=0;			//Gyro base build count 陀螺仪中点建立计数器
int GyroRol,GyroPit,GyroYaw;
int GyroBaseRol,GyroBasePit,GyroBaseYaw;
schar GyroRolPN=0,GyroPitPN=0,GyroYawPN=0;

//--------------------------------------
//##Integral angle 角度积分
#define I_RETSTEP  2	//Integral return step 积分回归幅度 2~4
#define I_MAX	   2000	//Integral max value 积分上限
int GyroRolI,GyroPitI,GyroYawI;//Gyro integral 各陀螺仪积分值

//--------------------------------------
//##Soft filter 软件滤波
#define FILTERNUM		4 //Soft filter buffer size
#define FILTERMASK		3 //Idx from 0~7
#define WARP(X)			(X=((X+1)&FILTERMASK))
uchar GyroRolIdx=0,GyroPitIdx=0,GyroYawIdx=0;
int GyroRolBuf[FILTERNUM],GyroPitBuf[FILTERNUM],GyroYawBuf[FILTERNUM];


//###############################################
//
//   Soft filter
//   软件滤波器
//
int Filter(int *value)
{
#if 0		   //中值滤波
 	bool swap;
	uchar i;
	do{
	 	swap=0;
		for(i=1;i<FILTERNUM;i++)
		{
		 	if(value[i-1]>value[i])
			{
			 	int t=value[i-1];
				value[i-1]=value[i];
				value[i]=t;
				swap=1;
			}
		}
	}while(swap);
	
	return value[FILTERNUM/2];
#else	   //滑动窗口滤波
	uchar i;
	int vs=0;
	for(i=0;i<FILTERNUM;i++)
	{
	 	vs+=value[i];
	}
	return vs/FILTERNUM;
#endif	
}


//###############################################
//
//    Gyro feature compensation
//     陀螺仪特性补偿
int GyroCompe(int gyro,schar pn)
{ 	
	if(pn==0)	  return gyro;
	
	if(gyro>0)
	{
	 	gyro*=GYROCOMPS+pn;
		gyro/=GYROCOMPS;	 		   
	}
	else
	{
	 	gyro*=GYROCOMPS-pn;
		gyro/=GYROCOMPS;
	}
	return gyro;
}

//###############################################
//
//   Integral value with limit and return
//   带限幅/带回归积分
//
int GyroIntegral(int v,int delta)
{
 	//Integral gain
	//内部积分增益
 	delta/=4;//delta=-511~511 -> -127~127
	v+=delta;
	
	//Limit value
	//限幅
	if(v>I_MAX)	 v=I_MAX;
	if(v<-I_MAX) v=-I_MAX;

	//Return value
	//回归
	v/=I_RETSTEP;

	return v;
}

//###############################################
//
//   Adc subroutine
//   AD转换子程序
//
uint ReadAdc(uchar ch)
{
 	ADMUX=ch;	   //Select channel
	
	//Read null once as AVR datasheet says
	ADCSRA=0xC6;   				//Start ADC
	while(ADCSRA&(1<<ADSC));	//Wait adc ok
	
	ADCSRA=0xC6;   				//Start ADC
	while(ADCSRA&(1<<ADSC));	//Wait adc ok
	
	return ADC;
}

//###############################################
//
//   Read signal for 3 gyros
//   读取三个陀螺仪的输出信号
//
void GyroRead(void)
{
 	DIDR0=0x3F;		//0b00111111
	ADCSRB=0;

	//Adc the gyro value
	//采样陀螺仪的值
	GyroRol=ReadAdc(GYRO_ROL);
	GyroPit=ReadAdc(GYRO_PIT);
	GyroYaw=ReadAdc(GYRO_YAW);
	
	if(BITTST(SoftSet,SOFT_FLT))
	{
		//Middle value filter
		//中值滤波
		GyroRolBuf[WARP(GyroRolIdx)]=GyroRol;
		GyroPitBuf[WARP(GyroPitIdx)]=GyroPit;
		GyroYawBuf[WARP(GyroYawIdx)]=GyroYaw;	
	
		GyroRol=Filter(GyroRolBuf);
		GyroPit=Filter(GyroPitBuf);
		GyroYaw=Filter(GyroYawBuf);
	}	
}

//###############################################
//
//   Read gain pots from 3 pots
//   读取三个感度电位器的信号
//
void GyroGainRead(void)
{
 	DIDR0=0x3F;		//0b00111111
	ADCSRB=0;
	GainRol=ReadAdc(GAIN_ROL)/8;	//0~127
	GainPit=ReadAdc(GAIN_PIT)/8;	//0~127
	GainYaw=ReadAdc(GAIN_YAW)/8;	//0~127
}
