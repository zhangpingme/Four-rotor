//###############################################
//
// Author  : Gale
// Date    : 2011-4
// Email   : galemx@126.com
// Location: FuZhou FuJian China
//
#include "KK_C.H"
#include "F_DELAY.H"

//--------------------------------------
//##Motor drive signal 马达驱动信号
uchar Motor1,Motor2,Motor3,Motor4; //Motor signals 各电机信号
//###############################################
//
//   Motor drive limit
//   电机驱动限幅
//
uchar MotorLimitValue(int v)
{
 	if(v>PPM_MAX)	return PPM_MAX;
	if(v<0)			return 0;
	return v;
}

//###############################################
//
//   Motor signal output
//   输出电机信号
//
//-----Consist of two function: MotorControlBegin & MotorControlEnd
//-----  MotorControlEnd set all signal-pin to high, and ppm need
//-----  a 1000us delay, so I use this period to do something else,
//-----  when MotorControlEnd execute, it delay to 1000us exactly,
//-----  then output ppm signal
//-----它由两个函数构成：MotorControlBegin 和 MotorControlEnd
//-----  MotorControlBegin 将所有信号脚拉高，由于PPM需要1000us的一个
//-----  延时，所以我们不妨利用这段延时去干点别的事情，等MotorControlEnd
//-----  执行的时候，它先精确延时到100us，然后再输出ppm信号
void MotorControlBegin(void)				
{
	//Reset softtimer
	TimerRst();
	
	//Rise all motor signal
 	MOTOR1_H();
	MOTOR2_H();
	MOTOR3_H();
	MOTOR4_H();
}
//######
void MotorControlEnd(void)
{		
	uchar Cnt;
	 
	//Wait until 1000us
	TimerTo(1000);
	TimerRst();
	
 	//8us per cycle
	for(Cnt=0;Cnt<125;Cnt++)
	{
 	 	if(Cnt>Motor1)	  MOTOR1_L();
 		else			  MOTOR1_H();
		if(Cnt>Motor2)	  MOTOR2_L();
		else			  MOTOR2_H();
		if(Cnt>Motor3)	  MOTOR3_L();
		else			  MOTOR3_H();
		if(Cnt>Motor4)	  MOTOR4_L();
		else			  MOTOR4_H();
		
		//Patch cycle to 8us
		TimerTo(8);
		TimerRst();
	}
	
 	MOTOR1_L();
	MOTOR2_L();
	MOTOR3_L();
	MOTOR4_L();
}
