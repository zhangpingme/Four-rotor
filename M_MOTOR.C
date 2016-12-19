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
//##Motor drive signal ���������ź�
uchar Motor1,Motor2,Motor3,Motor4; //Motor signals ������ź�
//###############################################
//
//   Motor drive limit
//   ��������޷�
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
//   �������ź�
//
//-----Consist of two function: MotorControlBegin & MotorControlEnd
//-----  MotorControlEnd set all signal-pin to high, and ppm need
//-----  a 1000us delay, so I use this period to do something else,
//-----  when MotorControlEnd execute, it delay to 1000us exactly,
//-----  then output ppm signal
//-----���������������ɣ�MotorControlBegin �� MotorControlEnd
//-----  MotorControlBegin �������źŽ����ߣ�����PPM��Ҫ1000us��һ��
//-----  ��ʱ���������ǲ������������ʱȥ�ɵ������飬��MotorControlEnd
//-----  ִ�е�ʱ�����Ⱦ�ȷ��ʱ��100us��Ȼ�������ppm�ź�
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