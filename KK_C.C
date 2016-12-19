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
//##Gyro signal 陀螺仪信号
uchar DevRev;	 				//Gyro rev flags 陀螺仪信号反转标识
uchar SoftSet;					//Fly control software setting 飞控软件设置
uchar AxisMode;					//Fly control mode 飞控模式

//--------------------------------------
//##Arm 锁定
bool InLock=1;	//model is in lock 模型处于锁定状态
uchar ArmCnt=0;	//Count for arm/disarm

//###############################################
//
//   Stick exp 
//   摇杆指数调整
//
int StickExp(int stk)
{
 	uchar neg=(stk<0);
	
	stk*=stk;
	stk/=128;
	
	if(neg) stk=-stk;
	
	return stk;
}

//###############################################
//
//   Stick angle limit
//   摇杆角度限幅
//
int StickLimitValue(int v)
{
 	if(v>PPM_MAX)	return PPM_MAX;
	if(v<0)			return 0;
	return v;
}

//###############################################
//
//   Board initialization
//   系统初始化
//
void Init(void)
{
 	//Disable all int 禁用全部中断
	CLI();	   
	
	//Port direction 设置端口方向
	DDRB=0x7F;		 //0b01111111
	DDRC=0xC0;		 //0b11000000
	DDRD=0xF1;		 //0b11110001
	
	//Interrupt setting 设置中断
	PCICR=0x05;		 //0b00000101 PB7,PB1
	PCMSK0=0x80;	 //0b10000000 PB7
	PCMSK2=0x02;	 //0b00000010 PD1
	EICRA=0x05;		 //0b00000101 PD2,PD3
	EIMSK=0x03;		 //0b00000011 PD2,PD3
	
	//Set timer1 to 1M 将定时器1设为1M
	TCCR1B=0x02; 	// 1/8 sysclk, 1us count
	
	//Set timer2 to 8us count
	TCCR2B=4;		// 1/64 sysclk,8us count
			
	//Enable interrupts 打开中断
	SEI();
	
	//Delay to avoid power jitter 
	//延时2秒避开电源不稳定阶段
	Delay100ms(20);
}

//###############################################
//
//   Test arming/disarming
//   判断是否需要锁定/解锁
//----Hold rud stick for a while, ARMING_TIME should mul main-loop cycle
//----保持方向摇杆一会后解/锁，ARMING_TIME乘以主循环周期就是时间
//
#define ARMING_TIME		  250
void ArmingRoutine(void)
{
 	//Count for anti-jitter 锁定消抖动
	if(RxRud<-STICKGATE || RxRud>STICKGATE)	ArmCnt++;
	else 				   					ArmCnt=0;
			
	//Hold rud stick for a while, the num should mul main-loop cycle
	if(ArmCnt>ARMING_TIME)		
	{ 	
		if(InLock)
		{
		 	if(RxRud>STICKGATE)	
			{
			 	GyroBaseCnt=GYROBASECNT;	
			 	InLock=0;
			}
		}
		else
		{
		 	if(RxRud<-STICKGATE)		InLock=1;
		}
	}
}

//###############################################
//
//   Gain scale (return gyro*gain/128)
//   感度调整
//
int GainAdj(int gyro,uchar gain)
{
 	int r;
	r=gyro/8;
	r*=gain;		   
	return r/(128/8);
}

//###############################################
//
//   Caculate plane attitude
//   计算飞行器姿态
//
void CaclAttitude(void)
{
	GyroRead();
	
	//If no gyro base, calibrate gyro
	//如果还未建立基准，建立它
	if(GyroBaseCnt)
	{			
	 	GyroBaseRol+=GyroRol;
		GyroBaseRol/=2;
		
		GyroBasePit+=GyroPit;
		GyroBasePit/=2;
		
		GyroBaseYaw+=GyroYaw;
		GyroBaseYaw/=2;
		
		GyroBaseCnt--;
		if(!(GyroBaseCnt&7)) LED0_TOG();	//Shine LED show gyro cali 闪烁LED表示在进行陀螺仪校准
		
		GyroRolI=GyroPitI=GyroYawI=0;//Reset I value 清空积分值
	}
	else
	{		
		if(InLock)
		{
		 	//熄灭LED表示在锁定中
			LED0_OFF();
		}
		else
		{
	 	 	//Remove base part from gyro value
			//减去基础值
	 		GyroRol-=GyroBaseRol;
	 		GyroPit-=GyroBasePit;
	 		GyroYaw-=GyroBaseYaw;
	
			//Reverse gyro signals if necessary
			//根据设置反转各个陀螺仪信号
			if(BITTST(DevRev,GYRO_ROL))	  GyroRol=-GyroRol;
			if(BITTST(DevRev,GYRO_PIT))	  GyroPit=-GyroPit;
			if(BITTST(DevRev,GYRO_YAW))	  GyroYaw=-GyroYaw;
			
			//Gyro feature compensation
			//陀螺仪特性补偿
			GyroRol=GyroCompe(GyroRol,GyroRolPN);
			GyroPit=GyroCompe(GyroPit,GyroPitPN);
			GyroYaw=GyroCompe(GyroYaw,GyroYawPN);
			
			//Sum integral value with return
			//带回归计算积分值		
			GyroRolI=GyroIntegral(GyroRolI,GyroRol);
			GyroPitI=GyroIntegral(GyroPitI,GyroPit);
			GyroYawI=GyroIntegral(GyroYawI,GyroYaw);
		
			//Light LED
			//点亮LED表示在工作中
	 		LED0_ON();
		}
	}
}

//###############################################
//
//   Axis signal mixer
//   计算电机输出信号
//
void AxisMixer(void)
{
 	int thr,ail,ele,rud; 
	
	//Stick exp 
	//摇杆指数
	if(BITTST(SoftSet,SOFT_EXP))
	{
	 	//Rudder do not need exp
	 	thr=StickExp(RxThr);
	 	ail=StickExp(RxAil)/2;
	 	ele=StickExp(RxEle)/2;
	}
	else
	{
	 	thr=RxThr;	
	 	ail=RxAil/4;
	 	ele=RxEle/4;
	}	
		
	//Add gyro to adjustment
	//将陀螺仪信号累加到调节量上
	ail+=GainAdj(GyroRol,GainRol)+GainAdj(GyroRolI,GainPit);
	ele+=GainAdj(GyroPit,GainRol)+GainAdj(GyroPitI,GainPit);
	rud=RxRud/4+GainAdj(GyroYaw,GainYaw);//+GainAdj(GyroYawI,GainPit);

	if(AxisMode==AXIS_CROSS)
	{
	 	// + Mode 十字模式
		//       1  
		//     3 + 2
		//       4	
	 	Motor1=MotorLimitValue(thr - ele + rud);
	 	Motor2=MotorLimitValue(thr - ail - rud);
	 	Motor3=MotorLimitValue(thr + ail - rud);
	 	Motor4=MotorLimitValue(thr + ele + rud);
	}
	else
	{
	 	// X Mode X模式
		//     1   2
		//       X
		//     3   4
	 	Motor1=MotorLimitValue(thr + ail - ele + rud);
	 	Motor2=MotorLimitValue(thr - ail - ele - rud);
	 	Motor3=MotorLimitValue(thr + ail + ele - rud);
	 	Motor4=MotorLimitValue(thr - ail + ele + rud);
	}
}

//###############################################
//
//   Main routine
//   主程序
//
void main(void)
{	
 	Init();	   	 		//Board init 初始化系统
	PpmWaitSignal();	//Wait rx signal, led will flash 等待接收机的信号，等待时LED闪烁
	Setup();			//Load & Adjust parameters 加载&调节参数
	
	//Main loop 主循环
 	LED0_OFF();
	while(1)
	{		
		TimerRst();
			CaclAttitude();	//Caculate plane attitude 计算飞行器姿态	
		TimerTo(1000);
				
		//Do something between MotorControlBegin() and MotorControlEnd
		//See functions declaration, can not exceed 1000us
		//在MotorControlBegin()和MotorControlEnd()之间干点事儿，注意不要超过1000us
		MotorControlBegin();	//Output head of ppm signal 输出PPM信号的头部分
			PpmReadSignal();	//Read rx 读取接收机信号 140us
			if(RxThr<RxThrLow)	//If thr shutdown 如果油门关闭
			{	
				GyroGainRead();		//Read gain 读取感度电位器 670us		
				ArmingRoutine();	//Arm/disarm 加锁解锁测试 5us
			}
			else
			{
				AxisMixer();		//Cacl motor signal 计算电机信号 305us
			}		
			
			//If locked(arm) or no gyro base, shutdown all motor
			//如果处于锁定态或者陀螺仪基准未建立，关闭所有马达
			if(InLock || GyroBaseCnt || RxThr<5)
			{
	 		 	Motor1=Motor2=Motor3=Motor4=0;
			}		
			//MOTOR6_L();			//I use it for test execute period 我用来观察执行时间的		
		MotorControlEnd();		//Output whole ppm signal 输出完整的PPM信号
	}
}