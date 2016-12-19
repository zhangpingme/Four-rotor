#ifndef _KK_C_H_
#define _KK_C_H_

#ifdef ATMega48
#include <iom48v.h>
#endif

#ifdef ATMega88P
#include <iom88v.h>
#endif

#ifdef ATMega168P
#include <iom168V.h>
#endif

typedef unsigned char bool;
typedef signed char schar;
typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

#define CLI()	 	  asm("cli")
#define SEI()	 	  asm("sei")
#define WDR()	 	  asm("wdr")

#define BIT(X)		  (1<<(X))
#define BITTST(X,Y)	  ((X)&BIT(Y))
#define BITSET(X,Y)	  ((X)|=BIT(Y))
#define BITCLR(X,Y)	  ((X)&=~BIT(Y))
#define BITTOG(X,Y)	  ((X)^=BIT(Y))	  	  

//###############################################
// Hardware defines
// 硬件参数定义
//LED0 : PORTB.6
	   #define LED0_ON()	  PORTB|=0x40
	   #define LED0_OFF()	  PORTB&=~0X40
	   #define LED0_TOG()	  BITTOG(PORTB,6)
//LED2 : PORTD.5
	   #define LED2_ON()	  PORTD|=0x20
	   #define LED2_OFF()	  PORTD&=~0X20
//LED3 : PORTD.6
	   #define LED3_ON()	  PORTD|=0x40
	   #define LED3_OFF()	  PORTD&=~0X40
//MOTOR OUT 1 : PORTB.2
//MOTOR OUT 2 : PORTB.1
//MOTOR OUT 3 : PORTB.0
//MOTOR OUT 4 : PORTD.7
//MOTOR OUT 5 : PORTD.6
//MOTOR OUT 6 : PORTD.5
		#define MOTOR1_H()  BITSET(PORTB,2)
		#define MOTOR2_H()  BITSET(PORTB,1)
		#define MOTOR3_H()  BITSET(PORTB,0)
		#define MOTOR4_H()  BITSET(PORTD,7)
		//#define MOTOR5_H()  BITSET(PORTD,6)
		//#define MOTOR6_H()  BITSET(PORTD,5)
		
		#define MOTOR1_L()  BITCLR(PORTB,2)
		#define MOTOR2_L()  BITCLR(PORTB,1)
		#define MOTOR3_L()  BITCLR(PORTB,0)
		#define MOTOR4_L()  BITCLR(PORTD,7)
		//#define MOTOR5_L()  BITCLR(PORTD,6)
		//#define MOTOR6_L()  BITCLR(PORTD,5)
//PPM IN CH1 : PORTD.1
//PPM IN CH2 : PORTD.2
//PPM IN CH3 : PORTD.3
//PPM IN CH4 : PORTB.7

#define GAIN_YAW	  0X05
#define GAIN_PIT	  0X04
#define GAIN_ROL	  0X03
#define GYRO_ROL	  0X02
#define	GYRO_PIT	  0X01
#define	GYRO_YAW	  0X00

//###############################################
// E2ROM config position
// E2ROM存储位置定义
#define EE_PWR_ON		0X00
#define EE_GYR_PN		0X02
#define EE_GYRO_REV		0X20
#define EE_SOFT_SET		0X21
#define EE_AXIS			0X22
#define EE_IDLE			0X23
#define EE_THRLOW		0X24
#define EE_MOTOR		0X30

//###############################################
//
//   Soft timer (0~2048us)
//   软定时器(0~2048us)
//
#define TimerRst()  TCNT2=0
#define TimerTo(X)	while(TCNT2<(X/8)) WDR()

//###############################################
// PPM signal define
// PPM 信号参数定义
#define	PPM_MAX			125
#define PPM_MIN			-125
#define PPM_HIGH		100
#define PPM_LOW			-100
#define PPM_SIG_HIGH	2000
#define PPM_SIG_NEU		1500
#define PPM_SIG_LOW		1000

//###############################################
// Options define
// 选项定义
#define	STICKGATE	   70	// Stick move gate when setting 设置时的摇杆摆动门限
#define SOFT_IDLE	 0X02
#define SOFT_EXP	 0X01
#define SOFT_FLT	 0X00

#define AXIS_CROSS	 0X00	//十字模式
#define AXIS_X		 0XFF	//X模式

//###############################################
// Functions and vars declaration
// 函数和变量声明

//in M_PPM.C
extern schar RxAil,RxEle,RxRud; 
extern uchar RxThr,RxThrLow;
void PpmReadSignal(void);
void PpmWaitSignal(void);

//in M_GYRO.C
extern int GyroBaseRol,GyroBasePit,GyroBaseYaw;
extern int GyroRol,GyroPit,GyroYaw;
extern schar GyroRolPN,GyroPitPN,GyroYawPN;
extern uchar GyroBaseCnt;
extern uchar GainRol,GainPit,GainYaw;			//0~127
extern int GyroRolI,GyroPitI,GyroYawI;			//Gyro integral 各陀螺仪积分值
#define GYROBASECNT 200	  						//Gyro base build times 陀螺仪中点建立次数 
void GyroRead(void);
int  GyroCompe(int gyro,schar pn);
int  GyroIntegral(int v,int delta);
void GyroGainRead(void);

//in KK_C.C
extern uchar InLock;
extern uchar DevRev;
extern uchar SoftSet;
extern uchar AxisMode;

//in M_MOTOR.C
extern uchar Motor1,Motor2,Motor3,Motor4;
extern uchar MotorIdle;
uchar MotorLimitValue(int v);
void MotorControlBegin(void);
void MotorControlEnd(void);

//in SETUP.C
void Setup(void);

#endif//_KK_C_H_