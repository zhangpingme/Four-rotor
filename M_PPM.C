#include "KK_C.H"
#include "F_DELAY.H"

//#######################################
// RxChStart? is PPM rising time 各通道PPM上升沿时刻
// RxCh? is PPM value 各通道的PPM时间
// RxCh?Base is neutral value 各通道的中立点时间 
uint RxChStart1,RxChStart2,RxChStart3,RxChStart4;
uint RxCh1,RxCh2,RxCh3,RxCh4;
#define RxCh1Base PPM_SIG_NEU
#define RxCh2Base PPM_SIG_NEU
#define RxCh3Base PPM_SIG_LOW
#define RxCh4Base PPM_SIG_NEU

//#######################################
//Rx??? is stick position, -125~125, thr is 0~125
//Rx???是各摇杆位置，取值范围从-125~+125，油门为0~125
schar RxAil,RxEle,RxRud;
uchar RxThr,RxThrLow;

//#######################################
//Busy flag, when true the RxCh? is invalid
//接收忙标志，为1的时候说明RxCh?不可读	 
bool RxChBusy;	

//#######################################
//Signal flag, each bit indicate a ch is got
//信号标志，每一位代表1个通道的信号被收到	 
uchar RxValid;	

//#######################################
//  Isr subroutine declare
//  中断服务声明
#pragma interrupt_handler IsrRxCh1:iv_PCINT2
#pragma interrupt_handler IsrRxCh2:iv_INT0
#pragma interrupt_handler IsrRxCh3:iv_INT1
#pragma interrupt_handler IsrRxCh4:iv_PCINT0

//#######################################
void IsrRxCh1(void)
{
 	BITSET(RxValid,0);
 	if(PIND&2) //Rising 上升沿
	{
	 	RxChStart1=TCNT1;
	}
	else//Falling 下降沿
	{
	 	RxChBusy=1;
	 	RxCh1=TCNT1-RxChStart1;
		RxChBusy=0;
	}
}

//#######################################
void IsrRxCh2(void)
{
 	BITSET(RxValid,1);
 	if(PIND&4) //Rising 上升沿
	{
	 	RxChStart2=TCNT1;
	}
	else//Falling 下降沿
	{
	 	RxChBusy=1;
	 	RxCh2=TCNT1-RxChStart2;
		RxChBusy=0;
	}
}

//#######################################
void IsrRxCh3(void) 
{
 	BITSET(RxValid,2);
 	if(PIND&8) //Rising 上升沿
	{
	 	RxChStart3=TCNT1;
	}
	else//Falling 下降沿
	{
	 	RxChBusy=1;
	 	RxCh3=TCNT1-RxChStart3;
		RxChBusy=0;
	}
}

//#######################################
void IsrRxCh4(void)
{
 	BITSET(RxValid,3);
 	if(PINB&0x80) //Rising 上升沿
	{
	 	RxChStart4=TCNT1;
	}
	else//Falling 下降沿
	{
	 	RxChBusy=1;
	 	RxCh4=TCNT1-RxChStart4;
		RxChBusy=0;
	}
}

//#######################################
//
// Limit ppm value to -125~125
// 将PPM信号值限定在-125~125之间
//
schar LimitPpmValue(int v)
{
 	if(v>PPM_MAX)	return PPM_MAX;
	if(v<PPM_MIN)	return PPM_MIN;
	return v;
}

//#######################################
//
// Read channel value from RX
// 读取接收机各个通道的值
//
void PpmReadSignal(void)
{
 	int t;
	
 	while(RxChBusy) WDR();
	t=RxCh1;		  	   	 	//900~2100
	t-=RxCh1Base;	  			//about -600~+600
	RxAil=LimitPpmValue(t/4);	//about -125~+125	
	
 	while(RxChBusy) WDR();
	t=RxCh2;
	t-=RxCh2Base;
	RxEle=LimitPpmValue(t/4);
	
 	while(RxChBusy) WDR();
	t=RxCh3;		  //900~2100
	t-=RxCh3Base;	  //about 0~1200
	if(t>0)	RxThr=LimitPpmValue(t/8);//about 0~125
	else	RxThr=0;
	
 	while(RxChBusy) WDR();
	t=RxCh4;
	t-=RxCh4Base;
	RxRud=LimitPpmValue(t/4);
}

//###############################################
//
//   Wait for all ch is got, LED will flash until got
//   等待所有通道的信号都收到，LED将一直闪烁直到收到
// 
void PpmWaitSignal(void)
{
 	 uchar rxcnt;
	 	 
	 //Wait for 5 times signal received
	 //等待收到5次信号
	 for(rxcnt=RxValid=0;rxcnt<20;)
	 {
	  	 //Read ppm signals
	  	 PpmReadSignal();
		
	  	 //If got signal, inc rxcnt
	 	 if(RxValid==0x0f)
		 {
		  	RxValid=0;
			rxcnt++;
		}
	  	LED0_TOG();
		Delay1ms(50);
	}
}
