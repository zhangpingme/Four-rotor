#include "KK_C.H"
#include "F_DELAY.H"
#include "F_EEROM.H"

//###############################################
//
// Setup subroutine 
//
//   Save on/off option
//   保存开关量设置
//
uchar SetupSave(uchar org,uchar cur,uint pos)
{
 	uchar mask=cur^org;//Find change 找到变化
	if(mask)	
	{		
	 	//Save it
	 	EEWriteB(pos,org=cur);
		
		//LED of/off 1s to indicate save ok
		//点亮/熄灭LED 1秒表示保存成功
		if(cur&mask)   LED0_ON();
		else		   LED0_OFF();
		Delay100ms(10);			
	}
	return org;
}

//###############################################
//
// Setup routine 1
//
//   Throttle calibration
//   油门校准
//
void SetupThrTravel(void)
{
 	uchar thrcnt;
	uchar thrmin;
	
	//Light LED to indicate thr cali mode
	//LED点亮表明进入油门校准
	LED0_ON();
	
	//Wait for thr goto highest again
	//等待油门杆再次拉高
	for(thrcnt=0;thrcnt<50;)
	{
	  	 //Read ppm signals
	  	 PpmReadSignal();		 
		 Delay1ms(20);
	
		 //Count if thr high
		 if(RxThr>110) thrcnt++;
		 else		   thrcnt=0;
	}
	
	//Loop to transfer thr signal
	//循环传送信号（直通）
	thrmin=120;
	while(1)
	{
		MotorControlBegin();	//Output head of ppm signal 输出PPM信号的头部分
			PpmReadSignal();	//Read rx 读取接收机信号
			Motor1=Motor2=Motor3=Motor4=MotorLimitValue(RxThr);//Transfer 直通
			if(thrmin>RxThr)	
			{
			 	thrmin=RxThr;
				EEWriteB(EE_THRLOW,thrmin);
			}
		MotorControlEnd();		//Output whole ppm signal 输出完整的PPM信号
				
	 	LED0_TOG();	   	   		//LED flash to indicate cali LED闪烁表示校准中
		Delay1ms(16);			//Make about 50Hz ppm freq 形成约50Hz PPM频率
	}
}

//###############################################
//
// Setup routine 2
//
//   Soft filter on/off
//   软件滤波器开关
//
void SetupFilter(void)
{
 	uchar rev;
	
	//Light LED to indicate soft filter setting
	//LED点亮表明进入软件滤波器设置
	LED0_ON();
	Delay100ms(10);	
	
	//Read origal setting
	//读出原有设置
 	SoftSet=rev=EEReadB(EE_SOFT_SET);
	
	//Loop
	while(1)
	{
	 	//Read ppm signal
		//读取PPM信号
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash to indicate working
		//LED闪烁表示可以开始调节
		LED0_TOG();
		
		//On/off if stick move
		//根据摇杆的高低设置对应参数的开关
		if(RxRud>STICKGATE) 	BITSET(rev,SOFT_FLT);  //Rud is filter on/off
		if(RxRud<-STICKGATE) 	BITCLR(rev,SOFT_FLT);
		
		if(RxEle>STICKGATE) 	BITSET(rev,SOFT_EXP);  //Ele is exp on/off
		if(RxEle<-STICKGATE) 	BITCLR(rev,SOFT_EXP);
				
		//If setting change,save it
		//如果改变了设置就保存
		SoftSet=SetupSave(SoftSet,rev,EE_SOFT_SET);
	}
}

//###############################################
//
// Setup routine 3
//
//   Gyro direction reversing
//   陀螺仪正反设置
//
void SetupGyroDir(void)
{
 	uchar rev;
	
	//Light LED to indicate gyro rev setup mode
	//LED点亮表明进入陀螺仪正反向设置
	LED0_ON();
	Delay100ms(10);
	
	//Read origal setting
	//读出原有设置
 	DevRev=rev=EEReadB(EE_GYRO_REV);
	
	//Loop
	while(1)
	{
		PpmReadSignal();			//Read ppm signal 读取PPM信号		
		GyroGainRead();				//Read gain 读取感度电位器 670us		
		Delay1ms(10+GainYaw/5);	//Flash freq decided by yaw pot 闪烁频率根据方向感度电位器来定
		
		//LED flash to indicate working
		//LED闪烁表示可以开始调节
		LED0_TOG();
		
		//Reverse if stick move
		//根据摇杆的高低设置正向或反向
		if(RxAil>STICKGATE) 	BITSET(rev,GYRO_ROL);
		if(RxAil<-STICKGATE) 	BITCLR(rev,GYRO_ROL);
		
		if(RxEle>STICKGATE) 	BITSET(rev,GYRO_PIT);
		if(RxEle<-STICKGATE) 	BITCLR(rev,GYRO_PIT);
		
		if(RxRud>STICKGATE) 	BITSET(rev,GYRO_YAW);
		if(RxRud<-STICKGATE) 	BITCLR(rev,GYRO_YAW);
		
		//If setting change,save it
		//如果改变了设置就保存
		DevRev=SetupSave(DevRev,rev,EE_GYRO_REV);
	}
}

//###############################################
//
// Setup routine 4
//
//   Axis mode
//   四轴模式
//
void SetupAxisMode(void)
{
 	uchar mode;
	
	//Light LED to indicate axis setting
	//LED点亮表明进入飞控模式设定
	LED0_ON();
	Delay100ms(10);
	
	//Read origal setting
	//读出原有设置
 	AxisMode=mode=EEReadB(EE_AXIS);
	
	//Loop
	while(1)
	{
	 	//Read ppm signal
		//读取PPM信号
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash to indicate working
		//LED闪烁表示可以开始调节
		LED0_TOG();
		
		//Set mode by stick
		//根据摇杆的高低设置设定模式
		if(RxRud>STICKGATE) 	mode=AXIS_CROSS;
		if(RxRud<-STICKGATE) 	mode=AXIS_X;
		
		//If setting change,save it
		//如果改变了设置就保存
		AxisMode=SetupSave(AxisMode,mode,EE_AXIS);
	}
}

//###############################################
//
// Setup routine 5
//
//   Reset to empty setting
//   还原回无设置状态
//
void SetupResetAll(void)
{
 	uint i;
	for(i=0;i<512;i++)	EEWriteB(i,0xFF);
	
	//Light LED to indicate finished
	//LED点亮表明设置清空完毕
	LED0_ON();
	while(1);
}

//###############################################
//
//   LED flash
//   LED闪烁
//
void LedFlash(uchar cnt)
{
 	while(cnt--)
	{
	 	LED0_ON();	Delay1ms(50);
		LED0_OFF();	Delay1ms(200);
	}
}

//###############################################
//
//   Setup routine & Config load
//   设置程序 & 读取设置
//
void Setup(void)
{
 	uchar idx,i;
	
	//Record power on times
	//记录上电次数
	EEWriteW(EE_PWR_ON,EEReadW(EE_PWR_ON)+1);
	
 	DevRev 	 =EEReadB(EE_GYRO_REV);
 	SoftSet	 =EEReadB(EE_SOFT_SET);
	AxisMode =EEReadB(EE_AXIS);
	RxThrLow =EEReadB(EE_THRLOW)+10;
				
	//Test gyro pos-neg compe
	if(EEReadB(EE_GYR_PN)=='P')
	{
	 	GyroRolPN=EEReadB(EE_GYR_PN+1);
		GyroPitPN=EEReadB(EE_GYR_PN+2);
		GyroYawPN=EEReadB(EE_GYR_PN+3);
	}
	
 	//If thr stick is high, enter setup
	//如果油门摇杆很高，进入设置，否则返回
 	if(RxThr<PPM_HIGH)
	{		
	 	return;
	}
	
	//Turn off LED 1s for recognize from PPM waiting
	//关闭LED 1秒与PPM等待区分开
	LED0_OFF();
	Delay100ms(10);
	
	//Loop every settings
	//循环设置项
	i=idx=100;//Set i=100 to trigger condition in loop
	while(1)
	{
	 	//Read ppm signal
		//读取PPM信号
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash interval 100 times(2s)
		//LED 闪烁提示的间隔为2秒
		if(i++>100)
		{
			i=0;   				 //Reset i
			if(++idx>5) idx=1;	 //Loop idx from 1 to 4
			
		 	//Flash idx to show subfunction
			//闪烁idx次来提示设置项
			LedFlash(idx);		
		}
		
		//If thr stick pulled low, enter item adjust
		//如果油门杆拉低，进入对应设置
		if(RxThr<30)	  
		{
			switch(idx)
			{
			 	case 1:	SetupThrTravel();	break;	//1.Motor thr cali		 
				case 2:	SetupFilter();		break;	//2.Software filter on/off
				case 3:	SetupGyroDir();		break;	//3.Gyro direction reverse
				case 4: SetupAxisMode();	break;	//4.Axis mode
				case 5: SetupResetAll();	break;	//5.Reset to empty
			}
		}
	}
}
