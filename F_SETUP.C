#include "KK_C.H"
#include "F_DELAY.H"
#include "F_EEROM.H"

//###############################################
//
// Setup subroutine 
//
//   Save on/off option
//   ���濪��������
//
uchar SetupSave(uchar org,uchar cur,uint pos)
{
 	uchar mask=cur^org;//Find change �ҵ��仯
	if(mask)	
	{		
	 	//Save it
	 	EEWriteB(pos,org=cur);
		
		//LED of/off 1s to indicate save ok
		//����/Ϩ��LED 1���ʾ����ɹ�
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
//   ����У׼
//
void SetupThrTravel(void)
{
 	uchar thrcnt;
	uchar thrmin;
	
	//Light LED to indicate thr cali mode
	//LED����������������У׼
	LED0_ON();
	
	//Wait for thr goto highest again
	//�ȴ����Ÿ��ٴ�����
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
	//ѭ�������źţ�ֱͨ��
	thrmin=120;
	while(1)
	{
		MotorControlBegin();	//Output head of ppm signal ���PPM�źŵ�ͷ����
			PpmReadSignal();	//Read rx ��ȡ���ջ��ź�
			Motor1=Motor2=Motor3=Motor4=MotorLimitValue(RxThr);//Transfer ֱͨ
			if(thrmin>RxThr)	
			{
			 	thrmin=RxThr;
				EEWriteB(EE_THRLOW,thrmin);
			}
		MotorControlEnd();		//Output whole ppm signal ���������PPM�ź�
				
	 	LED0_TOG();	   	   		//LED flash to indicate cali LED��˸��ʾУ׼��
		Delay1ms(16);			//Make about 50Hz ppm freq �γ�Լ50Hz PPMƵ��
	}
}

//###############################################
//
// Setup routine 2
//
//   Soft filter on/off
//   �����˲�������
//
void SetupFilter(void)
{
 	uchar rev;
	
	//Light LED to indicate soft filter setting
	//LED�����������������˲�������
	LED0_ON();
	Delay100ms(10);	
	
	//Read origal setting
	//����ԭ������
 	SoftSet=rev=EEReadB(EE_SOFT_SET);
	
	//Loop
	while(1)
	{
	 	//Read ppm signal
		//��ȡPPM�ź�
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash to indicate working
		//LED��˸��ʾ���Կ�ʼ����
		LED0_TOG();
		
		//On/off if stick move
		//����ҡ�˵ĸߵ����ö�Ӧ�����Ŀ���
		if(RxRud>STICKGATE) 	BITSET(rev,SOFT_FLT);  //Rud is filter on/off
		if(RxRud<-STICKGATE) 	BITCLR(rev,SOFT_FLT);
		
		if(RxEle>STICKGATE) 	BITSET(rev,SOFT_EXP);  //Ele is exp on/off
		if(RxEle<-STICKGATE) 	BITCLR(rev,SOFT_EXP);
				
		//If setting change,save it
		//����ı������þͱ���
		SoftSet=SetupSave(SoftSet,rev,EE_SOFT_SET);
	}
}

//###############################################
//
// Setup routine 3
//
//   Gyro direction reversing
//   ��������������
//
void SetupGyroDir(void)
{
 	uchar rev;
	
	//Light LED to indicate gyro rev setup mode
	//LED����������������������������
	LED0_ON();
	Delay100ms(10);
	
	//Read origal setting
	//����ԭ������
 	DevRev=rev=EEReadB(EE_GYRO_REV);
	
	//Loop
	while(1)
	{
		PpmReadSignal();			//Read ppm signal ��ȡPPM�ź�		
		GyroGainRead();				//Read gain ��ȡ�жȵ�λ�� 670us		
		Delay1ms(10+GainYaw/5);	//Flash freq decided by yaw pot ��˸Ƶ�ʸ��ݷ���жȵ�λ������
		
		//LED flash to indicate working
		//LED��˸��ʾ���Կ�ʼ����
		LED0_TOG();
		
		//Reverse if stick move
		//����ҡ�˵ĸߵ������������
		if(RxAil>STICKGATE) 	BITSET(rev,GYRO_ROL);
		if(RxAil<-STICKGATE) 	BITCLR(rev,GYRO_ROL);
		
		if(RxEle>STICKGATE) 	BITSET(rev,GYRO_PIT);
		if(RxEle<-STICKGATE) 	BITCLR(rev,GYRO_PIT);
		
		if(RxRud>STICKGATE) 	BITSET(rev,GYRO_YAW);
		if(RxRud<-STICKGATE) 	BITCLR(rev,GYRO_YAW);
		
		//If setting change,save it
		//����ı������þͱ���
		DevRev=SetupSave(DevRev,rev,EE_GYRO_REV);
	}
}

//###############################################
//
// Setup routine 4
//
//   Axis mode
//   ����ģʽ
//
void SetupAxisMode(void)
{
 	uchar mode;
	
	//Light LED to indicate axis setting
	//LED������������ɿ�ģʽ�趨
	LED0_ON();
	Delay100ms(10);
	
	//Read origal setting
	//����ԭ������
 	AxisMode=mode=EEReadB(EE_AXIS);
	
	//Loop
	while(1)
	{
	 	//Read ppm signal
		//��ȡPPM�ź�
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash to indicate working
		//LED��˸��ʾ���Կ�ʼ����
		LED0_TOG();
		
		//Set mode by stick
		//����ҡ�˵ĸߵ������趨ģʽ
		if(RxRud>STICKGATE) 	mode=AXIS_CROSS;
		if(RxRud<-STICKGATE) 	mode=AXIS_X;
		
		//If setting change,save it
		//����ı������þͱ���
		AxisMode=SetupSave(AxisMode,mode,EE_AXIS);
	}
}

//###############################################
//
// Setup routine 5
//
//   Reset to empty setting
//   ��ԭ��������״̬
//
void SetupResetAll(void)
{
 	uint i;
	for(i=0;i<512;i++)	EEWriteB(i,0xFF);
	
	//Light LED to indicate finished
	//LED������������������
	LED0_ON();
	while(1);
}

//###############################################
//
//   LED flash
//   LED��˸
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
//   ���ó��� & ��ȡ����
//
void Setup(void)
{
 	uchar idx,i;
	
	//Record power on times
	//��¼�ϵ����
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
	//�������ҡ�˺ܸߣ��������ã����򷵻�
 	if(RxThr<PPM_HIGH)
	{		
	 	return;
	}
	
	//Turn off LED 1s for recognize from PPM waiting
	//�ر�LED 1����PPM�ȴ����ֿ�
	LED0_OFF();
	Delay100ms(10);
	
	//Loop every settings
	//ѭ��������
	i=idx=100;//Set i=100 to trigger condition in loop
	while(1)
	{
	 	//Read ppm signal
		//��ȡPPM�ź�
		PpmReadSignal();		
		Delay1ms(20);
		
		//LED flash interval 100 times(2s)
		//LED ��˸��ʾ�ļ��Ϊ2��
		if(i++>100)
		{
			i=0;   				 //Reset i
			if(++idx>5) idx=1;	 //Loop idx from 1 to 4
			
		 	//Flash idx to show subfunction
			//��˸idx������ʾ������
			LedFlash(idx);		
		}
		
		//If thr stick pulled low, enter item adjust
		//������Ÿ����ͣ������Ӧ����
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