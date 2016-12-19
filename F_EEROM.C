#include "F_EEROM.H"

#define WDR()	 	  asm("wdr")

void EEWriteB(uint uiAddress, uchar ucData)
{
	/* 等待上一次写操作结束 */
	while(EECR & (1<<EEWE));

	/* 设置地址和数据寄存器 */
	EEAR = uiAddress;
	EEDR = ucData;
	
	/* 置位EEMWE */
	EECR |= (1<<EEMWE);

	/* 置位EEWE以启动写操作 */
	EECR |= (1<<EEWE);
	
	WDR();
}

void EEWriteW(uint uiAddress,uint uiData)
{
 	EEWriteB(uiAddress,uiData>>8);
	EEWriteB(uiAddress+1,uiData);
}

uchar EEReadB(uint uiAddress)
{
	/* 等待上一次写操作结束 */
	while(EECR & (1<<EEWE));
	/* 设置地址寄存器 */
	EEAR = uiAddress;
	/* 设置EERE以启动读操作 */
	EECR |= (1<<EERE);
	/* 自数据寄存器返回数据 */
	WDR();
	return EEDR;
}

uint EEReadW(uint uiAddress)
{
 	uint r;
 	r=EEReadB(uiAddress);
	r<<=8;
	r+=EEReadB(uiAddress+1);
	return r;
}
