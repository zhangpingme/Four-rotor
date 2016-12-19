#include "F_DELAY.H"

void Delay10us(uchar c)
{
	while(c--)
	{
		uchar i;
		for(i=0;i<15;i++) asm("nop;");
	}
	WDR();
}

void Delay1ms(uchar c)
{
	while(c--)	Delay10us(100);
}

void Delay100ms(uchar c)
{
	while(c--)	Delay1ms(100);
}
