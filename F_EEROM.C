#include "F_EEROM.H"

#define WDR()	 	  asm("wdr")

void EEWriteB(uint uiAddress, uchar ucData)
{
	/* �ȴ���һ��д�������� */
	while(EECR & (1<<EEWE));

	/* ���õ�ַ�����ݼĴ��� */
	EEAR = uiAddress;
	EEDR = ucData;
	
	/* ��λEEMWE */
	EECR |= (1<<EEMWE);

	/* ��λEEWE������д���� */
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
	/* �ȴ���һ��д�������� */
	while(EECR & (1<<EEWE));
	/* ���õ�ַ�Ĵ��� */
	EEAR = uiAddress;
	/* ����EERE������������ */
	EECR |= (1<<EERE);
	/* �����ݼĴ����������� */
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