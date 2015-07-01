#include <c8051f020.h>
#include <intrins.h>
#include "CPU_Init.h"
#include "FNT_GB16.h"

#define uchar unsigned char

#define Hang 8 //�еĵ���ģ����
#define Lie 2  //�еĵ���ģ����
#define Fre 100 //ɨ��֡��,�Ӷ�ÿ�е�Ƶ��Ϊ80Hz*8=640Hz
#define SYSCLOCK 22118400


sbit SI1A=P3^0;
sbit SI1B=P3^1;
sbit SI1C=P3^2;
sbit SI1D=P3^3;

sbit CLK=P3^4;
sbit RCK=P3^5;
sbit OE=P3^6;

unsigned char Counte_Hang=0x00;//��ɨ�������.
unsigned char counter=0x00;
unsigned char YiWei=0x00;

void delay(void);
void send_byte(uchar byte1,uchar byte2);
void out595(void);

void Timer3_Init (int counts);
void Timer3_ISR (void);

struct typFNT_GB16 *pointFNT;

void main(void)
{	
	WDTCN = 0xde; // ��ֹ���Ź���ʱ��
    WDTCN = 0xad;
	Init_Device();
	Timer3_Init(SYSCLOCK/12/800);
	EA=1;
	while(1)
	{;}


}

void send_byte(uchar byte1,uchar byte2)
{    
	uchar num1,num2,c; 
	num1=~byte1;
	num2=~byte2;	
	for(c=0;c<8;c++)
	{
		if(num1&0x01){SI1A=1;SI1C=0;}
		else {SI1A=0;SI1C=1;}
		num1=num1>>1;
		CLK=0;
		if(num2&0x01){SI1B=1;SI1D=0;}
		else {SI1B=0;SI1D=1;}
		num2=num2>>1;		
		CLK=1;
		
	}

}

void out595(void)
{
	RCK=0;
	_nop_ ();
	_nop_ ();
	_nop_ ();
	_nop_ ();
	_nop_ ();
	RCK=1;
}
/*
void Display(void)
{
	uchar i;
	pointFNT=GB_16;
	for(i=0;i<8;i++)
	{
		send_byte(pointFNT->Msk[2*i],pointFNT->Msk[2*i+16]);	
	}
}*/

/*************************************************************************************
**��������: Timer3_Init
**��������: T3��ʼ���Ӻ���
**�������: 
**�������: 
**˵    ��: 
*************************************************************************************/
void Timer3_Init (int counts)
{
   TMR3CN = 0x00;                  /*ֹͣT3,����TF3����*/
                                   /*ʹ�õ�ʱ����ϵͳʱ��12��Ƶ*/
   TMR3RL  = -counts;              /*�趨��ʱ����װ�ص�ֵ*/
   TMR3    = 0xffff;               /*����װ��TMR3RL�е�ֵ*/
   EIE2   |= 0x01;                 /*ʹ�ܶ�ʱ���ж�*/
   TMR3CN |= 0x04;                 /*Timer3��ʼ����*/
}
/*************************************************************************************
**��������: Timer3_ISR
**��������: T3���жϴ�����
**�������: 
**�������: 
**˵    ��: 
*************************************************************************************/
void Timer3_ISR (void) interrupt 14
{
 	static uchar i;
	static uchar j;
	uchar k;
	uchar a1,a2,b1,b2;
	static uchar a2_old[8];
	static uchar b2_old[8];
	static bit flag;
	EA=0;
	TMR3CN &= ~(0x80); /*��TF3�������*/
	//ɨ�蹦��ʵ��
	OE=1;//��ֹ138������ź�
	P5=Counte_Hang;
	counter=2*Counte_Hang;
	
	for(k=5;k>0;)
	{	
		k--;
		pointFNT=GB_16+j+k;	
		//send_byte(pointFNT->Msk[counter+1],pointFNT->Msk[counter+17]);
		//send_byte(pointFNT->Msk[counter],pointFNT->Msk[counter+16]);
		if(flag==1)
		{
			a1=pointFNT->Msk[counter+1];//��N���Ұ�λ
			b1=pointFNT->Msk[counter+17];//��N+8���Ұ�λ
			a2=pointFNT->Msk[counter];	//��N�����λ
			b2=pointFNT->Msk[counter+16];//��N+8�����λ
		}
		else
		{
			a1=(pointFNT+1)->Msk[counter];//��N���Ұ�λ
			b1=(pointFNT+1)->Msk[counter+16];//��N+8���Ұ�λ
			a2=pointFNT->Msk[counter+1];	//��N�����λ
			b2=pointFNT->Msk[counter+17];//��N+8�����λ
			//j++;
		}
		send_byte(a1<<YiWei|a2_old[Counte_Hang]>>(8-YiWei),b1<<YiWei|b2_old[Counte_Hang]>>(8-YiWei));
		send_byte(a2<<YiWei|a1>>(8-YiWei),b2<<YiWei|b1>>(8-YiWei));
		a2_old[Counte_Hang]=a2;
		b2_old[Counte_Hang]=b2;
		//b2_old=b2;
		
	}/*/
	/*pointFNT=GB_16+j+3;
	send_byte(pointFNT->Msk[counter+1],pointFNT->Msk[counter+17]);
	send_byte(pointFNT->Msk[counter],pointFNT->Msk[counter+16]);
	pointFNT=GB_16+j+2;
	send_byte(pointFNT->Msk[counter+1],pointFNT->Msk[counter+17]);
	send_byte(pointFNT->Msk[counter],pointFNT->Msk[counter+16]);
	pointFNT=GB_16+j+1;
	send_byte(pointFNT->Msk[counter+1],pointFNT->Msk[counter+17]);
	send_byte(pointFNT->Msk[counter],pointFNT->Msk[counter+16]);
	pointFNT=GB_16+j;
	send_byte(pointFNT->Msk[counter+1],pointFNT->Msk[counter+17]);
	send_byte(pointFNT->Msk[counter],pointFNT->Msk[counter+16]);
	/*
	for(i=Hang;i>0;i--)
	{
		send_byte(GB_16[4][2*Counte_Hang+1],GB_16[4][2*Counte_Hang+1+16]);
		send_byte(GB_16[4][2*Counte_Hang],GB_16[4][2*Counte_Hang+16]);
	}*/
	out595();//������ź�
	OE=0;//����5138������ź�
	if(Counte_Hang==7)Counte_Hang=0;
	else Counte_Hang++;
	i++;

	if(i>40)
	{	
		YiWei++;
		i=0;
		if(YiWei>7)
		{
			YiWei=0;
			flag=!flag;
			if(flag==1)j++;
		}
		if(j>11)j=0;
	}
	EA=1;
}
