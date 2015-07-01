#include <c8051f020.h>
#include <intrins.h>
#include "CPU_Init.h"
#include "FNT_GB16.h"

#define uchar unsigned char

#define Hang 8 //行的点阵模块数
#define Lie 2  //列的点阵模块数
#define Fre 100 //扫描帧率,从而每行的频率为80Hz*8=640Hz
#define SYSCLOCK 22118400


sbit SI1A=P3^0;
sbit SI1B=P3^1;
sbit SI1C=P3^2;
sbit SI1D=P3^3;

sbit CLK=P3^4;
sbit RCK=P3^5;
sbit OE=P3^6;

unsigned char Counte_Hang=0x00;//行扫描计数器.
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
	WDTCN = 0xde; // 禁止看门狗定时器
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
**函数名称: Timer3_Init
**函数功能: T3初始化子函数
**输入参数: 
**输出参数: 
**说    明: 
*************************************************************************************/
void Timer3_Init (int counts)
{
   TMR3CN = 0x00;                  /*停止T3,并对TF3清零*/
                                   /*使用的时基是系统时钟12分频*/
   TMR3RL  = -counts;              /*设定定时器重装载的值*/
   TMR3    = 0xffff;               /*立即装入TMR3RL中的值*/
   EIE2   |= 0x01;                 /*使能定时器中断*/
   TMR3CN |= 0x04;                 /*Timer3开始运行*/
}
/*************************************************************************************
**函数名称: Timer3_ISR
**函数功能: T3的中断处理函数
**输入参数: 
**输出参数: 
**说    明: 
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
	TMR3CN &= ~(0x80); /*对TF3清零操作*/
	//扫描功能实现
	OE=1;//禁止138输出行信号
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
			a1=pointFNT->Msk[counter+1];//第N行右八位
			b1=pointFNT->Msk[counter+17];//第N+8行右八位
			a2=pointFNT->Msk[counter];	//第N行左八位
			b2=pointFNT->Msk[counter+16];//第N+8行左八位
		}
		else
		{
			a1=(pointFNT+1)->Msk[counter];//第N行右八位
			b1=(pointFNT+1)->Msk[counter+16];//第N+8行右八位
			a2=pointFNT->Msk[counter+1];	//第N行左八位
			b2=pointFNT->Msk[counter+17];//第N+8行左八位
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
	out595();//输出列信号
	OE=0;//允许5138输出行信号
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
