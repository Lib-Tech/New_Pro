//=========================AD9850.c=================================//
#include "stm32f10x_lib.h"
#include "AD9850.h"
/********************************************
函数名称：AD9850_Delay
功    能：AD9850延时函数
参    数：z - 延时长度
返回值  ：无
*********************************************/
void AD9850_Delay(unsigned int z)
{
    for(;z>0;z--)
    {;}
}
/*------------------------串行模式-------------------------*/
/********************************************
函数名称：AD9850_Reset_Sreial
功    能：在串行模式下复位AD9850
参    数：无
返回值  ：无
*********************************************/
void AD9850_Reset_Serial(void)
{
    AD9850_WCLK_CLR();
    AD9850_FQUD_CLR();
    //RST信号
    AD9850_RST_CLR();
    AD9850_RST_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_RST_CLR();
    //WCLK信号
    AD9850_WCLK_CLR();
    AD9850_WCLK_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_WCLK_CLR();
    //FQUD信号
    AD9850_FQUD_CLR();
    AD9850_FQUD_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_FQUD_CLR();
	//AD9850_Write_Serial(0,0,0);
}
/********************************************
函数名称：AD9850_Write_Serial
功    能：在串行模式下写AD9850寄存器
			W0-W31:Freq b0-b31;W32-W33:Control,工厂测试，必须为0;W34:Power-down;W35-W39:phase b0-b4
参    数：W0 - W0寄存器的值
          freq - 频率值
返回值  ：无
*********************************************/
void AD9850_Write_Serial(unsigned long freq,unsigned char phase,unsigned char PDN)
{
    unsigned char i,wdata ;
    unsigned long  y ;
	double x;
	x=4294967296/125;  //2^32/125
    y =(x*freq)/999983;  
    
    wdata = y>>0 ;   //写w4
    for(i=0 ;i<8 ;i++)
    {
        if(wdata & 0x01)
          AD9850_DATA_Write_1();
        else
          AD9850_DATA_Write_0();
        AD9850_WCLK_SET();
        wdata >>= 1 ;
        AD9850_WCLK_CLR();
    }
    wdata = y>>8 ;  //写w3
     for(i=0 ;i<8 ;i++)
    {
        if(wdata & 0x01)
          AD9850_DATA_Write_1();
        else
          AD9850_DATA_Write_0();
        AD9850_WCLK_SET();
        wdata >>= 1 ;
        AD9850_WCLK_CLR();
    }
    wdata = y>>16 ;  //写w2
     for(i=0 ;i<8 ;i++)
    {
        if(wdata & 0x01)
          AD9850_DATA_Write_1();
        else
          AD9850_DATA_Write_0();
        AD9850_WCLK_SET();
        wdata >>= 1 ;
        AD9850_WCLK_CLR();
    }
    wdata = y>>24 ;  //写w1
     for(i=0 ;i<8 ;i++)
    {
        if(wdata & 0x01)
          AD9850_DATA_Write_1();
        else
          AD9850_DATA_Write_0();
        AD9850_WCLK_SET();
        wdata >>= 1 ;
        AD9850_WCLK_CLR();
    }
    wdata = (phase<<3) | (PDN<<2) | 0x00;  //写w0
     for(i=0 ;i<8 ;i++)
    {
        if(wdata & 0x01)
          AD9850_DATA_Write_1();
        else
          AD9850_DATA_Write_0();
        AD9850_WCLK_SET();
        wdata >>= 1 ;
        AD9850_WCLK_CLR();
    }   
    AD9850_FQUD_SET();  //移入使能
    AD9850_Delay(0xFFFF) ;
    AD9850_FQUD_CLR();
}

/*------------------------串行模式-------------------------*/
/*---------------------------------------------------------*/
/********************************************
函数名称：Init_AD9834
功    能：初始化AD9834
参    数：无
返回值  ：无
*********************************************/
void Init_AD9850(void)
{
    GPIO_InitTypeDef GPIO_InitStructure ;
    GPIO_InitStructure.GPIO_Pin = AD9850_WCLK | AD9850_FQUD | AD9850_RST | AD9850_DATA ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(AD9850_CONTROL_PORT ,&GPIO_InitStructure) ;
    
    AD9850_Reset_Serial() ;
    
}
