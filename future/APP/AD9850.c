//=========================AD9850.c=================================//
#include "stm32f10x_lib.h"
#include "AD9850.h"
/********************************************
�������ƣ�AD9850_Delay
��    �ܣ�AD9850��ʱ����
��    ����z - ��ʱ����
����ֵ  ����
*********************************************/
void AD9850_Delay(unsigned int z)
{
    for(;z>0;z--)
    {;}
}
/*------------------------����ģʽ-------------------------*/
/********************************************
�������ƣ�AD9850_Reset_Sreial
��    �ܣ��ڴ���ģʽ�¸�λAD9850
��    ������
����ֵ  ����
*********************************************/
void AD9850_Reset_Serial(void)
{
    AD9850_WCLK_CLR();
    AD9850_FQUD_CLR();
    //RST�ź�
    AD9850_RST_CLR();
    AD9850_RST_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_RST_CLR();
    //WCLK�ź�
    AD9850_WCLK_CLR();
    AD9850_WCLK_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_WCLK_CLR();
    //FQUD�ź�
    AD9850_FQUD_CLR();
    AD9850_FQUD_SET();
    AD9850_Delay(0xFFFF) ;
    AD9850_FQUD_CLR();
	//AD9850_Write_Serial(0,0,0);
}
/********************************************
�������ƣ�AD9850_Write_Serial
��    �ܣ��ڴ���ģʽ��дAD9850�Ĵ���
			W0-W31:Freq b0-b31;W32-W33:Control,�������ԣ�����Ϊ0;W34:Power-down;W35-W39:phase b0-b4
��    ����W0 - W0�Ĵ�����ֵ
          freq - Ƶ��ֵ
����ֵ  ����
*********************************************/
void AD9850_Write_Serial(unsigned long freq,unsigned char phase,unsigned char PDN)
{
    unsigned char i,wdata ;
    unsigned long  y ;
	double x;
	x=4294967296/125;  //2^32/125
    y =(x*freq)/999983;  
    
    wdata = y>>0 ;   //дw4
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
    wdata = y>>8 ;  //дw3
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
    wdata = y>>16 ;  //дw2
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
    wdata = y>>24 ;  //дw1
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
    wdata = (phase<<3) | (PDN<<2) | 0x00;  //дw0
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
    AD9850_FQUD_SET();  //����ʹ��
    AD9850_Delay(0xFFFF) ;
    AD9850_FQUD_CLR();
}

/*------------------------����ģʽ-------------------------*/
/*---------------------------------------------------------*/
/********************************************
�������ƣ�Init_AD9834
��    �ܣ���ʼ��AD9834
��    ������
����ֵ  ����
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
