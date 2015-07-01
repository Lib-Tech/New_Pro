//========================AD9850.h=============================//
/*--------------------------------------------
            AD9850��������
----------------------------------------------*/
#ifndef __AD9850_H
#define __AD9850_H
/*ѡ������ͨ�ŷ�ʽ��ֻ��ѡ��һ��*/
#define MODE_SEIAL   //����ģʽ

#define AD9850_SYSTEM_COLCK     150000000
/* AD9850 ���ƺ��������Ŷ��� */
#define AD9850_CONTROL_PORT  GPIOB

#define AD9850_FQUD   GPIO_Pin_0 //��FQUP
#define AD9850_WCLK   GPIO_Pin_9 //��WCLK
#define AD9850_RST    GPIO_Pin_10 //��RESET
#define AD9850_DATA   GPIO_Pin_11 //��D7
  
#define AD9850_DATA_Write_1()     GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_DATA,Bit_SET)
#define AD9850_DATA_Write_0()     GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_DATA,Bit_RESET)

#define AD9850_WCLK_SET()    	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_WCLK,Bit_SET)
#define AD9850_WCLK_CLR()    	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_WCLK,Bit_RESET)
#define AD9850_FQUD_SET()    	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_FQUD,Bit_SET)
#define AD9850_FQUD_CLR()    	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_FQUD,Bit_RESET)
#define AD9850_RST_SET()     	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_RST,Bit_SET)
#define AD9850_RST_CLR()     	GPIO_WriteBit(AD9850_CONTROL_PORT,AD9850_RST,Bit_RESET)

/* ����ģʽ�������� */
extern void AD9850_Reset_Serial(void) ;
extern void AD9850_Write_Serial(unsigned long freq,unsigned char phase,unsigned char PDN);
extern void Init_AD9850(void) ;
#endif /* AD8950_H */
