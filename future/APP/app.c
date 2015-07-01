/****************************************************************************
* Copyright (C), 2011 �ܶ�Ƕ��ʽ������ ourstm.5d6d.com
*
* �������� �ܶ���STM32������Tiny�ϵ���ͨ��           
* QQ: 9191274, ������sun68, Email: sun68@163.com 
* �Ա����̣�ourstm.taobao.com  
*
* �ļ���: app.c
* ���ݼ���:
*       �����̲���ϵͳ����ucos2.86a�汾�� ������5������
			������											 ���ȼ�
			APP_TASK_START_PRIO                               2	        ������	  		
            Task_Com1_PRIO                                    4			COM1ͨ������
            Task_Led1_PRIO                                    7			LED1 ��˸����
            Task_Led2_PRIO                                    8			LED2 ��˸����
            Task_Led3_PRIO                                    9			LED3 ��˸����
		 ��Ȼ��������ϵͳ����
		    OS_TaskIdle                  ��������-----------------���ȼ����
			OS_TaskStat                  ͳ������ʱ�������-------���ȼ��ε�
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2011-05-26 sun68  �������ļ�
*


���ڷܶ���STM32 Tiny ��NRF24L01תUSB���ڽӿڰ壩
�ܶ�STM32��������̳��http://ourstm.5d6d.com	
                     http://www.ourstm.net
�ܶ�STM32 QQ����4Ⱥ: 133971340�����޷ܶ����û���
�ܶ�STM32 QQ����3Ⱥ���߼�Ⱥ��: 115132365������
�ܶ�STM32 QQ����1Ⱥ���߼�Ⱥ��: 42465044������
�ܶ�STM32 QQ����2Ⱥ���߼�Ⱥ��: 105680620������
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define GLOBALS

#include "stdarg.h"
 
#include "includes.h"
#include "globals.h"

#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

#include "spi.h"
#include "lcd_dis24.h"
//#include "fsmc_sram.h"
//#include "nrf24l01.h"
#include "sensor.h"
//#include "AD9850.h"

#include "oled.h"

OS_EVENT* Com1_SEM;
OS_EVENT* Com1_MBOX;  

OS_EVENT *SeMbox;   //���巢����Ϣ��������ָ��
OS_EVENT *Key_BOX;   //���巢����Ϣ��������ָ��  

tg_HMC5883L_TYPE hmc5883l;
tg_MPU6050_g_TYPE MPU6050_g;
tg_MPU6050_a_TYPE MPU6050_a;
tg_EulerAngle_TYPE EulerAngle_Current;
tg_EulerAngle_TYPE EulerAngle_Before; 
tg_Quaternion_TYPE Q;
//static unsigned long Freq_SET=0;


#define FREQ_LCD_ADD_X 10*12
#define FREQ_LCD_ADD_Y 5*24

#define ADC1_DR_Address    ((u32)0x4001244C)

void LED_N_STATUS(unsigned char N,unsigned char STATUS);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
extern void fun_para(void);

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_STK App_TaskStartStk[APP_TASK_START_STK_SIZE];
static  OS_STK Task_Com1Stk[Task_Com1_STK_SIZE];
static  OS_STK Task_Led1Stk[Task_Led1_STK_SIZE];
static  OS_STK Task_Led2Stk[Task_Led2_STK_SIZE];
static  OS_STK Task_Led3Stk[Task_Led3_STK_SIZE];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void App_TaskCreate(void);

static  void App_TaskStart(void* p_arg);

static  void Task_Com1(void* p_arg);
static  void Task_Led1(void* p_arg);
static  void Task_Led2(void* p_arg);
static  void Task_Led3(void* p_arg);
/*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument : none.
*
* Return   : none.
*********************************************************************************************************/

int main(void)
{
	CPU_INT08U os_err;
	
	//��ֹCPU�ж�
	CPU_IntDis();
	
	//UCOS ��ʼ��
	OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel".         */
	
	//Ӳ��ƽ̨��ʼ��
	BSP_Init();                                                 /* Initialize BSP functions.  */
	/* Configure FSMC Bank1 NOR/PSRAM */

	I2C_Ini();
    HMC5883L_Init();
	HMC5883L_Start();
	MPU6050_Init();	 

		
   //���������� ���ȼ����  ���������������һ����;��Ϊ���Ժ�ʹ��ͳ������
   os_err = OSTaskCreate((void (*) (void *)) App_TaskStart,	  		  		//ָ����������ָ��
                          (void *) 0,								  		//����ʼִ��ʱ�����ݸ�����Ĳ�����ָ��
               (OS_STK *) &App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],	//���������Ķ�ջ��ջ��ָ��   �Ӷ����µݼ�
               (INT8U) APP_TASK_START_PRIO);								//�������������ȼ�
   os_err =os_err;
   //ucos�Ľ��ļ�������0    ���ļ�������0-4294967295    ���ڽ���Ƶ��100hzʱ�� ÿ��497������¼��� 
   OSTimeSet(0);
   OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II).  */
                                                 /* Start multitasking (i.e. give control to uC/OS-II).  */
 
   return (0);
}




/*
*********************************************************************************************************
*                                          App_TaskStart()
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return   : none.
*
* Caller   : This is a task.
*
* Note     : none.
*********************************************************************************************************
*/

static  void App_TaskStart(void* p_arg)//�����ȡ������
{

	(void) p_arg;
	
	//��ʼ��ucosʱ�ӽ���
	OS_CPU_SysTickInit();                                       /* Initialize the SysTick.       */
	
	//ʹ��ucos ��ͳ������
	#if (OS_TASK_STAT_EN > 0)
	//----ͳ�������ʼ������  
	OSStatInit();                                               /* Determine CPU capacity.                              */
	#endif
	LED_Init();
	//��������������
	//LED_P8x16Str(0,0,"Hello",0);	
	//LED_P8x16Str(0,1,"Dolphin",0);
	//AD9850_Write_Serial(10000,0,0);
	App_TaskCreate();

	while (1)
	{
		
		MPU6050_Read_GYRO(&MPU6050_g);	//780us
		MPU6050_Read_ACCEL(&MPU6050_a);	//
		HMC5883L_MultRead(&hmc5883l);	//
		LED_N_STATUS(1,1);
		AHRS_position(&hmc5883l,&MPU6050_a,&MPU6050_g,&EulerAngle_Current);//164us for fast invsqrt 256us for 1.0 / sqrt.
		//350us with EulerAngle
		//IMU_getYawPitchRoll(&EulerAngle_Current);//178us
		LED_N_STATUS(1,0);
		OSTimeDlyHMSM(0, 0, 0, 10);//5ms	
		//LED_N_STATUS(1,1);	
	}
}


//----------------------------------------
static  void Task_Led1(void* p_arg)//���������̬
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	(void) p_arg;
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE); 
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				                 //LED1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	    
	while (1)
	{		
		
		
		//LED_N_STATUS(1,0);		
		UART1_ReportIMU(&EulerAngle_Current);//1.5ms
		//LED_N_STATUS(1,0);
		OSTimeDlyHMSM(0, 0, 0, 40);
	}
}
//----------------------------------------
static  void Task_Led2(void* p_arg)//����ɼ���ѹ
{
	GPIO_InitTypeDef GPIO_InitStructure;

	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
  
	INT16U shu[4]={0}; 
	INT16U *s; 

	vu16 ADC_ConvertedValue;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* DMA channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);	
	/* Enable DMA channel1 */
	//DMA_Cmd(DMA1_Channel1, ENABLE);	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//����ģʽ  CR1
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;						    //ɨ��ģʽʹ��
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//����ת��ģʽʹ��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//����ת�����ⲿ�¼�--��  CR2
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ת����������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//ת����ͨ����Ϊ1
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	/* ADC1 regular channel14 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);	  	//ͨ��10����ʱ��														  
	//ADC_DMACmd(ADC1, ENABLE);		                          						  //����ADC1����DMA����	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);														  //ʹ��ADC1	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);													  //����ADC1��λУ׼�Ĵ��� 
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));									  //���У׼�Ĵ����Ƿ�λ��� 	
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);													  //����ADC1 У׼
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));									      //���У׼�Ƿ���� 	
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);										  //�����������ADC1ת��


	(void) p_arg;	    
	while (1)
	{
		//while(0 == ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
		if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
		{
			shu[2]=shu[1];	
			shu[1]=shu[0];
			shu[0]=ADC_GetConversionValue(ADC1);//���ADC1��ת������
		}
	
		
		//shu[3]=0xff;						
		//USART_OUT(USART1,"ADC_%d!\r\n",shu[0]);
		s=&shu[0]; //ȡ��shu�еĵ�ַ����ָ��s 
		OSMboxPost(SeMbox,s); 		
		OSTimeDlyHMSM(0, 0, 0, 500);	
	}
}

//----------------------------------------
static  void Task_Led3(void* p_arg)//������ʾ
{
	INT8U err; 
	//INT16U *s; 
	int ADC_tmp=0;
   	//unsigned int tmp=0;

	//s=OSMboxPend(SeMbox,0,&err);

	(void) p_arg;	    
	while (1)
	{
		//ADC_tmp= (int)(fb[0] * 10);
		ADC_tmp= (int)(EulerAngle_Current.Repx * 10000);
		LED_P8x16Str(0 + 32 ,0 , ".", 1);
		LED_PrintChar(0 + 0 ,0,ADC_tmp/10);
		LED_P8x16Char(0 + 40,0,(unsigned int)(abs(ADC_tmp)%10)+48); 
		//ADC_tmp= (int)(fb[1] * 10);
		ADC_tmp= (int)(EulerAngle_Current.Repy * 10000);
		LED_PrintChar(0  + 64,0,ADC_tmp/10);
		LED_P8x16Char(64 + 40,0,(unsigned int)(abs(ADC_tmp)%10)+48); 
		LED_P8x16Str(64 + 32 ,0 , ".", 1);
		ADC_tmp= (int)(EulerAngle_Current.Repz * 10000);
		//ADC_tmp= (int)(fb[2] * 10);
		LED_P8x16Str(0 + 32 ,1 , ".", 1);
		LED_PrintChar(0  + 0,1,ADC_tmp/10);
		LED_P8x16Char(0 + 40,1,(unsigned int)(abs(ADC_tmp)%10)+48);   	
		ADC_tmp=(int)(EulerAngle_Current.YAW * 10);//hmc5883l.m_a;//EulerAngle_Current
		LED_P8x16Str(0 + 32 ,2 , ".", 1);
		LED_PrintChar(0 + 0,2,ADC_tmp/10);
		LED_P8x16Char(0 + 40,2,(unsigned int)(abs(ADC_tmp)%10)+48); 
		ADC_tmp=(int)(EulerAngle_Current.PITCH * 10);//hmc5883l.m_a;//EulerAngle_Current
		LED_PrintChar(0  + 64,2,ADC_tmp/10);
		LED_P8x16Char(64 + 40,2,(unsigned int)(abs(ADC_tmp)%10)+48); 
		LED_P8x16Str(64 + 32 ,2 , ".", 1);
		ADC_tmp=(int)(EulerAngle_Current.ROLL*10);//hmc5883l.m_a;//EulerAngle_Current
		LED_P8x16Str(0 + 32 ,3 , ".", 1);
		LED_PrintChar(0  + 0,3,ADC_tmp/10);
		LED_P8x16Char(0 + 40,3,(unsigned int)(abs(ADC_tmp)%10)+48);   
					
	}
}

//++++++++++++COM1��������++++++++++++++++++++++++++
static  void Task_Com1(void *p_arg)
{     
   	INT8U err;   				  
	INT8U *s; 
	INT8U i;
	INT8U flag=0;
	static unsigned long d=0;
	static unsigned int freq_tmp=0;
	static unsigned int freq_tmp1=0;
	INT8U ADD_x=FREQ_LCD_ADD_X;  
	s=OSMboxPend(Com1_MBOX,0,&err);	
	//Init_AD9850();   	
	(void)p_arg;	
	while(1)
	{	
		//USART_OUT(USART1,"Task_Com1\r\n");
		if(*s=='L')
		{
			for(i=1;i<50;i++)
			{
				if(*(s+i)!='F')
				{
					d=d*10+(*(s+i)-0x30);
				}
				else
				{
					//AD9850_Write_Serial(d,0,0);
					//KEY_FLAG=0;
					//Freq_SET=d;
					//if(d/1000000>0)
					//{
					freq_tmp=d/1000000;
					if(freq_tmp>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"USART is %d",freq_tmp);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);
					ADD_x+=12;						
					freq_tmp=d%1000000/1000;
					freq_tmp1=(freq_tmp)/100;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						//USART_OUT(USART1,"%d",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);

					ADD_x+=12;
					freq_tmp1=(freq_tmp%100)/10;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"%d",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);
				
					ADD_x+=12;
					freq_tmp1=(freq_tmp%100)%10;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"%d",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);
					ADD_x+=12;
					freq_tmp=d%1000;
					freq_tmp1=(freq_tmp)/100;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"%d",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);
					ADD_x+=12;
					freq_tmp1=(freq_tmp%100)/10;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"%d",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);

					ADD_x+=12;
					freq_tmp1=(freq_tmp%100)%10;
					if(freq_tmp1>0)flag=1;
					if(flag==1)
					{
						USART_OUT(USART1,"%dHz\r\n",freq_tmp1);
						//TFT_PutChar12x24(ADD_x,FREQ_LCD_ADD_Y,freq_tmp1+0x30,0xffff,0x0000);							
					}
					//else GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y," ",1,0xffff,0x0000);
					
					ADD_x+=12;

					//GUI_Text12x24(ADD_x,FREQ_LCD_ADD_Y,"Hz",2,0xffff,0x0000);
					//}

					//TFT_PutChar12x24(00,200-24,tmp+0x30,0xffff,0x0000);
					//GUI_Text12x24(48,200-24,"KHz",3,0xffff,0x0000);
//					Select_DA();
//					SPI_SendData(d<<6);
//					NotSelect_DA();
					break;	
				}
			}
				
			*s=0;
			d=0;
			ADD_x=FREQ_LCD_ADD_X;
			flag=0;
			OSMboxPost(Com1_MBOX,(void *)&s);
		}
		OSTimeDlyHMSM(0, 0, 0, 200);
	}
}
/*
*********************************************************************************************************
*                                            App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument : none.
*
* Return   : none.
*
* Caller   : App_TaskStart().
*
* Note     : none.
*********************************************************************************************************
*/

static  void App_TaskCreate(void)
{
   //CPU_INT08U os_err;
 
   //Com1_SEM=OSSemCreate(1);		     //��������1�жϵ��ź���
   	Com1_MBOX=OSMboxCreate((void *) 0);		     //��������1�жϵ���Ϣ����
   	SeMbox = OSMboxCreate((void *)0);//����һ������ 
	Key_BOX= OSMboxCreate((void *)0);//����һ������
	//Key_MBOX=OSMboxCreate((void *)0);
   
   
   //����1���ռ���������---------------------------------------------------------    
   OSTaskCreateExt(Task_Com1,									  //ָ����������ָ��
   					(void *)0,									  //����ʼִ��ʱ�����ݸ�����Ĳ�����ָ��
					(OS_STK *)&Task_Com1Stk[Task_Com1_STK_SIZE-1],//���������Ķ�ջ��ջ��ָ��   �Ӷ����µݼ�
					Task_Com1_PRIO,								  //�������������ȼ�
					Task_Com1_PRIO,								  //Ԥ�����Ժ�汾�������ʶ���������а汾ͬ�������ȼ�
					(OS_STK *)&Task_Com1Stk[0],					  //ָ�������ջջ�׵�ָ�룬���ڶ�ջ�ļ���
                    Task_Com1_STK_SIZE,							  //ָ����ջ�����������ڶ�ջ�ļ���
                    (void *)0,									  //ָ���û����ӵ��������ָ�룬������չ�����������ƿ�
                    OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);	  //ѡ�ָ���Ƿ������ջ���飬�Ƿ񽫶�ջ��0,�����Ƿ�Ҫ���и�������ȵȡ�
   //LED1 ��˸����------------------------------------------------------
   OSTaskCreateExt(Task_Led1,(void *)0,(OS_STK *)&Task_Led1Stk[Task_Led1_STK_SIZE-1],Task_Led1_PRIO,Task_Led1_PRIO,(OS_STK *)&Task_Led1Stk[0],
                    Task_Led1_STK_SIZE,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
   
   //LED2 ��˸����------------------------------------------------------
   OSTaskCreateExt(Task_Led2,(void *)0,(OS_STK *)&Task_Led2Stk[Task_Led2_STK_SIZE-1],Task_Led2_PRIO,Task_Led2_PRIO,(OS_STK *)&Task_Led2Stk[0],
                    Task_Led2_STK_SIZE,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR); 
   
   //LED3 ��˸����------------------------------------------------------
   OSTaskCreateExt(Task_Led3,
   					(void *)0,
					(OS_STK *)&Task_Led3Stk[Task_Led3_STK_SIZE-1],
					Task_Led3_PRIO,
					Task_Led3_PRIO,
					(OS_STK *)&Task_Led3Stk[0],
                    Task_Led3_STK_SIZE,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);  
}

void LED_N_STATUS(unsigned char N,unsigned char STATUS)
{
	switch(N)
	{
		case 1:
			if(STATUS==0)
			{
				GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			}
			else
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_13);			
			}
		break;
		case 2:
			if(STATUS==0)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
			 	GPIO_SetBits(GPIOB, GPIO_Pin_1);
			}
			else
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
			 	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);
				GPIO_ResetBits(GPIOA, GPIO_Pin_1);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);			
			}
		break;
		case 3:
			if(STATUS==0)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
			 	GPIO_SetBits(GPIOB, GPIO_Pin_1);
			}
			else
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
			 	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
				GPIO_ResetBits(GPIOA, GPIO_Pin_2);			
			}
		break;

	}
}


int SendChar (int ch)  
{                /* Write character to Serial Port     */
	
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}


/******************************************************
		��ʽ�������������
        "\r"	�س���	   USART_OUT(USART1, "abcdefg\r")   
		"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
		"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
{ 

	const char *s;
    int d;
   
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0)
	{				                          //�ж��Ƿ񵽴��ַ���������
		if(*Data==0x5c)						  //'\'
		{									  
			switch (*++Data){
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);	   
					Data++;
					break;
				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;			
				default:
					Data++;
				    break;
			}			 
		}
		else if(*Data=='%')
		{							
			switch (*++Data){				
				case 's':										  //�ַ���
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) 
					{
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //ʮ����
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) 
					{
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else 
		{
			USART_SendData(USARTx, *Data++);
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
			//GUI_Text12x24(48,100," Hz",3,0xffff,0x0000);
		}
		
	}
}

/******************************************************
		��������ת�ַ�������
        char *itoa(int value, char *string, int radix)
		radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

	    ����d=-379;
		ִ��	itoa(d, buf, 10); ��
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */


/*
*********************************************************************************************************
*********************************************************************************************************
*                                          uC/OS-II APP HOOKS
*********************************************************************************************************
*********************************************************************************************************
*/

#if (OS_APP_HOOKS_EN > 0)
/*
*********************************************************************************************************
*                                      TASK CREATION HOOK (APPLICATION)
*
* Description : This function is called when a task is created.
*
* Argument : ptcb   is a pointer to the task control block of the task being created.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/

void App_TaskCreateHook(OS_TCB* ptcb)
{
}

/*
*********************************************************************************************************
*                                    TASK DELETION HOOK (APPLICATION)
*
* Description : This function is called when a task is deleted.
*
* Argument : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/

void App_TaskDelHook(OS_TCB* ptcb)
{
   (void) ptcb;
}

/*
*********************************************************************************************************
*                                      IDLE TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskIdleHook(), which is called by the idle task.  This hook
*               has been added to allow you to do such things as STOP the CPU to conserve power.
*
* Argument : none.
*
* Note     : (1) Interrupts are enabled during this call.
*********************************************************************************************************
*/

#if OS_VERSION >= 251
void App_TaskIdleHook(void)
{
}
#endif

/*
*********************************************************************************************************
*                                        STATISTIC TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskStatHook(), which is called every second by uC/OS-II's
*               statistics task.  This allows your application to add functionality to the statistics task.
*
* Argument : none.
*********************************************************************************************************
*/

void App_TaskStatHook(void)
{
}

/*
*********************************************************************************************************
*                                        TASK SWITCH HOOK (APPLICATION)
*
* Description : This function is called when a task switch is performed.  This allows you to perform other
*               operations during a context switch.
*
* Argument : none.
*
* Note     : 1 Interrupts are disabled during this call.
*
*            2  It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                   will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the
*                  task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/

#if OS_TASK_SW_HOOK_EN > 0
void App_TaskSwHook(void)
{
}
#endif

/*
*********************************************************************************************************
*                                     OS_TCBInit() HOOK (APPLICATION)
*
* Description : This function is called by OSTCBInitHook(), which is called by OS_TCBInit() after setting
*               up most of the TCB.
*
* Argument : ptcb    is a pointer to the TCB of the task being created.
*
* Note     : (1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/

#if OS_VERSION >= 204
void App_TCBInitHook(OS_TCB* ptcb)
{
   (void) ptcb;
}
#endif

#endif
