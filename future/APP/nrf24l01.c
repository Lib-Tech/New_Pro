#include "stm32f10x.h"
#include "string.h"
#include "spi.h"

#define TX_PLOAD_WIDTH  20  	// 20 uints TX payload

u8 TxBuf[32]={0};
u8 RxBuf[32]={0};
///******************************************************************************************
//uint   sta;   //状态标志
//#define	RX_DR	sta & 0x40
//#define	TX_DS	sta & 0x20
//#define	MAX_RT	sta & 0x10
///****************************************IO端口定义***************************************

//Chip Enable Activates RX or TX mode
//#define CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_10) 
//#define CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_10)

//SPI Chip Select
//#define CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_11)
//#define CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_11)

//#define CE_H()   ( GPIOB->ODR |= (1<<10) )
//#define CE_L()   ( GPIOB->ODR &= ~(1<<10) )
//#define CSN_H()  ( GPIOB->ODR |= (1<<11) )
//#define CSN_L()  ( GPIOB->ODR &= ~(1<<11) )


///*********************************************NRF24L01*************************************
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width

#define RX_PLOAD_WIDTH  20  	// 20 uints TX payload
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};	//本地地址
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};	//接收地址
///***************************************NRF24L01寄存器指令*******************************************************
#define READ_REGI        0x00  	// 读寄存器指令
#define WRITE_REGI       0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
///*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
///**************************************************************************************

u8 SPI_WR_Reg(u8 reg, u8 value);
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 Len);
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 Len);
u8 nRF24L01_RxPacket(unsigned char* rx_buf);
void nRF24L01_TxPacket(unsigned char * tx_buf);
u8 SPI_RD_Reg(u8 reg);

#define NRF24SPI_Send_Byte   SPI_Send_Byte

///******************************************************************************************
///*延时函数,非精确延时
///******************************************************************************************/
void Delay_us(u32 n)
{
	u32 i;
	
	while(n--)
	{
 	   i=2;
 	   while(i--);
  }
}

///****************************************************************************************
///*NRF24L01初始化
///***************************************************************************************/
void init_NRF24L01(void)
{
	//u8 buf[5]={0};
	
	Delay_us(100);
	
	Enable_RF();    // chip enable
	NotSelect_RF();   // Spi disable 		SPI_Read_Buf(TX_ADDR, buf, TX_ADR_WIDTH);//debug 测试原来的本地地址：复位值是：0xE7 0xE7 0xE7 0xE7 0xE7
  
//	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
//	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址

//	
//	SPI_WR_Reg(WRITE_REG + EN_AA, 0x01);      //  频道0自动	ACK应答允许	
//	SPI_WR_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
//	SPI_WR_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 设置自动重发时间和次数：500us + 86us, 10 retrans...
//	SPI_WR_Reg(WRITE_REG + RF_CH, 22);        //   设置信道工作为2.4GHZ，收发必须一致
//	SPI_WR_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
//	SPI_WR_Reg(WRITE_REG + RF_SETUP, 0x07);   		//设置发射速率为1MHZ，发射功率为最大值0dB
//  
//  SPI_RD_Reg(WRITE_REG + EN_AA);
//  SPI_RD_Reg(WRITE_REG + EN_RXADDR);
//  SPI_RD_Reg(WRITE_REG + RF_CH);
//  SPI_RD_Reg(WRITE_REG + RX_PW_P0);
//  SPI_RD_Reg(WRITE_REG + RF_SETUP);
}

///****************************************************************************************************
///*函数：uchar SPI_Read(u8 reg)
///*功能：NRF24L01的SPI时序
///****************************************************************************************************/
u8 SPI_RD_Reg(u8 reg)
{
	u8 reg_val;
	
	Select_RF();                 // CSN low, initialize SPI communication...
	NRF24SPI_Send_Byte(reg);            // Select register to read from..
	reg_val = NRF24SPI_Send_Byte(0);    // ..then read registervalue
	NotSelect_RF();                // CSN high, terminate SPI communication
	
	return(reg_val);        // return register value
}

//****************************************************************************************************/
//*功能：NRF24L01读写寄存器函数
//****************************************************************************************************/
u8 SPI_WR_Reg(u8 reg, u8 value)
{
	u8 status;
	
	Select_RF();                   // CSN low, init SPI transaction
	status = NRF24SPI_Send_Byte(reg);// select register
	NRF24SPI_Send_Byte(value);             // ..and write value to it..
	NotSelect_RF();                   // CSN high again
	
	return(status);            // return nRF24L01 status u8
}
///****************************************************************************************************/
//*函数：uint SPI_Read_Buf(u8 reg, u8 *pBuf, u8 Len)
//*功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
//****************************************************************************************************/
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 Len)
{
	u32 status,i;
	
	Select_RF();                    		// Set CSN low, init SPI tranaction
	status = NRF24SPI_Send_Byte(reg);       		// Select register to write to and read status u8	
	for(i=0;i<Len;i++)
	{
		pBuf[i] = NRF24SPI_Send_Byte(0);
	}	
	NotSelect_RF();                           	
	return(status);                    // return nRF24L01 status u8
}
//*********************************************************************************************************
//*函数：uint SPI_Write_Buf(u8 reg, u8 *pBuf, u8 Len)
//*功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
//*********************************************************************************************************/
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 Len)
{
	u32 status,i;
	
	Select_RF();            //SPI使能       
	status = NRF24SPI_Send_Byte(reg);   
	for(i=0; i<Len; i++) //
	{
	  	NRF24SPI_Send_Byte(*pBuf);
		pBuf ++;
	}
	NotSelect_RF();           //关闭SPI
	return(status);    // 
}

//****************************************************************************************************/
//*函数：void SetRX_Mode(void)
//*功能：数据接收配置 
//****************************************************************************************************/
void RX_Mode(void)
{
	u8 buf[5]={0};
	Disable_RF();//CE_L();
	
	SPI_Read_Buf(TX_ADDR, buf, TX_ADR_WIDTH);//debug 测试原来的本地地址：复位值是：0xE7 0xE7 0xE7 0xE7 0xE7
	//SPI_WR_Reg(WRITE_REGI + CONFIG, 0x03);//SPI_WR_Reg(WRITE_REG + CONFIG, 0x0f); // IRQ收发完成中断响应，16位CRC	，主接收
	
	//SPI_Write_Buf(WRITE_REGI + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	SPI_Write_Buf(WRITE_REGI + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址
	
	SPI_WR_Reg(WRITE_REGI + EN_AA, 0);//0x01);      //  频道0自动	ACK应答允许	
	SPI_WR_Reg(WRITE_REGI + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
	SPI_WR_Reg(WRITE_REGI + SETUP_RETR, 0x1a); // 设置自动重发时间和次数：500us + 86us, 10 retrans...
	SPI_WR_Reg(WRITE_REGI + RF_CH, 40);        //   设置信道工作为2.4GHZ，收发必须一致
	SPI_WR_Reg(WRITE_REGI + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	SPI_WR_Reg(WRITE_REGI + RF_SETUP, 0x0F);   		//设置发射速率为1MHZ，发射功率为最大值0dB
	
	SPI_WR_Reg(WRITE_REGI + CONFIG, 0x03);//0x0F);	
	Enable_RF();//CE_H(); 
	Delay_us(200);
}

void TX_Mode(void)
{
 	Disable_RF();//CE_L();
	//SPI_WR_Reg(WRITE_REG + CONFIG, 0x02);   //0x0E		 // IRQ收发完成中断响应，16位CRC，主发送
	
	SPI_Write_Buf(WRITE_REGI + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	SPI_Write_Buf(WRITE_REGI + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址	
	SPI_WR_Reg(WRITE_REGI + EN_AA, 0);//0x01);      //  频道0自动	ACK应答允许	
	SPI_WR_Reg(WRITE_REGI + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
	SPI_WR_Reg(WRITE_REGI + SETUP_RETR, 0x1a); // 设置自动重发时间和次数：500us + 86us, 10 retrans...
	SPI_WR_Reg(WRITE_REGI + RF_CH, 40);        //   设置信道工作为2.4GHZ，收发必须一致
//	SPI_WR_Reg(WRITE_REGI + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	SPI_WR_Reg(WRITE_REGI + RF_SETUP, 0x0F);   		//设置发射速率为1MHZ，发射功率为最大值0dB

	SPI_WR_Reg(WRITE_REGI + CONFIG, 0x02);//0x0E);
 	Enable_RF();//CE_H();
}
//***********************************************************************************************************
//*函数：void nRF24L01_TxPacket(unsigned char * tx_buf)
//*功能：发送 tx_buf中数据
//**********************************************************************************************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	Disable_RF();			//StandBy I模式	CE=0
	SPI_Write_Buf(WRITE_REGI + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
//	SPI_WR_Reg(WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
	Enable_RF();		 //置高CE，激发数据发送
	//Delay_us(10);
}

//******************************************************************************************************/
//*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
//*功能：数据读取后放如rx_buf接收缓冲区中
//******************************************************************************************************/
u8 nRF24L01_RxPacket(unsigned char* rx_buf)
{
	u8 flag=0;
	u8 status;
 	
	status=SPI_RD_Reg(NRFRegSTATUS);	// 读取状态寄存其来判断数据接收状况	
	if(status & 0x40)				// 判断是否接收到数据
	{
	   //CE_L(); 			//SPI使能
		 SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		 flag =1;			//读取数据完成标志
	}
	SPI_WR_Reg(WRITE_REGI+NRFRegSTATUS, status);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	return flag;
}


///************************************主函数************************************************************
void NRF_Receive(void)
{     
 	//static u32 counter=0;
	//static u8 flag=0;   
   	//TX_Mode();
   	//IntervalTimems(10); 
   	if(nRF24L01_RxPacket(RxBuf))            //if 数据读取成功
   	{
		if( memcmp(RxBuf, "1234567890abcdefghij", 20)==0 )
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_11);
		}
		else if( memcmp(RxBuf, "abcdefghij1234567890", 20)==0 )
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_11);
		}
      
   	}  
   	memset(RxBuf, 0, 20);
   //memcpy(TxBuf, "1234567890abcdefghijklmn", 20);
   //nRF24L01_TxPacket(TxBuf);
}

void NRF_Send(void)
{
	//static u32 counter=0;
	static u8 flag=0;  
	//TX_Mode();	
	//IntervalTimems(100);
	if(flag==0)
	{
		flag=1;
		memcpy(TxBuf, "1234567890abcdefghij", 20);
		nRF24L01_TxPacket(TxBuf);
	}
	else
	{
		flag=0;
		memcpy(TxBuf, "abcdefghij1234567890", 20);
		nRF24L01_TxPacket(TxBuf);
	}
	//GPIO_Write(GPIOC, (u16)~GPIO_ReadOutputData(GPIOC));
}
