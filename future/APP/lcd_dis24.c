/*******************************************

				  2.4寸 QVGA显示驱动程序







**********************************************/

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "fsmc_sram.h"
#include "ili9320_font.h"

#include "ASCII12X24.h"
#include "nAsciiDot12x24.h"
#include "ASCII8X16.h"
#include "typFNT_GB24.h"

#define Bank1_LCD_D    ((uint32_t)0x60010000)    //disp Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 //disp Reg ADDR

#define	LCD_REG_OTM4802 LCD_WRITE_CMD
#define	LCD_DAT_OTM4802 LCD_WRITE_CMD_DATA

unsigned long color1=0;
//void MUC_Init();
void LCD_Init(void);
void LCD_WR_REG(unsigned int index);
void LCD_WR_CMD(unsigned int index,unsigned int val);

void LCD_WR_Data(unsigned int val);
void LCD_WR_Data_8(unsigned int val);
void LCD_test(void);
void LCD_clear(unsigned int p);
void LCD_Clear_Screen(u16 color);
void LCD_Set_Area(u16 xstart,u16 ystart,u16 xend,u16 yend);
void lcd_wr_zf(unsigned int a, unsigned int b, unsigned int a1,unsigned int b1, unsigned int d,unsigned int e, unsigned char g, unsigned char *f); 
void lcd_wr_pixel(unsigned int a, unsigned int b, unsigned int e);
void LCD_Color_Show(void);
void GUI_Text12x24(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor);

unsigned int color[]={0xf800,0x07e0,0x001f,0xffe0,0x0000,0xffff,0x07ff,0xf81f};

unsigned int LCD_RD_data(void);
extern void lcd_rst(void);
extern void Delay(__IO uint32_t nCount);
extern void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);

struct typFNT_GB24 *pointFNT;

static void delay(u16 cnt) 
{
  while (cnt--);
}
static void delay_ms(u16 cnt)
{
	do{delay(10);}while(cnt--);	
}

//写寄存器地址函数
void LCD_WR_REG(unsigned int index)
{
	*(__IO uint8_t *) (Bank1_LCD_C)= index;

}

void LCD_WRITE_CMD(u16 index)		   	//写命令到TFT
{
	*(__IO uint8_t *) (Bank1_LCD_C)=(index);	
}
void LCD_WRITE_CMD_DATA(u16 Data)		//写数据到TFT
{
	*(__IO uint8_t *) (Bank1_LCD_D)=(Data);	
}

void LCD_WRITE_DATA(u16 Data)			//写数据到TFT
{
	unsigned int i;
	*(__IO uint8_t *) (Bank1_LCD_D)=(Data>>8);
	for(i=0;i<5;i++);
	*(__IO uint8_t *) (Bank1_LCD_D)=(Data & 0x00ff);	
}



/****************************************************************************
* 名    称：u16 ili9320_BGR2RGB(u16 c)
* 功    能：RRRRRGGGGGGBBBBB 改为 BBBBBGGGGGGRRRRR 格式
* 入口参数：c      BRG 颜色值
* 出口参数：RGB 颜色值
* 说    明：内部函数调用
* 调用方法：
****************************************************************************/
u16 BGR2RGB(u16 c)
{
	u16  r, g, b;
	b = (c>>0)  & 0x1f;
	g = (c>>5)  & 0x3f;
	r = (c>>11) & 0x1f;
	return( (b<<11) + (g<<5) + (r<<0) );
}
/*******************************************************************************/
void LCD_Color_Show(void)
{
	LCD_Set_Area(0,0,320-1,240-1);
	LCD_Clear_Screen(0xf800);//G
	delay_ms(40000);
	delay_ms(40000);
	delay_ms(40000);
	LCD_Clear_Screen(0x7e0);
	delay_ms(40000);
	delay_ms(40000);
	delay_ms(40000);
	LCD_Clear_Screen(0x001f);
	delay_ms(40000);
	delay_ms(40000);
	delay_ms(40000);
	LCD_Clear_Screen(0x0000);
	delay_ms(40000);
	delay_ms(40000);
	GUI_Text12x24(30,0,"Hello ",6,0xf800,0x0000);
	delay_ms(40000);
	delay_ms(40000);
	GUI_Text12x24(30,25,"Dolphin",7,0xf800,0x0000);
	delay_ms(40000);
	delay_ms(40000);
}



//初始化函数
void LCD_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 //LCD-RST
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
 
    GPIO_SetBits(GPIOE, GPIO_Pin_1);		 	 
	delay_ms(15000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    delay_ms(15000);					   				   
    //GPIO_SetBits(GPIOE, GPIO_Pin_1);		 	 
	delay_ms(400);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    delay_ms(1200);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1);		 	 
	delay_ms(1200);
#if 0
	//------------------------------------Display Control Setting--------------------------------------//
	LCD_WRITE_CMD(0x0001);  
	LCD_WRITE_DATA(0x0000);	// Output Direct 0100  0000|0 SM 0 SS|0000|0000		   SM=0 SS=0
	LCD_WRITE_CMD(0x0002);
	LCD_WRITE_DATA(0x0700); 	// Line Inversion
	
	LCD_WRITE_CMD(0x0003);
	LCD_WRITE_DATA(0x10B0);	// Entry Mode (65K, BGR)//0x1030   
	LCD_WRITE_CMD(0x0004);
	LCD_WRITE_DATA(0x0000);	// Set Scaling function off		
	LCD_WRITE_CMD(0x0008);
	LCD_WRITE_DATA(0x0200);	// Porch Setting202	
	LCD_WRITE_CMD(0x0009);
	LCD_WRITE_DATA(0x0000);	// Scan Cycle	
	LCD_WRITE_CMD(0x000a);
	LCD_WRITE_DATA(0x0000);	// FMARK off
//-----------------------------------End Display Control setting-----------------------------------------//
//-------------------------------- Power Control Registers Initial --------------------------------------//
	LCD_WRITE_CMD(0x0010);
	LCD_WRITE_DATA(0x0000);//Power Control1  07 90	
	LCD_WRITE_CMD(0x0011);
	LCD_WRITE_DATA(0x0007);//Power Control2	
	LCD_WRITE_CMD(0x0012);
	LCD_WRITE_DATA(0x0000);//Power Control3	
	LCD_WRITE_CMD(0x0013);
	LCD_WRITE_DATA(0x1300);//Power Control4  00 04
//---------------------------------End Power Control Registers Initial -------------------------------//
	delay_ms(60);
//--------------------------------- Power Supply Startup 1 Setting------------------------------------//
	LCD_WRITE_CMD(0x0010);
	LCD_WRITE_DATA(0x1290); //Power Control1  	1490
	delay_ms(100);	
	LCD_WRITE_CMD(0x0011);
	LCD_WRITE_DATA(0x0527); //Power Control2  		0227
	delay_ms(100);
//--------------------------------- End Power Supply Startup 1 Setting------------------------------//
	LCD_WRITE_CMD(0x0012);
	LCD_WRITE_DATA(0x0018); //Power Control3  		 1C
	LCD_WRITE_CMD(0x0013);
	LCD_WRITE_DATA(0x1000); //Power Control4  1b 00	
	LCD_WRITE_CMD(0x0029);
	LCD_WRITE_DATA(0x001E); //VCOMH setting   00 22	   1E
	LCD_WRITE_CMD(0x002b);
	LCD_WRITE_DATA(0x000D); 						//	0b
	LCD_WRITE_CMD(0x0020);
	LCD_WRITE_DATA(0x0000); 
	LCD_WRITE_DATA(0x0000); 
	LCD_WRITE_CMD(0x0021);
	LCD_WRITE_DATA(0x0000); 
	LCD_WRITE_DATA(0x0000); 
//--------------------------------- End Power Supply Startup 2 Setting------------------------------//
	delay_ms(100);
//-------------------------------------Gamma Cluster Setting-------------------------------------------//
	LCD_WRITE_CMD(0x0030);
	LCD_WRITE_DATA(0x0000);	
	LCD_WRITE_CMD(0x0031);
	LCD_WRITE_DATA(0x0407);	
	LCD_WRITE_CMD(0x0032);
	LCD_WRITE_DATA(0x0601);	
	LCD_WRITE_CMD(0x0035);
	LCD_WRITE_DATA(0x0104);	
	LCD_WRITE_CMD(0x0036);
	LCD_WRITE_DATA(0x0e06);	
	LCD_WRITE_CMD(0x0037);
	LCD_WRITE_DATA(0x0106);	
	LCD_WRITE_CMD(0x0038);
	LCD_WRITE_DATA(0x0704);	
	LCD_WRITE_CMD(0x0039);
	LCD_WRITE_DATA(0x0300);	
	LCD_WRITE_CMD(0x003c);
	LCD_WRITE_DATA(0x0401);	
	LCD_WRITE_CMD(0x003d);
	LCD_WRITE_DATA(0x060e);
	
//---------------------------------------End Gamma Setting---------------------------------------------//
//----------------------------------Display Windows 240 X 320----------------------------------------//
	LCD_WRITE_CMD(0x0050);
	LCD_WRITE_DATA(0x0000); 	// Horizontal Address Start Position
	
	LCD_WRITE_CMD(0x0051);
	LCD_WRITE_DATA(0x00ef);	// Horizontal Address End Position
	
	LCD_WRITE_CMD(0x0052);
	LCD_WRITE_DATA(0x0000);	// Vertical Address Start Position
	
	LCD_WRITE_CMD(0x0053);
	LCD_WRITE_DATA(0x013f);	// Vertical Address End Position
//----------------------------------End Display Windows 240 X 320----------------------------------//
	
	LCD_WRITE_CMD(0x0060);
	LCD_WRITE_DATA(0xa700);	// Gate scan control	GS=0
	
	LCD_WRITE_CMD(0x0061);	//
	LCD_WRITE_DATA(0x0001);	// Non-display Area setting
	LCD_WRITE_CMD(0x006a);
	LCD_WRITE_DATA(0x0000);	

//--------------- OTM3225 Panel Image Control -----------------//
	LCD_WRITE_CMD(0x80);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 1
	LCD_WRITE_CMD(0x81);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 1
	LCD_WRITE_CMD(0x82);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 1
	LCD_WRITE_CMD(0x83);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 2
	LCD_WRITE_CMD(0x84);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 2
	LCD_WRITE_CMD(0x85);   
	LCD_WRITE_DATA( 0x0000); // Set Partial Display 2
	
	LCD_WRITE_CMD(0x0090);
	LCD_WRITE_DATA(0x0010);	// RTNI setting
	LCD_WRITE_CMD(0x0092);
	LCD_WRITE_DATA(0x0600);
	LCD_WRITE_CMD(0x0093);
	LCD_WRITE_DATA(0x0003);		
	LCD_WRITE_CMD(0x0095);
	LCD_WRITE_DATA(0x0110);
	LCD_WRITE_CMD(0x0097);
	LCD_WRITE_DATA(0x0000);
	LCD_WRITE_CMD(0x0098);
	LCD_WRITE_DATA(0x0000);										
	LCD_WRITE_CMD(0x0007);
	LCD_WRITE_DATA(0x0173);	// Display Control1
	delay_ms(100);	
#endif

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);//This command is empty command,it can be used to terminate RAM data write
	LCD_DAT_OTM4802(0x80);

	LCD_REG_OTM4802(0xFF);//
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);

	LCD_REG_OTM4802(0x00);//This command is empty command,it can be used to terminate RAM data write
	LCD_DAT_OTM4802(0xB1);

	LCD_REG_OTM4802(0xC5);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0xA6);

	LCD_REG_OTM4802(0xB3);
	LCD_DAT_OTM4802(0x20);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0xB4);

	LCD_REG_OTM4802(0xC0);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x83);

	LCD_REG_OTM4802(0xC5);
	LCD_DAT_OTM4802(0x07);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x90);

	LCD_REG_OTM4802(0xC5);
	LCD_DAT_OTM4802(0x47);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x81);
	LCD_REG_OTM4802(0xC1);
	LCD_DAT_OTM4802(0x77);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0xD8);
	LCD_DAT_OTM4802(0x7E);
	LCD_DAT_OTM4802(0x7E);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0xD9);
	LCD_DAT_OTM4802(0xC8);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0xA1);
	LCD_REG_OTM4802(0xB3);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0xA7);
	LCD_REG_OTM4802(0xB3);
	LCD_DAT_OTM4802(0x11);


	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0xE1);//Gamma Correct ion Characteristics Setting
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x03);
	LCD_DAT_OTM4802(0x09);
	LCD_DAT_OTM4802(0x06);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x1C);
	LCD_DAT_OTM4802(0x0D);
	LCD_DAT_OTM4802(0x0E);
	LCD_DAT_OTM4802(0x01);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x04);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x1E);
	LCD_DAT_OTM4802(0x1B);
	LCD_DAT_OTM4802(0x04);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);


	LCD_REG_OTM4802(0xE1);//Gamma Correct ion Characteristics Setting
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x03);
	LCD_DAT_OTM4802(0x09);
	LCD_DAT_OTM4802(0x06);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x1C);
	LCD_DAT_OTM4802(0x0D);
	LCD_DAT_OTM4802(0x0E);
	LCD_DAT_OTM4802(0x01);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x04);
	LCD_DAT_OTM4802(0x05);
	LCD_DAT_OTM4802(0x1E);
	LCD_DAT_OTM4802(0x1B);
	LCD_DAT_OTM4802(0x04);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x2A); //Column Address Set,define area of frame memory XS [15:0] and XE [15:0] 
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x50);//XS=80
	LCD_DAT_OTM4802(0x01);
	LCD_DAT_OTM4802(0x8F);//XE=399
						  //XE-XS=319
	LCD_REG_OTM4802(0x2B);//Row Address Set,
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x28);//YS=40
	LCD_DAT_OTM4802(0x01);//
	LCD_DAT_OTM4802(0x17);//YE=279
						  //YE-YS=239
	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x80);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x00);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0xFF);
	LCD_DAT_OTM4802(0xFF);
	LCD_DAT_OTM4802(0xFF);

	LCD_REG_OTM4802(0x36);//defines read/ write scanning direction of frame memory
	LCD_DAT_OTM4802(0xA1);//LCD vertical refresh Top to Bottom
						  //D3: Color selector ‘0’ = RGB  ‘1’ = BGR

	LCD_REG_OTM4802(0x3a);//define the format of RGB picture data to be transferred via the MCU interface 
	LCD_DAT_OTM4802(0x55);//16-bits/pixel 

	LCD_REG_OTM4802(0xFF);//Enable access command 2 registers:
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x80);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);//END Enable access command 2 registers:

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x90);

	LCD_REG_OTM4802(0xff);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x93);

	LCD_REG_OTM4802(0xff);
	LCD_DAT_OTM4802(0x20);


	delay_ms(10);
	LCD_REG_OTM4802(0x11);//This command turns off sleep mode. 

	LCD_REG_OTM4802(0x0c);//This command indicates the Pixel Display Format	 
	LCD_DAT_OTM4802(0x05);//16-bits/pixel 

//	LCD_REG_OTM4802(0x36);//defines read/ write scanning direction of frame memory
//	LCD_DAT_OTM4802(0xc0);//LCD vertical refresh Top to Bottom
//						  //D3: Color selector ‘0’ = RGB  ‘1’ = BGR


	delay_ms(100);
	LCD_REG_OTM4802(0x36);//defines read/ write scanning direction of frame memory
	LCD_DAT_OTM4802(0x80);//LCD vertical refresh Top to Bottom
						  //D3: Color selector ‘0’ = RGB  ‘1’ = BGR
						  //scan X:0 Y=1
						  //-----<------
						  //|         -
						  //|       -
						  //^     -
						  //|   -
						  //|-
						  //-----<-----    
	delay_ms(10);
	LCD_REG_OTM4802(0x29);//recover from DISPLAY OFF  mode----Display On 
	LCD_REG_OTM4802(0x2C);//Memory Write --transfer data from MCU to frame memory.
	//for(;;)LCD_REG_OTM4802(0x22);
#if 0	
	while(1)
	{
		LCD_Color_Show();
		//LCD_Clear_Screen(0x07);
		//LCD_REG_OTM4802(0x22);
		//delay_ms(500);
		delay_ms(50000);
		LCD_REG_OTM4802(0x23);
		delay_ms(50000);
		LCD_REG_OTM4802(0x13);			
	}
 #endif
	//After the initialization, Set the DWIDTH as 8bit twice for data transfer.
	//plcd_mode->slcd_cfg &= ~(7<<10);
	//plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_TWICE;
	//OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);


}	
		


void LCD_Set_Location(u16 i,u16 j)		//设置X、Y坐标，做好写RAM的准备
{
	LCD_WRITE_CMD(0x2b);
	LCD_WRITE_DATA(i+0x50); // xstart
	LCD_WRITE_DATA(i+0x50+1); // xend
	LCD_WRITE_CMD(0x2a);
	LCD_WRITE_DATA(j+0x28); // xstart
	LCD_WRITE_DATA(j+0x28+1); // xend
	LCD_REG_OTM4802(0x2C);//Memory Write	
}

void LCD_Set_Area(u16 xstart,u16 ystart,u16 xend,u16 yend)
{
//#if 0
//	LCD_WRITE_CMD(0x0052);
//	LCD_WRITE_DATA(xstart); // xstart
//	
//	LCD_WRITE_CMD(0x0053);
//	LCD_WRITE_DATA(xend); // xend
//	
//	LCD_WRITE_CMD(0x0050);
//	LCD_WRITE_DATA(ystart); // ystart
//	LCD_WRITE_CMD(0x0051);
//	LCD_WRITE_DATA( yend); // yend
//	LCD_WRITE_CMD(0x0022);
//#endif
	LCD_WRITE_CMD(0x2b);
	LCD_WRITE_DATA(xstart+0x50); // xstart 0x50
	LCD_WRITE_DATA(xend+0x50); // xend
	
	LCD_WRITE_CMD(0x2a);
	LCD_WRITE_DATA(ystart+0x28); // ystart	0x28
	LCD_WRITE_DATA(yend+0x28); // yend

	LCD_REG_OTM4802(0x2C);//Memory Write	
	
}
/****************************************************************************
* 名    称：void ili9320_Clear(u16 dat)
* 功    能：将屏幕填充成指定的颜色，如清屏，则填充 0xffff
* 入口参数：dat      填充值
* 出口参数：无
* 说    明：
* 调用方法：ili9320_Clear(0xffff);
****************************************************************************/
void TFT_Clear(u16 dat)
{
	unsigned long j;
	LCD_Set_Area(0,0,240-1,320-1);
	for(j=0;j<320*240;j++)	
		LCD_WRITE_DATA(dat); 
}
/****************************************************************************
* 名    称：void TFT_SetPoint(u16 x,u16 y,u16 point)
* 功    能：在指定座标画点
* 入口参数：x      行座标
*           y      列座标
*           point  点的颜色
* 出口参数：无
* 说    明：
* 调用方法：TFT_SetPoint(10,10,0x0fe0);
****************************************************************************/
void TFT_SetPoint(u16 x,u16 y,u16 point)
{	
	LCD_Set_Location(x,y);	
	LCD_WRITE_DATA(point); 
}



void LCD_Clear_Screen(u16 color)
{
	//显示单色数据到LCD
	unsigned long j;
	LCD_Set_Area(0,0,320-1,240-1);
	for(j=0;j<320*240;j++)	
	{
		LCD_WRITE_DATA(color);
		//delay_ms(40000);	
	}
}


void LCD_SetPixel(u16 x, u16 y, u16 color)
{
	LCD_Set_Location(x,y);
	LCD_WRITE_DATA( color);
}
/****************************************************************************
* 名    称：void TFT_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
* 功    能：在指定座标显示一个8x16点阵的ascii字符
* 入口参数：x          行座标
*           y          列座标
*           charColor  字符的颜色
*           bkColor    字符背景颜色
* 出口参数：无
* 说    明：显示范围限定为可显示的ascii码
* 调用方法：TFT_PutChar(10,10,'a',0x0000,0xffff);
****************************************************************************/
void TFT_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
	u16 i=0;
	u16 j=0;  
	u8 tmp_char=0;
	charColor 	=BGR2RGB(charColor);
	bkColor		=BGR2RGB(bkColor);
	//x=320-8-x;
	for (i=0;i<16;i++)
	{
		tmp_char=ascii_8x16[((c-0x20)*16)+i];
		for (j=0;j<8;j++)
		{
			TFT_SetPoint(x+j,y+i,0xffff);		//清除该坐标点原来的数据
			if ( (tmp_char >> 7-j) & 0x01 == 0x01)
			{				
				TFT_SetPoint(x+j,y+i,charColor); // 字符颜色
			}
			else
			{
				TFT_SetPoint(x+j,y+i,bkColor); 	// 背景颜色
			}
		}
	}
}

/****************************************************************************
* 名    称：void TFT_PutChar12x24(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
* 功    能：在指定座标显示一个12x24点阵的ascii字符
* 入口参数：x          行座标
*           y          列座标
*           charColor  字符的颜色
*           bkColor    字符背景颜色
* 出口参数：无
* 说    明：显示范围限定为可显示的ascii码
* 调用方法：TFT_PutChar(10,10,'a',0x0000,0xffff);
****************************************************************************/
void TFT_PutChar12x24(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
	u16 i=0;
	u16 j=0;  
	u8 tmp_char=0;
	charColor 	=BGR2RGB(charColor);
	bkColor		=BGR2RGB(bkColor);	
	//x=320-12-x;
#if 0
	for (i=0;i<48;)//
	{
		
		tmp_char=nAscii12x24Dot[((c-0x20)*48)+i];
		for (j=0;j<8;j++)
		{
			delay_ms(40000);
			TFT_SetPoint(x+j,y+(i>>1),0xffff);	  		//清除该坐标点原来的数据
			if ( (tmp_char >> 7-j) & 0x01 == 0x01)
			{	
				TFT_SetPoint(x+j,y+(i>>1),charColor); 	// 字符颜色
			}
			else
			{
				TFT_SetPoint(x+j,y+(i>>1),bkColor); 	// 背景颜色
			}
		}
		tmp_char=nAscii12x24Dot[((c-0x20)*48)+i+1];
		for (j=0;j<4;j++)
		{
			delay_ms(40000);
			TFT_SetPoint(x+8+j,y+(i>>1),0xffff);	  		//清除该坐标点原来的数据
			if ( (tmp_char >> 7-j) & 0x01 == 0x01)
			{
				TFT_SetPoint(x+8+j,y+(i>>1),charColor); // 字符颜色
			}
			else
			{
				TFT_SetPoint(x+8+j,y+(i>>1),bkColor); 	// 背景颜色
			}
		}
		i=i+2;
	}
#else
	LCD_Set_Area(x,y,x+11,y+23);
	for (i=0;i<36;i++)//每个字符48个字节
	{
		tmp_char=nAsciiDot12x24[((c-0x20)*36)+i];
		for(j=0;j<8;j++)
		{
			//delay_ms(40000);
			//if (pointFNT->Msk[j] & (0x01<<i))
			if( (tmp_char) & (0x80>>j))
			{
				LCD_WRITE_DATA(charColor);//写有效点
			}
			else
			{
				LCD_WRITE_DATA(bkColor);//写底色
			}
		}
	}
#endif
}

/****************************************************************************
* 名    称：void GUI_Text(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
* 功    能：在指定座标显示字符串
* 入口参数：x      行座标
*           y      列座标
*           *str   字符串
*           len    字符串长度
*           Color  字符颜色
*           bkColor字符背景颜色
* 出口参数：无
* 说    明：
* 调用方法：GUI_Text(0,0,"0123456789",10,0x0000,0xffff);
****************************************************************************/
void GUI_Text(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
{
	u8 i;	
	for (i=0;i<len;i++)
	{
		TFT_PutChar((x+8*i),y,*str++,Color,bkColor);
	}
}
/****************************************************************************
* 名    称：void GUI_Text12x24(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
* 功    能：在指定座标显示字符串
* 入口参数：x      行座标
*           y      列座标
*           *str   字符串
*           len    字符串长度
*           Color  字符颜色
*           bkColor字符背景颜色
* 出口参数：无
* 说    明：
* 调用方法：GUI_Text(0,0,"0123456789",10,0x0000,0xffff);
****************************************************************************/
void GUI_Text12x24(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
{
	u8 i;	
	for (i=0;i<len;i++)
	{
		TFT_PutChar12x24((x+12*i),y,*str++,Color,bkColor);
	}
}
/****************************************************************************
* 名    称：void GUI_Chinese24x24(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
* 功    能：在指定座标显示字符串
* 入口参数：x      行座标
*           y      列座标
*           *str   字符串
*           len    字符串长度
*           Color  字符颜色
*           bkColor字符背景颜色
* 出口参数：无
* 说    明：
* 调用方法：GUI_Text(0,0,"0123456789",10,0x0000,0xffff);
****************************************************************************/

void GUI_Chinese24x24(u16 x, u16 y, u8 *str, u16 len,u16 Color, u16 bkColor)
{
	unsigned char i,j,k;
	unsigned char str_cnt;
	unsigned int Chinese_Code;
	unsigned char region   = *str-0xa0;//得到区号  gb2312
	unsigned char location = *(str+1)-0xa0;//得到位号
	for(str_cnt=0;str_cnt<2*len;str_cnt+=2)
	{
		Chinese_Code=*(str+str_cnt);
		USART_OUT(USART1,"-%d\r\n",(Chinese_Code<<8)+*(str+str_cnt+1));//打印汉字内码
		LCD_Set_Area(x,y,x+23,y+23);//设置写显存区域
		for(k=0;k<24;k++)//按从左到右，从上到下，纵向8点上高位取模
		{
			for(j=0;j<3;j++)
			{
				for (i = 0; i < 8; i++)//
				{
					if( (GB_24[j*24+k]) & (0x80>>i))
						LCD_WRITE_DATA(Color);//写有效点
					else
						LCD_WRITE_DATA(bkColor);//写底色
				}     
			}
		}
		x+=24;//下一个字的位置
	} 
}

void TFT_PutChinese(u16 x, u16 y, u16 chn_num,u16 Color, u16 bkColor)
{
	unsigned char i,j,k;
	unsigned char str_cnt;

	chn_num=chn_num*72;
	for(str_cnt=0;str_cnt<2;str_cnt+=2)
	{
		LCD_Set_Area(x,y,x+23,y+23);//设置写显存区域
		for(k=0;k<24;k++)//按从左到右，从上到下，纵向8点上高位取模
		{
			for(j=0;j<3;j++)
			{
				for (i = 0; i < 8; i++)//
				{
					if( (GB_24[chn_num+j*24+k]) & (0x80>>i))
						LCD_WRITE_DATA(Color);//写有效点
					else
						LCD_WRITE_DATA(bkColor);//写底色
				}     
			}
		}
		x+=24;//下一个字的位置
	} 
}


//演示程序
void LCD_test(void)
{
	unsigned int tmp;
//	struct typFNT_GB *pointFNT;
	//struct typFNT_GB16 
	//typFNT_GB24
	//ili9320_Clear(0xffff);

	GUI_Text(0,0,"Hello world.",12,0x0000,0xf800);
	//GUI_Text12x24(0,50,"Hello world.",12,0x0000,0x001f);
	tmp=color1/10000;
	TFT_PutChar(0,100,tmp+0x30,0x0000,0xffff);
	TFT_PutChar12x24(0,150,tmp+0x30,0x0000,0xffff);
	  
	tmp=(color1%10000)/1000;
	TFT_PutChar(8,100,tmp+0x30,0x0000,0xffff);
	TFT_PutChar12x24(12,150,tmp+0x30,0x0000,0xffff);
 
	tmp=((color1%10000)%1000)/100;
	TFT_PutChar(16,100,tmp+0x30,0x0000,0xffff);
	TFT_PutChar12x24(24,150,tmp+0x30,0x0000,0xffff);
 
	tmp=(((color1%10000)%1000)%100)/10;
	TFT_PutChar(24,100,tmp+0x30,0x0000,0xffff);
	TFT_PutChar12x24(36,150,tmp+0x30,0x0000,0xffff);
	tmp=color1%10;
	TFT_PutChar(32,100,tmp+0x30,0x0000,0xffff); 
	TFT_PutChar12x24(48,150,tmp+0x30,0x0000,0xffff);
	color1++; 
	if(color1==65536) color1=0; 
	//tmp=GB_24x24;
//	pointFNT=GB_24x+73;//GB_24x24[2]+10; 
//	tmp=pointFNT->Index[1];			
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
