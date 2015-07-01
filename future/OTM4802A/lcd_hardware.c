
#include "tsys.h"
#include "lcd.h"



#define W_COM(cmd)					Mcupanel_Command(cmd)
#define W_DATA(data1, data2)		Mcupanel_Data(((unsigned short)data1<<8)|data2)
#define W_DATA16(data)				Mcupanel_Data(data&0xffff)
///Add By Ryan 2012-02-13
///DEFINATION FOR OTM4802A
#define	LCD_REG_OTM4802(cmd)		Mcupanel_Command(cmd)
#define	LCD_DAT_OTM4802(data1)		Mcupanel_Data(data1)

extern PLCD_MODE	plcd_mode;
extern void pwm_init(unsigned int chn, unsigned int frequence, unsigned int mode);
extern void pwm_set_duty(unsigned int chn, unsigned int percent);
extern void mdelay(unsigned int msec);

static void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
{
	cmd = (cmd & 0x0000ffff);
	data = (data &0x0000ffff);

	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	__gpio_clear_pin(SLCD_CS_PIN);
	OUTREG32(A_LCD_MDATA,((1 << 31)|((cmd&0xff00)>>8)));
	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	OUTREG32(A_LCD_MDATA,((1 << 31)|(cmd&0xff)));
	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	OUTREG32(A_LCD_MDATA,((0 << 31)|data));
	__gpio_set_pin(SLCD_CS_PIN);
}

void Mcupanel_Command(unsigned int cmd)
{
	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	OUTREG32(A_LCD_MDATA,(1 << 31) | ((cmd & 0xff00)>>8));
	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	OUTREG32(A_LCD_MDATA,(1 << 31) | (cmd & 0xff));
}

void Mcupanel_Data(unsigned int data)
{
	while (INREG32(A_LCD_MSTATE)&(1 << 0))
	{
		INTWHILEBUG();
	}
	OUTREG32(A_LCD_MDATA,(0 << 31) | (data & 0x0000ffff));
}

void lcd_special_pin_init()
{
	//__gpio_as_output(SLCD_RD_PIN);
	//__gpio_set_pin(SLCD_RD_PIN);		// no read operation, always high

	__gpio_as_output(SLCD_CS_PIN);
	__gpio_clear_pin(SLCD_CS_PIN);		// in select condition

	__gpio_as_output(SLCD_RESET_PIN);	// init lcd reset pin and hardware reset lcd
	__gpio_set_pin(SLCD_RESET_PIN);
	//mdelay(100);
	//__gpio_clear_pin(SLCD_RESET_PIN);
	//mdelay(10);
	//__gpio_set_pin(SLCD_RESET_PIN);
	//mdelay(120);
}

/* Set the start address of screen, for example (0, 0) */
void Mcupanel_SetAddr(unsigned int x, unsigned int y)
{
	Mcupanel_RegSet(0x20, x);   // Set GRAM Address (GRAM地址选择）
	mdelay(1);
	Mcupanel_RegSet(0x21, y);  // Set GRAM Address
	mdelay(1);
	Mcupanel_Command(0x22);

}

//----------------------------------------------------------------------------
#define PWM_FREQ		(50 * 1000)	// Condigure according to the BKL chip
unsigned int level = 0;

#ifdef SLCD_BACKLIGHT_PWM_CHN
void lcd_set_backlight(unsigned int percent)
{
	pwm_set_duty(SLCD_BACKLIGHT_PWM_CHN, 100-percent);
	level = percent;

	__gpio_as_output(LCD_BACKLIGHT_EN);
	if (level != 0)
	{
		__gpio_set_pin(LCD_BACKLIGHT_EN);
	}
	else
	{
		__gpio_clear_pin(LCD_BACKLIGHT_EN);
	}
}

//----------------------------------------------------------------------------
void lcd_close_backlight(void)
{
	pwm_set_duty(SLCD_BACKLIGHT_PWM_CHN, 100);
	__gpio_as_output(LCD_BACKLIGHT_EN);
	__gpio_clear_pin(LCD_BACKLIGHT_EN);
}

unsigned int lcd_get_backlight(void)
{
	return(level);
}

/************************************************************************
函数功能：    PWM测试程序--将PWM设置为IO口，并控制其高低电平
输入参数：    1	表示PWM输出高电平；		0 表示PWM输出低电平
输出参数：    无
返 回 值：    无
版本历史：    2012-03-07		Ryan		基本功能完成
************************************************************************/
void	lcd_test_set_port_level(int level)
{
	int ioPort = GPIO_GROUP_C + 14;
	printk("lcd_test_set_port_level level = %d\r\n", level);

	__gpio_as_output(ioPort);

	if (0 == level)
	{
		__gpio_clear_pin(ioPort);
	}
	else
	{
		__gpio_set_pin(ioPort);
	}
}

/************************************************************************
函数功能：    PWM测试程序--设置为功能脚
输入参数：    无
输出参数：    无
返 回 值：    无
版本历史：    2012-03-07		Ryan		基本功能完成
************************************************************************/
void	lcd_test_enable_function(void)
{
	int chn = 4;
	unsigned int full_cycles;

	printk("lcd_test_enable_function 设置为功能脚! \r\n");

#if 1	//按照DATASHEET写的代码
	OUTREG32(A_GPIO_PXFUNC(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXTRGC(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXSELC(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXFUNS(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXPES(2), 1 << (10 + chn));
#else	//原来的代码
	OUTREG32(A_GPIO_PXFUNS(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXSELC(2), 1 << (10 + chn));
	OUTREG32(A_GPIO_PXPES(2), 1 << (10 + chn));
#endif

	full_cycles = INREG16(A_TCU_TDFR(chn));
	CLRREG32((A_TCU_TCSR(chn)), TCSR_PWM_EN);
	OUTREG16(A_TCU_TDHR(chn), full_cycles - (full_cycles * 50) / 110);
	OUTREG16(A_TCU_TCNT(chn), 0x0000);
	OUTREG16(A_TCU_TESR, TCU_TIMER(chn));
	SETREG32((A_TCU_TCSR(chn)), TCSR_PWM_EN);
}


#else
void lcd_set_backlight(unsigned int percent)
{
	level = percent;
	__gpio_as_output(LCD_BACKLIGHT_EN);
	if (level != 0)
	{
		__gpio_set_pin(LCD_BACKLIGHT_EN);
	}
	else
	{
		__gpio_clear_pin(LCD_BACKLIGHT_EN);
	}
}

//-----------------------------------------------------------------------------
void lcd_close_backlight(void)
{
	__gpio_as_output(LCD_BACKLIGHT_EN);
	__gpio_clear_pin(LCD_BACKLIGHT_EN);

}

unsigned int lcd_get_backlight(void)
{
	return(100);
}

#endif

void lcd_board_init(void)
{
	__gpio_as_slcd_8bit();		// mcu lcd interface control and data pin init: RS WR D0~D7 pin
	lcd_special_pin_init();		// mcu lcd interface control pin init: RD CS RESET pin
}

void display_rgb(unsigned int color)
{
	unsigned int i,j;

	for (i=0;i<320;i++)
	{
		for (j=0;j<240;j++)
		{
			W_DATA16(color);
		}
	}
}

#if (LCD_MODEL == OTM3225C)

void Init_otm3225c()
{
	printk("Init_otm3225c start\r\n");

	//---------------OTM3225 LCD Setting--------------------------//
	W_COM(0xE3);   W_DATA( 0x30,0x08); // This is an empty register. No side-effect after setting it with any value.
	W_COM(0xE7);   W_DATA( 0x00,0x12); // This is an empty register. No side-effect after setting it with any value.
	W_COM(0xEF);   W_DATA( 0x12,0x31); // This is an empty register. No side-effect after setting it with any value.
	W_COM(0x01);   W_DATA( 0x01,0x00); // Set SS and SM bit
	W_COM(0x02);   W_DATA( 0x07,0x00); // Set line inversion
	W_COM(0x03);   W_DATA( 0x10,0xB0); // Set Write direction
	W_COM(0x04);   W_DATA( 0x00,0x00); // Set Scaling function off
	W_COM(0x08);   W_DATA( 0x02,0x07); // Set BP and FP
	W_COM(0x09);   W_DATA( 0x00,0x00); // Set non-display area
	W_COM(0x0A);   W_DATA( 0x00,0x00); // Frame marker control
	W_COM(0x0C);   W_DATA( 0x00,0x00); // Set interface control
	W_COM(0x0D);   W_DATA( 0x00,0x00); // Frame marker Position
	W_COM(0x0F);   W_DATA( 0x00,0x00); // Set RGB interface polarity.
	//---------------OTM3225 Power On Sequence----------------//
	W_COM(0x10);   W_DATA( 0x00,0x00); // Set SAP);   W_DATA(BT[3:0]);   W_DATA(AP);   W_DATA(SLP);
	W_COM(0x11);   W_DATA( 0x00,0x07); // Set DC1[2:0]);   W_DATA(DC0[2:0]);   W_DATA(VC[2:0]
	W_COM(0x12);   W_DATA( 0x00,0x00); // Set VREG1OUT voltage
	W_COM(0x13);   W_DATA( 0x00,0x00); // Set VCOM AMP voltage
	mdelay(200);
	W_COM(0x10);   W_DATA( 0x14,0x90); // Set SAP);   W_DATA(BT[3:0]);   W_DATA(AP);   W_DATA(SLP);
	W_COM(0x11);   W_DATA( 0x02,0x27); // Set DC1[2:0]);   W_DATA(DC0[2:0]);   W_DATA(VC[2:0]
	mdelay(50);
	W_COM(0x12);   W_DATA( 0x00,0x1c); // Set VREG1OUT voltage
	mdelay(50);
	W_COM(0x13);   W_DATA( 0x10,0x00); // Set VCOM AMP voltage
	W_COM(0x29);   W_DATA( 0x00,0x10); // Set VCOMH voltage
	W_COM(0x2B);   W_DATA( 0x00,0x0e); // Set Frame rate.
	mdelay(50);
	W_COM(0x20);   W_DATA( 0x00,0x00); // Set GRAM Horizontal Address
	W_COM(0x21);   W_DATA( 0x00,0x00); // Set GRAM Vertical Address
	//---------------OTM3225 Gamma Control----------------------//
	W_COM(0x30);   W_DATA( 0x00,0x03);
	W_COM(0x31);   W_DATA( 0x04,0x07);
	W_COM(0x32);   W_DATA( 0x06,0x01);
	W_COM(0x35);   W_DATA( 0x01,0x04);
	W_COM(0x36);   W_DATA( 0x0E,0x06);
	W_COM(0x37);   W_DATA( 0x01,0x06);
	W_COM(0x38);   W_DATA( 0x07,0x04);
	W_COM(0x39);   W_DATA( 0x03,0x00);
	W_COM(0x3C);   W_DATA( 0x04,0x01);
	W_COM(0x3D);   W_DATA( 0x06,0x0e);
	//--------------- OTM3225 RAM Address Control ----------------//
	W_COM(0x50);   W_DATA( 0x00,0x00); // Set GRAM Horizontal Start Address
	W_COM(0x51);   W_DATA( 0x00,0xEF); // Set GRAM Horizontal End Address
	W_COM(0x52);   W_DATA( 0x00,0x00); // Set GRAM Vertical Start Address
	W_COM(0x53);   W_DATA( 0x01,0x3F); // Set GRAM Vertical End Address
	//--------------- OTM3225 Panel Image Control -----------------//
	W_COM(0x60);   W_DATA( 0xA7,0x00); // Set Gate Scan line
	W_COM(0x61);   W_DATA( 0x00,0x01); // Set NDL);   W_DATA( VLE);   W_DATA( REV
	W_COM(0x6A);   W_DATA( 0x00,0x00); // Set Scrolling line
	//--------------- OTM3225 Panel Image Control -----------------//
	W_COM(0x80);   W_DATA( 0x00,0x00); // Set Partial Display 1
	W_COM(0x81);   W_DATA( 0x00,0x00); // Set Partial Display 1
	W_COM(0x82);   W_DATA( 0x00,0x00); // Set Partial Display 1
	W_COM(0x83);   W_DATA( 0x00,0x00); // Set Partial Display 2
	W_COM(0x84);   W_DATA( 0x00,0x00); // Set Partial Display 2
	W_COM(0x85);   W_DATA( 0x00,0x00); // Set Partial Display 2
	//--------------- OTM3225 Panel Interface Control---------------//
	W_COM(0x90);   W_DATA( 0x00,0x10);
	W_COM(0x92);   W_DATA( 0x06,0x00);
	W_COM(0x93);   W_DATA( 0x00,0x03);
	W_COM(0x95);   W_DATA( 0x01,0x10);
	W_COM(0x97);   W_DATA( 0x00,0x00);
	W_COM(0x98);   W_DATA( 0x00,0x00);
	//---------------OTM3225 Display On-------------------------------//
	W_COM(0x07);   W_DATA( 0x01,0x33); // Display on#endif

	mdelay(200);

	W_COM(0x50);   W_DATA( 0x00,0x00); // Set GRAM Horizontal Start Address
	W_COM(0x51);   W_DATA( 0x00,0xEF); // Set GRAM Horizontal End Address
	W_COM(0x52);   W_DATA( 0x00,0x00); // Set GRAM Vertical Start Address
	W_COM(0x53);   W_DATA( 0x01,0x3F); // Set GRAM Vertical End Address
	W_COM(0x20);   W_DATA( 0x00,0xef);
	W_COM(0x21);   W_DATA( 0x01,0x3f);
	W_COM(0x22);

	mdelay(50);

	display_rgb(0xf800);
	mdelay(500);
	display_rgb(0x07e0);
	mdelay(500);
	display_rgb(0x001f);

	printk("Init_otm3225c end\r\n");
}

#endif

#if (LCD_MODEL == OTM4802A)

///Add By Ryan 2012-02-13
///The Initialization for OTM4802A
void	Init_otm4802A()
{
	printk("----------------------------------- 2012-06-05 Init_otm4802A Start! \r\n");

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x80);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);

	LCD_REG_OTM4802(0x00);
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

	LCD_REG_OTM4802(0xE1);
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


	LCD_REG_OTM4802(0xE1);
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

	LCD_REG_OTM4802(0x2A);
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x50);
	LCD_DAT_OTM4802(0x01);
	LCD_DAT_OTM4802(0x8F);

	LCD_REG_OTM4802(0x2B);
	LCD_DAT_OTM4802(0x00);
	LCD_DAT_OTM4802(0x28);
	LCD_DAT_OTM4802(0x01);
	LCD_DAT_OTM4802(0x17);

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

	LCD_REG_OTM4802(0x36);
	LCD_DAT_OTM4802(0xA8);


	LCD_REG_OTM4802(0x3a);
	LCD_DAT_OTM4802(0x55);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x80);

	LCD_REG_OTM4802(0xFF);
	LCD_DAT_OTM4802(0x48);
	LCD_DAT_OTM4802(0x02);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x90);

	LCD_REG_OTM4802(0xff);
	LCD_DAT_OTM4802(0x01);

	LCD_REG_OTM4802(0x00);
	LCD_DAT_OTM4802(0x93);

	LCD_REG_OTM4802(0xff);
	LCD_DAT_OTM4802(0x20);


	mdelay(10);
	LCD_REG_OTM4802(0x11);

	LCD_REG_OTM4802(0x0c);		//Add By Ryan--Read Pixel Display Format
	LCD_DAT_OTM4802(0x05);

//	LCD_REG_OTM4802(0x22);
//	LCD_REG_OTM4802(0x13);

	mdelay(100);
	LCD_REG_OTM4802(0x29);

	LCD_REG_OTM4802(0x2C);

	//After the initialization, Set the DWIDTH as 8bit twice for data transfer.
	plcd_mode->slcd_cfg &= ~(7<<10);
	plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_TWICE;
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
}

void OTM4802_DisplayOff(void)
{
	//Set the DWIDTH as 8bit once for data transfer.
	plcd_mode->slcd_cfg &= ~(7<<10);
	plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_ONCE;
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);

   LCD_REG_OTM4802(0x28);
   LCD_REG_OTM4802(0x10);

   //Set the DWIDTH as 8bit twice for data transfer.
   plcd_mode->slcd_cfg &= ~(7<<10);
   plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_TWICE;
   OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
}

void OTM4802_DisplayOn(void)
{
	//Set the DWIDTH as 8bit once for data transfer.
	plcd_mode->slcd_cfg &= ~(7<<10);
	plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_ONCE;
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);

   LCD_REG_OTM4802(0x11); 
   LCD_REG_OTM4802(0x29); 

   //Set the DWIDTH as 8bit twice for data transfer.
   plcd_mode->slcd_cfg &= ~(7<<10);
   plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_TWICE;
   OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
}

#endif
