/******************************************************************************

                  版权所有 (C), 2001-2011, 广东步步高教育电子有限公司

 ******************************************************************************
  文 件 名   : lcd_hardware.c
  版 本 号   : 初稿
  作    者   : mj
  生成日期   : 2011年9月26日
  最近修改   :
  功能描述   : lcd屏驱动
  函数列表   :
  修改历史   :
  1.日    期   : 2011年9月26日
    作    者   : mj
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
//#include "tsys.h"
//#include "lcd.h"

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
//#define LcdID			(('L' << 24) | ('C' << 16) | ('D' << 16) | (':' << 16))

#define LCD_RESET 			(GPIO_GROUP_D + 23)
#define LCD_CTRSIGNAL 		(GPIO_GROUP_D + 24)
#define LCD_SPICHIPSELECT 	(GPIO_GROUP_C + 19)
#define LCD_LIGHT			(GPIO_GROUP_C + 14)

//>lfx #define FRAME_BUFF_ADDRESS		(0x81f80000)
//>lfx #define	FRAME_BUFF_DECS		    (0x81fA8000)
//<lfx
#define FRAME_BUFF_ADDRESS			PHY_ADDR_LCD_BUFF
#define	FRAME_BUFF_DECS		    	PHY_ADDR_LCD_BUFF2

#define PANEL_MODE_LCD_PANEL		0
#define PANEL_MODE_TVE_PAL			IOCTL_LCD_TO_TVE_PAL
#define PANEL_MODE_TVE_NTSC			IOCTL_LCD_TO_TVE_NTSC

#define NR_PALETTE	256

/***************************************************************************************************************/
/**新加宏定义声明*/
#define	LCD_TEST_WRITE_DATA			0


typedef struct jzfb_info
{
	unsigned char *cpal;						// Cacheable Palette Buffer
	unsigned char *pal;							// Non-cacheable Palette Buffer
	unsigned char *cframe;						// Cacheable Frame Buffer
	unsigned char *frame;						// Non-cacheable Frame Buffer

	struct
	{
		unsigned char red;
		unsigned char green;
		unsigned char blue;
	} palette[NR_PALETTE];
}JZFB_INFO, *PJZFB_INFO;

#undef LCD_UNCACHED
#define LCD_UNCACHED(addr)	((unsigned int)(addr) | 0xa0000000)

/*----------------------------------------------*
 * 外部变量                                     *
 *----------------------------------------------*/
extern LCD_PARAM lcd_param;

/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
static struct com_device lcd;
static struct device_graphic_info _lcd_info;

PLCD_MODE	plcd_mode = (PLCD_MODE)(&lcd_param.lcd_mode);
POSD_MODE	posd_mode = (POSD_MODE)(&lcd_param.osd_mode);
static JZFB_INFO jzfb;
static LCD_DESCRIPTOR_NEW lcd_palette_desc __attribute__ ((aligned (32)));
static PLCD_DESCRIPTOR_NEW lcd_frame_desc2;
static PLCD_DESCRIPTOR_NEW fg0_frame_desc;
static PLCD_DESCRIPTOR_NEW fg1_frame_desc;

static unsigned char *fg0_frame;
static unsigned int needroom;

#ifdef FRAME_BUFF_ADDRESS
static unsigned char *_framebuffer = (unsigned char *)FRAME_BUFF_ADDRESS;
static LCD_DESCRIPTOR_NEW *lcd_frame_desc2_room;
#else
static unsigned short _framebuffer[LCD_HEIGHT][LCD_WIDTH] __attribute__((aligned(4096))); // 4Kb align
static LCD_DESCRIPTOR_NEW lcd_frame_desc2_room[4] __attribute__ ((aligned (32)));
#endif

unsigned int *pscreen_light = UBOOT_SCREEN_LIGHT_ADDR;

//unsigned char LcdTmpBuffer[LCD_WIDTH*(LCD_HEIGHT+2)*2];

extern void lcd_board_init(void);
extern unsigned int GetCurrentPLLClock(bool bCheckPCSBit);
extern void serial_waitfinish();
extern unsigned int Get_ScreenHardVer();
extern void mdelay( unsigned int msec );
extern void PM_AddWakeUp(int id,void* preWakeUp,void* WakeUp);
extern void lcd_set_backlight(unsigned int percent);
extern void lcd_close_backlight(void);
extern void	lcd_test_set_port_level(int level);
extern void	lcd_test_enable_function(void);

extern void Init_otm3225c();
extern void Init_otm4802A();


extern void Mcupanel_Data(unsigned int data);
extern void Mcupanel_Command(unsigned int cmd);

/***************************************************************************************************************/
/**新加接口声明*/
		void	start_slcd_dma(void);
		void	ddi_slcd_flush_ram(void);
		unsigned char* 	ddi_slcd_get_mem_ptr(void);
		void	ddi_slcd_flush_ram_to_reg(char* buff);
/***************************************************************************************************************/


#define DEBUG_MODE
#ifdef DEBUG_MODE
void print_reg(void)
{
	printk("####### LCD registers #######\r\n");
	printk("A_LCD_LCDCFG[%#x]\r\n", INREG32(A_LCD_LCDCFG));
	printk("A_LCD_LCDSTATE[%#x]\r\n", INREG32(A_LCD_LCDSTATE));
	printk("A_LCD_LCDCTRL[%#x]\r\n", INREG32(A_LCD_LCDCTRL));
	printk("A_LCD_LCDVSYNC[%#x]\r\n", INREG32(A_LCD_LCDVSYNC));
	printk("A_LCD_LCDHSYNC[%#x]\r\n", INREG32(A_LCD_LCDHSYNC));
	printk("A_LCD_LCDVAT[%#x]\r\n", INREG32(A_LCD_LCDVAT));
	printk("A_LCD_LCDDAH[%#x]\r\n", INREG32(A_LCD_LCDDAH));
	printk("A_LCD_LCDDAV[%#x]\r\n", INREG32(A_LCD_LCDDAV));
	printk("####### DMA0 registers #######\r\n");
	printk("A_LCD_LCDDA0[%#x]\r\n", INREG32(A_LCD_LCDDA0));
	printk("A_LCD_LCDSA0[%#x]\r\n", INREG32(A_LCD_LCDSA0));
	printk("A_LCD_LCDFID0[%#x]\r\n", INREG32(A_LCD_LCDFID0));
	printk("A_LCD_LCDCMD0[%#x]\r\n", INREG32(A_LCD_LCDCMD0));
	printk("A_LCD_LCDOFFS0[%#x]\r\n", INREG32(A_LCD_LCDOFFS0));
	printk("A_LCD_LCDPW0[%#x]\r\n", INREG32(A_LCD_LCDPW0));
	printk("A_LCD_LCDCNUM0[%#x]\r\n", INREG32(A_LCD_LCDCNUM0));
	printk("A_LCD_LCDDESSIZE0[%#x]\r\n", INREG32(A_LCD_LCDDESSIZE0));
	printk("####### DMA1 registers #######\r\n");
	printk("A_LCD_LCDDA1[%#x]\r\n", INREG32(A_LCD_LCDDA1));
	printk("A_LCD_LCDSA1[%#x]\r\n", INREG32(A_LCD_LCDSA1));
	printk("A_LCD_LCDFID1[%#x]\r\n", INREG32(A_LCD_LCDFID1));
	printk("A_LCD_LCDCMD1[%#x]\r\n", INREG32(A_LCD_LCDCMD1));
	printk("A_LCD_LCDOFFS1[%#x]\r\n", INREG32(A_LCD_LCDOFFS1));
	printk("A_LCD_LCDPW1[%#x]\r\n", INREG32(A_LCD_LCDPW1));
	printk("A_LCD_LCDCNUM1[%#x]\r\n", INREG32(A_LCD_LCDCNUM1));
	printk("A_LCD_LCDDESSIZE1[%#x]\r\n", INREG32(A_LCD_LCDDESSIZE1));
	printk("####### OSD registers #######\r\n");
	printk("LCD_LCDOSDC[%#x]\r\n", INREG16(A_LCD_LCDOSDC));
	printk("LCD_LCDOSDCTRL[%#x]\r\n", INREG16(A_LCD_LCDOSDCTRL));
	printk("LCD_LCDOSDS[%#x]\r\n", INREG16(A_LCD_LCDOSDS));
	printk("A_LCD_LCDBGC[%#x]\r\n", INREG32(A_LCD_LCDBGC));
	printk("A_LCD_LCDKEY0[%#x]\r\n", INREG32(A_LCD_LCDKEY0));
	printk("A_LCD_LCDKEY1[%#x]\r\n", INREG32(A_LCD_LCDKEY1));
	printk("A_LCD_LCDALPHA[%#x]\r\n", INREG8(A_LCD_LCDALPHA));
	printk("A_LCD_LCDIPUR[%#x]\r\n", INREG32(A_LCD_LCDIPUR));
	printk("A_LCD_LCDXYP0[%#x]\r\n", INREG32(A_LCD_LCDXYP0));
	printk("A_LCD_LCDXYP1[%#x]\r\n", INREG32(A_LCD_LCDXYP1));
	printk("A_LCD_LCDSIZE0[%#x]\r\n", INREG32(A_LCD_LCDSIZE0));
	printk("A_LCD_LCDSIZE1[%#x]\r\n", INREG32(A_LCD_LCDSIZE1));
	printk("A_LCD_LCDRGBC[%#x]\r\n", INREG32(A_LCD_LCDRGBC));
	printk("####### slcd registers #######\r\n");
	printk("A_LCD_MCFG[%#x]\r\n", INREG32(A_LCD_MCFG));
	printk("A_LCD_MCTRL[%#x]\r\n", INREG32(A_LCD_MCTRL));
}

void print_lcd_param(void)
{
	printk("lcd vendor:%s\r\n", plcd_mode->vendor);
	printk("lcd type:%s\r\n", plcd_mode->type);
	printk("lcd width:%d\r\n", plcd_mode->width);
	printk("lcd height:%d\r\n", plcd_mode->height);
	printk("lcd bpp:%d\r\n", posd_mode->foreground0.bpp);
	printk("lcd freq:%d\r\n", plcd_mode->freq);
	printk("lcd hsync:%d\r\n", plcd_mode->hsync);
	printk("lcd vsync:%d\r\n", plcd_mode->vsync);
	printk("lcd elw:%d\r\n", plcd_mode->elw);
	printk("lcd blw:%d\r\n", plcd_mode->blw);
	printk("lcd efw:%d\r\n", plcd_mode->efw);
	printk("lcd bfw:%d\r\n", plcd_mode->bfw);
	printk("lcd cfg:0x%x\r\n", plcd_mode->cfg);
}
void print_osd_param()
{
	printk("lcd osd_cfg:%d\r\n", posd_mode->osd_cfg);
	printk("lcd osd_ctrl:%d\r\n", posd_mode->osd_ctrl);
	printk("lcd rgb_ctrl:%d\r\n", posd_mode->rgb_ctrl);
	printk("lcd bgcolor:%d\r\n", posd_mode->bgcolor);
	printk("lcd colorkey0:%d\r\n", posd_mode->colorkey0);
	printk("lcd colorkey1:%d\r\n", posd_mode->colorkey1);
	printk("lcd alpha:%d\r\n", posd_mode->alpha);
	printk("lcd ipu_restart:%d\r\n", posd_mode->ipu_restart);
	printk("lcd fg_change:%d\r\n", posd_mode->fg_change);

	printk("lcd foreground0:%d\r\n", posd_mode->foreground0.bpp);
	printk("lcd foreground0:%d\r\n", posd_mode->foreground0.x);
	printk("lcd foreground0:%d\r\n", posd_mode->foreground0.y);
	printk("lcd foreground0:%d\r\n", posd_mode->foreground0.w);
	printk("lcd foreground0:%d\r\n", posd_mode->foreground0.h);

	printk("lcd foreground1:%d\r\n", posd_mode->foreground1.bpp);
	printk("lcd foreground1:%d\r\n", posd_mode->foreground1.x);
	printk("lcd foreground1:%d\r\n", posd_mode->foreground1.y);
	printk("lcd foreground1:%d\r\n", posd_mode->foreground1.w);
	printk("lcd foreground1:%d\r\n", posd_mode->foreground1.h);
}
void print_fb_buff(void)
{
	printk("cpal:%#x\r\n", jzfb.cpal);
	printk("pal:%#x\r\n", jzfb.pal);
	printk("cframe:%#x\r\n", jzfb.cframe);
	printk("frame:%#x\r\n", jzfb.frame);
}
#endif


unsigned int lcd_get_bpp(void)
{
	if (posd_mode->foreground1.bpp > 16)
	{
		return (32);
	}
	return (posd_mode->foreground1.bpp);
}

unsigned int lcd_get_width(void)
{
	return (posd_mode->foreground1.w);
}

unsigned int lcd_get_height(void)
{
	return (posd_mode->foreground1.h);
}

static void lcd_set_wakeup()
{
}

static void lcd_wakeup()
{
	lcd_board_init();
}

unsigned char* lcd_get_frame(void)
{
	return (jzfb.frame);
}

/*****************************************************************************
 函 数 名  : lcd_interrupt_handler
 功能描述  : lcd中断回调
 输入参数  : unsigned int irq
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年9月26日     mj         基本功能的完成

*****************************************************************************/
static void lcd_interrupt_handler(unsigned int irq)
{
	unsigned int state;

	state = INREG32(A_LCD_LCDSTATE);

	if (state & LCD_STATE_EOF) // End of frame
	{
		CLRREG8(A_LCD_LCDSTATE, LCD_STATE_EOF);
	}

	if (state & LCD_STATE_IFU0)
	{
		CLRREG8(A_LCD_LCDSTATE, LCD_STATE_IFU0);
	}

	if (state & LCD_STATE_IFU1)
	{
		CLRREG8(A_LCD_LCDSTATE, LCD_STATE_IFU1);
	}

	if (state & LCD_STATE_OUF)		// Out fifo underrun
	{
		CLRREG8(A_LCD_LCDSTATE, LCD_STATE_OUF);
	}
}

static int fb_malloc(void)
{
	unsigned int bpp;

	if (posd_mode->foreground1.bpp == 15)
	{
		bpp = 16;
	}

	if(posd_mode->foreground1.bpp > 16)
	{
		bpp = 32;
	}
	else
	{
		bpp = posd_mode->foreground1.bpp;
	}

	//needroom = ((plcd_mode->width * t + 7) >> 3) * plcd_mode->height;
	needroom = (((posd_mode->foreground1.w * bpp + 7) >> 3) * posd_mode->foreground1.h + 63) & ~63;
//	printk("slcd 1frame buff addr = %x %d\n", _framebuffer, needroom);
#ifdef FRAME_BUFF_ADDRESS
	jzfb.cpal = (unsigned char*) _framebuffer;

	jzfb.cframe = (unsigned char *)((unsigned int)_framebuffer);
//	printk("slcd 2frame buff addr = %x %d\n", jzfb.cframe, needroom );
	// lcd when 3buf is 16 bit
#ifdef FRAME_BUFF_DECS
	lcd_frame_desc2_room = (LCD_DESCRIPTOR_NEW *)FRAME_BUFF_DECS;
#else
	lcd_frame_desc2_room =  (LCD_DESCRIPTOR_NEW *)((((unsigned int)_framebuffer + (unsigned int)needroom) + 63) & ~63);
#endif
#else
	jzfb.cpal = (unsigned char *)(((unsigned int)_framebuffer) & ~0xfff);
	jzfb.cframe = (unsigned char *)((unsigned int)jzfb.cpal + 0x1000);
#endif

	jzfb.pal = (unsigned char *)LCD_UNCACHED(jzfb.cpal);
	jzfb.frame = (unsigned char *)LCD_UNCACHED(jzfb.cframe);
	fg0_frame = jzfb.frame + needroom;

	memset(jzfb.cpal, 0, 512);

	return (0);
}

static void foreground_param_check(POSD_FG posd_fg, unsigned int fg_num)
{
	unsigned int width = plcd_mode->width;
	unsigned int height = plcd_mode->height;
	unsigned int bpp;

	// foreground width height x and y check
	posd_fg->x = (posd_fg->x >= width) ? 0 : posd_fg->x;
	posd_fg->y = (posd_fg->y >= width) ? 0 : posd_fg->y;
	posd_fg->w = (posd_fg->w < width) ? posd_fg->w : width;
	posd_fg->h = (posd_fg->h < height) ? posd_fg->h : height;
	posd_fg->x = (posd_fg->x + posd_fg->w > width) ? (width - posd_fg->w) / 2 : posd_fg->x;
	posd_fg->y = (posd_fg->y + posd_fg->h > height) ? (height - posd_fg->h) / 2 : posd_fg->y;

	if (fg_num == 0)
	{
		bpp = posd_mode->foreground0.bpp;
	}
	else
	{
		bpp = posd_mode->foreground1.bpp;
	}

	if (bpp == 15)
	{
		posd_fg->bpp = 16;
	}
	else if (bpp > 16)
	{
		posd_fg->bpp = 32;
	}
	else
	{
		posd_fg->bpp = bpp;
	}

}

static void lcd_descriptor_init(void)
{
	//int i;
	unsigned int pal_size;
	unsigned int frm_size1, frm_size0;
	unsigned char dual_panel = 0;
	LCD_DESCRIPTOR_NEW *pal_desc, *frame_desc0, *frame_desc1;
	LCD_DESCRIPTOR_NEW *frame_desc_cmd0;
	lcd_frame_desc2 = (LCD_DESCRIPTOR_NEW *)LCD_UNCACHED(lcd_frame_desc2_room);
	OSD_FG osd_fg0;
	OSD_FG osd_fg1;

	pal_desc	= &lcd_palette_desc;
	frame_desc_cmd0	= fg0_frame_desc = &lcd_frame_desc2[0];
	frame_desc0	= &lcd_frame_desc2[1];
	frame_desc1	= fg1_frame_desc = &lcd_frame_desc2[4];

	memcpy(&osd_fg0, &posd_mode->foreground0, sizeof(OSD_FG));
	memcpy(&osd_fg1, &posd_mode->foreground1, sizeof(OSD_FG));
	foreground_param_check(&osd_fg0, 0);
	foreground_param_check(&osd_fg1, 1);

	frm_size1 = (osd_fg1.w * osd_fg1.h * osd_fg1.bpp) >> 3;
	frm_size0 = (osd_fg0.w * osd_fg0.h * osd_fg0.bpp) >> 3;

	if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
	{
		dual_panel = 1;
		frm_size1 >>= 1;
	}

	frm_size1 /= 4;
	frm_size0 /= 4;

	switch (osd_fg0.bpp)
	{
		case 1:
			pal_size = 4;
			break;
		case 2:
			pal_size = 8;
			break;
		case 4:
			pal_size = 32;
			break;
		case 8:
		default:
			pal_size = 512;
			break;
	}

	pal_size /= 4;

	// Palette Descriptor
	pal_desc->NEXTDESC_PA	= PHYS(frame_desc0);
	pal_desc->DATA_PA		= PHYS(jzfb.pal);
	pal_desc->FRAME_ID		= 0xdeadbeaf;
	pal_desc->COMMAND		= pal_size | LCD_CMD_PAL; // Palette Descriptor

	//if (!(dual_panel))
	//	return;

  	// Frame Descriptor 0
	frame_desc0->NEXTDESC_PA	= PHYS(frame_desc_cmd0);
	frame_desc0->DATA_PA		= PHYS(jzfb.pal);
	frame_desc0->FRAME_ID		= 0xbeafbeaf;
	frame_desc0->COMMAND		= frm_size0; //LCD_CMD_SOFINT | LCD_CMD_EOFINT | frm_size;
	frame_desc0->DESC_SIZE	 	= (osd_fg0.h << 16) | osd_fg0.w;
	frame_desc0->OFFSET	 		= 0;
	frame_desc0->PAGE_WIDTH		= 0;
	memcpy(&fg0_frame_desc[3], frame_desc0, sizeof(LCD_DESCRIPTOR_NEW));

	frame_desc_cmd0->NEXTDESC_PA	= PHYS(frame_desc0);
	frame_desc_cmd0->DATA_PA		= 0;
	frame_desc_cmd0->FRAME_ID		= 0xbeafbeae;
	frame_desc_cmd0->COMMAND		= LCD_CMD_CMD | 0;
	frame_desc_cmd0->OFFSET	 		= 0;
	frame_desc_cmd0->PAGE_WIDTH		= 0;
	frame_desc_cmd0->CMD_NUM		= 0;
	frame_desc_cmd0->DESC_SIZE	 	= 0;
	memcpy(&fg0_frame_desc[2], frame_desc_cmd0, sizeof(LCD_DESCRIPTOR_NEW));

	// Frame Descriptor 1
	frame_desc1->NEXTDESC_PA	= PHYS(frame_desc1);
	frame_desc1->DATA_PA		= PHYS(jzfb.frame);
	frame_desc1->FRAME_ID		= 0xdeaddead;
  	frame_desc1->COMMAND	 	= frm_size1;
  	frame_desc1->DESC_SIZE 		= (osd_fg1.h << 16) | osd_fg1.w;
  	frame_desc1->OFFSET	 		= 0;
	frame_desc1->PAGE_WIDTH		= 0;
}

#if 0
static int print_lcd_clock(void)
{
	unsigned int pll_clock = GetCurrentPLLClock(1);
	unsigned int pix_clock = pll_clock / ((INREG32(A_CPM_LPCDR) & 0x7ff) + 1);

	trace1("pll_clock[%d], ", (unsigned long)pll_clock);
	trace1("pixel clk[%d]\r\n", (unsigned long)pix_clock);

	return 0;
}

int osd_init(void)
{
	OSD_FG osd_fg0;
	OSD_FG osd_fg1;

	memcpy(&osd_fg0, &posd_mode->foreground0, sizeof(OSD_FG));
	memcpy(&osd_fg1, &posd_mode->foreground1, sizeof(OSD_FG));
	foreground_param_check(&osd_fg0, 0);
	foreground_param_check(&osd_fg1, 1);

	if(osd_fg1.bpp > 16)
	{
		posd_mode->osd_ctrl = (posd_mode->osd_ctrl & ~LCD_BPP_MASK) | LCD_CTRL_BPP_18_24;
	}
	else
	{
		posd_mode->osd_ctrl = (posd_mode->osd_ctrl & ~LCD_BPP_MASK) | LCD_CTRL_BPP_16;
	}

	// OSD init
	OUTREG16(A_LCD_LCDOSDC, posd_mode->osd_cfg);
	OUTREG16(A_LCD_LCDOSDCTRL, posd_mode->osd_ctrl);
	OUTREG16(A_LCD_LCDRGBC, posd_mode->rgb_ctrl);
	OUTREG32(A_LCD_LCDBGC, posd_mode->bgcolor);
	OUTREG32(A_LCD_LCDKEY0, posd_mode->colorkey0);
	OUTREG32(A_LCD_LCDKEY1, posd_mode->colorkey1);
	OUTREG8(A_LCD_LCDALPHA, (unsigned char)posd_mode->alpha);
	OUTREG32(A_LCD_LCDIPUR, posd_mode->ipu_restart);
	OUTREG32(A_LCD_LCDXYP0, (posd_mode->foreground0.y << 16) | (posd_mode->foreground0.x << 0));
	OUTREG32(A_LCD_LCDXYP1, (posd_mode->foreground1.y << 16) | (posd_mode->foreground1.x << 0));
	OUTREG32(A_LCD_LCDSIZE0, (posd_mode->foreground0.h << 16) | (posd_mode->foreground0.w << 0));
	OUTREG32(A_LCD_LCDSIZE1, (posd_mode->foreground1.h << 16) | (posd_mode->foreground1.w << 0));
	//OUTREG16(A_LCD_LCDOSDS, 0);

	return 0;
}

static int controller_init(void)
{
	unsigned int val = 0/*, reg*/;
	unsigned int pclk;
	unsigned int stnH;
	int ret = 0;

	val = plcd_mode->ctrl;

	switch (posd_mode->foreground0.bpp)
	{
		case 1:
			val |= LCD_CTRL_BPP_1;
			break;
		case 2:
			val |= LCD_CTRL_BPP_2;
			break;
		case 4:
			val |= LCD_CTRL_BPP_4;
			break;
		case 8:
			val |= LCD_CTRL_BPP_8;
			break;
		case 15:
			val |= LCD_CTRL_RGB555;
		case 16:
			val |= LCD_CTRL_BPP_16;
			break;
		case 17 ... 32:
			val |= LCD_CTRL_BPP_18_24;
			break;
		default:
			printk("The BPP %d is not supported\n", posd_mode->foreground0.bpp);
			val |= LCD_CTRL_BPP_16;
			break;
	}

	switch (plcd_mode->cfg & MODE_MASK)
	{
		case MODE_STN_MONO_DUAL:
		case MODE_STN_COLOR_DUAL:
		case MODE_STN_MONO_SINGLE:
		case MODE_STN_COLOR_SINGLE:
			switch (posd_mode->foreground0.bpp)
			{
				case 1:
					// val |= LCD_CTRL_PEDN;
				case 2:
					val |= LCD_CTRL_FRC_2;
					break;
				case 4:
					val |= LCD_CTRL_FRC_4;
					break;
				case 8:
				default:
					val |= LCD_CTRL_FRC_16;
					break;
			}
			break;
		default:
			break;
	}

	switch (plcd_mode->cfg & MODE_MASK)
	{
		case MODE_STN_MONO_DUAL:
		case MODE_STN_COLOR_DUAL:
		case MODE_STN_MONO_SINGLE:
		case MODE_STN_COLOR_SINGLE:
			switch (plcd_mode->cfg & STN_DAT_PINMASK)
			{
				#define align2(n) (n)=((((n)+1)>>1)<<1)
				#define align4(n) (n)=((((n)+3)>>2)<<2)
				#define align8(n) (n)=((((n)+7)>>3)<<3)
				case STN_DAT_PIN1:
					// Do not adjust the hori-param value.
					break;
				case STN_DAT_PIN2:
					align2(plcd_mode->hsync);
					align2(plcd_mode->elw);
					align2(plcd_mode->blw);
					break;
				case STN_DAT_PIN4:
					align4(plcd_mode->hsync);
					align4(plcd_mode->elw);
					align4(plcd_mode->blw);
					break;
				case STN_DAT_PIN8:
					align8(plcd_mode->hsync);
					align8(plcd_mode->elw);
					align8(plcd_mode->blw);
					break;
			}
			break;
		default:
			break;
	}

	OUTREG32(A_LCD_LCDCTRL, val);

	switch (plcd_mode->cfg & MODE_MASK)
	{
		case MODE_STN_MONO_DUAL:
		case MODE_STN_COLOR_DUAL:
		case MODE_STN_MONO_SINGLE:
		case MODE_STN_COLOR_SINGLE:
			if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_DUAL) ||
			    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL))
			{
				stnH = plcd_mode->height >> 1;
			}
			else
			{
				stnH = plcd_mode->height;
			}

			val = (0 << 16) | plcd_mode->vsync;
			OUTREG32(A_LCD_LCDVSYNC, val);
			val = ((plcd_mode->blw + plcd_mode->width) << 16) |
					(plcd_mode->blw + plcd_mode->width + plcd_mode->hsync);
			OUTREG32(A_LCD_LCDHSYNC, val);

			// Screen setting
			val = ((plcd_mode->blw + plcd_mode->width + plcd_mode->hsync + plcd_mode->elw) << 16) |
					(stnH + plcd_mode->vsync + plcd_mode->bfw + plcd_mode->efw);
			OUTREG32(A_LCD_LCDVAT, val);
			val = (plcd_mode->blw << 16) | (plcd_mode->blw + plcd_mode->width);
			OUTREG32(A_LCD_LCDDAH, val);
			val = (0 << 16) | (stnH);
			OUTREG32(A_LCD_LCDDAV, val);

			// AC BIAs signal
			val = (0 << 16) | (stnH+plcd_mode->vsync+plcd_mode->efw+plcd_mode->bfw);
			OUTREG32(A_LCD_LCDPS, val);
			break;

		case MODE_TFT_GEN:
		case MODE_TFT_SHARP:
		case MODE_TFT_CASIO:
		case MODE_8BIT_SERIAL_TFT:
		case MODE_TFT_SAMSUNG:
		case MODE_CCIR656_INT:
		case LCDCFG_MODE_SLCD:
			val = (0 << 16) | plcd_mode->vsync;
			OUTREG32(A_LCD_LCDVSYNC, val);
			val = ((plcd_mode->vsync + plcd_mode->bfw) << 16) |
				  (plcd_mode->vsync + plcd_mode->bfw + plcd_mode->height);
			OUTREG32(A_LCD_LCDDAV, val);
			val = (((plcd_mode->blw + plcd_mode->width + plcd_mode->elw + plcd_mode->hsync)) << 16) | (plcd_mode->vsync + plcd_mode->bfw + plcd_mode->height + plcd_mode->efw);
			OUTREG32(A_LCD_LCDVAT, val);
			val = (0 << 16) | plcd_mode->hsync;
			OUTREG32(A_LCD_LCDHSYNC, val);
			val = ((plcd_mode->hsync + plcd_mode->blw) << 16) | (plcd_mode->hsync + plcd_mode->blw + plcd_mode->width);
			OUTREG32(A_LCD_LCDDAH, val);
			break;
	}

	switch (plcd_mode->cfg & MODE_MASK)
	{
		case MODE_TFT_SAMSUNG:
		case MODE_TFT_SHARP:
		case MODE_TFT_CASIO:
			printk("LCD DOES NOT supported.\n");
			break;
	}

	// Configure the LCD panel
	OUTREG32(A_LCD_LCDCFG, plcd_mode->cfg);
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
	SETREG32(A_LCD_MCTRL,LCD_MCTRL_DMA_EN | LCD_MCTRL_DMAMODE);// | LCD_MCTRL_DMASTART);

	// init osd if needed
	if ((posd_mode->osd_cfg & LCD_OSDC_OSDEN) != 0)
	{
		osd_init();
	}

	// Timing setting
	CLRREG32(A_CPM_CLKGR, CLKGR_STOP_LCD);

	val = plcd_mode->freq; // frame clk

	pclk = val * (plcd_mode->width + plcd_mode->hsync + plcd_mode->elw + plcd_mode->blw) *
	       (plcd_mode->height + plcd_mode->vsync + plcd_mode->efw + plcd_mode->bfw); // Pixclk

	if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_SINGLE) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL))
	{
		pclk = (pclk * 3);
	}

	if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_SINGLE) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_SINGLE) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
	{
		pclk = pclk >> ((plcd_mode->cfg & STN_DAT_PINMASK) >> 4);
	}

	if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
	{
		pclk >>= 1;
	}

	val = GetCurrentPLLClock(1) / pclk;
	OUTREG32(A_CPM_LPCDR, val - 1);
	SETREG32(A_CPM_CPCCR, CPCCR_CHANGE_EN);

#ifdef  TEST_FOR_FPGA
	OUTREG32(A_LCD_LCDREV, 1);
#endif

#ifdef DEBUG_MODE
//	print_lcd_param();
//	print_reg();
#endif

	print_lcd_clock();

	CLRREG32(A_CPM_CLKGR, CLKGR_STOP_LCD);
	return (ret);
}
#endif

static void clear_screen(unsigned char *buf, int size, int color)
{
	int i;
	for (i = 0; i < size / 4; i++)
	{
		*((unsigned int *)buf) = color;
		buf += 4;
	}
}

void lcd_clean_frame_all()
{
	clear_screen(jzfb.cframe, lcd_get_width() * lcd_get_height() * lcd_get_bpp() / 8, 0);
}

void lcd_flush_frame_all()
{
	__dcache_writeback_all();
}

static void lcd_reinit()
{
    //int t, i;

	CLRREG32(A_LCD_LCDCTRL, LCD_CTRL_ENA);
	fb_malloc();

	__dcache_writeback_all();
	trace("lcd_descriptor_init\n");
	lcd_descriptor_init();
	//controller_init();
	if (posd_mode->foreground0.bpp <= 8)
	{
		OUTREG32(A_LCD_LCDDA0, PHYS(&lcd_palette_desc));
	}
	else
	{
		OUTREG32(A_LCD_LCDDA0, PHYS(fg0_frame_desc));
	}

	if (((plcd_mode->cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((plcd_mode->cfg & MODE_MASK) == MODE_STN_MONO_DUAL)  ||
	    ((posd_mode->osd_cfg & LCD_OSDC_F1EN) != 0))
	{
		OUTREG32(A_LCD_LCDDA1, PHYS(fg1_frame_desc));
	}

	//>lfx if ((INREG32(A_LCD_LCDCFG) & LCDCFG_TVEN) == 0)
	//>lfx {
	//>lfx 	clear_screen((unsigned char *)jzfb.cframe, needroom, 0);
	//>lfx 	lcd_flush_frame_all();
	//>lfx }
}

int lcd_open(void)
{
	//unsigned int enable = 1, i;

	//if(!(INREG32(A_LCD_LCDCTRL) & LCD_CTRL_ENA))
    {
		SETREG32(A_LCD_LCDCTRL, LCD_CTRL_ENA);
    }
#if 0
	for (i = 0; i < 10; i++)
	{
		if (!(INREG32(A_LCD_LCDCTRL) & LCD_CTRL_ENA))
		{
			enable = 0;
			break;
		}
	}
	trace1("i = %d\n",(unsigned long)i);
	if (enable == 0)
	{
		printk("Put CPU into hibernate mode.\n");
		serial_waitfinish();

		RTC_OUTREG32(A_RTC_HWRSR, 0);
		RTC_OUTREG32(A_RTC_RTCSAR, RTC_INREG32(A_RTC_RTCSR) + 2);
		RTC_SETREG32(A_RTC_HWCR, HWCR_EALM);
		RTC_SETREG32(A_RTC_HRCR, 0x0fe0);
		RTC_SETREG32(A_RTC_HWFCR, (0x00FF << 4));
	   	RTC_SETREG32(A_RTC_HCR, HCR_PD);
	}
	else
	{
		if (plcd_mode->cfg & LCDCFG_RECOVER)
        {
            CLRREG32(A_LCD_LCDCTRL, LCD_CTRL_ENA);
            SETREG32(A_LCD_LCDCFG, LCDCFG_RECOVER);
            SETREG32(A_LCD_LCDCTRL, LCD_CTRL_ENA);
        }
		trace1("get the ENA BIT 0x%x!!\n", (unsigned long)INREG32(A_LCD_LCDCTRL));
	}
#endif
	return (0);
}

void lcd_slcd_special_on()
{
	int SpeLinda; //没加电阻的凌达屏PD4为1
	unsigned int m_hardwarever = Get_ScreenHardVer();

	SpeLinda = m_hardwarever >> 3;

#if (LCD_MODEL == OTM3225C)
	Init_otm3225c();
#elif (LCD_MODEL == OTM4802A)
	Init_otm4802A();
#endif
}

void DrawBeginMap(unsigned char *buf, unsigned short x, unsigned short y, unsigned short w, unsigned short h,unsigned char setTrans,unsigned short transColor)
{
#if 0
	unsigned int i,j;
	U32 offsetPos;
	U32  desbufAdr;
	unsigned short *sptr,*dptr;
	unsigned char *LCDSTARTADDR;
	unsigned short validy;

    if(buf == NULL)
    {
        return ;
    }
	LCDSTARTADDR   = LcdTmpBuffer;

	sptr = (unsigned short*)buf;
	if(w > LCD_WIDTH)
    {
		w= LCD_WIDTH;
    }
	if(h > LCD_HEIGHT)
    {
		h = LCD_HEIGHT;
    }
	if(x + w > LCD_WIDTH)
    {
		x= LCD_WIDTH - w;
    }
	if(y + h > LCD_HEIGHT)
    {
		y= LCD_HEIGHT - h;
    }

	for(i=y;i< y+h;i++)
	{
		validy = LCD_HEIGHT - i - 1;
		offsetPos = LCD_HEIGHT*x + validy;
		desbufAdr = (unsigned long)LCDSTARTADDR + offsetPos * 2;

		dptr=(unsigned short*)desbufAdr;

		for(j=0;j<w;j++)
		{
			if ( setTrans && (transColor == *sptr) )
			{
				;
			}
			else
            {
				*dptr = *sptr;
            }

			sptr++;
			dptr += LCD_HEIGHT;
		}
	}

//DMA_Init();
//lcd_StartAudioPlay(LcdTmpBuffer,lcd_get_frame(),153600);

	memcpy(lcd_get_frame(), LcdTmpBuffer, 153600);   //shh 可以用dma吗？
#endif
}

//画点
void lcd_SetPixel(int x, int y, unsigned short color)
{
	U32 pos_sdramAddr;
    U32 offsetPos;
    U16 *pos_ptr;
	unsigned char *cur_vram_lcdc_addr = lcd_get_frame();

	offsetPos = x + y*LCD_WIDTH;
    pos_sdramAddr = (unsigned long)cur_vram_lcdc_addr + offsetPos * 2;
    pos_ptr = (U16*)pos_sdramAddr;
    *pos_ptr = color;
}

/*****************************************************************************
 函 数 名  : jzlcd_init
 功能描述  : 平台lcd初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年9月26日     mj         基本功能的完成

*****************************************************************************/
int jzlcd_init(void)
{
	PM_AddWakeUp(LcdID, lcd_set_wakeup, lcd_wakeup);

	request_irq(IRQ_LCD, lcd_interrupt_handler, 0);

	lcd_board_init();

	//>lfx lcd_set_backlight(*pscreen_light);

	lcd_reinit();

	//lcd_slcd_special_on();

	lcd_open();

#ifdef DEBUG_MODE
//	print_reg();
#endif

	return (0);
}

/*****************************************************************************
 函 数 名  : lcd_copy_screen
 功能描述  : 拷贝屏幕数据并将图片旋转回来
 输入参数  : void *des_buf
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年11月8日     mj         基本功能的完成

*****************************************************************************/
void lcd_copy_screen(void *des_buf)
{
    int i = 0;
	int j = 0;
	//int y = 0;
	unsigned short *sptr,*dptr;
	U32  srcbufAdr;
	unsigned short validy;
	unsigned char *LCDSTARTADDR;
    //U32	offsetPos;
	//U32	desbufAdr;

    LCDSTARTADDR = lcd_get_frame();

	dptr = (unsigned short *)des_buf;

	for(i=0;i< SCREEN_HEIGHT;i++)
	{
		validy = SCREEN_HEIGHT - i - 1;
		srcbufAdr = (unsigned long)LCDSTARTADDR + validy * LCD_BPP_UNIT;

		sptr=(unsigned short*)srcbufAdr;

		for(j=0;j<SCREEN_WIDTH;j++)
		{
			*dptr = *sptr;
			dptr++;
			sptr += SCREEN_HEIGHT;
		}
	}
}


/*****************************************************************************
 函 数 名  : lcd_init
 功能描述  : lcd设备驱动初始化
 输入参数  : rt_device_t dev
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年9月26日     mj         基本功能的完成

*****************************************************************************/
static device_err_t lcd_init (device_t dev)
{
    //int m_order;

	//unsigned char * pLcdMemPtr;

	if ((jzlcd_init() < 0))
	{
	    trace("lcd 初始化失败\n");
		return (-1);
	}

    _lcd_info.framebuffer = lcd_get_frame();

    trace("lcd 初始化成功\n");

    //显示第一个画面
    //sys_res_drawpic(WELCOME_BK_PATH);

	extern unsigned char GetBootManager(void);

	if (GetBootManager() == BOOT_TO_USB && 0 == __gpio_get_pin(GPIO_USB_INT))
	{
		draw_kernel_bitmap_with_backcolor(USB_LINKING, -1, -1, -1);
	}
	else
	{
		draw_kernel_bitmap_with_backcolor(LOGO, -1, -1, 0xffff);
	}
	lcd_set_backlight(*pscreen_light);

	return (0);
}

/*****************************************************************************
 函 数 名  : lcd_control
 功能描述  : lcd传入一个控制参数给设备驱动
 输入参数  : device_t dev
             u8 cmd
             void *args
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年9月26日     mj         基本功能的完成

*****************************************************************************/
static device_err_t lcd_control (device_t dev, u8 cmd, void *args)
{
	switch (cmd)
	{
    	case GRAPHIC_CTRL_RECT_UPDATE:
    		break;
    	case GRAPHIC_CTRL_POWERON:
    		break;
    	case GRAPHIC_CTRL_POWEROFF:
    		break;
    	case GRAPHIC_CTRL_GET_INFO:
        {
    		memcpy(args, &_lcd_info, sizeof(_lcd_info));
        }
        break;
    	case GRAPHIC_CTRL_SET_MODE:
    		break;
        case GRAPHIC_CTRL_MEM_TO_SCREEN:
        {
            if(args == NULL)
            {
                return -1;
            }

            DrawBeginMap((unsigned char *)args,0,0,LCD_WIDTH,LCD_HEIGHT,0,0);
        }
        break;

        case GRAPHIC_CTRL_SET_PIXEL:
        {
            struct device_graphic_setpixel *m_setpixel;

            if(args == NULL)
            {
                return -1;
            }

            m_setpixel = (struct device_graphic_setpixel *)args;
            lcd_SetPixel(m_setpixel->x,m_setpixel->y,m_setpixel->color);
        }
        break;

        case GRAPHIC_CTRL_DRAW_BEGIN_MAP:
        {
            struct device_graphic_drawbeginmap_info *m_beginmap;

            if(args == NULL)
            {
                return -1;
            }

            m_beginmap = (struct device_graphic_drawbeginmap_info *)args;

            DrawBeginMap(m_beginmap->buf, m_beginmap->x, m_beginmap->y, m_beginmap->w, m_beginmap->h,m_beginmap->setTrans,m_beginmap->transColor);
        }
        break;

        case GRAPHIC_CTRL_OPEN_BACKLIGHT:
        {
			printk("lcd_control 打开背光 = %d\r\n", *pscreen_light);
			if (*pscreen_light > 0 && *pscreen_light <= 100)
			{
				lcd_set_backlight(*pscreen_light);	//打开背光
			}
			else
			{
				lcd_set_backlight(SCREEN_LIGHT_MID);	//打开背光
			}
        }
        break;

        case GRAPHIC_CTRL_CLOSE_BACKLIGHT:
        {
			printk("lcd_control 关闭背光! \r\n");
            lcd_close_backlight();//关闭背光
        }
        break;

		case GRAPHIC_CTRL_TEST_SET_IO_HIGH:
			lcd_test_set_port_level(1);
			break;
		case GRAPHIC_CTRL_TEST_SET_IO_LOW:
			lcd_test_set_port_level(0);
			break;
		case GRAPHIC_CTRL_TEST_SET_FUNCTION:
			lcd_test_enable_function();
			break;

		case GRAPHIC_CTRL_SET_BACKLIGHT:	//设置背光
		{
			int backlight = *((int *)args);
		//	printk("lcd_control GRAPHIC_CTRL_SET_BACKLIGHT backlight = %d\r\n", backlight);

			if (backlight > 100)
			{
				backlight = 100;
			}
			else if (backlight < 0)
			{
				backlight = 0;
			}

		//	printk("lcd_control GRAPHIC_CTRL_SET_BACKLIGHT backlight = %d\r\n", backlight);
			lcd_set_backlight(backlight);
		}
		break;

        case GRAPHIC_CTRL_PLAY_CARTOON:
        {
            //显示第一个画面
            //sys_res_drawpic(WELCOME_BK_PATH);

            lcd_flush_frame_all();
        }
        break;

        case GRAPHIC_CTRL_CLEAN_SCREEN:
        {
            //lcd_clean_frame_all();

        //    memset(LcdTmpBuffer,0x0,153600);
            memset(lcd_get_frame(), 0, 153600);
			ddi_slcd_flush_ram();
        }
        break;

        case GRAPHIC_CTRL_SET_WHITEBK:
        {
            clear_screen(lcd_get_frame(), lcd_get_width() * lcd_get_height() * lcd_get_bpp() / 8, 0xFFFFFFFF);
        }
        break;

        case GRAPHIC_CTRL_COPY_SCREEN:
        {
            lcd_copy_screen(args);
        }
        break;
	case GRAPHIC_CTRL_GET_MEM_PTR:		//获取LCD缓存指针
		*((unsigned char **)args) = ddi_slcd_get_mem_ptr();
		break;
	case GRAPHIC_CTRL_UPDATE_SCR:		//将数据刷新到LCD物理屏幕上
		ddi_slcd_flush_ram();
		break;
	case GRAPHIC_CTRL_GET_DMA_STATE:
		if ( !(INREG32(A_LCD_MSTATE) & LCD_MSTATE_BUSY))
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
        default:
            break;
	}

	return (0);
}

/*****************************************************************************
 函 数 名  : lcd_read
 功能描述  : 获取屏上图像
 输入参数  : device_t dev
             device_off_t pos
             void* buffer
             device_size_t size
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年10月14日     mj         基本功能的完成

*****************************************************************************/
static device_size_t lcd_read(device_t dev, device_off_t pos, void* buffer, device_size_t size)
{
    memcpy(buffer, lcd_get_frame(), size);
	return size;
}

/*****************************************************************************
 函 数 名  : lcd_write
 功能描述  : 往屏上写数据
 输入参数  : device_t dev
             device_off_t pos
             const void* buffer
             device_size_t size
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年10月14日     mj         基本功能的完成

*****************************************************************************/
static device_size_t lcd_write(device_t dev, device_off_t pos, const void* buffer, device_size_t size)
{
    DrawBeginMap((unsigned char *)buffer,0,0,LCD_WIDTH,LCD_HEIGHT,0,0);
	return size;
}

/*****************************************************************************
 函 数 名  : hw_lcd_init
 功能描述  : 注册lcd驱动
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2011年9月26日     mj         基本功能的完成

*****************************************************************************/
void hw_lcd_init(void)
{
    _lcd_info.bits_per_pixel = 16;
	_lcd_info.pixel_format = GRAPHIC_PIXEL_FORMAT_RGB565;
	_lcd_info.width = LCD_WIDTH;
	_lcd_info.height = LCD_HEIGHT;

    lcd.type = Device_Class_Unknown;
	lcd.init = lcd_init;
	lcd.open = NULL;
	lcd.close = NULL;
	lcd.control = lcd_control;
    lcd.read = lcd_read;
    lcd.write = lcd_write;
	lcd.user_data = (void*)&_lcd_info;

    device_register(&lcd, "lcd", DEVICE_FLAG_RDWR);
}

/*****************************************************************************
 函 数 名  : start_slcd_dma
 功能描述  : 开始使用DMA传输数据
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2012年2月14日     Ryan         基本功能的完成

*****************************************************************************/
void	start_slcd_dma(void)
{
	int ready = 0;
	int waitCount = 0;


	if ((INREG32(A_LCD_MCTRL) & LCD_MCTRL_DMAMODE) && (INREG32(A_LCD_LCDCTRL)& LCD_CTRL_ENA))
	{
		while (1)
		{
			if ( !(INREG32(A_LCD_MSTATE) & LCD_MSTATE_BUSY))
			{
				ready = 1;
				break;
			}
			waitCount++;

			if (waitCount > 100000)
			{
				printk(" LCD ERROR: DMA wait timeout! \r\n");
				break;
			}
		}
	}
	else
	{
		printk(" LCD ERROR: DMA environment is not ready! \r\n");
	}

	if (1 == ready)
	{
		OUTREG32(A_LCD_LCDDA0, PHYS(fg0_frame_desc));
		SETREG32(A_LCD_MCTRL,  LCD_MCTRL_DMASTART);
	}
}

/*****************************************************************************
 函 数 名  : ddi_slcd_flush_ram
 功能描述  : 将LCD缓存中的数据刷新到物理屏幕
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2012年2月14日     Ryan         基本功能的完成

*****************************************************************************/
void	ddi_slcd_flush_ram(void)
{
#if 1
	plcd_mode->slcd_cfg &= ~(7<<10);
	plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_ONCE;
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
#endif
	Mcupanel_Command(0x2A);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x50);
	Mcupanel_Data(0x01);
	Mcupanel_Data(0x8F);

	Mcupanel_Command(0x2B);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x28);
	Mcupanel_Data(0x01);
	Mcupanel_Data(0x17);

	Mcupanel_Command(0x29);
	Mcupanel_Command(0x2C);
#if 1
	plcd_mode->slcd_cfg &= ~(7<<10);
	plcd_mode->slcd_cfg |= LCD_MCFG_DWIDTH_8BIT_TWICE;
	OUTREG32(A_LCD_MCFG, plcd_mode->slcd_cfg);
#endif

	lcd_flush_frame_all();
	start_slcd_dma();
}

#define W_DATA(data1, data2)		Mcupanel_Data(((unsigned short)data1<<8)|data2)
void	ddi_slcd_flush_ram_to_reg(char* buff)
{
	int i = 0;
	unsigned char* lcd_buff = lcd_get_frame();

	lcd_flush_frame_all();

	for (i=0; i < 320 * 240 * 2;i += 2)
	{
		W_DATA(lcd_buff[i+1], lcd_buff[i]);
	}
}

/*****************************************************************************
 函 数 名  : ddi_slcd_get_mem_ptr
 功能描述  : 获取LCD缓存的指针
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2012年2月14日     Ryan         基本功能的完成

*****************************************************************************/
unsigned char* 	ddi_slcd_get_mem_ptr(void)
{
	return lcd_get_frame();
}

/*****************************************************************************
 函 数 名  : ryan_lcd_set_backlight_test_entry
 功能描述  : kernel中的调节背光测试函数
 输入参数  : void
 输出参数  : 无
 返 回 值  : 无

 修改历史      :

 版本		日期		作者		修改内容及原因
 1.0    2012年2月14日     Ryan         基本功能的完成

*****************************************************************************/
void	ryan_lcd_set_backlight_test_entry(void)
{
	int bk = 50;

	printk("                                          ryan_lcd_set_backlight_test_entry \r\n");

	while (1)
	{
		bk += 10;
		if (bk > 100)
		{
			bk = 10;
		}

		printk("ryan_lcd_set_backlight_test_entry = %d\r\n", bk);
		device_control(device_find("lcd"), GRAPHIC_CTRL_SET_BACKLIGHT, &bk);

		mdelay(1000);
	}
}

