/******************************************************************************

          版权所有 (C), 2011-2012, 信意电子科技(http://xydz123.taobao.com/)

 ******************************************************************************
  文 件 名   : oled.h
  版 本 号   : v1.0
  作    者   : Guokaiyi
  生成日期   : 2012-11-12
  最近修改   : 
  功能描述   : oled.c 的头文件
  修改历史   :
  1.日    期   : 2012-11-12
    作    者   : Guokaiyi
    修改内容   : 创建文件

******************************************************************************/
#ifndef __OLED_H__
#define __OLED_H__

/*----------------------------------------------------------------------------*
 * 包含头文件                                                                 *
 *----------------------------------------------------------------------------*/
//#include "common.h"

/*----------------------------------------------------------------------------*
 * 宏定义                                                                     *
 *----------------------------------------------------------------------------*/
#define LED_IMAGE_WHITE       1
#define LED_IMAGE_BLACK       0

#define LED_MAX_ROW_NUM      64
#define LED_MAX_COLUMN_NUM  128

/*----------------------------------------------------------------------------*
 * 宏定义                                                                     *
 *----------------------------------------------------------------------------*/
#ifndef VOID
    #define VOID void
#endif /* VOID */

#ifndef UCHAR8
    #define UCHAR8 unsigned char
#endif /* UCHAR8 */

#ifndef CHAR8
    #define CHAR8 char  
#endif /* CHAR8 */

#ifndef USHORT16
    #define USHORT16 unsigned short
#endif /* USHORT16 */

#ifndef SHORT16
    #define SHORT16 short
#endif /* SHORT16 */

#ifndef ULONG32
    #define ULONG32 unsigned long int  
#endif /* ULONG32 */

#ifndef LONG32
    #define LONG32 long int
#endif /* LONG32 */

#ifndef ULONG64
    #define ULONG64 unsigned long long int
#endif /* ULONG64 */

#ifndef LONG64
    #define LONG64 long long int
#endif /* LONG64 */

#ifndef TRUE
    #define TRUE 1
#endif /* TRUE */

#ifndef FALSE
    #define FALSE 0
#endif /* FALSE */

/*----------------------------------------------------------------------------*
 * 全局变量                                                                   *
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 * 内部函数原型                                                               *
 *----------------------------------------------------------------------------*/
#define OLED_CONTROL_PORT  GPIOB
#define OLED_RST   GPIO_Pin_1 	//接RST
#define OLED_CMD   GPIO_Pin_12 	//接CMD
#define OLED_CS    GPIO_Pin_2 	//接CS

/*----------------------------------------------------------------------------*
 * 外部函数原型                                                               *
 *----------------------------------------------------------------------------*/
extern VOID LED_Init(VOID);
extern VOID LED_SetPos(UCHAR8 ucIdxX, UCHAR8 ucIdxY); 
extern VOID LED_P6x8Char(UCHAR8 ucIdxX,UCHAR8 ucIdxY,UCHAR8 ucData);
extern VOID LED_P6x8Str(UCHAR8 ucIdxX,UCHAR8 ucIdxY,UCHAR8 ucDataStr[]);
extern VOID LED_P8x16Str(UCHAR8 ucIdxX,UCHAR8 ucIdxY,UCHAR8 ucDataStr[],UCHAR8 ucFlag);
extern VOID LED_P14x16Str(UCHAR8 ucIdxX,UCHAR8 ucIdxY,UCHAR8 ucDataStr[]);
extern VOID LED_P8x16Char(UCHAR8 ucIdxX, UCHAR8 ucIdxY, UCHAR8 ucData);
extern VOID LED_PXx16MixStr(UCHAR8 ucIdxX, UCHAR8 ucIdxY, UCHAR8 ucDataStr[]);
extern VOID LED_Fill(UCHAR8 ucData);
extern VOID LED_PrintChar(UCHAR8 ucIdxX, UCHAR8 ucIdxY, int cData);
extern VOID LED_PrintUnChar(UCHAR8 ucIdxX, UCHAR8 ucIdxY, unsigned int cData);
extern VOID LED_Printint(UCHAR8 ucIdxX, UCHAR8 ucIdxY, int cData);
extern VOID LED_PrintShort(UCHAR8 ucIdxX, UCHAR8 ucIdxY, SHORT16 sData);
extern VOID LED_PrintImage(UCHAR8 *pucTable, USHORT16 usRowNum, USHORT16 usColumnNum);
extern VOID LED_P20x40Char(UCHAR8 ucIdxX, UCHAR8 ucIdxY, UCHAR8 ucData,UCHAR8 ucFlag);

#endif

