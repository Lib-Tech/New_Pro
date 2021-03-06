/*********************************Copyright (c)*********************************
**                               
**
**--------------File Info-------------------------------------------------------
** File Name:               sim_i2c.h
** Last modified Date:      
** Last Version:            
** Description:             i2c配置文件
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
更新说明:
    
******************************************************************************/

/******************************************************************************
*********************************  应 用 资 料 ********************************
******************************************************************************/


#ifndef _SIM_I2C_H_
#define _SIM_I2C_H_

#ifdef __cplusplus
 extern "C" {
#endif
/******************************************************************************
********************************* 文件引用部分 ********************************
******************************************************************************/
#include "stm32f10x.h"
#include <stdbool.h>


/******************************************************************************
********************************** 参数配置 ***********************************
******************************************************************************/
/*---------------------*
*    A- 数据类型定义   * 
*----------------------*/
//none
/*
typedef uint8_t       uint8_t ;   // 重定义8位无符号整形数据类型
typedef unsigned int        uint16_t;   // 注意MCS51 int类型占2个字节
typedef unsigned long       uint32_t;
typedef signed char         int8_t  ;
typedef signed int          int16_t ;
typedef signed long         int32_t ;
*/

/*---------------------*
*    B- 错误代码列表   *
*----------------------*/
//none
/*---------------------*
*   C- 版本与系统信息  *
*----------------------*/
//none
/*---------------------* 
*    D- 芯片时钟设定   * 
*----------------------*/
//none
/******************************************************************************
********************************** 需自定义 ***********************************
****************************** 参数配置、引脚定义  ****************************
******************************************************************************/
//************************** ICC引脚 *********************************
//传感器I2C
#define AHRS_I2C                    I2C1
#define AHRS_I2C_GPIO               GPIOB
#define AHRS_I2C_CLK                RCC_APB1Periph_I2C1
#define AHRS_I2C_GPIO_CLK           RCC_APB2Periph_GPIOB
#define AHRS_I2C_SclPin             GPIO_Pin_6
#define AHRS_I2C_SdaPin             GPIO_Pin_7
/***** 操作宏定义 *****/
#define SCL_H         (AHRS_I2C_GPIO->BSRR = AHRS_I2C_SclPin)
#define SCL_L         (AHRS_I2C_GPIO->BRR  = AHRS_I2C_SclPin)    
#define SDA_H         (AHRS_I2C_GPIO->BSRR = AHRS_I2C_SdaPin)
#define SDA_L         (AHRS_I2C_GPIO->BRR  = AHRS_I2C_SdaPin)
#define SCL_read      (AHRS_I2C_GPIO->IDR  & AHRS_I2C_SclPin)
#define SDA_read      (AHRS_I2C_GPIO->IDR  & AHRS_I2C_SdaPin)


//********************** 定义LED信号指示灯(输出) ********************
#define LED1_GPIO_PORT              GPIOA       //L1信号指示
#define LED1_GPIO_CLK               RCC_APB2Periph_GPIOA 
#define LED1_GPIO_PIN               GPIO_Pin_15
#define LED2_GPIO_PORT              GPIOB       //L2信号指示
#define LED2_GPIO_CLK               RCC_APB2Periph_GPIOB 
#define LED2_GPIO_PIN               GPIO_Pin_3
/***** 操作宏定义 *****/
#define LED1_ON()                   {LED1_GPIO_PORT->BRR  = LED1_GPIO_PIN;}
#define LED1_OFF()                  {LED1_GPIO_PORT->BSRR = LED1_GPIO_PIN;}
#define LED1_DIV()                  {LED1_GPIO_PORT->ODR ^= LED1_GPIO_PIN;}
#define LED2_ON()                   {LED2_GPIO_PORT->BRR  = LED2_GPIO_PIN;}
#define LED2_OFF()                  {LED2_GPIO_PORT->BSRR = LED2_GPIO_PIN;}
#define LED2_DIV()                  {LED2_GPIO_PORT->ODR ^= LED2_GPIO_PIN;}


//*********************** 面板按键(输入)******************************
#define KEY_GPIO_PORT               GPIOB       //按下为高,空闲为低
#define KEY_GPIO_CLK                RCC_APB2Periph_GPIOB  
#define KEY_GPIO_PIN                GPIO_Pin_2
/***** 操作宏定义 *****/
#define KEY_IS_H                    (KEY_GPIO_PORT->IDR & KEY_GPIO_PIN)
#define KEY_IS_L                    (!(KEY_GPIO_PORT->IDR & KEY_GPIO_PIN))

/******************************************************************************
******************************* 自定义参数配置 ********************************
******************************************************************************/
//none
#define I2C_DELAY_VAL   2    //30对应100K,经测试最低到4还能写�,取值[7..255]
                             //4对应400K
                                                          

/******************************************************************************
******************************** 数据类型定义 *********************************
******************************************************************************/
//none

/******************************************************************************
********************************* 数 据 声 明 *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//none

/******************************************************************************
********************************* 函 数 声 明 *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//I2C完整读写函数
extern void    I2C_Ini(void);               //配置I2C接口
extern bool    Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
extern uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address);
extern bool    Fast_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
extern bool    Mult_Read(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t * ptChar,uint8_t size);

//I2C分立操作函数
extern bool I2C_Start(void);              //总线忙活或出错则返回false
extern void I2C_Stop(void);
extern void I2C_Ack(void);
extern void I2C_NoAck(void);
extern bool I2C_WaitAck(void); 	          //返回为:=1有ACK,=0无ACK
extern void I2C_SendByte(u8 SendByte);    //数据从高位到低位
extern uint8_t I2C_RadeByte(void);  //数据从高位到低位

//杂项函数
extern void Delayms(uint32_t time);

/*************************************************************
************************* 其 它 杂 项 ************************
*************************************************************/
//none
 
/******************************************************************************
***********************************   END  ************************************
******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif



