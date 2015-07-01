/*********************************Copyright (c)*********************************
**                               
**
**--------------File Info-------------------------------------------------------
** File Name:               sim_i2c.c
** Last modified Date:      
** Last Version:            
** Description:             Ä£ÄâI2C½Ó¿Ú(Ä¬ÈÏ100kbps)
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
¸üÐÂËµÃ÷:
    
******************************************************************************/

/******************************************************************************
*********************************  Ó¦ ÓÃ ×Ê ÁÏ ********************************
******************************************************************************/

/******************************************************************************
********************************* ÎÄ¼þÒýÓÃ²¿·Ö ********************************
******************************************************************************/
#include "sim_i2c.h"


/******************************************************************************
******************************* ×Ô¶¨Òå²ÎÊýÅäÖÃ ********************************
******************************************************************************/


/******************************************************************************
********************************* Êý ¾Ý Éù Ã÷ *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:ÓÉÍâÌá¹©   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:ÏòÍâÌá¹©   * 
*----------------------*/
//none

/******************************************************************************
********************************* º¯ Êý Éù Ã÷ *********************************
******************************************************************************/
char  test=0;
/*---------------------* 
*    IMPORT:ÓÉÍâÌá¹©   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:ÏòÍâÌá¹©   * 
*----------------------*/
//none



/******************************************************************************
*********************************  ³ÌÐò¿ªÊ¼  **********************************
******************************************************************************/
/******************************************************************************
/ º¯Êý¹¦ÄÜ:ms¼¶ÑÓÊ±º¯Êý
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:none
/ Êä³ö²ÎÊý:none
/ Ê¹ÓÃËµÃ÷:none
******************************************************************************/
void Delayms(uint32_t time)
{
   uint16_t i=0;  
   while(time--)
   {
      i=8000;
      while(i--);
   }  
}

/******************************************************************************
/ º¯Êý¹¦ÄÜ:I2C½Ó¿ÚÅäÖÃ³ÌÐò
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:none
/ Êä³ö²ÎÊý:none
/ Ê¹ÓÃËµÃ÷:none
******************************************************************************/
void I2C_Ini(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
    
  //¹Ø±ÕJTAGÊ¹ÄÜSWDÒÔÊÍ·ÅIO¿Ú
  uint32_t u32Temp;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
  u32Temp = AFIO->MAPR;
  u32Temp &= ~AFIO_MAPR_SWJ_CFG ;
  u32Temp |= AFIO_MAPR_SWJ_CFG_1;
  AFIO->MAPR = u32Temp;
  
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
  
  //ÅäÖÃI2CµÄSCLÒý½ÅºÍSDAÒý½Å
  RCC_APB2PeriphClockCmd(AHRS_I2C_GPIO_CLK , ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin =  AHRS_I2C_SclPin;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin =   AHRS_I2C_SdaPin;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  SCL_H;SDA_H;    //×ÜÏßÊÍ·Å
  
  //ÅäÖÃÆäËûIO¿
//  RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK , ENABLE);
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Pin =  LED1_GPIO_PIN;
//  GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Pin =  LED2_GPIO_PIN;
//  GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
//  
//  //ÅäÖÃÊäÈë
//  RCC_APB2PeriphClockCmd(KEY_GPIO_CLK, ENABLE);
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//  GPIO_InitStructure.GPIO_Pin =  KEY_GPIO_PIN;
//  GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);
}

/******************************************************************************
/ º¯Êý¹¦ÄÜ:Simulation IIC Timing series delay
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:none
/ Êä³ö²ÎÊý:none
/ Ê¹ÓÃËµÃ÷:none
******************************************************************************/
__inline void I2C_delay(void)
{
        
   u8 i=I2C_DELAY_VAL; //ÕâÀï¿ÉÒÔÓÅ»¯ËÙ¶È   £¬¾­²âÊÔ×îµÍµ½5»¹ÄÜÐ´Èë
   while(i) 
   { 
     i--; 
   }  
}


/******************************************************************************
/ º¯Êý¹¦ÄÜ:ÑÓÊ±5msÊ±¼ä
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:none
/ Êä³ö²ÎÊý:none
/ Ê¹ÓÃËµÃ÷:none
******************************************************************************/
void delay5ms(void)
{
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather  Start
****************************************************************************** */
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read)return false;  //SDAÏßÎªµÍµçÆ½Ôò×ÜÏßÃ¦,ÍË³ö
    SDA_L;
    I2C_delay();
    if(SDA_read) return false;  //SDAÏßÎª¸ßµçÆ½Ôò×ÜÏß³ö´í,ÍË³ö
    SDA_L;
    I2C_delay();
    return true;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{   
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{   
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather  Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void)   //·µ»ØÎª:=1ÓÐACK,=0ÎÞACK
{
    SCL_L;
    I2C_delay();
    SDA_H;          
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_read)
    {
      SCL_L;
      I2C_delay();
      return false;
    }
    SCL_L;
    I2C_delay();
    return true;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
uint8_t I2C_RadeByte(void)  //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;              
    while(i--)
    {
        ReceiveByte<<=1;      
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();  
        if(SDA_read)
        {
          ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
} 
/******************************************************************************
/ º¯Êý¹¦ÄÜ:µ¥×Ö½ÚÐ´Èë
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:
/   @arg SlaveAddress   ´ÓÆ÷¼þµØÖ·
/   @arg REG_Address    ¼Ä´æÆ÷µØÖ·
/   @arg REG_data       ÓûÐ´ÈëµÄ×Ö½ÚÊý¾Ý
/ Êä³ö²ÎÊý: ¶Á³öµÄ×Ö½ÚÊý¾Ý
/ Ê¹ÓÃËµÃ÷:ÕâÊ±Ò»¸öÍêÕûµÄµ¥×Ö½Ú¶ÁÈ¡º¯Êý
******************************************************************************/
bool Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);   //·¢ËÍÉè±¸µØÖ·+Ð´ÐÅºÅ//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//ÉèÖÃ¸ßÆðÊ¼µØÖ·+Æ÷¼þµØÖ· 
    if(!I2C_WaitAck()){I2C_Stop(); return false;}
    I2C_SendByte(REG_Address );   //ÉèÖÃµÍÆðÊ¼µØÖ·      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    //delay5ms();
    return true;
}

/******************************************************************************
/ º¯Êý¹¦ÄÜ:µ¥×Ö½ÚÐ´Èë
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:
/   @arg SlaveAddress   ´ÓÆ÷¼þµØÖ·
/   @arg REG_Address    ¼Ä´æÆ÷µØÖ·
/   @arg REG_data       ÓûÐ´ÈëµÄ×Ö½ÚÊý¾Ý
/ Êä³ö²ÎÊý: ¶Á³öµÄ×Ö½ÚÊý¾Ý
/ Ê¹ÓÃËµÃ÷:ÕâÊ±Ò»¸öÍêÕûµÄµ¥×Ö½Ú¶ÁÈ¡º¯Êý
******************************************************************************/
bool Fast_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);   //·¢ËÍÉè±¸µØÖ·+Ð´ÐÅºÅ//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//ÉèÖÃ¸ßÆðÊ¼µØÖ·+Æ÷¼þµØÖ· 
    if(!I2C_WaitAck()){I2C_Stop(); return false;}
    I2C_SendByte(REG_Address );   //ÉèÖÃµÍÆðÊ¼µØÖ·      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    return true;
}



/******************************************************************************
/ º¯Êý¹¦ÄÜ:µ¥×Ö½ÚÐ´Èë
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:
/   @arg SlaveAddress   ´ÓÆ÷¼þµØÖ·
/   @arg REG_Address    ¼Ä´æÆ÷µØÖ·
/ Êä³ö²ÎÊý: ¶Á³öµÄ×Ö½ÚÊý¾Ý
/ Ê¹ÓÃËµÃ÷:ÕâÊ±Ò»¸öÍêÕûµÄµ¥×Ö½Ú¶ÁÈ¡º¯Êý
******************************************************************************/
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
    uint8_t REG_data;       
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//ÉèÖÃ¸ßÆðÊ¼µØÖ·+Æ÷¼þµØÖ· 
    if(!I2C_WaitAck()){I2C_Stop();return false;}
    I2C_SendByte((u8) REG_Address);   //ÉèÖÃµÍÆðÊ¼µØÖ·      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

    REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return true;
    return REG_data;
}

/******************************************************************************
/ º¯Êý¹¦ÄÜ:¶à×Ö½Ú¶Á³öº¯Êý
/ ÐÞ¸ÄÈÕÆÚ:none
/ ÊäÈë²ÎÊý:
/   @arg SlaveAddress   ´ÓÆ÷¼þµØÖ·
/   @arg REG_Address    ¼Ä´æÆ÷µØÖ·
/   @arg ptChar         Êä³ö»º³å
/   @arg size           ¶Á³öµÄÊý¾Ý¸öÊý,size±ØÐë´óÓÚ=1
/ Êä³ö²ÎÊý: ³É¹¦Ê§°Ü±ê¼Ç
/ Ê¹ÓÃËµÃ÷:none
******************************************************************************/
bool Mult_Read(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t * ptChar,uint8_t size)
{
    uint8_t i;
    
    if(size < 1)return false;
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);
    if(!I2C_WaitAck()){I2C_Stop();return false;}
    I2C_SendByte(REG_Address);    
    I2C_WaitAck();
    
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();
    
    //Á¬Ðø¶Á³öax,ay,azÊý¾Ý
    for(i=1;i<size; i++)
    {
        *ptChar++ = I2C_RadeByte();
        I2C_Ack();
    }
    *ptChar++ = I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    return true;    
}




