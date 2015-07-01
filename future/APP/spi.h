#ifndef _SPI_H_
#define _SPI_H_

#define Select_DA()     	GPIO_ResetBits(GPIOE, GPIO_Pin_0)
/* Deselect SPI FLASH: ChipSelect pin high */
#define NotSelect_DA()    	GPIO_SetBits(GPIOE, GPIO_Pin_0)

#define Select_RF()     	GPIO_ResetBits(GPIOB, GPIO_Pin_12)
/* Deselect SPI FLASH: ChipSelect pin high */
#define NotSelect_RF()    	GPIO_SetBits(GPIOB, GPIO_Pin_12)

#define Enable_RF()		GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define Disable_RF()	GPIO_SetBits(GPIOA, GPIO_Pin_8)

void SPI_DEV_Init(void);
u16 SPI_ReadData(u16 Dummy_Byte);
u16 SPI_SendData(u16 data);
u8 SPI_Send_Byte(u8 byte);


#endif
