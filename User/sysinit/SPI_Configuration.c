#include "Configuration.h"

/**************************************************************************************
函数名：SPI2_Configuration
描述　：配置SPI2
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void SPI2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable SPI2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);

	/* Configure SPI2 pins: NSS, SCK, MISO and MOSI */
	/*PB13,PB14,PB15开漏复用功能*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* Configure PB12 as Output push-pull, used as Flash Chip select */
	/*PB12开漏输出功能*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Deselect the Chip*/
	SPI2_Deselected();

	SPI2_HighSpeed(DISABLE);


#if defined (CONFIG_DEBUG)
	printf("Initializing SPI2 ... Done\n");
	DebugPrintf();
#endif
 
}


void SPI2_HighSpeed(FunctionalState newState)
{
	SPI_InitTypeDef  SPI_InitStructure;

	/* Deselect the Chip*/
	SPI2_Deselected();

	SPI_Cmd(SPI2, DISABLE);	// disable first

	/* SPI2 configuration */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //选择2线单向数据模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					   //主设备配置
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;				  //8为数据长度
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  			//空闲时SCK为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 		//数据采样从第二个时钟边沿开始
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	   	//软件从设备管理使能，NSS引脚输入被CR1中SSI位中的值替代
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;   //APB1时钟64分频, 36/64 = 562.5kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //先发送高位
	SPI_InitStructure.SPI_CRCPolynomial = 7;							  //多项式为7

	if(newState == ENABLE)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;   //APB2时钟2分频, 36 /2 = 18MHz
	}
	
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);   										  //使能SPI2

}

/**************************************************************************************
函数名：SPI2_SendByte
描述　：SPI2发送
输入　：无
输出　：无
返回　：无
**************************************************************************************/
uint8 SPI2_SendByte(uint8 byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//	while((SPI2->SR & 0x02) == 0);
	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI2, byte);
//	SPI2->DR = byte;
	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
//	while((SPI2->SR & 0x01) == 0);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
//	return (SPI2->DR);

}



/**************************************************************************************
函数名：SPI3_Configuration
描述　：配置SPI3
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void SPI3_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable SPI3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3 , ENABLE);

	// pin remap
	GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);

	/* Configure SPI3 pins: NSS, SCK, MISO and MOSI */
	/*PC10,PC11,PC12开漏复用功能*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	/* Configure PA15 as Output push-pull, used as Flash Chip select */
	/*PA15开漏输出功能*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Deselect the Chip*/
	SPI3_Deselected();

	/* SPI2 configuration */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //选择2线单向数据模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					   //主设备配置
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;				  //8为数据长度
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  			//空闲时SCK为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 		//数据采样从第二个时钟边沿开始
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	   	//软件从设备管理使能，NSS引脚输入被CR1中SSI位中的值替代
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;   //APB1时钟8分频, 36/8 = 4.5 MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //先发送高位
	SPI_InitStructure.SPI_CRCPolynomial = 7;							  //多项式为7
	SPI_Init(SPI3, &SPI_InitStructure);

	/* Enable SPI3  */
	SPI_Cmd(SPI3, ENABLE);   										  //使能SPI3

#if defined (CONFIG_DEBUG)
	printf("Initializing SPI3 ... Done\n");
	DebugPrintf();
#endif
 
}

/**************************************************************************************
函数名：SPI3_SendByte
描述　：SPI3发送
输入　：无
输出　：无
返回　：无
**************************************************************************************/
uint8 SPI3_SendByte(uint8 byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
//	while((SPI3->SR & 0x02) == 0);

	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI3, byte);
//	SPI3->DR = byte;

	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
//	while((SPI3->SR & 0x01) == 0);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI3);
//	return (SPI3->DR);

}

