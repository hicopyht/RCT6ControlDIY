#include "Configuration.h"


/***************Configuration of USART1************************************************/
/**************************************************************************************
函数名：USART1_Configuration
描述　：配置USART1
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART1时钟

	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	//配置USART1
	USART_InitStructure.USART_BaudRate = 230400; //115200, 230400;	//波特率115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 	//发送、接收使能
	//配置USART1时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;		//时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;		//SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;		//时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);

	/* Enable USART1 DMA TX request */
  	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	//使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART1
	USART_Cmd(USART1, ENABLE);	   

#ifdef CONFIG_DEBUG
	printf("Initializing USART1 ... Done.\r\n");
	DebugPrintf();
#endif

}


/**************************************************************************************
函数名：USART1_SendData
描述　：USART1发送函数
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART1_SendData(uint16_t Data)
{ 
	USART1->DR = (Data & (uint16_t)0x01FF);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/***************Configuration of USART2************************************************/
/**************************************************************************************
函数名：USART2_Configuration
描述　：配置USART2
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟

	//配置PA2作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置PA3作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置USART2
	USART_InitStructure.USART_BaudRate = 115200;		  			//波特率115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制使能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 	//发送、接收使能
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;		//时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;		//SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;		//时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStruct);

	/* Enable USART2 DMA TX request */
  	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	
	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE);	   

#ifdef CONFIG_DEBUG
	printf("Initializing USART2 ... Done.\r\n");
	DebugPrintf();
#endif
	
}

/**************************************************************************************
函数名：USART2_SendData
描述　：USART2发送函数
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART2_SendData(uint16_t Data)
{ 
	USART2->DR = (Data & (uint16_t)0x01FF);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}


/***************Configuration of USART3************************************************/
/**************************************************************************************
函数名：USART3_Configuration
描述　：配置USART3
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART3时钟

	//配置PB10作为USART3　Tx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB , &GPIO_InitStructure);

	//配置PB11作为USART3　Rx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB , &GPIO_InitStructure);

	//配置USART3
	USART_InitStructure.USART_BaudRate = 115200;						//波特率9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 	//发送、接收使能
	//配置USART3时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;		//时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;		//SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;		//时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStruct);

	/* Enable USART3 DMA TX request */
  	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

	//使能USART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(USART3, ENABLE);	   

#ifdef CONFIG_DEBUG
	printf("Initializing USART3 ... Done\n");
	DebugPrintf();
#endif

}

/**************************************************************************************
函数名：USART3_SendData
描述　：USART3发送函数
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART3_SendData(uint16_t Data)
{ 
	USART3->DR = (Data & (uint16_t)0x01FF);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}


