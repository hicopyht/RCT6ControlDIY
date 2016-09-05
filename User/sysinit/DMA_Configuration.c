#include "Configuration.h"

#define ADC1_DR_Adress 		((u32)0x4001244C)
#define ADC3_DR_Address    	((u32)0x40013C4C)
#define USART1_DR_Adress 	((u32)0x40013804)
#define USART2_DR_Adress 	((u32)0x40004404)
#define USART3_DR_Adress 	((u32)0x40004804)
/**************************************************************************************
函数名：DMA_Configuration
描述　：配置DMA. USART1_TX,USART2_TX,USART3_TX,ADC1_2,ADC3使用DMA.
			ADC1_2		DMA1 Channel1
			USART3_TX	DMA1_Channel2
			USART1_TX	DMA1_Channel4
			USART2_TX	DMA1_Channel7
			ADC3		DMA2_Channel5
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//ADC1_2  use DMA1 Channel1
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Adress;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC1_2_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC1_2_CHANNEL_NUM / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);				//允许DMA通道1
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);	//传输完成中断

	//USART1_TX use DMA1_Channel4
	DMA_DeInit(DMA1_Channel4); 	//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Adress;	//以定义DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_Send_Buf ; //定义DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	//外设作为数据传输的目的
	DMA_InitStructure.DMA_BufferSize = USART1_BUF;	//定义指定DMA通道的DMA缓存的大小，单位为数据单位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		//工作在 模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;		//DMA通道1拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	//DMA_Cmd(DMA1_Channel4, ENABLE);		//允许DMA通道4
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);	 //传输完成中断

	//USART2_TX use DMA1_Channel7
	DMA_DeInit(DMA1_Channel7); 	//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Adress;	//以定义DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_Send_Buf ; //定义DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	//外设作为数据传输的目的
	DMA_InitStructure.DMA_BufferSize = USART2_BUF;	//定义指定DMA通道的DMA缓存的大小，单位为数据单位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		//工作在 模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;		//DMA通道1拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	//DMA_Cmd(DMA1_Channel7, ENABLE);		//允许DMA通道7
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);	 //传输完成中断

	/*
	//USART3_TX use DMA1_Channel2
	DMA_DeInit(DMA1_Channel2); 	//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Adress;	//以定义DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART3_Send_Buf ; //定义DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	//外设作为数据传输的目的
	DMA_InitStructure.DMA_BufferSize = USART3_BUF;	//定义指定DMA通道的DMA缓存的大小，单位为数据单位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		//工作在 模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;		//DMA通道1拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel2, ENABLE);		//允许DMA通道2
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);	 //传输完成中断
	*/
	

#if defined (CONFIG_DEBUG)
	printf("Initializing DMA ... Done.\r\n");
	DebugPrintf();
#endif
	
}
