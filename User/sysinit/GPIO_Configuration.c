#include "Configuration.h"

/**************************************************************************************
函数名：GPIO_Configuration
描述　：配置GPIO端口
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//SWJ enabled, JTAG disabled
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);	//SWJ disabled
	

	//PA1,PA4 配置为输出，控制LED1, LED2
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_4);	//输出高电平，LED 不亮
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//最高输出速度10MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//开漏输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Motor A IN1, IN2
	// Motor B IN1, IN2
	// PC2, PC3, PC4, PC5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Motor standby
	// PD2
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);	// PA2 outputs low to disable TB6612
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Breaker
	// PB8, PB9
	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

#if defined (CONFIG_DEBUG)
	printf("Initializing GPIO ... Done.\r\n");
	DebugPrintf();
#endif
	
}

void setBreakerStatus( U8 index, U8 value )
{
	if( index == 0 )
	{
		if( value == 0 )
			PB8_LOW;
		else if( value == 1 )
			PB8_HIGH;
	}
	else if( index == 1 )
	{
		if( value == 0 )
			PB9_LOW;
		else if( value == 1 )
			PB9_HIGH;
	}
}


