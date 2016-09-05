#include "Configuration.h"

/**************************************************************************************
函数名：SysTick_Configuration
描述　：配置系统时钟，每1ms产生一次中断	(系统滴答定时器）
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void  SysTick_Configuration(void)
{
	 //SysTick以9MHz计数到9000，时间1ms
	 //SysTick以9MHz计数到45000，时间5ms
	// Setup SysTick Timer for 1 msec interrupts  SystemFrequency = 72M
	if (SysTick_Config(SystemFrequency / 8000))	//9000
	{ 
		/* Capture error */ 
		while (1);
	}
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //将HCLK/8作为SysTick时钟源9MHz
	NVIC_SetPriority(SysTick_IRQn, 0x0C);	//Set SysTick Preemption Priority to 3
//	SysTick->CTRL |= CTRL_TICKINT_Set;
	SysTick->VAL = SysTick_Counter_Clear;		 // 清除SysTick计数器
	SysTick->CTRL |= SysTick_Counter_Enable;	 // 使能SysTick计数器
//	SysTick->CTRL &= SysTick_Counter_Disable;	 // 关闭SysTick计数器

#ifdef CONFIG_DEBUG
	printf("Initializing Systick ... Done.\r\n");
	DebugPrintf();
#endif

}
