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

void getTimeStamp( struct TIME_STAMP * stamp)
{
	u16 us =  getSystemTickValue() /9;
	stamp->usec = Time * 1000 + us;
	stamp->sec = Second;
}

// cur >= pre
void deltaTimeStamp( struct TIME_STAMP* cur, struct TIME_STAMP* pre, struct TIME_STAMP* delta)
{
	delta->sec = cur->sec - pre->sec;
	if( cur->usec < pre->usec )
	{
		delta->usec = 1000000 + cur->usec - pre->usec;
		delta->sec --;
	}
	else
	{
		delta->usec = cur->usec - pre->usec;
	}
}

double deltaTimeStampDouble( struct TIME_STAMP* cur, struct TIME_STAMP* pre )
{
	U32 sec, usec;
	sec = cur->sec - pre->sec;
	if( cur->usec < pre->usec )
	{
		usec = 1000000 + cur->usec - pre->usec;
		sec --;
	}
	else
	{
		usec = cur->usec - pre->usec;
	}

	return (double) (sec + usec /1000000.0);
}

float deltaTimeStampFloat( struct TIME_STAMP* cur, struct TIME_STAMP* pre )
{
	U32 sec, usec;
	sec = cur->sec - pre->sec;
	if( cur->usec < pre->usec )
	{
		usec = 1000000 + cur->usec - pre->usec;
		sec --;
	}
	else
	{
		usec = cur->usec - pre->usec;
	}

	return (float) (sec + usec /1000000.0);
}


