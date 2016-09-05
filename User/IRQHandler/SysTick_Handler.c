#include "Configuration.h"


/**************************************************************************************
函数名：SysTick_Handler
描述　：处理SysTick中断
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void SysTick_Handler(void)
{
	if(TimingDelay != 0)	//精确延时
		TimingDelay --;
	
	Time++;
	if(Time >= 1000)
	{
	 	Time = 0;
		sec ++;
	}

	if( Time % controller_cycle == 7 )
		is_control_motor = TRUE;
	

	if(Time % 100 == 33)
		IsU1Send = TRUE;


	if(Time % 100 == 43)
		IsU2Send = TRUE;

	

	//系统运行LED闪烁
	if(Time == 500)
	{
		LED1_ON;
		LED2_ON;
	}
	else if(Time == 800)
	{
		LED1_OFF;
		LED2_OFF;
	}

}
