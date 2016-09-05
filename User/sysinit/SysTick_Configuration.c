#include "Configuration.h"

/**************************************************************************************
��������SysTick_Configuration
������������ϵͳʱ�ӣ�ÿ1ms����һ���ж�	(ϵͳ�δ�ʱ����
���롡����
���������
���ء�����
**************************************************************************************/
void  SysTick_Configuration(void)
{
	 //SysTick��9MHz������9000��ʱ��1ms
	 //SysTick��9MHz������45000��ʱ��5ms
	// Setup SysTick Timer for 1 msec interrupts  SystemFrequency = 72M
	if (SysTick_Config(SystemFrequency / 8000))	//9000
	{ 
		/* Capture error */ 
		while (1);
	}
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //��HCLK/8��ΪSysTickʱ��Դ9MHz
	NVIC_SetPriority(SysTick_IRQn, 0x0C);	//Set SysTick Preemption Priority to 3
//	SysTick->CTRL |= CTRL_TICKINT_Set;
	SysTick->VAL = SysTick_Counter_Clear;		 // ���SysTick������
	SysTick->CTRL |= SysTick_Counter_Enable;	 // ʹ��SysTick������
//	SysTick->CTRL &= SysTick_Counter_Disable;	 // �ر�SysTick������

#ifdef CONFIG_DEBUG
	printf("Initializing Systick ... Done.\r\n");
	DebugPrintf();
#endif

}
