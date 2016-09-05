#include "Configuration.h"

void InitDevice(void)
{		

	SystemInit();
	RCC_Configuration();		// Clock
	USART2_Configuration();	// UART2, If DebugPrintf() is U2Printf(), UART2 should be initilized before UART1.
	//
	paramsInit();
	//
	USART1_Configuration();	// UART1, If DebugPrintf() is U1Printf(), UART1 should be initilized before UART2.
	GPIO_Configuration(); 	// GPIO	
	DMA_Configuration();	// DMA
	EXTI_Configuration();	// External interrupt
	//
	ADC_Configuration();	// ADC
	TIM1_Configuration();	// ADC trigger
	//
	TIM2_Configuration();	// TIM2, Encoder A
	TIM4_Configuration();	// TIM4, Encoder B
	TIM3_Configuration();	// TIM3, PWM for Motors
	//
	I2C2_Init();
	Init_ITG3205();
	//
	Motor_Init();
	SysTick_Configuration();	// System tick
	NVIC_Configuration();  	// Interrupt source configuration
	
}

