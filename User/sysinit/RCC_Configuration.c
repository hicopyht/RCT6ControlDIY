#include "Configuration.h"

/**************************************************************************************
函数名：RCC_Configuration
描述　：配置不同的系统时钟
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void RCC_Configuration(void)
{
 	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();					//将外设RCC寄存器重设为默认值

	RCC_HSEConfig(RCC_HSE_ON);		//开启HSE（外部高速晶振）
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)	//HSE是否成功启动
	{
		//设置AHB时钟（HCLK）　AHB时钟＝系统时钟  72M
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		//设置高速AHB时钟（PCLK2）　APB2时钟＝HCLK　72M
		RCC_PCLK2Config(RCC_HCLK_Div1);

		//设置低速AHB时钟（PCLK1）	APB1时钟＝HCLK/2	36M
		RCC_PCLK1Config(RCC_HCLK_Div2);

		 //设置ADC时钟频率 ADC时钟 = PCLK2 / 8	9M
        	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

		//设置FLASH存储器延时时钟周期数	2延时周期
		FLASH_SetLatency(FLASH_Latency_2);

		//预取指缓存使能
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		//设置PLL时钟源及倍频系数 PLLCLK = 8MHz * 9 = 72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		//使能PLL
		RCC_PLLCmd(ENABLE);

		//等待PLL有效
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		//以PLL作为系统时钟源
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		//等待PLL成为系统时钟源有效
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}


	//使能APB2外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
							RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
							RCC_APB2Periph_AFIO, ENABLE);

	//PWR_BackupAccessCmd(ENABLE);	//允许修改RTC 和后备寄存器
	//RCC_LSEConfig(RCC_LSE_OFF);		//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。
	//BKP_TamperPinCmd(DISABLE);	//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用

}
