#include "Configuration.h"
/*
	ADC共采集12个通道分别为:
		ADC_Channel_16	:	芯片内部温度传感器
		ADC_Channel_17	:	芯片内部1.2V基准
		ADC_Channel_10	:	
		ADC_Channel_11	:		
*/

/**************************************************************************************
函数名：ADC_Configuration
描述　：配置ADC, ADC1 及ADC2  工作在同步规则模式
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void ADC_Configuration(void)
{
	ADC_InputPin_Configuration();	//配置ADC输入引脚
	ADC1_2_Configuration();			//配置ADC1和ADC2工作在同步规则转换模式

#if defined (CONFIG_DEBUG)
	printf("Initializing ADC1_2 ... Done\n");
	DebugPrintf();
#endif

}

/**************************************************************************************
函数名：ADC_InputPin_Configuration
描述　：配置ADC输入引脚
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void ADC_InputPin_Configuration(void)
{
 	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//PC0, PC1模拟输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 |GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}

/**************************************************************************************
函数名：ADC1_2_Configuration
描述　：配置ADC, ADC2工作在同步规则模式
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void ADC1_2_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	//允许ADC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);


	//ADC1初始化
	ADC_DeInit(ADC1);									//将外设ADC1的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;	//ADC1和ADC2工作在同步规则模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC3;	//
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = ADC1_2_CHANNEL_NUM / 2;			//规定了顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);

	//ADC2初始化
	ADC_DeInit(ADC2);									//将外设ADC2的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;	//ADC1和ADC2工作在同步规则模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC3;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = ADC1_2_CHANNEL_NUM / 2;			//规定了顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC2, &ADC_InitStructure);
		
	 
	ADC_TempSensorVrefintCmd(ENABLE);	//ADC内置温度传感器使能（要使用片内温度传感器，切忌要开启它）
	
	//ADC1规则转换通道设置
    	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_71Cycles5);	//常规转换序列1：通道16（内部温度传感器）要求采样时间大于2.2us	
    	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 4, ADC_SampleTime_71Cycles5);	//
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 5, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 6, ADC_SampleTime_71Cycles5);
	//ADC2规则转换通道设置
    	ADC_RegularChannelConfig(ADC2, ADC_Channel_10,  1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10,  2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10,  3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11,  4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11,  5, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11,  6, ADC_SampleTime_71Cycles5);

	
	ADC_DMACmd(ADC1, ENABLE);	//允许ADC1进行DMA传送
//	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);	//允许ADC转换结束中断 (所有转换结束后中断)

	ADC_Cmd(ADC1, ENABLE);		//允许ADC1
	ADC_Cmd(ADC2, ENABLE);		//允许ADC2
	/* Enable ADC1 external trigger */ 
 	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	//ADC1校准
	ADC_ResetCalibration(ADC1);	//允许ADC1复位校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1)){}	  //检测ADC1校准寄存器是否复位完成
	ADC_StartCalibration(ADC1);	//启动ADC1校准
	while(ADC_GetCalibrationStatus(ADC1)){}		//检测ADC1校准是否完成
	//ADC2校准
	ADC_ResetCalibration(ADC2);	//允许ADC1复位校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC2)){}	  //检测ADC1校准寄存器是否复位完成
	ADC_StartCalibration(ADC2);	//启动ADC2校准
	while(ADC_GetCalibrationStatus(ADC2)){}		//检测ADC1校准是否完成
}


