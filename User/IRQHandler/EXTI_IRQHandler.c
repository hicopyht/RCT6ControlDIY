#include "Configuration.h"

/**************************************************************************************
函数名：EXTI0_IRQHandler
描述　：处理EXTI0中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI0_IRQHandler(void)
{
	// Read encoder and gyroscope measurement
	/*
	encoder_stamp = getTimeStamp();
	encoder_left_value = readEncoderL();
	encoder_right_value = readEncoderR();
	Read_ITG3205_Z();
	gyro_update = TRUE;
	*/
	//清除EXTI Line0挂起位
	EXTI_ClearITPendingBit(EXTI_Line0);

}


/**************************************************************************************
函数名：EXTI1_IRQHandler
描述　：处理EXTI1中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI1_IRQHandler(void)
{
	printf("ET1_IRQ...\r\n");
	DebugPrintfDMA();
	//清除EXTI Line1挂起位
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/**************************************************************************************
函数名：EXTI2_IRQHandler
描述　：处理EXTI2中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI2_IRQHandler(void)
{
	
	printf("ET2_IRQ...\r\n");
	DebugPrintfDMA();
	//清除EXTI Line2挂起位
	EXTI_ClearITPendingBit(EXTI_Line2);
}

/**************************************************************************************
函数名：EXTI3_IRQHandler
描述　：处理EXTI3中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI3_IRQHandler(void)
{
	printf("ET3_IRQ...\r\n");
	DebugPrintfDMA();
	//清除EXTI Line3挂起位
	EXTI_ClearITPendingBit(EXTI_Line3);
}

/**************************************************************************************
函数名：EXTI4_IRQHandler
描述　：处理EXTI4中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI4_IRQHandler(void)
{
	printf("ET4_IRQ...\r\n");
	DebugPrintfDMA();
	//清除EXTI Line4挂起位
	EXTI_ClearITPendingBit(EXTI_Line4);
}

/**************************************************************************************
函数名：EXTI15_10_IRQHandler
描述　：处理EXTI0中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	//U8 tmpU8 = 1;

	/*
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		printf("ET10_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
	else if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		printf("ET11_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	else if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		printf("ET12_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	else if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		printf("ET13_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		printf("ET14_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	else if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		printf("ET15_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line15);
	} 
	*/

	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		printf("ET13_IRQ...\r\n");
		paramsReset(1);
		EXTI_ClearITPendingBit(EXTI_Line13);
	}

	DebugPrintfDMA();

}

/**************************************************************************************
函数名：EXTI9_5_IRQHandler
描述　：处理EXTI0中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		printf("ET5_IRQ...\r\n");
	 	EXTI_ClearITPendingBit(EXTI_Line5);
	}
	else if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		printf("ET6_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	else if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		printf("ET7_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		printf("ET8_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	else if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		printf("ET9_IRQ...\r\n");
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
	DebugPrintfDMA();
}


