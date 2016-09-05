#include "Configuration.h"

/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA Stream 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{	
	static u32 ADC1_2_ConvertedResultTemp[ADC1_2_CHANNEL_NUM] = {0};
	u32 i;
	
	if(DMA_GetITStatus(DMA1_IT_TC1))
	{
		for(i = 0; i< (ADC1_2_CHANNEL_NUM/2); i ++)
		{
			*(ADC1_2_ConvertedResultTemp + i) = *(ADC1_2_ConvertedValue + i) & 0x0FFF;
			*(ADC1_2_ConvertedResultTemp + i + (ADC1_2_CHANNEL_NUM/2)) = (*(ADC1_2_ConvertedValue + i) >> 16) & 0x0FFF;
		}

		*(ADC_ConvertedValue + ADC_CHANNEL_TEMP) = (*(ADC1_2_ConvertedResultTemp + 0) + 
			*(ADC1_2_ConvertedResultTemp + 1) + *(ADC1_2_ConvertedResultTemp + 2)) / ADC_CHANNEL_TEMP_FILTER_NUM;

		*(ADC_ConvertedValue + ADC_CHANNEL_1V2) = (*(ADC1_2_ConvertedResultTemp + 3) + 
			*(ADC1_2_ConvertedResultTemp + 4) + *(ADC1_2_ConvertedResultTemp + 5)) / ADC_CHANNEL_1V2_FILTER_NUM;

		*(ADC_ConvertedValue + ADC_CHANNEL_10) = (*(ADC1_2_ConvertedResultTemp + 6) + 
			*(ADC1_2_ConvertedResultTemp + 7) + *(ADC1_2_ConvertedResultTemp + 8)) / ADC_CHANNEL_10_FILTER_NUM;

		*(ADC_ConvertedValue + ADC_CHANNEL_11) = (*(ADC1_2_ConvertedResultTemp + 9) + 
			*(ADC1_2_ConvertedResultTemp + 10) + *(ADC1_2_ConvertedResultTemp + 11)) / ADC_CHANNEL_11_FILTER_NUM;

		ADC1_2_OK = TRUE;
		
		DMA_ClearITPendingBit(DMA1_IT_TC1);//清中断	

		DMA1_Channel1->CCR &= 0xfffffffe;  //关闭DMA
		DMA1_Channel1->CMAR	= (u32)ADC1_2_ConvertedValue;  //重置传输地址
		DMA1_Channel1->CNDTR = ADC1_2_CHANNEL_NUM / 2;		   //设置数据传输量
		DMA1_Channel1->CCR |= 0x01;  //开启DMA
	}
}


/*******************************************************************************
函数名：DMA1_Channel2_IRQHandler
描述　：处理DMA中断的服务子程序, USART3_TX
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
	IsU3TxEmpty = 1;
	if(U3SendBufPo != U3SendBufPi)
	{
		U3SendCn = 0;	//串口发送计数器清零
		while(U3SendBufPo != U3SendBufPi)
		{
			*(USART3_Send_Buf + U3SendCn) = *(U3SendBuf + U3SendBufPo);		// 发送数据
			U3SendBufPo = (U3SendBufPo+1)%USART3_BUF;	
			U3SendCn ++;
			if(U3SendCn>=USART3_BUF)
			{
				break;
			}
		}	        
		DMA1_Channel2->CCR &= 0xfffffffe;  //关闭DMA
		DMA1_Channel2->CMAR	= (u32)USART3_Send_Buf;  //重置传输地址
		DMA1_Channel2->CNDTR = U3SendCn;		   //设置数据传输量
		DMA1_Channel2->CCR |= 0x01;  //开启DMA

		IsU3TxEmpty = 0;
	}
	DMA_ClearITPendingBit(DMA1_IT_TC2);//清中断
}

/*******************************************************************************
函数名：DMA1_Channel4_IRQHandler
描述　：处理DMA中断的服务子程序, USART1_TX
*******************************************************************************/

void DMA1_Channel4_IRQHandler(void)  
{	
	IsU1TxEmpty = 1;
	if(U1SendBufPo != U1SendBufPi)
	{
		U1SendCn = 0;	//串口发送计数器清零
		while(U1SendBufPo != U1SendBufPi)
		{
			*(USART1_Send_Buf + U1SendCn) = *(U1SendBuf + U1SendBufPo);		// 发送数据
			U1SendBufPo = (U1SendBufPo+1) % USART1_BUF;	
			U1SendCn ++;
			if(U1SendCn >= USART1_BUF)
			{
				break;
			}
		}	        
		DMA1_Channel4->CCR &= 0xfffffffe;  //关闭DMA
		DMA1_Channel4->CMAR	= (u32)USART1_Send_Buf;  //重置传输地址
		DMA1_Channel4->CNDTR = U1SendCn;		   //设置数据传输量
		DMA1_Channel4->CCR |= 0x01;  //开启DMA

		IsU1TxEmpty = 0;
	}
	DMA_ClearITPendingBit(DMA1_IT_TC4);//清中断
}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request. USART2_TX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	IsU2TxEmpty = 1;
	if(U2SendBufPo != U2SendBufPi)
	{
		U2SendCn = 0;	//串口发送计数器清零
		while(U2SendBufPo != U2SendBufPi)
		{
			*(USART2_Send_Buf + U2SendCn) = *(U2SendBuf + U2SendBufPo);		// 发送数据
			U2SendBufPo = (U2SendBufPo+1)%USART2_BUF;	
			U2SendCn ++;
			if(U2SendCn>=USART2_BUF)
			{
				break;
			}
		}	        
		DMA1_Channel7->CCR &= 0xfffffffe;  //关闭DMA
		DMA1_Channel7->CMAR	= (u32)USART2_Send_Buf;  //重置传输地址
		DMA1_Channel7->CNDTR = U2SendCn;		   //设置数据传输量
		DMA1_Channel7->CCR |= 0x01;  //开启DMA

		IsU2TxEmpty = 0;
	}
	DMA_ClearITPendingBit(DMA1_IT_TC7);//清中断
}


