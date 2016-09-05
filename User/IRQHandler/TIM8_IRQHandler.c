#include "Configuration.h"

/**************************************************************************************
函数名：TIM8_CC_IRQHandler
描述　：处理TIM2中断
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void TIM8_CC_IRQHandler(void)
{
	u16 temptime;
	/*通道1输入比较中断*/
	if(TIM_GetITStatus(TIM8, TIM_IT_CC1) == SET)	  		//
	{
		if(TIM8_flag[0] == 0)
		{
			TIM8_RT[0] = TIM_GetCapture1(TIM8);  //上升沿时刻
			TIM8->CCER |=0x0002;	//设置下次捕获下降沿
			TIM8_flag[0] = 1;		
		}
		else if(TIM8_flag[0] == 1)
		{
			TIM8_FT[0] = TIM_GetCapture1(TIM8);   //下降沿时刻
			TIM8->CCER &=0xfffd;	//设置下次捕获上升沿
			TIM8_flag[0] = 0;
	
			if(TIM8_FT[0] > TIM8_RT[0])
			{
				temptime = TIM8_FT[0] - TIM8_RT[0];	  //高电平时间
			}
			else
			{
				temptime = 65536 - TIM8_RT[0] + TIM8_FT[0];
			}
			if(temptime < 2000 && temptime > 1000)
			{
				inPWM1 = temptime;
				//TIM2_T[0] = (temptime-1500)*2 + 1500;	 //扩展2倍
				//TIM2_T[0] = temptime;
				//TIM2->CCR1 = TIM2_T[0];
			}

		}
		
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
	}

	
	/*通道2输入比较中断*/	
	else if(TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET)	  		//
	{
		if(TIM8_flag[1] == 0)
		{
			TIM8_RT[1] = TIM_GetCapture2(TIM8);  //上升沿时刻
			TIM8->CCER |=0x0020;	//设置下次捕获下降沿
			TIM8_flag[1] = 1;		
		}
		else if(TIM8_flag[1] == 1)
		{
			TIM8_FT[1] = TIM_GetCapture2(TIM8);   //下降沿时刻
			TIM8->CCER &=0xffdf;	//设置下次捕获上升沿
			TIM8_flag[1] = 0;
	
			if(TIM8_FT[1] > TIM8_RT[1])
			{
				temptime = TIM8_FT[1] - TIM8_RT[1];	  //高电平时间
			}
			else
			{
				temptime = 65536 - TIM8_RT[1] + TIM8_FT[1];
			}
			if(temptime < 2000 && temptime > 1000)
			{
				inPWM2 = temptime;
				//TIM3_T[1] = (temptime-1500)*2 + 1500;
				//TIM3_T[1] = temptime;
				//TIM2->CCR2 = TIM3_T[1];
			}

		}
		
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

	}

	/*通道3输入比较中断*/
	else if(TIM_GetITStatus(TIM8, TIM_IT_CC3) == SET)
	{
		if(TIM8_flag[2] == 0)
		{
			TIM8_RT[2] = TIM_GetCapture3(TIM8);  //上升沿时刻
			TIM8->CCER |=0x0200;	//设置下次捕获下降沿
			TIM8_flag[2] = 1;		
		}
		else if(TIM8_flag[2] == 1)
		{
			TIM8_FT[2] = TIM_GetCapture3(TIM8);   //下降沿时刻
			TIM8->CCER &=0xfdff;	//设置下次捕获上升沿
			TIM8_flag[2] = 0;
	
			if(TIM8_FT[2] > TIM8_RT[2])
			{
				temptime = TIM8_FT[2] - TIM8_RT[2];	  //高电平时间
			}
			else
			{
				temptime = 65536 - TIM8_RT[2] + TIM8_FT[2];
			}
			if(temptime < 2000 && temptime > 1000)
	 		{
				inPWM3 = temptime;
				//TIM3_T[2] = temptime;
			   	//TIM2->CCR3 = TIM3_T[2];
			}
		}
		
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);

	}

	 /*通道4输入比较中断*/

	else if(TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET)
	{
		if(TIM8_flag[3] == 0)
		{
			TIM8_RT[3] = TIM_GetCapture4(TIM8);  //上升沿时刻
			TIM8->CCER |=0x2000;	//设置下次捕获下降沿
			TIM8_flag[3] = 1;		
		}
		else if(TIM8_flag[3] == 1)
		{
			TIM8_FT[3] = TIM_GetCapture4(TIM8);   //下降沿时刻
			TIM8->CCER &=0xdfff;	//设置下次捕获上升沿
			TIM8_flag[3] = 0;
	
			if(TIM8_FT[3] > TIM8_RT[3])
			{
				temptime = TIM8_FT[3] - TIM8_RT[3];	  //高电平时间
			}
			else
			{
				temptime = 65536 - TIM8_RT[3] + TIM8_FT[3];
			}
			if(temptime < 2000 && temptime > 1000)
			{
				inPWM4 = temptime;
				//TIM3_T[3] = temptime;
				//TIM2->CCR4 = TIM3_T[3];

			}
		}
		
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);

	}

}
