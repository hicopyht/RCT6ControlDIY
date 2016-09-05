#include "Configuration.h"

double getTimeStamp(void)
{
	u16 nsec =  getSystemTickValue() /9;
	return (sec + (double)(Time + nsec / 1000.0)  / 1000.0);
}

void delayus( vu32 count )
{
	vu32 i = 0;
	for(;count != 0;count--)
	{
		for(i = 35; i != 0; i--)
		{
		}
	}
}

/*************************************************************************************
函数名：Delay1
描述　：插入延迟时间 (不精确）
输入　：nCount:制定延迟时间长度	  0.1ms	  STM32执行一条指令大约为几十ns
输出　：无
返回　：无
*************************************************************************************/
void delayms(vu32 nCount)
{
	int i;
	for(;nCount != 0;nCount--)
	{
		for(i=0;i<1500;i++)
		{
		}
	}	
}

/*************************************************************************************
函数名：Delay2
描述　：插入延迟时间 (精确） 使用SysTick，
	（在不使用SysTick中断功能时可利用其产生精确延时）
输入　：nCount:制定延迟时间长度	  ms
输出　：无
返回　：无
注	　：注释内容为库1.0内函数，在库3.0中没有，所以直接操作寄存器
*************************************************************************************/
void delayms2(u32 nTime)
{
	//SysTick_CounterCmd(SysTick_Counter_Enable);		 // 使能SysTick计数器
//	SysTick->VAL = SysTick_Counter_Clear;		 // 清除SysTick计数器
	//SysTick->CTRL |= SysTick_Counter_Enable;	 // 使能SysTick计数器		
	TimingDelay = nTime; 				// 读取延时时间
	while(TimingDelay != 0); 			// 判断延时是否结束
	//SysTick_CounterCmd(SysTick_Counter_Disable); 	// 关闭SysTick计数器
	//SysTick->CTRL &= SysTick_Counter_Disable;		//关闭SysTick计数器
	//SysTick_CounterCmd(SysTick_Counter_Clear); 		// 清除SysTick计数器
//	SysTick->VAL = SysTick_Counter_Clear;		 // 清除SysTick计数器
}

/******************************************************************************* 
* Function Name  : GetTemp 
* Description    : 根据ADC结果计算温度 
* Input          : u16 advalue 
* Output         : 
* Return         : u16 power, 0.01V
*******************************************************************************/ 
u16 GetPower(u16 advalue) 
{ 
	//return (U16)(advalue/0.95); 
	return (U16)(advalue); 
} 

/******************************************************************************* 
* Function Name  : u16 GetTemp(u16 advalue) 
* Description    : 根据ADC结果计算供电电压 
* Input          : u16 advalue 
* Output         : 
* Return         : u16 temp 
*******************************************************************************/ 
u16 GetTemp(u16 advalue) 
{ 
	u32 Vtemp_sensor; 
	s32 Current_Temp; 	  //signed int
	
	//    ADC转换结束以后，读取ADC_DR寄存器中的结果，转换温度值计算公式如下： 
	//          V25 - VSENSE 
	//  T(℃) = ------------  + 25 
	//           Avg_Slope 
	//   V25：  温度传感器在25℃时 的输出电压，典型值1.43 V。 
	//  VSENSE：温度传感器的当前输出电压，与ADC_DR 寄存器中的结果ADC_ConvertedValue之间的转换关系为： 
	//            ADC_ConvertedValue * Vdd 
	//  VSENSE = -------------------------- 
	//            Vdd_convert_value(0xFFF) 
	//  Avg_Slope：温度传感器输出电压和温度的关联参数，典型值4.3 mV/℃。 

	Vtemp_sensor = advalue * 330 / 4095;
	Current_Temp = (s32)(143 - Vtemp_sensor)*1000/43 + 2500;

	//   Current_Temp = (s32)((1.42 - advalue*3.3/4095)*1000/4.35) + 25; 
	return (s16)Current_Temp; 
} 

/*************************************************************************************
函数名：fputc
描述　：将C库中printf函数重定向到USART
输入　：无
输出　：无
返回　：无
*************************************************************************************/

int fputc(int ch, FILE *f)
{
	//在此实现自己需要的fputc
	//用向USART发送一个字符作为fputc
	/*USART_SendData(USART1, (u8)ch);
	//等待传输结束
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
		;
	}
	return ch;
	*/

	if( PrintfCn < PrintfBuf_Len )
	{
   		PrintfBuf[PrintfCn] = ch;
		PrintfCn++;
	}
	
	return ch;
}

/*************************************************************************************
函数名：average()
描述　：去极值均值滤波
输入　：数列、序列个数、起始位置
输出　：均值
返回　：无
*************************************************************************************/
u16 average(u16 *ADdata, u8 Num, u8 ADi)
{
	u16 max=0;
	u16 min=0xffff;
	u32 Sum=0;
	for(;ADi<Num*2;ADi=ADi+2)
	{
		Sum += ADdata[ADi];
		if(ADdata[ADi] > max)
		{
			max = ADdata[ADi];
		}	
		if(ADdata[ADi] < min)
		{
			min = ADdata[ADi];
		}
	}
	return (u16)((Sum-max-min)/(Num-2));

}

/*************************************************************************************
函数名：average2()
描述　：去极值均值滤波,取中间值（3/5)求均值
输入　：数列、序列个数、起始位置
输出　：均值
返回　：无
*************************************************************************************/
u16 average2(u16 *ADdata, u8 Num, u8 ADi)
{
	u16 tempdata[50];
	u16 temp;
	u8 ti,tj;
	u32 Sum = 0;
	u8 cutnum;
	for(tj=0,ti=ADi;ti<Num*2;ti=ti+2,tj++)
	{
		tempdata[tj] = ADdata[ti];
	}
	for(ti=0;ti<Num;ti++)
	{
		for(tj=0;tj<Num-ti-1;tj++)
		{
			if(tempdata[tj] > tempdata[tj+1])
			{
				temp = tempdata[tj];
				tempdata[tj] = tempdata[tj+1];
				tempdata[tj+1] = temp;
			}
		}
	}
	if(Num > 5)
	{
		cutnum = Num/5;
	}
	for(ti=cutnum;ti<Num-cutnum;ti++)
	{
		Sum +=tempdata[ti];
	}
	return (u16)(Sum/(Num-2*cutnum));

}

float s16s8toFloat( S16 number, S8 evalue)
{
	float f = number;
	
	if( number == 0 )
		return 0;
	
	else if( evalue > 0 )
	{
		for( ; evalue > 0; evalue--)
			f = f/10.0;
	}

	return f;
}

void floatToBytes4( float *f, U8* b0, U8* b1, U8* b2, U8* b3 )
{
	FLOAT4 data;
	data.bytes[0] = *b0;
	data.bytes[1] = *b1;
	data.bytes[2] = *b2;
	data.bytes[3] = *b3;
	*f = data.f;
}

float bytes4ToFloat( U8 b0, U8 b1, U8 b2, U8 b3)
{
	FLOAT4 data;
	data.bytes[0] = b0;
	data.bytes[1] = b1;
	data.bytes[2] = b2;
	data.bytes[3] = b3;

	return data.f;
}


void U16ToChar(uint16 data, char * pk_ch)
{
/*
	uint8 i;
	for(i = 0; i < 5; i ++)
	{
		* (pk_ch + i) = data % 10 + '0';
		data = data / 10;
	}
*/
	* (pk_ch + 0) = data % 10 + '0';
	data = data / 10;
	* (pk_ch + 1) = data % 10 + '0';
	data = data / 10;
	* (pk_ch + 2) = data % 10 + '0';
	data = data / 10;
	* (pk_ch + 3) = data % 10 + '0';
	data = data / 10;
	* (pk_ch + 4) = data % 10 + '0';

}

void U8ToChar(uint8 data, char * pk_ch)
{
	* (pk_ch + 0) = data % 10 + '0';
	data = data / 10;
	* (pk_ch + 1) = data % 10 + '0';
}

