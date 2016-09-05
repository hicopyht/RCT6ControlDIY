#include "Configuration.h"
/**************************************************************************************
函数名：USART3_IRQHandler
描述　：处理USART3中断的服务子程序
输入　：无
输出　：无
返回　：无
**************************************************************************************/
void USART3_IRQHandler(void)  
{	
	U8 t = 0;    
	static U8 FLen = 10; 

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		t = USART_ReceiveData(USART3);

		if(U3BufP < 5)
		{
			if(t == DH0 && U3BufP == H0)
			{
				U3Buf[U3BufP] = t;
				U3BufP++;
			}
			else if(t == DH1 && U3BufP == H1)
			{
				U3Buf[U3BufP] = t;
				U3BufP++;
			}
			else if (U3BufP == Dest)
			{

				U3Buf[U3BufP] = t;
				U3BufP++;
			}
			else if( t <= 64 && U3BufP == Dnum)
			{
				FLen = t + 6;
				U3Buf[U3BufP] = t;
				U3BufP++;
			}
			else
			{
				U3BufP = 0;
			}
		}
		else if(U3BufP < FLen)		//接受数据和校验值
		{
			U3Buf[U3BufP] = t;
			U3BufP++;
			
			if(U3BufP >= FLen)
			{
			//收满一帧
				if (U3RecBufP == 0)
				{
					for (t = 0; t < U3BufP; t++)
					{
						U3RecBuf[t] = U3Buf[t];
					//	USART1_SendData(U2Buf[t]);
					}
					
					U3RecBufP = U3BufP;
					U3BufP = 0;
					IsU3RecOK = TRUE;
				//	LED2_TOGGLE;
				}
				else
				{
					//现在上一帧还没有处理完，丢弃此帧
					U3BufP = 0;
				}
			}
		}
	}

	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}


//UART3帧处理程序－上位机
U8 UART3_CAL(void)
{
//	U8 i;
//	U8 TempU8=0;
//	S8 TempS8 = 0;
//	U16 TempU16=0;
//	S16 TempS16=0;
//	S32 TempS32=0;

	U8 DLen;//数据长度
//	U8 Chk;	//校验字位置
	U8 DestU8;//地址

	DLen = U3RecBuf[Dnum];
/*	Chk = DLen + 5;
	   
	TempU8 = 0;
	for (i = 0; i < Chk; i++)
	       TempU8 = TempU8 + U3RecBuf[i];

	if (TempU8 != U3RecBuf[Chk])
	{  			  	  
		  ////校验和错误，将帧处理缓冲区清空
		  U3RecBufP = 0;
		  return FALSE;
	}
*/

	DestU8 = U3RecBuf[Dest] & 0x0F;
	if((DestU8 == FOMC_IP)||(DestU8 == ALL_IP))
	{
		//判断是发送到该编号控制器的数据,或者是广播数据
		DestU8 = (U3RecBuf[Dest] >> 4) & 0x0F;
		if((DestU8 == PC_IP))//||(DestU8== ALL_IP))  
		{  
			switch(U3RecBuf[CMD])
			{
				case	0x50:	
						//测试命令
						if(DLen < 2)	//少于2 个字节出错
						{
							break;
						}
						//回复状态
						tU3SendBuf[Dest]  = ALL_IP << 4 | FOMC_IP;  //源/目标地址设置
						tU3SendBuf[Dnum]  = 0x02;  				//返回数据个数
						tU3SendBuf[CMD]  = 0x50;					//命令ID
						tU3SendBuf[D0] = 0x55;
						tU3SendBuf[D1] = 0xAA;
						InU3FrameBuf();
						break;

				default:

						break;	   
			}    
			
		}
		
	}
			
	//将帧处理缓冲区清空
	U3RecBufP = 0;
	return TRUE;
}


// 2、发送到协议桢缓冲区内 FrameBuf            定时或指令触发
// tU3SendBuf --> U3SendBuf
U8 InU3FrameBuf(void)
{
	U8 i=0;
	U8 TempU8=0; 
	U8   Chk=0;

	//根据协议增加的内容
	tU3SendBuf[H0] =  DH0;
	tU3SendBuf[H1] =  DH1;


	Chk = tU3SendBuf[Dnum] + 5;

	for(i = 0; i< Chk;i ++)
		TempU8 += tU3SendBuf[i]; 

	tU3SendBuf[Chk] = TempU8;
	//桢长度 Chk+1 
	memcpy(U3FrameBuf[U3FrameBufPi], tU3SendBuf, Chk+1);  //注入 缓冲区    
	U3FrameBufPi=(U3FrameBufPi+1)%U3_FRAMEBUF_LEN;  //指针移位    

	return TRUE;
}

// 3、将协议桢缓冲区的数据发送到串口发送缓冲区  U3SendBuf    定时
// FrameBuf --> U3SendBuf
U8 OutU3FrameBuf(void)
{
    
    U8   Chk=0;
    U8   i=0;
   
	if(U3FrameBufPo != U3FrameBufPi) //协议桢缓冲区不空
	{

		Chk = U3FrameBuf[U3FrameBufPo][Dnum] + 5;
		 //桢长度 Chk+1 
		for(i=0;i<Chk+1;i++)
		{
			U3SendBuf[U3SendBufPi] = U3FrameBuf[U3FrameBufPo][i];
			U3SendBufPi = (U3SendBufPi +1)%USART3_BUF;  //输出指针移位 
		}		
	       U3FrameBufPo=(U3FrameBufPo+1)%U3_FRAMEBUF_LEN;  //指针移位      

		if(IsU3TxEmpty == 1)
		{
			DMA1_Channel2_IRQHandler();	
		}
			
//		USART3->CR1 |= 0x80;  // 同时允许THRE中断,即发送空中断
	}	
	else 	//空
	{
		return FALSE;
	}
	
    return TRUE;
}


//直接送入发送缓冲，当前帧发送结束后立即发送
U8 InU3SendBuf(void)
 {
 
	U8 i=0;
	U8 TempU8=0; 
	U8   Chk=0;

	//根据协议增加的内容
	tU3SendBuf[H0] =  DH0;
	tU3SendBuf[H1] =  DH1;

	Chk = tU3SendBuf[Dnum] + 5;

	for(i=0;i<Chk;i++)
		TempU8 += tU3SendBuf[i]; 

	tU3SendBuf[Chk] = TempU8;
	//桢长度 Chk+1 
	for(i=0;i<Chk+1;i++)			
	{
		U3SendBuf[U3SendBufPi] = tU3SendBuf[i];
		U3SendBufPi = (U3SendBufPi +1)%USART3_BUF;  //输出指针移位 
	}	
	if(IsU3TxEmpty == 1)
	{
		DMA1_Channel2_IRQHandler();	
	}		
	//USART3->CR1 |= 0x80;  // 同时允许THRE中断,即发送空中断

	return TRUE;
 }

//USART3 输出printf
void U3Printf(void)
{
	U16 i=0;
	
	for(i = 0; i < PrintfCn; i++)			
	{
		U3SendBuf[U3SendBufPi] = PrintfBuf[i];
		U3SendBufPi = (U3SendBufPi +1) % USART3_BUF;  //输出指针移位 
	}	

	PrintfCn = 0;
	
	if(IsU3TxEmpty == 1)
	{
		DMA1_Channel2_IRQHandler();	
	}

}


