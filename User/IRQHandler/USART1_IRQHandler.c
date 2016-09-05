#include "Configuration.h"


/**************************************************************************************
函数名：USART1_IRQHandler
描述　：处理USART1_IRQHandler中断的服务子程序
**************************************************************************************/
void USART1_IRQHandler(void)  
{
	U8 t = 0;    	
	static U8 FLen = 10;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		t = USART_ReceiveData(USART1);
		inBaseBuf( t );

		/*
		//从接收缓冲区中读取数据
		//USART1_SendByte(t);
		if(U1BufP < 4)
		{
			if(t == DH0 && U1BufP == H0)
			{
				U1Buf[U1BufP] = t;
				U1BufP++;
			}
			else if(t == DH1 && U1BufP == H1)
			{
				U1Buf[U1BufP] = t;
				U1BufP++;
			}
			else if(U1BufP == Dest) 
			{

				U1Buf[U1BufP] = t;
				U1BufP++;
			}
			else if( t <= 64 && U1BufP == Dnum)
			{
				FLen = t + 6;
				U1Buf[U1BufP] = t;
				U1BufP++;
			}
			else
			{
				U1BufP = 0;
			}
		}
		else if(U1BufP < FLen)		//接受数据和校验值
		{
			U1Buf[U1BufP] = t;
			U1BufP++;
			
			if(U1BufP >= FLen)
			{
			//收满一帧
				if (U1RecBufP == 0)
				{
					for (t = 0; t < U1BufP; t++)
					{
						U1RecBuf[t] = U1Buf[t];
					//	USART1_SendData(U0Buf[t]);
					}
					
					U1RecBufP = U1BufP;
					U1BufP = 0;
					IsU1RecOK = TRUE;
				//	LED2_TOGGLE;
				}
				else
				{
					//现在上一帧还没有处理完，丢弃此帧
					U1BufP = 0;
				}
			}
		}*/
		
	}
	
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
}

//UART1帧处理程序－上位机
U8 UART1_CAL(void)
{
//	U8 i;
//	U8 TempU8=0;
//	S8 TempS8 = 0;
	U16 TempU16=0;
//	S16 TempS16=0;
//	S32 TempS32=0;

	U8 DLen;//数据长度
//	U8 Chk;	//校验字位置
	U8 DestU8;//地址

	DLen = U1RecBuf[Dnum];
/*	Chk = DLen + 5;
	   
	TempU8 = 0;
	for (i = 0; i < Chk; i++)
	       TempU8 = TempU8 + U1RecBuf[i];

	if (TempU8 != U1RecBuf[Chk])
	{  			  	  
		  ////校验和错误，将帧处理缓冲区清空
		  U1RecBufP = 0;
		  return FALSE;
	}
*/

	DestU8 = U1RecBuf[Dest] & 0x0F;
	if((DestU8 == LOCAL_IP)||(DestU8 == ALL_IP))
	{
		//判断是发送到该编号控制器的数据,或者是广播数据
		DestU8 = (U1RecBuf[Dest] >> 4) & 0x0F;
		if((DestU8 == PC_IP) || (DestU8== ALL_IP))  
		{  
			switch(U1RecBuf[CMD])
			{
				case	't':	// t = 0x74
				case	'T':	// T = 0x54
						//测试命令
						if(DLen < 2)	//少于2 个字节出错
						{
							break;
						}
						//回复状态
						tU1SendBuf[Dest]  = ALL_IP << 4 | LOCAL_IP;  //源/目标地址设置
						tU1SendBuf[Dnum]  = 0x02;  				//返回数据个数
						tU1SendBuf[CMD]  = 'T';					//命令ID
						tU1SendBuf[D0] = 0x55;
						tU1SendBuf[D1] = 0xAA;
						InU1SendBuf();
						break;

				case	'a':	// a = 0x61
				case	'A':	// A = 0x41
						//测试命令
						if(DLen < 2)	//少于2 个字节出错
						{
							break;
						}
						TempU16 = (U1RecBuf[D0] << 8) + U1RecBuf[D1];

						if( TempU16 > 300 )
							TempU16 = 300;
						if( TempU16 < 30 )
							TempU16 = 0;
						PWML_Output(TempU16);
						printf("\r\nSet PWM A to %d", TempU16);
						DebugPrintfDMA();
						
						break;

				case	'b':	// b = 0x62
				case	'B':	// B = 0x42
						//测试命令
						if(DLen < 2)	//少于2 个字节出错
						{
							break;
						}
						TempU16 = (U1RecBuf[D0] << 8) + U1RecBuf[D1];

						if( TempU16 > 300 )
							TempU16 = 300;
						if( TempU16 < 30 )
							TempU16 = 0;

						PWMR_Output(TempU16);
						printf("\r\nSet PWM B to %d", TempU16);
						DebugPrintfDMA();
						
						break;


				case	'd':	// d = 0x64
				case	'D':	// D = 0x44
						//测试命令
						if(DLen < 2)	//少于2 个字节出错
						{
							break;
						}
						
						if( U1RecBuf[D0] == 0 )
						{
							MOTOR_LEFT_STOP;
							MOTOR_RIGHT_STOP;
							printf("\r\nMOTOR A&B STOP");
						}
						else if( U1RecBuf[D0] == 1 )
						{
							MOTOR_LEFT_FORWARD;
							MOTOR_RIGHT_FORWARD;
							printf("\r\nMOTOR A&B FORWARD");
						}
						else if( U1RecBuf[D0] == 2 )
						{
							MOTOR_LEFT_BACKWARD;
							MOTOR_RIGHT_BACKWARD;
							printf("\r\nMOTOR A&B BACKWARD");
						}
						DebugPrintfDMA();
						
						break;

				default:

						break;	   
			}    
			
		}
		
	}
			
	//将帧处理缓冲区清空
	U1RecBufP = 0;
	return TRUE;
}


// 2、发送到协议桢缓冲区内 FrameBuf            定时或指令触发
// tU1SendBuf --> U1SendBuf
U8 InU1FrameBuf(void)
{
	U8 i=0;
	U8 TempU8=0; 
	U8   Chk=0;

	//根据协议增加的内容
	tU1SendBuf[H0] =  DH0;
	tU1SendBuf[H1] =  DH1;


	Chk = tU1SendBuf[Dnum] + 5;

	for(i = 0; i< Chk;i ++)
		TempU8 += tU1SendBuf[i]; 

	tU1SendBuf[Chk] = TempU8;
	//桢长度 Chk+1 
	memcpy(U1FrameBuf[U1FrameBufPi], tU1SendBuf, Chk+1);  //注入 缓冲区    
	U1FrameBufPi=(U1FrameBufPi+1)%U1_FRAMEBUF_LEN;  //指针移位    

	return TRUE;
}

// 3、将协议桢缓冲区的数据发送到串口发送缓冲区  U3SendBuf    定时
// FrameBuf --> U3SendBuf
U8 OutU1FrameBuf(void)
{
    
    U8   Chk=0;
    U8   i=0;
   
	if(U1FrameBufPo != U1FrameBufPi) //协议桢缓冲区不空
	{

		Chk = U1FrameBuf[U1FrameBufPo][Dnum] + 5;
		 //桢长度 Chk+1 
		for(i=0;i<Chk+1;i++)
		{
			U1SendBuf[U1SendBufPi] = U1FrameBuf[U1FrameBufPo][i];
			U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
		}		
	       U1FrameBufPo=(U1FrameBufPo+1)%U1_FRAMEBUF_LEN;  //指针移位      

		if(IsU1TxEmpty == 1)
		{
			DMA1_Channel4_IRQHandler();	
		}
			
//		USART2->CR1 |= 0x80;  // 同时允许THRE中断,即发送空中断
	}	
	else 	//空
	{
		return FALSE;
	}
	
    return TRUE;
}


//直接送入发送缓冲，当前帧发送结束后立即发送
U8 InU1SendBuf(void)
 {
 
	U8 i=0;
	U8 TempU8=0; 
	U8   Chk=0;

	//根据协议增加的内容
	tU1SendBuf[H0] =  DH0;
	tU1SendBuf[H1] =  DH1;

	Chk = tU1SendBuf[Dnum] + 5;

	for(i=0;i<Chk;i++)
		TempU8 += tU1SendBuf[i]; 

	tU1SendBuf[Chk] = TempU8;
	//桢长度 Chk+1 
	for(i=0;i<Chk+1;i++)			
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();	
	}		
	//USART2->CR1 |= 0x80;  // 同时允许THRE中断,即发送空中断

	return TRUE;
 }



//USART1 输出printf
void U1Printf(void)
{
	U16 i=0;
	
	for(i = 0; i < PrintfCn; i++)			
	{
		U1SendBuf[U1SendBufPi] = PrintfBuf[i];
		U1SendBufPi = (U1SendBufPi +1) % USART1_BUF;  //输出指针移位 
	}	

	PrintfCn = 0;
	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();	
	}

}

void inBaseBuf( U8 byte )
{
	BaseBuf[BaseBufPi] = byte;
	BaseBufPi = (BaseBufPi + 1) % BASE_BUF_LEN;
}

U8 outBaseBuf( U8* byte )
{
	if( BaseBufPi == BaseBufPo )
		return FALSE;

	*byte = BaseBuf[BaseBufPo];
	BaseBufPo = (BaseBufPo + 1) % BASE_BUF_LEN;
	return TRUE;
}

void calBaseBuf(void)
{
	U8 byte = 0;
	
	while( outBaseBuf(&byte) )
	{
		ReceiveBaseCommand(byte);
	}
}

// 9 bytes, # E C LM LS RM RS SUM \n
void SendEncoders( U16 left, U16 right)
{
	U8 i=0;
	U8 TempU8=0; 

	//simulateEncoder( &left, &right );

	tU1SendBuf[0] =  '#';
	tU1SendBuf[1] =  'E';
	tU1SendBuf[2] = 'C';
	tU1SendBuf[3] = (left  >> 8) & 0xFF;
	tU1SendBuf[4] = left & 0xFF;
	tU1SendBuf[5] = (right  >> 8) & 0xFF;
	tU1SendBuf[6] = right & 0xFF;
	tU1SendBuf[7] = 0;
	tU1SendBuf[8] = '\n';
	

	for(i=0;i<7;i++)
		TempU8 += tU1SendBuf[i]; 
	tU1SendBuf[7] = TempU8;
	
	//桢长度 Chk+1 
	for(i=0;i<9;i++)			
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();	
	}		

}

// 11 bytes, # E C LH LL RH RL GH GL SUM \n
void SendEncodersAndGyro( U16 left, U16 right, S16 gyro )
{
	U8 i=0;
	U8 TempU8=0; 

	tU1SendBuf[0] =  '#';
	tU1SendBuf[1] =  'E';
	tU1SendBuf[2] = 'C';
	tU1SendBuf[3] = (left  >> 8) & 0xFF;
	tU1SendBuf[4] = left & 0xFF;
	tU1SendBuf[5] = (right  >> 8) & 0xFF;
	tU1SendBuf[6] = right & 0xFF;
	tU1SendBuf[7] = (gyro  >> 8) & 0xFF;
	tU1SendBuf[8] = gyro & 0xFF;
	tU1SendBuf[9] = 0;
	tU1SendBuf[10] = '\n';
	

	for(i=0;i<9;i++)
		TempU8 += tU1SendBuf[i]; 
	tU1SendBuf[9] = TempU8;
	
	//桢长度 Chk+1 
	for(i=0;i<11;i++)			
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();	
	}		

}



// 9 bytes, # B S b0 b1 b2 b3 SUM \n
void SendBatteryState( U16 voltage_adc )
{
	U8 i=0;
	U8 TempU8=0; 

	//simulateEncoder( &left, &right );

	tU1SendBuf[0] =  '#';
	tU1SendBuf[1] =  'B';
	tU1SendBuf[2] = 'S';
	tU1SendBuf[3] = (voltage_adc >> 8) & 0xFF;
	tU1SendBuf[4] = voltage_adc & 0xFF;
	tU1SendBuf[5] = (4096 >> 8) & 0xFF;
	tU1SendBuf[6] = 4096 & 0xFF;
	tU1SendBuf[7] = 0;
	tU1SendBuf[8] = '\n';

	for(i=0;i<7;i++)
		TempU8 += tU1SendBuf[i]; 
	tU1SendBuf[7] = TempU8;
	
	//桢长度 Chk+1 
	for(i=0;i<9;i++)
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();
	}		

}

void SendBreakerStatus( U8 b0, U8 b1)
{
	U8 i=0;
	U8 TempU8=0; 

	//simulateEncoder( &left, &right );

	tU1SendBuf[0] =  '#';
	tU1SendBuf[1] =  'B';
	tU1SendBuf[2] = 'K';
	tU1SendBuf[3] = b0;
	tU1SendBuf[4] = b1;
	tU1SendBuf[5] = 0;
	tU1SendBuf[6] = '\n';

	for(i=0;i<5;i++)
		TempU8 += tU1SendBuf[i]; 
	tU1SendBuf[5] = TempU8;
	
	//桢长度 Chk+1 
	for(i=0;i<7;i++)
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();
	}	
}

// 9 bytes, # E C LM LS RM RS SUM \n
void SendParam( float value )
{
	U8 i=0;
	U8 TempU8=0; 
	ParamValue param;

	param.f = value;

	tU1SendBuf[0] =  '#';
	tU1SendBuf[1] =  'G';
	tU1SendBuf[2] = 'P';
	tU1SendBuf[3] = param.bytes[0];
	tU1SendBuf[4] = param.bytes[1];
	tU1SendBuf[5] = param.bytes[2];
	tU1SendBuf[6] = param.bytes[3];
	tU1SendBuf[7] = 0;
	tU1SendBuf[8] = '\n';
	

	for(i=0;i<7;i++)
		TempU8 += tU1SendBuf[i]; 

	tU1SendBuf[7] = TempU8;
	//桢长度 Chk+1 
	for(i=0;i<9;i++)			
	{
		U1SendBuf[U1SendBufPi] = tU1SendBuf[i];
		U1SendBufPi = (U1SendBufPi +1)%USART1_BUF;  //输出指针移位 
	}	
	if(IsU1TxEmpty == 1)
	{
		DMA1_Channel4_IRQHandler();	
	}		

}

void ReceiveBaseCommand( U8 byte )
{
	U8 i = 0;

	if( BaseCmdBufP == 0  && byte == '#' )
	{
		BaseCmdBuf[0] = '#';
		BaseCmdBufP = 1;
	}
	else if( BaseCmdBufP > 0 && BaseCmdBufP < 32)
	{
		if( byte != '\n' )
		{
			BaseCmdBuf[BaseCmdBufP] = byte;
			BaseCmdBufP ++;
		}
		else
		{
			BaseCmdBuf[BaseCmdBufP] = byte;
			BaseCmdBufP ++;
			
			if( BaseRecBufP == 0 )
			{
				for( i = 0; i < BaseCmdBufP; i++)
				{
					BaseRecBuf[i] = BaseCmdBuf[i];
				}
				BaseRecBufP = BaseCmdBufP;
				IsBaseCmdOK = TRUE;
				calBaseCmd();
				IsBaseCmdOK = FALSE;
			}
		}
	}
	else
	{
		BaseCmdBufP = 0;
	}
}

U8 calBaseCmd(void)
{
	U8 tempU8;
	float tempF;

	if( BaseRecBufP == 13 && BaseRecBuf[1] == 'M' && BaseRecBuf[2] == 'S' )	// 0x4d, 0x53,  12 valid bytes, 13rh byte is '\n' 
	{
		// Check sum ?

		// Read speed
		VelocityBuff.velocity_left = bytes4ToFloat( BaseRecBuf[3], BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6] );
		VelocityBuff.velocity_right = bytes4ToFloat( BaseRecBuf[7], BaseRecBuf[8], BaseRecBuf[9], BaseRecBuf[10] );
		VelocityBuff.stamp = getTimeStamp();

		// Limit max
		if( VelocityBuff.velocity_left > VelocityBuff.velocity_max )
			VelocityBuff.velocity_left = VelocityBuff.velocity_max;
		else if( VelocityBuff.velocity_left < VelocityBuff.velocity_min )
			VelocityBuff.velocity_left = VelocityBuff.velocity_min;

		if( VelocityBuff.velocity_right > VelocityBuff.velocity_max )
			VelocityBuff.velocity_right = VelocityBuff.velocity_max;
		else if( VelocityBuff.velocity_right < VelocityBuff.velocity_min )
			VelocityBuff.velocity_right = VelocityBuff.velocity_min;

		// Deadzone
		/*
		if( VelocityBuff.velocity_left < VelocityBuff.velocity_deadzone && VelocityBuff.velocity_left > -VelocityBuff.velocity_deadzone)
			VelocityBuff.velocity_left = 0;
		if( VelocityBuff.velocity_right < VelocityBuff.velocity_deadzone && VelocityBuff.velocity_right > -VelocityBuff.velocity_deadzone)
			VelocityBuff.velocity_right = 0;
		*/

#if defined (CONFIG_DEBUG)
		// Print info
		//printf(" MS %d: %f %f rad/s\n", BaseRecBufP, VelocityBuff.velocity_left, VelocityBuff.velocity_right );
		printf(" MS = %d\r\n", Time );
		DebugPrintfDMA();
#endif
				
	}
	else if( BaseRecBufP == 7 && BaseRecBuf[1] == 'B' && BaseRecBuf[2] == 'K' )
	{
		setBreakerStatus( BaseRecBuf[3], BaseRecBuf[4] );
#if defined (CONFIG_DEBUG)
		// Print info
		printf(" BK %d set %d = %d\n", Time, BaseRecBuf[3], BaseRecBuf[4] );
		DebugPrintfDMA();
#endif
	}
	else if( BaseRecBufP == 10 && BaseRecBuf[1] == 'S' && BaseRecBuf[2] == 'P' )
	{
		tempU8 = BaseRecBuf[3];
		if( tempU8 < PARAM_NUM )
		{
			tempF= bytes4ToFloat( BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6], BaseRecBuf[7] );
			paramWriteFloat( param_vector[tempU8].name, tempF );
			paramsSave();
		}
				
#if defined (CONFIG_DEBUG)
		// Print info
		if( tempU8 < PARAM_NUM )
		{
			//printf(" SP %d, %s = %f\n", BaseRecBufP, param_vector[tempU8].name, tempF );
			//DebugPrintfDMA();
		}
#endif

	}
	else if( BaseRecBufP == 6 && BaseRecBuf[1] == 'G' && BaseRecBuf[2] == 'P' )
	{
		tempU8 = BaseRecBuf[3];
		if( tempU8 < PARAM_NUM )
		{
			tempF= param_vector[tempU8].value.f;
			SendParam( tempF );
		}
				
#if defined (CONFIG_DEBUG)
		// Print info
		if( tempU8 < PARAM_NUM )
		{
			//printf(" GP %d, %s = %f\n", BaseRecBufP, param_vector[tempU8].name, param_vector[tempU8].value.f );
			//DebugPrintfDMA();
		}
#endif

	}
	else if( BaseRecBufP == 9 && BaseRecBuf[1] == 'P' && BaseRecBuf[2] == 'P' )
	{
		PidBuff = VelocityPid[MOTOR_LEFT].pid.gains;
		PidBuff.p_gain = bytes4ToFloat( BaseRecBuf[3], BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6] );
		VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
		VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;

		paramWriteFloat( "velocity_pid_p", PidBuff.p_gain );
		paramsSave();
				
#if defined (CONFIG_DEBUG)
		// Print info
		//printf(" PP %d: p = %f\n", BaseRecBufP, PidBuff.p_gain );
		//DebugPrintfDMA();
#endif

	}
	else if( BaseRecBufP == 9 && BaseRecBuf[1] == 'P' && BaseRecBuf[2] == 'I' )
	{
		PidBuff = VelocityPid[MOTOR_LEFT].pid.gains;
		PidBuff.i_gain = bytes4ToFloat( BaseRecBuf[3], BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6] );
		VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
		VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;

		paramWriteFloat( "velocity_pid_i", PidBuff.i_gain );
		paramsSave();

#if defined (CONFIG_DEBUG)
		// Print info
		printf(" PI %d: i = %f\n", BaseRecBufP, PidBuff.i_gain );
		DebugPrintfDMA();
#endif

	}
	else if( BaseRecBufP == 9 && BaseRecBuf[1] == 'P' && BaseRecBuf[2] == 'D' )
	{
		PidBuff = VelocityPid[MOTOR_LEFT].pid.gains;
		PidBuff.d_gain = bytes4ToFloat( BaseRecBuf[3], BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6] );
		VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
		VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;

		paramWriteFloat( "velocity_pid_d", PidBuff.d_gain );
		paramsSave();
				
#if defined (CONFIG_DEBUG)
		// Print info
		printf(" PD %d: d = %f\n", BaseRecBufP, PidBuff.d_gain );
		DebugPrintfDMA();
#endif

	}
	else if( BaseRecBufP == 9 && BaseRecBuf[1] == 'P' && BaseRecBuf[2] == 'C' )
	{
		PidBuff = VelocityPid[MOTOR_LEFT].pid.gains;
		PidBuff.i_max = bytes4ToFloat( BaseRecBuf[3], BaseRecBuf[4], BaseRecBuf[5], BaseRecBuf[6] );
		PidBuff.i_min = -PidBuff.i_max;
		VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
		VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;

		paramWriteFloat( "velocity_pid_clamp", PidBuff.i_max );
		paramsSave();
				
#if defined (CONFIG_DEBUG)
		// Print info
		printf(" PC %d: i_max = %f, i_min = %f \n", BaseRecBufP, PidBuff.i_max, PidBuff.i_min );
		DebugPrintfDMA();
#endif

	}

	BaseRecBufP = 0;
	return TRUE;
}

void simulateEncoder( U16* left, U16 * right )
{
	static U16 l = 0, r = 0;
	U16 value = getSystemTickValue();

	l += value % 23;
	r += value % 17;

	*left = l;
	*right = r;
}


