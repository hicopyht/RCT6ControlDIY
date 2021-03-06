#include "Configuration.h"

//芯片内部温度
u16 InternalTemp;
//电源电压
u16 Power;
float battery_voltage = 0;

/*计时变量*/
u32 Time = 0;
u32 sec = 0;
/*精确延时函数的辅助变量*/
u32 TimingDelay = 0;  			//延时时间

u32 ADC1_2_OK = FALSE;
u32 ADC1_2_ConvertedValue[ADC1_2_CHANNEL_NUM/2];	//ADC1、2转换结果DMA临时缓存
u32 ADC_ConvertedValue[ADC_CHANNEL_NUM];	//ADC转换最终结果
/***************************************************************
***************************************************************/
//define of PWM input
U16 TIM8_RT[4] = {0,0,0,0};		//记录TIM8输入PWM上升沿时间
U16 TIM8_FT[4] = {0,0,0,0};		//记录TIM8输入PWM下降沿时间
U8  TIM8_flag[4] = {0,0,0,0};		//标志此刻输入PWM是高电平还是低电平	
//RC输入鉴宽
U16 inPWM1 = 1500;		//inPWM1  脉宽数据
U16 inPWM2 = 1500;		//inPWM2  脉宽数据
U16 inPWM3 = 1500;		//inPWM3  脉宽数据
U16 inPWM4 = 1500;		//inPWM4  脉宽数据

/***************************************************************
**串口相关变量
***************************************************************/
/***********************Para of stdio*****************************/
U8 PrintfBuf[PrintfBuf_Len] = {0};
U16 PrintfCn = 0;

/***********************Para of USART1**************************/
U8 IsU1Send = FALSE;	//串口1发送控制

U8 USART1_Send_Buf[USART1_BUF] = {0};	//USART1发送缓冲区(DMA)
U16 U1SendCn = 0;						//串口1发送计数器
U8 IsU1TxEmpty = 1;						//标志USART1发送缓冲区是否为空

U8  tU1SendBuf[U1_FRAME_LEN]={0};	     	//USART1发送临时缓冲区

U8  U1SendBuf[USART1_BUF]={0};	     		//USART1发送缓冲区
U16  U1SendBufPi=0;	                                 	//USART1发送缓冲区输出指针	
U16  U1SendBufPo=0;	                          		//USART1发送缓冲区输出指针

U8  U1FrameBuf[U1_FRAMEBUF_LEN][U1_FRAME_LEN]={0};   //桢发送缓冲区
U8  U1FrameBufPi = 0;					//桢发送缓冲区输入指针
U8  U1FrameBufPo = 0;					//桢发送缓冲区输出指�

U8  U1BufP = 0;                                		//USART1接收缓冲区指针
U8  U1Buf[U1_FRAME_LEN]={0};		 		//USART1接收缓冲区		 

U8  IsU1RecOK = FALSE;
U8  U1RecBufP = 0;				  		//USART1接收临时缓冲区指针
U8  U1RecBuf[U1_FRAME_LEN]={0};	  		//USART1接收临时缓冲区

/*************/
U8 BaseBuf[BASE_BUF_LEN] = {0};
U16 BaseBufPi = 0;
U16 BaseBufPo = 0;

U8 BaseCmdBufP = 0;
U8 BaseCmdBuf[32] = {0};

U8 IsBaseCmdOK = FALSE;
U8 BaseRecBufP = 0;
U8 BaseRecBuf[32] = {0};


/***********************Para of USART2**************************/
U8 IsU2Send = FALSE;	//串口2发送控制

U8 USART2_Send_Buf[USART2_BUF] = {0};	//USART2发送缓冲区(DMA)
U16 U2SendCn = 0;						//串口2发送计数器
U8 IsU2TxEmpty = 1;						//标志USART2发送缓冲区是否为空

U8  tU2SendBuf[U2_FRAME_LEN]={0};	     	//USART2发送临时缓冲区

U8  U2SendBuf[USART2_BUF]={0};	     		//USART2发送缓冲区
U16  U2SendBufPi=0;	                                	//USART2发送缓冲区输出指针	
U16  U2SendBufPo=0;	                          		//USART2发送缓冲区输出指针

U8  U2FrameBuf[U2_FRAMEBUF_LEN][U2_FRAME_LEN]={0};   //桢发送缓冲区
U8  U2FrameBufPi = 0;					//桢发送缓冲区输入指针
U8  U2FrameBufPo = 0;					//桢发送缓冲区输出指针

U8  U2BufP = 0;                                		//USART2接收缓冲区指针
U8  U2Buf[U2_FRAME_LEN]={0};		 		//USART2接收缓冲区		 

U8  IsU2RecOK = FALSE;
U8  U2RecBufP = 0;				  		//USART2接收临时缓冲区指针
U8  U2RecBuf[U2_FRAME_LEN]={0};	  		//USART2接收临时缓冲区

/***********************Para of USART3**************************/
U8 IsU3Send = FALSE;	//串口3发送控制

U8 USART3_Send_Buf[USART3_BUF] = {0};	//USART3发送缓冲区(DMA)
U16 U3SendCn = 0;						//串口3发送计数器
U8 IsU3TxEmpty = 1;						//标志USART3发送缓冲区是否为空

U8  tU3SendBuf[U3_FRAME_LEN]={0};	     	//USART3发送临时缓冲区

U8  U3SendBuf[USART3_BUF]={0};	     		//USART3发送缓冲区
U16  U3SendBufPi=0;	                                 	//USART3发送缓冲区输出指针	
U16  U3SendBufPo=0;	                          		//USART3发送缓冲区输出指针

U8  U3FrameBuf[U3_FRAMEBUF_LEN][U3_FRAME_LEN]={0};   //桢发送缓冲区
U8  U3FrameBufPi = 0;					//桢发送缓冲区输入指针
U8  U3FrameBufPo = 0;					//桢发送缓冲区输出指针

U8  U3BufP = 0;                                		//USART3接收缓冲区指针
U8  U3Buf[U3_FRAME_LEN]={0};		 		//USART3接收缓冲区		 

U8  IsU3RecOK = FALSE;
U8  U3RecBufP = 0;				  		//USART3接收临时缓冲区指针
U8  U3RecBuf[U3_FRAME_LEN]={0};	  		//USART3接收临时缓冲区


