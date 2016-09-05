#ifndef VAR_H
#define VAR_H

#define true 		TRUE
#define false 		FALSE

#define BIT(x)	(1 << (x))
#define set_bit(reg, bit)		((reg) |= (1 << (bit)))
#define clear_bit(reg, bit)		((reg) &= ~ (1 << (bit)))
#define toggle_bit(reg, bit)	((reg) ^= (1 << (bit)))
#define get_bit(reg, bit)		(((reg) >> (bit)) & 1)
#define get_twobit(reg, bit)	(((reg) >> (bit)) & 3)

#define getSystemTickValue()	(SysTick->VAL)

//芯片内部温度
extern u16 InternalTemp;
//电源电压
extern u16 Power;
extern float battery_voltage;

/*计时变量*/
extern u32 Time;
extern u32 Second;
extern bool time_locked;
/*精确延时函数的辅助变量*/
extern u32 TimingDelay;  			//延时时间

/***************************************************************
***************************************************************/
	//define of PWM input
extern U16 TIM8_RT[4];		//记录TIM8输入PWM上升沿时间
extern U16 TIM8_FT[4];		//记录TIM8输入PWM下降沿时间
extern U8  TIM8_flag[4];		//标志此刻输入PWM是高电平还是低电平	

//RC输入鉴宽
extern U16 inPWM1 ;		//inPWM1  脉宽数据
extern U16 inPWM2 ;		//inPWM2  脉宽数据
extern U16 inPWM3 ;		//inPWM3  脉宽数据
extern U16 inPWM4 ;		//inPWM4  脉宽数据
//

#endif

#ifndef    ADCVAR_H
#define    ADCVAR_H
//ADC转换通道定义
#define ADC_CHANNEL_NUM	4
#define ADC_CHANNEL_TEMP	0
#define ADC_CHANNEL_1V2	1
#define ADC_CHANNEL_10		2
#define ADC_CHANNEL_11 	3
//转化结果校准值定义

//转换结果寄存器定义
#define ADC1_2_CHANNEL_NUM 12
#define ADC_CHANNEL_TEMP_FILTER_NUM	3
#define ADC_CHANNEL_1V2_FILTER_NUM	3
#define ADC_CHANNEL_10_FILTER_NUM	3
#define ADC_CHANNEL_11_FILTER_NUM	3

extern u32 ADC1_2_OK;
extern u32 ADC1_2_ConvertedValue[ADC1_2_CHANNEL_NUM / 2];//ADC1、2转换结果DMA临时缓存
extern u32 ADC_ConvertedValue[ADC_CHANNEL_NUM];	//ADC转换最终结果

#endif //ADCVAR


/***************************************************************
**串口相关变量
***************************************************************/
#ifndef USARTVAR
#define USARTVAR

/***********************Para of stdio*****************************/
#define PrintfBuf_Len	500

extern U8 PrintfBuf[PrintfBuf_Len];
extern U16 PrintfCn;

/**********************define of devices address**************************/
#define PC_IP		0x01	//上位机地址
#define LOCAL_IP		0x02	//本机地址
#define ALL_IP		0x00	//广播地址

//串口协议
#define  DH0      	0xA5      //桢头0  数据
#define  DH1    	0x5A      //桢头0   数据

#define  H0        	0      //桢头0   位置
#define  H1        	1      //桢头1    位置

#define  Dest  	2      //目标 位置
#define  Dnum	3      //源      位置
#define  CMD    	4      //命令字 位置
#define  D0	5
#define  D1	6
#define  D2	7
#define  D3	8
#define  D4	9
#define  D5	10
#define  D6	11
#define  D7	12
#define  D8	13
#define  D9	14
#define  D10	15
#define  D11	16
#define  D12	17
#define  D13	18
#define  D14	19
#define  D15	20


#define CMD_ST	0xA0
/////////////////////////////////////////////////
#define FOMC_IP		0x01	//床椅主控制器地址
#define ST188_IP		0x03	//红外寻线处理模块

/***********************Para of USART1**************************/
#define USART1_BUF			200
#define U1_FRAME_LEN		30
#define U1_FRAMEBUF_LEN	10

extern U8 IsU1Send;						//串口1发送控制
extern U8 USART1_Send_Buf[USART1_BUF];	//USART1发送缓冲区(DMA)
extern U16 U1SendCn;						//串口1发送计数器
extern U8 IsU1TxEmpty;					//标志USART1发送缓冲区是否为空

extern U8  tU1SendBuf[U1_FRAME_LEN];	     	//USART1发送临时缓冲区

extern U8  U1SendBuf[USART1_BUF];	     	//USART1发送缓冲区
extern U16  U1SendBufPi;	                            //USART1发送缓冲区输出指针	
extern U16  U1SendBufPo;	                          	//USART1发送缓冲区输出指针

extern U8  U1FrameBuf[U1_FRAMEBUF_LEN][U1_FRAME_LEN];   //桢发送缓冲区
extern U8  U1FrameBufPi;                                       //桢发送缓冲区输入指针
extern U8  U1FrameBufPo;                                      //桢发送缓冲区输出指针

extern U8  U1BufP;                                		//USART1接收缓冲区指针
extern U8  U1Buf[U1_FRAME_LEN];		 	//USART1接收缓冲区		 

extern U8  IsU1RecOK;
extern U8  U1RecBufP;				  		//USART1接收临时缓冲区指针
extern U8  U1RecBuf[U1_FRAME_LEN];	  	//USART1接收临时缓冲区

/*************/
#define BASE_BUF_LEN	500
extern U8 BaseBuf[BASE_BUF_LEN];
extern U16 BaseBufPi;
extern U16 BaseBufPo;

extern U8 BaseCmdBufP;
extern U8 BaseCmdBuf[32];

extern U8 IsBaseCmdOK;
extern U8 BaseRecBufP;
extern U8 BaseRecBuf[32];

/***********************Para of USART2**************************/
#define USART2_BUF			500
#define U2_FRAME_LEN		30
#define U2_FRAMEBUF_LEN	5

extern U8 IsU2Send;						//串口2发送控制
extern U8 USART2_Send_Buf[USART2_BUF];	//USART2发送缓冲区(DMA)
extern U16 U2SendCn;						//串口2发送计数器
extern U8 IsU2TxEmpty;					//标志USART2发送缓冲区是否为空

extern U8  tU2SendBuf[U2_FRAME_LEN];	     	//USART2发送临时缓冲区

extern U8  U2SendBuf[USART2_BUF];	     	//USART2发送缓冲区
extern U16  U2SendBufPi;	                            //USART2发送缓冲区输出指针	
extern U16  U2SendBufPo;	                          	//USART2发送缓冲区输出指针

extern U8  U2FrameBuf[U2_FRAMEBUF_LEN][U2_FRAME_LEN];   //桢发送缓冲区
extern U8  U2FrameBufPi;                                       //桢发送缓冲区输入指针
extern U8  U2FrameBufPo;                                      //桢发送缓冲区输出指针

extern U8  U2BufP;                                		//USART2接收缓冲区指针
extern U8  U2Buf[U2_FRAME_LEN];		 	//USART2接收缓冲区		 

extern U8  IsU2RecOK;
extern U8  U2RecBufP;				  		//USART2接收临时缓冲区指针
extern U8  U2RecBuf[U2_FRAME_LEN];	  	//USART2接收临时缓冲区

/***********************Para of USART3**************************/
#define USART3_BUF			200
#define U3_FRAME_LEN		70
#define U3_FRAMEBUF_LEN	10

extern U8 IsU3Send;						//串口3发送控制
extern U8 USART3_Send_Buf[USART3_BUF];	//USART3发送缓冲区(DMA)
extern U16 U3SendCn;						//串口3发送计数器
extern U8 IsU3TxEmpty;					//标志USART3发送缓冲区是否为空

extern U8  tU3SendBuf[U3_FRAME_LEN];	     	//USART3发送临时缓冲区

extern U8  U3SendBuf[USART3_BUF];	     	//USART3发送缓冲区
extern U16  U3SendBufPi;	                            //USART3发送缓冲区输出指针	
extern U16  U3SendBufPo;	                          	//USART3发送缓冲区输出指针

extern U8  U3FrameBuf[U3_FRAMEBUF_LEN][U3_FRAME_LEN];   //桢发送缓冲区
extern U8  U3FrameBufPi;                                       //桢发送缓冲区输入指针
extern U8  U3FrameBufPo;                                      //桢发送缓冲区输出指针

extern U8  U3BufP;                                		//USART3接收缓冲区指针
extern U8  U3Buf[U3_FRAME_LEN];		 	//USART3接收缓冲区		 

extern U8  IsU3RecOK;
extern U8  U3RecBufP;				  		//USART3接收临时缓冲区指针
extern U8  U3RecBuf[U3_FRAME_LEN];	  	//USART3接收临时缓冲区


#endif//USARTVAR


