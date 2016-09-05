#include "Configuration.h"

//–Њ∆ђƒЏ≤њќ¬ґ»
u16 InternalTemp;
//µз‘іµз—є
u16 Power;
float battery_voltage = 0;

/*Љ∆ ±±дЅњ*/
u32 Time = 0;
u32 Second = 0;
bool time_locked = FALSE;
/*ЊЂ»Ј—” ±Їѓ эµƒЄ®÷ъ±дЅњ*/
u32 TimingDelay = 0;  			//—” ± ±Љд

u32 ADC1_2_OK = FALSE;
u32 ADC1_2_ConvertedValue[ADC1_2_CHANNEL_NUM/2];	//ADC1°Ґ2„™їїљбєыDMAЅў ±їЇіж
u32 ADC_ConvertedValue[ADC_CHANNEL_NUM];	//ADC„™її„о÷’љбєы
/***************************************************************
***************************************************************/
//define of PWM input
U16 TIM8_RT[4] = {0,0,0,0};		//Љ«¬ЉTIM8 д»лPWM…ѕ…э—Ў ±Љд
U16 TIM8_FT[4] = {0,0,0,0};		//Љ«¬ЉTIM8 д»лPWMѕ¬љµ—Ў ±Љд
U8  TIM8_flag[4] = {0,0,0,0};		//±к÷ЊіЋњћ д»лPWM «Єяµз∆љїє «µЌµз∆љ	
//RC д»лЉшњн
U16 inPWM1 = 1500;		//inPWM1  ¬цњн эЊЁ
U16 inPWM2 = 1500;		//inPWM2  ¬цњн эЊЁ
U16 inPWM3 = 1500;		//inPWM3  ¬цњн эЊЁ
U16 inPWM4 = 1500;		//inPWM4  ¬цњн эЊЁ

/***************************************************************
**іЃњЏѕаєЎ±дЅњ
***************************************************************/
/***********************Para of stdio*****************************/
U8 PrintfBuf[PrintfBuf_Len] = {0};
U16 PrintfCn = 0;

/***********************Para of USART1**************************/
U8 IsU1Send = FALSE;	//іЃњЏ1ЈҐЋЌњЎ÷∆

U8 USART1_Send_Buf[USART1_BUF] = {0};	//USART1ЈҐЋЌїЇ≥е«ш(DMA)
U16 U1SendCn = 0;						//іЃњЏ1ЈҐЋЌЉ∆ э∆ч
U8 IsU1TxEmpty = 1;						//±к÷ЊUSART1ЈҐЋЌїЇ≥е«ш «Јсќ™њ’

U8  tU1SendBuf[U1_FRAME_LEN]={0};	     	//USART1ЈҐЋЌЅў ±їЇ≥е«ш

U8  U1SendBuf[USART1_BUF]={0};	     		//USART1ЈҐЋЌїЇ≥е«ш
U16  U1SendBufPi=0;	                                 	//USART1ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л	
U16  U1SendBufPo=0;	                          		//USART1ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л

U8  U1FrameBuf[U1_FRAMEBUF_LEN][U1_FRAME_LEN]={0};   //иеЈҐЋЌїЇ≥е«ш
U8  U1FrameBufPi = 0;					//иеЈҐЋЌїЇ≥е«ш д»л÷Є’л
U8  U1FrameBufPo = 0;					//иеЈҐЋЌїЇ≥е«ш д≥ц÷Є’

U8  U1BufP = 0;                                		//USART1љ” ’їЇ≥е«ш÷Є’л
U8  U1Buf[U1_FRAME_LEN]={0};		 		//USART1љ” ’їЇ≥е«ш		 

U8  IsU1RecOK = FALSE;
U8  U1RecBufP = 0;				  		//USART1љ” ’Ѕў ±їЇ≥е«ш÷Є’л
U8  U1RecBuf[U1_FRAME_LEN]={0};	  		//USART1љ” ’Ѕў ±їЇ≥е«ш

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
U8 IsU2Send = FALSE;	//іЃњЏ2ЈҐЋЌњЎ÷∆

U8 USART2_Send_Buf[USART2_BUF] = {0};	//USART2ЈҐЋЌїЇ≥е«ш(DMA)
U16 U2SendCn = 0;						//іЃњЏ2ЈҐЋЌЉ∆ э∆ч
U8 IsU2TxEmpty = 1;						//±к÷ЊUSART2ЈҐЋЌїЇ≥е«ш «Јсќ™њ’

U8  tU2SendBuf[U2_FRAME_LEN]={0};	     	//USART2ЈҐЋЌЅў ±їЇ≥е«ш

U8  U2SendBuf[USART2_BUF]={0};	     		//USART2ЈҐЋЌїЇ≥е«ш
U16  U2SendBufPi=0;	                                	//USART2ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л	
U16  U2SendBufPo=0;	                          		//USART2ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л

U8  U2FrameBuf[U2_FRAMEBUF_LEN][U2_FRAME_LEN]={0};   //иеЈҐЋЌїЇ≥е«ш
U8  U2FrameBufPi = 0;					//иеЈҐЋЌїЇ≥е«ш д»л÷Є’л
U8  U2FrameBufPo = 0;					//иеЈҐЋЌїЇ≥е«ш д≥ц÷Є’л

U8  U2BufP = 0;                                		//USART2љ” ’їЇ≥е«ш÷Є’л
U8  U2Buf[U2_FRAME_LEN]={0};		 		//USART2љ” ’їЇ≥е«ш		 

U8  IsU2RecOK = FALSE;
U8  U2RecBufP = 0;				  		//USART2љ” ’Ѕў ±їЇ≥е«ш÷Є’л
U8  U2RecBuf[U2_FRAME_LEN]={0};	  		//USART2љ” ’Ѕў ±їЇ≥е«ш

/***********************Para of USART3**************************/
U8 IsU3Send = FALSE;	//іЃњЏ3ЈҐЋЌњЎ÷∆

U8 USART3_Send_Buf[USART3_BUF] = {0};	//USART3ЈҐЋЌїЇ≥е«ш(DMA)
U16 U3SendCn = 0;						//іЃњЏ3ЈҐЋЌЉ∆ э∆ч
U8 IsU3TxEmpty = 1;						//±к÷ЊUSART3ЈҐЋЌїЇ≥е«ш «Јсќ™њ’

U8  tU3SendBuf[U3_FRAME_LEN]={0};	     	//USART3ЈҐЋЌЅў ±їЇ≥е«ш

U8  U3SendBuf[USART3_BUF]={0};	     		//USART3ЈҐЋЌїЇ≥е«ш
U16  U3SendBufPi=0;	                                 	//USART3ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л	
U16  U3SendBufPo=0;	                          		//USART3ЈҐЋЌїЇ≥е«ш д≥ц÷Є’л

U8  U3FrameBuf[U3_FRAMEBUF_LEN][U3_FRAME_LEN]={0};   //иеЈҐЋЌїЇ≥е«ш
U8  U3FrameBufPi = 0;					//иеЈҐЋЌїЇ≥е«ш д»л÷Є’л
U8  U3FrameBufPo = 0;					//иеЈҐЋЌїЇ≥е«ш д≥ц÷Є’л

U8  U3BufP = 0;                                		//USART3љ” ’їЇ≥е«ш÷Є’л
U8  U3Buf[U3_FRAME_LEN]={0};		 		//USART3љ” ’їЇ≥е«ш		 

U8  IsU3RecOK = FALSE;
U8  U3RecBufP = 0;				  		//USART3љ” ’Ѕў ±їЇ≥е«ш÷Є’л
U8  U3RecBuf[U3_FRAME_LEN]={0};	  		//USART3љ” ’Ѕў ±їЇ≥е«ш


