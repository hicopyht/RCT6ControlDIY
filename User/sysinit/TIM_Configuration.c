#include "Configuration.h"

/***************Configuration of TIM1****************************************************/
//TIM1_CC3ÊÂ¼þÓÃÓÚ´¥·¢ADC1_2µÄ¹æÔòÊäÈëÍ¨µÀ×ª»»
/**************************************************************************************
º¯ÊýÃû£ºTIM1_Configuration
ÃèÊö¡¡£ºÅäÖÃ¶¨Ê±Æ÷
ÊäÈë¡¡£ºÎÞ
Êä³ö¡¡£ºÎÞ
·µ»Ø¡¡£ºÎÞ
**************************************************************************************/
void TIM1_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//Æô¶¯TIM1Ê±ÖÓ
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,  ENABLE);	//TIM1¸´ÓÃ¹¦ÄÜÍêÈ«Ó³Éä
	
	TIM_DeInit(TIM1);			//½«ÍâÉèTIM8¼Ä´æÆ÷ÖØÉèÎªÈ±Ê¡Öµ

	//TIM1»ù±¾¼ÆÊýÆ÷ÉèÖÃ£¨ÉèÖÃPWMÆµÂÊ£©ÖÜÆÚ20ms	 
	// TIM1µÄÊ±ÖÓÔ´Îª72M	  ÆäÓÉAHBÊ±ÖÓ2·ÖÆµÔÙ2±¶Æµ£¨¶¨Ê±Æ÷±¶ÆµÆ÷£©µÃµ½
	//¼ÆÊýÆ÷Ê±ÖÓÆµÂÊ£ºCK_CNT = f_CK_PSC/(Prescaler + 1);
	//PWMÆµÂÊ:f=TIMCLK/(TIM_Period + 1)
	TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;	   // 2khz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseStructure.TIM_Period = 400-1;		 // 5hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		  
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;	 //´Ë²ÎÊý½ö¶ÔTIM1ºÍTIM8ÓÐÓÃ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	//ÉèÖÃÍ¨µÀPWMÄ£Ê½¼°Õ¼¿Õ±È
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				 //CCMR1 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		 //OC1ÐÅºÅÊä³öµ½¶ÔÓ¦µÄÊä³öÒý½Å CCER
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;	 //£¿£¿£¿±£ÁôÎ»CCER
	TIM_OCInitStructure.TIM_Pulse = 50 ;								   
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	   //OC1µÍµçÆ½ÓÐÐ§¡¡CCER
	//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;	   //£¿£¿£¿±£ÁôÎ»CCER
	//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		//???±£ÁôÎ»CR2	 
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	//CR2

//	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
//	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);

	TIM_OCInitStructure.TIM_Pulse = 200;
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);
	

	TIM_Cmd(TIM1, ENABLE);                         //Æô¶¯TIM1
	/* TIM1 main Output Enable */
  	TIM_CtrlPWMOutputs(TIM1, ENABLE); 

#ifdef CONFIG_DEBUG
	printf("Initializing TIM1 ... Done\n");
	DebugPrintf();
#endif

}


// TIM2 CH1, CH2 as encoder input A and B.
void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	

 	//Enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Remap pin
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	// PA15 CH1, PB3 CH2


	// PA15 to TIM1 CH1
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	// PB3 to TIM2 CH2
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB , &GPIO_InitStructure);

	TIM_DeInit(TIM2);


	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	TIM_TimeBaseStructure.TIM_Period = 65535;	// Reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// Clock divided by 1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// Encoder mode 3

	// Config capture channels
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 10;	//0x4;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);


	// Clear update flag
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	// Enable interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	

	
	//Reset counter
	TIM_SetCounter(TIM2,0);
	// Enable TIM2
	TIM_Cmd(TIM2, ENABLE); 


#ifdef CONFIG_DEBUG
	printf("Initializing TIM2 ... Done\r\n");
	DebugPrintf();
#endif
	
}	



void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

 	// Enable TIM3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);


	/*
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	*/

	// PB0, PB1 as PWM output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB , &GPIO_InitStructure);

	TIM_DeInit(TIM3);

	//TIM3 clock source 72MHz
	// CK_CNT = f_CK_PSC/(Prescaler + 1)
	// feq = TIMCLK/(Period + 1)
	TIM_TimeBaseStructure.TIM_Prescaler = 24 -1;	// 6MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// Count up
	TIM_TimeBaseStructure.TIM_Period = 300 - 1;	// 20kHZ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		  
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;	// Only valid for TIM1 and TIM8
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//ÉèÖÃÍ¨µÀPWMÄ£Ê½¼°Õ¼¿Õ±È
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0 ;								   
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; 
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	

	/*
	TIM_OCInitStructure.TIM_Pulse = 0 ; 
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0 ; 
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	*/

	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	
	//ÔÊÐíTIM3
	TIM_Cmd(TIM3, ENABLE);
	//Ö÷Êä³öÔÊÐí
	TIM_CtrlPWMOutputs(TIM3, ENABLE);	

#ifdef CONFIG_DEBUG
	printf("Initializing TIM3 ... Done.\r\n");
	DebugPrintf();
#endif
	
}	



// TIM4 CH1, CH2 as encoder input A and B.
void TIM4_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	

 	//Enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);


	// PB6 PB7 to TIM1 CH1 CH2
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB , &GPIO_InitStructure);
	

	TIM_DeInit(TIM4);


	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	TIM_TimeBaseStructure.TIM_Period = 65535;	// Reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// Clock divided by 1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// Encoder mode 3

	// Config capture channels
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 10;	//0x4;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);


	// Clear update flag
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);	
	// Enable interrupt
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	

	
	//Reset counter
	TIM_SetCounter(TIM4, 0);
	// Enable TIM2
	TIM_Cmd(TIM4, ENABLE); 


#ifdef CONFIG_DEBUG
	printf("Initializing TIM4 ... Done.\r\n");
	DebugPrintf();
#endif
	
}	

// TIM5 CH1, CH2 as PWM output.
void TIM5_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

 	//Ê¹ÄÜAPB1ÍâÉèÊ±ÖÓ
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);


	// PA0 to TIM5 CH1
	// PA1 to TIM5 CH2
	/*
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	*/

	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	TIM_DeInit(TIM5);

	//TIM4 clock source 72MHz
	// CK_CNT = f_CK_PSC/(Prescaler + 1)
	// feq = TIMCLK/(Period + 1)
	TIM_TimeBaseStructure.TIM_Prescaler = 12 -1;	// 6MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 300-1;	// 20kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		  
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;	// only for TIM1 and TIM8
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// Set channel
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			//CCMR1 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;								   
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 0 ; 
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);

	
	// Enable TIM5
	TIM_Cmd(TIM5, ENABLE);
	// Enable PWM output
	TIM_CtrlPWMOutputs(TIM5, ENABLE);	

#ifdef CONFIG_DEBUG
	printf("Initializing TIM5 ... Done\n");
	DebugPrintf();
#endif
	
}	

/***************Configuration of TIM8****************************************************/
//ÉèÖÃ¶¨Ê±Æ÷8ÎªËÄÍ¨µÀPWMÊäÈë
void TIM8_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

 	//Ê¹ÄÜAPB2ÍâÉèÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	/* PC6 7 8 9ÉèÖÃÎª¹¦ÄÜ½Å*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_DeInit(TIM8);			//½«ÍâÉèTIM8¼Ä´æÆ÷ÖØÉèÎªÈ±Ê¡Öµ

	//TIM8»ù±¾¼ÆÊýÆ÷ÉèÖÃ£¨ÉèÖÃPWMÆµÂÊ£©ÖÜÆÚ20ms	 
	// TIM8µÄÊ±ÖÓÔ´Îª72M	  ÆäÓÉAHBÊ±ÖÓ2·ÖÆµÔÙ2±¶Æµ£¨¶¨Ê±Æ÷±¶ÆµÆ÷£©µÃµ½
	//¼ÆÊýÆ÷Ê±ÖÓÆµÂÊ£ºCK_CNT = f_CK_PSC/(Prescaler + 1)=1M;
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	   //ÉèÖÃÁËÓÃÀ´×÷ÎªTIMÊ±ÖÓÆµÂÊ³ýÊýµÄÔ¤·ÖÆµÖµ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseStructure.TIM_Period = 65535; 	//5m, 14.7ms, 29412; // 10m, 29.4ms //65535;		 	//ÉèÖÃÁËÔÚÏÂÒ»¸ö¸üÐÂÊÂ¼þ×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ,¼´¿É²âÖÜÆÚ0£­65535
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		  
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;	 //´Ë²ÎÊý½ö¶ÔTIM1ºÍTIM8ÓÐÓÃ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	//ÉèÖÃÍ¨µÀ1Îª£ºÊäÈë²¶»ñÄ£Ê½
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//Í¨µÀÑ¡Ôñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//CCMR->CCxS¹Ü½ÅÓë¼Ä´æÆ÷¶ÔÓ¦¹ØÏµ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//·ÖÆµÆ÷
	TIM_ICInitStructure.TIM_ICFilter = 0x4;        //ÂË²¨ÉèÖÃ£¬¾­Àú¼¸¸öÖÜÆÚÌø±äÈÏ¶¨²¨ÐÎÎÈ¶¨0x0¡«0xF
	TIM_ICInit(TIM8, &TIM_ICInitStructure);        //³õÊ¼»¯
	
 	//ÉèÖÃÍ¨µÀ2Îª£ºÊäÈë²¶»ñÄ£Ê½
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//Í¨µÀÑ¡Ôñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//CCMR->CCxS¹Ü½ÅÓë¼Ä´æÆ÷¶ÔÓ¦¹ØÏµ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//·ÖÆµÆ÷
	TIM_ICInitStructure.TIM_ICFilter = 0x4;        //ÂË²¨ÉèÖÃ£¬¾­Àú¼¸¸öÖÜÆÚÌø±äÈÏ¶¨²¨ÐÎÎÈ¶¨0x0¡«0xF
	TIM_ICInit(TIM8, &TIM_ICInitStructure);        //³õÊ¼»¯

 	//ÉèÖÃÍ¨µÀ3Îª£ºÊäÈë²¶»ñÄ£Ê½
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;//Í¨µÀÑ¡Ô
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//CCMR->CCxS¹Ü½ÅÓë¼Ä´æÆ÷¶ÔÓ¦¹ØÏµ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//·ÖÆµÆ÷
	TIM_ICInitStructure.TIM_ICFilter = 0x4;        //ÂË²¨ÉèÖÃ£¬¾­Àú¼¸¸öÖÜÆÚÌø±äÈÏ¶¨²¨ÐÎÎÈ¶¨0x0¡«0xFñ
	TIM_ICInit(TIM8, &TIM_ICInitStructure);        //³õÊ¼»¯

 	//ÉèÖÃÍ¨µÀ4Îª£ºÊäÈë²¶»ñÄ£Ê½
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;//Í¨µÀÑ¡Ôñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//CCMR->CCxS¹Ü½ÅÓë¼Ä´æÆ÷¶ÔÓ¦¹ØÏµ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//·ÖÆµÆ÷
	TIM_ICInitStructure.TIM_ICFilter = 0x4;        //ÂË²¨ÉèÖÃ£¬¾­Àú¼¸¸öÖÜÆÚÌø±äÈÏ¶¨²¨ÐÎÎÈ¶¨0x0¡«0xF
	TIM_ICInit(TIM8, &TIM_ICInitStructure);        //³õÊ¼»¯


	TIM_SelectInputTrigger(TIM8, TIM_TS_TI2FP2);   //SMCR->TS¡¡101Ñ¡ÔñÊ±ÖÓ´¥·¢Ô´£ºÂË²¨ºóµÄ¶¨Ê±Æ÷ÊäÈë1
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);//SMCR SMS 100 ´¥·¢·½Ê½  Ñ¡ÔñTIMx´ÓÄ£Ê½,Ñ¡ÖÐ´¥·¢ÐÅºÅ£¨TRGI£©µÄÉÏÉýÑØÖØ³õÊ¼»¯¼ÆÊýÆ÷²¢´¥·¢¼Ä´æÆ÷µÄ¸üÐÂ
	TIM_SelectMasterSlaveMode(TIM8, TIM_MasterSlaveMode_Enable); //Æô¶¯¶¨Ê±Æ÷µÄ±»¶¯´¥·¢
	
	TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);        //´ò¿ªÍ¨µÀ1²¶»ñÖÐ¶Ï
	TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);        //´ò¿ªÍ¨µÀ2²¶»ñÖÐ¶Ï
 	TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);        //´ò¿ªÍ¨µÀ3²¶»ñÖÐ¶Ï
	TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);        //´ò¿ªÍ¨µÀ4²¶»ñÖÐ¶Ï

	//TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 	   //´ò¿ªTIM8Òç³öÖÐ¶Ï

	//TIM_SetCounter(TIM8, 0);
	TIM_Cmd(TIM8, ENABLE);                         //Æô¶¯TIM8

#ifdef CONFIG_DEBUG
	printf("Initializing TIM8 ... Done\n");
	DebugPrintf();
#endif
	
}


