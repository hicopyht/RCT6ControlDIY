#ifndef _TIM_Configuration_H
#define _TIM_Configuration_H

#define PWML_Output(x)	(TIM3->CCR3 = (x))	// PB4 as PWMA, max = 300 at 10kHz
#define PWMR_Output(x)	(TIM3->CCR4 = (x))	// PB5 as PWMB

#define readEncoderL()	(TIM2 ->CNT)
#define readEncoderR()	(TIM4 ->CNT)


void TIM1_Configuration(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);
void TIM4_Configuration(void);
void TIM5_Configuration(void);
void TIM8_Configuration(void);


#endif /* __TIM_Configuration_H */
