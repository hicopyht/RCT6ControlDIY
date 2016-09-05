#ifndef _GPIO_Configuration_H
#define _GPIO_Configuration_H

//define of 2 System LED
#define LED1_ON			WRITE_REG(GPIOA->BRR, GPIO_Pin_1)
#define LED1_OFF		WRITE_REG(GPIOA->BSRR, GPIO_Pin_1)
#define LED1_TOGGLE		(GPIOA->ODR ^= GPIO_Pin_1)
#define LED2_ON			WRITE_REG(GPIOA->BRR, GPIO_Pin_4)
#define LED2_OFF		WRITE_REG(GPIOA->BSRR, GPIO_Pin_4)
#define LED2_TOGGLE		(GPIOA->ODR ^= GPIO_Pin_4)

/*
#define LED_ON			WRITE_REG(GPIOA->BRR, GPIO_Pin_8)
#define LED_OFF			WRITE_REG(GPIOA->BSRR, GPIO_Pin_8)
#define LED_TOGGLE		(GPIOA->ODR ^= GPIO_Pin_8)
*/

#define PD2_HIGH	WRITE_REG(GPIOD->BSRR, GPIO_Pin_2)
#define PD2_LOW		WRITE_REG(GPIOD->BRR, GPIO_Pin_2)

#define PC2_HIGH	WRITE_REG(GPIOC->BSRR, GPIO_Pin_2)
#define PC2_LOW		WRITE_REG(GPIOC->BRR, GPIO_Pin_2)
#define PC3_HIGH	WRITE_REG(GPIOC->BSRR, GPIO_Pin_3)
#define PC3_LOW		WRITE_REG(GPIOC->BRR, GPIO_Pin_3)
#define PC4_HIGH	WRITE_REG(GPIOC->BSRR, GPIO_Pin_4)
#define PC4_LOW		WRITE_REG(GPIOC->BRR, GPIO_Pin_4)
#define PC5_HIGH	WRITE_REG(GPIOC->BSRR, GPIO_Pin_5)
#define PC5_LOW		WRITE_REG(GPIOC->BRR, GPIO_Pin_5)

#define PB8_LOW		WRITE_REG(GPIOB->BRR, GPIO_Pin_8)
#define PB8_HIGH	WRITE_REG(GPIOB->BSRR, GPIO_Pin_8)
#define PB9_LOW		WRITE_REG(GPIOB->BRR, GPIO_Pin_9)
#define PB9_HIGH	WRITE_REG(GPIOB->BSRR, GPIO_Pin_9)
//
#define PB8_VALUE()	(!(!(GPIOB->IDR & GPIO_Pin_8)))
#define PB9_VALUE()	(!(!(GPIOB->IDR & GPIO_Pin_9)))


void GPIO_Configuration(void);
void setBreakerStatus( U8 index, U8 value );

#endif /* __GPIO_Configuration_H */
