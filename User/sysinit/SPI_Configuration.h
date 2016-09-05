#ifndef _SPI_Configuration_H
#define _SPI_Configuration_H

#define SPI2_CS_LOW()     	WRITE_REG(GPIOB->BRR, GPIO_Pin_12)
#define SPI2_CS_HIGH()    	WRITE_REG(GPIOB->BSRR, GPIO_Pin_12)
//
#define SPI2_Selected()		SPI2_CS_LOW()
#define SPI2_Deselected()	SPI2_CS_HIGH()
//
//#define MPU_Selected()		WRITE_REG(GPIOB->BRR, GPIO_Pin_12)	//set low
//#define MPU_Deselected()		WRITE_REG(GPIOB->BSRR, GPIO_Pin_12)

#define SPI3_CS_LOW()     	WRITE_REG(GPIOA->BRR, GPIO_Pin_15)
#define SPI3_CS_HIGH()    	WRITE_REG(GPIOA->BSRR, GPIO_Pin_15)
#define SPI3_Selected()		SPI3_CS_LOW()
#define SPI3_Deselected()	SPI3_CS_HIGH()


void SPI2_Configuration(void);
void SPI2_HighSpeed(FunctionalState newState);
uint8 SPI2_SendByte(uint8);

void SPI3_Configuration(void);
uint8 SPI3_SendByte(uint8);


#endif /* __SPI_Configuration_H */
