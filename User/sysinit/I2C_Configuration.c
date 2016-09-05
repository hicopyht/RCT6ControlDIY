#include "Configuration.h"

void I2C1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure; 

	/* I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   
  
	/* Configure I2C pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 300000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

#if defined (CONFIG_DEBUG)
	printf("Initializing  I2C1 ... Done\n");
	DebugPrintf();
#endif
  
}

bool I2C1_WriteByte(uint8_t device, uint8_t reg, uint8_t data)
{
	U16 tmpU16 = 0;

	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
	/*
	tmpU16 = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && (tmpU16 < I2C_TIMEOUT_VALUE))
	{
		tmpU16 ++;
	}
	if(tmpU16 >= I2C_TIMEOUT_VALUE)
		return FALSE;
	*/
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	
	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, device, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the EEPROM's internal address to write to : only one byte Address */
	I2C_SendData(I2C1, reg);

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(I2C1, data); 

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);

	return TRUE;
}

bool  I2C1_ReadBuffer(uint8_t device, uint8_t reg, uint8_t *pdata, uint8_t num)
{  
	U16 tmpU16 = 0;
	U16 tmpCount = 0;

	/* While the bus is busy */
	//while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_WaitFor(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, device, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the EEPROM's internal address to read from: Only one byte address */
	I2C_SendData(I2C1, reg);  

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for read */
	I2C_Send7bitAddress(I2C1, device, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* While there is data to be read */
	tmpU16 = 0;
	tmpCount = I2C_TIMEOUT_VALUE * num;
	while(num && (tmpU16 < tmpCount))  
	{
		if(num == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(I2C1, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		/* Test on EV7 and clear it */
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
		{      
			/* Read a byte from the EEPROM */
			*pdata = I2C_ReceiveData(I2C1);

			/* Point to the next location where the byte read will be saved */
			pdata++; 

			/* Decrement the read bytes counter */
			num--;        
		}   
		tmpU16 ++;
	}
	if(tmpU16 >= tmpCount)
		return FALSE;

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return TRUE;
}


///*********************************************************************************************///


void I2C2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure; 

	/* I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);   
  
	/* Configure I2C  pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 300000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C2, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C2, &I2C_InitStructure);

#if defined (CONFIG_DEBUG)
	printf("Initializing  I2C2 ... Done\n");
	DebugPrintf();
#endif
  
}


bool I2C2_WriteByte(uint8_t device, uint8_t reg, uint8_t data)
{
	U16 tmpU16 = 0;

	/* Send STRAT condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  
	/*
	tmpU16 = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && (tmpU16 < I2C_TIMEOUT_VALUE))
	{
		tmpU16 ++;
	}
	if(tmpU16 >= I2C_TIMEOUT_VALUE)
		return FALSE;
	*/
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	
	
	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the EEPROM's internal address to write to : only one byte Address */
	I2C_SendData(I2C2, reg);

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(I2C2, data); 

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	return TRUE;
}

bool  I2C2_ReadBuffer(uint8_t device, uint8_t reg, uint8_t *pdata, uint8_t num)
{  
	U16 tmpU16 = 0;
	U16 tmpCount = 0;

	/* While the bus is busy */
	//while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_WaitFor(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the EEPROM's internal address to read from: Only one byte address */
	I2C_SendData(I2C2, reg);  

	/* Test on EV8 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on EV5 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	//while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_WaitFor(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* While there is data to be read */
	tmpU16 = 0;
	tmpCount = I2C_TIMEOUT_VALUE * num;
	while(num && (tmpU16 < tmpCount))  
	{
		if(num == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(I2C2, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(I2C2, ENABLE);
		}

		/* Test on EV7 and clear it */
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))  
		{      
			/* Read a byte from the EEPROM */
			*pdata = I2C_ReceiveData(I2C2);

			/* Point to the next location where the byte read will be saved */
			pdata++; 

			/* Decrement the read bytes counter */
			num--;        
		}   
		tmpU16 ++;
	}
	if(tmpU16 >= tmpCount)
		return FALSE;

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	return TRUE;
}


