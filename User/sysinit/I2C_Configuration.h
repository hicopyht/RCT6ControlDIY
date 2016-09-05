#ifndef _I2C_Configuration
#define	_I2C_Configuration

#define I2C_TIMEOUT_VALUE	500		// 1800 system calculation equals to 10 communication clock cycle
#define I2C_WaitFor(status)  {tmpU16 = 0; \
	while((!(status))  && (tmpU16 < I2C_TIMEOUT_VALUE))	\
	{	\
		tmpU16 ++;	\
	}	\
	if(tmpU16 >= I2C_TIMEOUT_VALUE)	\
		return FALSE;}


void I2C1_Init(void);
bool I2C1_WriteByte(uint8_t device, uint8_t reg, uint8_t data);
bool  I2C1_ReadBuffer(uint8_t device, uint8_t reg, uint8_t *pdata, uint8_t num);

void I2C2_Init(void);
bool I2C2_WriteByte(uint8_t device, uint8_t reg, uint8_t data);
bool  I2C2_ReadBuffer(uint8_t device, uint8_t reg, uint8_t *pdata, uint8_t num);

#endif
