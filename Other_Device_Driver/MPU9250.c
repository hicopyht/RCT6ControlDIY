#include "Configuration.h"

struct XYZ_DATA accel_data;
struct XYZ_DATA gyro_data;
struct XYZ_DATA magnet_data;
U16 mpu_temp_value = 0;

U8 accel_buffer[14] = {0};
U8 xyz_buffer[6] = {0};

//
struct LowPassFilter2p gyro_filter_z;
struct XYZ_DATA gyro_data_filtered;

// Initialize MPU9250
void MPU9250_Init(void)
{
	U8 status;

	// Init filter
	LowPassFilter2p_Init(&gyro_filter_z, 50, 20);
	
	SPI2_HighSpeed(DISABLE);
	//delayms(100);
	// 16bits gyroscope and acceleration
	MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x00);	// Internal 20MHz oscillator
	MPU9250_WriteReg(MPU9250_REG_SIGNAL_PATH_RESET, 0x07); // reset all signal path
	MPU9250_WriteReg(MPU9250_REG_SMPLRT_DIV, 0);	// sample rate = output rate / (div + 1), 1k / (div+1), 1kHz
	MPU9250_WriteReg(MPU9250_REG_CONFIG, 0x03);	// DLPF 3 <41Hz>, gyro 1khz, delay 5.9ms
	MPU9250_WriteReg(MPU9250_REG_GYRO_CONFIG, 0x0B);	// +-500 deg/sec
	MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG, 0x18);	// +-16g
	MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG2, 0x0B); // DLPF 3 <41Hz>, accel 1khz, delay 11.8ms
	//
	
	//MPU9250_WriteReg(MPU9250_REG_INT_PIN_CFG, 0xD0); // active low, open-drain, 50us pulse, clear by any read operation
	//MPU9250_WriteReg(MPU9250_REG_INT_ENABLE, 0x01);	// enable data ready interrupt 
	status = MPU9250_ReadReg(MPU9250_REG_SMPLRT_DIV);
	
	SPI2_HighSpeed(ENABLE);
	//delayms(100);

#if defined (CONFIG_DEBUG)
	printf("Initializing MPU9250... div = %d ... Done \r\n", status);
	DebugPrintf();
#endif

}

// Read acceleration result
void MPU9250_ReadResult(void)
{
	/*
	float stamp = getTimeStamp();
	// ax, ay, az, T, gx, gy, gz
	MPU9250_ReadBuffer(MPU9250_ACCEl_RESULT_FIRST, accel_buffer, 14);
	MPU9250_ReadBuffer(MPU9250_MAG_RESULT_FIRST, magnet_buffer, 6);
	//
	accel_data.x = U8toS16( accel_buffer[1], accel_buffer[0]);
	accel_data.y = U8toS16( accel_buffer[3], accel_buffer[2]);
	accel_data.z = U8toS16( accel_buffer[5], accel_buffer[4]);
	mpu_temp_value= U8toU16( accel_buffer[7], accel_buffer[6]);
	gyro_data.x = U8toS16( accel_buffer[9], accel_buffer[8]);
	gyro_data.y = U8toS16( accel_buffer[11], accel_buffer[10]);
	gyro_data.z = U8toS16( accel_buffer[13], accel_buffer[12]);
	magnet_data.x = U8toS16( accel_buffer[0], accel_buffer[1]);
	magnet_data.y = U8toS16( accel_buffer[2], accel_buffer[3]);
	magnet_data.z = U8toS16( accel_buffer[4], accel_buffer[5]);
	*/
	MPU9250_ReadBuffer(MPU9250_REG_GYRO_ZOUT_H, xyz_buffer, 2);
	gyro_data.z = U8toS16( xyz_buffer[1], xyz_buffer[0] );
	gyro_data_filtered.z = LowPassFilter2p_apply(&gyro_filter_z, gyro_data.z);

}

// Write byte
void MPU9250_WriteReg(U8 reg, U8 byte)
{
	MPU_Selected();		// select chip
	delayms(1);
	MPU_SPI_RW(MPU_WRITE_REG | reg);	// send register address
	MPU_SPI_RW(byte);		// send byte
	MPU_Deselected();	// deselect chip
	delayms(1);
}

// Read byte
U8 MPU9250_ReadReg(U8 reg)
{
	U8 byte;
	MPU_Selected();		// select chip
	MPU_SPI_RW(MPU_READ_REG | reg);	// send register address
	byte = MPU_SPI_RW(0xFF);		// read byte
	MPU_Deselected();	// deselect chip
	return byte;
}

// Write buffer
void MPU9250_WriteBuffer(U8 reg, U8*buffer, U8 num)
{
	MPU_Selected();		// select chip
	delayms(1);
	MPU_SPI_RW(MPU_WRITE_REG | reg);	// send register address
	while(num > 0)
	{
		MPU_SPI_RW(*buffer);		// send byte
		buffer++;
		num --;
	}
	MPU_Deselected();	// deselect chip
	delayms(1);
}

// Read buffer
void MPU9250_ReadBuffer(U8 reg, U8*buffer, U8 num)
{
	MPU_Selected();		// select chip
	MPU_SPI_RW(MPU_READ_REG | reg);	// send register address
	while(num > 0)
	{
		*buffer = MPU_SPI_RW(0xFF);		// read byte
		buffer++;
		num--;
	}
	MPU_Deselected();	// deselect chip
}


