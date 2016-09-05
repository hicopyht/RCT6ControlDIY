#include "Configuration.h"

//
#define GYRO_SAMP_FRQ    125
#define GYRO_CUTOFF_FRQ  20
#define ACC_SAMP_FRQ    125
#define ACC_CUTOFF_FRQ  20

//
U8 mpu_buffer[14] = {0};
U8 mpu_temperature[2] = {0};

//
struct IMU_DATA_PARAMS acc_params;
struct IMU_DATA_RAW acc_raw;
struct LowPassFilter2p acc_filter_x;
struct LowPassFilter2p acc_filter_y;
struct LowPassFilter2p acc_filter_z;
struct IMU_DATA_FILTERED acc_filtered;
//
//
struct IMU_DATA_PARAMS gyro_params;
struct IMU_DATA_RAW gyro_raw;
struct LowPassFilter2p gyro_filter_x;
struct LowPassFilter2p gyro_filter_y;
struct LowPassFilter2p gyro_filter_z;
struct IMU_DATA_FILTERED gyro_filtered;


//
U16 MpuFiFoPi = 0;
U16 MpuFiFoPo = 0;
struct IMU_DATA_RAW gyro_buffer[MPU_FIFO_LEN];
struct IMU_DATA_RAW acc_buffer[MPU_FIFO_LEN];

//
U8 Is_RecordIMUData = true;


// Initialize MPU6000
void MPU6000_Init(void)
{
	U8 status;

	gyro_params.range_scale = 2000.0f / 180.0 * M_PI / 32768.0f;
	gyro_params.x_offset = 15.0f;
	gyro_params.y_offset = 7.0f;
	gyro_params.z_offset = -1.0f;
	gyro_params.x_scale = 1.0f;
	gyro_params.y_scale = 1.0f;
	gyro_params.z_scale = 1.0f;

	// -2156/2103	-2122/2052	-2344/2096
	// -2099/2051	-2096/2036	-2168/2011
	acc_params.range_scale = 16.0f * M_G / 32768.0f;
	//acc_params.range_scale = 1.0f * M_G;
	acc_params.x_offset = -24.0f;
	acc_params.y_offset = -30.0f;
	acc_params.z_offset = -29.0f;
	acc_params.x_scale = 1.0f;
	acc_params.y_scale = 1.0f;
	acc_params.z_scale = 1.0f;
	//acc_params.x_scale = 2.0f / (2051.0f + 2099.0f);
	//acc_params.y_scale = 2.0f / (2036.0f + 2096.0f);
	//acc_params.z_scale = 2.0f / (2011.0f + 2168.0f);



	LowPassFilter2p_Init(&gyro_filter_x, GYRO_SAMP_FRQ, GYRO_CUTOFF_FRQ);
	LowPassFilter2p_Init(&gyro_filter_y, GYRO_SAMP_FRQ, GYRO_CUTOFF_FRQ);
	LowPassFilter2p_Init(&gyro_filter_z, GYRO_SAMP_FRQ, GYRO_CUTOFF_FRQ);

	LowPassFilter2p_Init(&acc_filter_x, ACC_SAMP_FRQ, ACC_CUTOFF_FRQ);
	LowPassFilter2p_Init(&acc_filter_y, ACC_SAMP_FRQ, ACC_CUTOFF_FRQ);
	LowPassFilter2p_Init(&acc_filter_z, ACC_SAMP_FRQ, ACC_CUTOFF_FRQ);


	
	SPI1_HighSpeed(DISABLE);
	//Delayms(100);
		// 16bits gyroscope and acceleration
	MPU6000_WriteReg(MPU6000_REG_PWR_MGMT_1, 0x03);	// PPL with gyroscope z clock
	Delayms(2000);
	MPU6000_WriteReg(MPU6000_REG_SIGNAL_PATH_RESET, 0x07); // reset all signal path
	Delayms(2000);
	//MPU6000_WriteReg(MPU6000_REG_SMPLRT_DIV, 15);	// sample rate = output rate / (div + 1), 8k / (15+1) = 500Hz
	//MPU6000_WriteReg(MPU6000_REG_SMPLRT_DIV, 39);	// sample rate = output rate / (div + 1), 8k / (39+1) = 200Hz
	MPU6000_WriteReg(MPU6000_REG_SMPLRT_DIV, 7);	// sample rate = output rate / (div + 1), 1k / (7+1) = 125Hz
	//MPU6000_WriteReg(MPU6000_REG_SMPLRT_DIV, 4);	// sample rate = output rate / (div + 1), 1k / (4+1) = 200Hz
	MPU6000_WriteReg(MPU6000_REG_CONFIG, BITS_DLPF_CFG_42HZ & BITS_DLPF_CFG_MASK);	// DLPF 42HZ, gyroscope output rate 1khz
	MPU6000_WriteReg(MPU6000_REG_GYRO_CONFIG, 0x18);	// +-2000 deg/sec, 16bits, -32768~32767
	MPU6000_WriteReg(MPU6000_REG_ACCEL_CONFIG, 0x18);	// +-16g, 16bits, -32768~32767
	//
	
	MPU6000_WriteReg(MPU6000_REG_INT_PIN_CFG, 0xD0); // active low, open-drain, 50us pulse, clear by any read operation
	MPU6000_WriteReg(MPU6000_REG_INT_ENABLE, 0x01);	// enable data ready interrupt 
	//status = MPU6000_ReadReg(MPU6000_REG_INT_STATUS);
	status = MPU6000_ReadReg(MPU6000_REG_SMPLRT_DIV);
	
	SPI1_HighSpeed(ENABLE);
	//Delayms(100);

#if defined (CONFIG_DEBUG)
	printf("\r\n Initializing MPU6000 ... Done div = %d", status);
	U1Printf();
#endif

}

// Read acceleration result
void MPU6000_ReadResult(void)
{
	U32 stamp = getTimeStamp();
	S16 ax, ay, az, gx, gy, gz;
	float x, y, z;
	// ax, ay, az, T, gx, gy, gz
	MPU6000_ReadBuffer(MPU6000_REG_RESULT_FIRST, mpu_buffer, 14);
	ax = (mpu_buffer[0] << 8) + mpu_buffer[1];
	ay = (mpu_buffer[2] << 8) + mpu_buffer[3];
	az = (mpu_buffer[4] << 8) + mpu_buffer[5];
	mpu_temperature[0] = mpu_buffer[6];
	mpu_temperature[1] = mpu_buffer[7];
	gx = (mpu_buffer[8] << 8) + mpu_buffer[9];
	gy = (mpu_buffer[10] << 8) + mpu_buffer[11];
	gz = (mpu_buffer[12] << 8) + mpu_buffer[13];

	// gyroscope data
	// adjust coordinate
	gyro_raw.x = gy;
	gyro_raw.y = ((gx == -32768) ? 32767 : -gx);
	gyro_raw.z = gz;
	//gyro_raw.x = ((gy == -32768) ? 32767 : -gy);
	//gyro_raw.y = ((gx == -32768) ? 32767 : -gx);
	//gyro_raw.z = gz;
	gyro_raw.stamp = stamp;
	// read real value
	x = (gyro_raw.x - gyro_params.x_offset) * gyro_params.range_scale * gyro_params.x_scale;
	y = (gyro_raw.y - gyro_params.y_offset) * gyro_params.range_scale * gyro_params.y_scale;
	z = (gyro_raw.z - gyro_params.z_offset) * gyro_params.range_scale * gyro_params.z_scale;
	//
	// 2nd order low pass filter
	gyro_filtered.x = LowPassFilter2p_apply(&gyro_filter_x, x);
	gyro_filtered.y = LowPassFilter2p_apply(&gyro_filter_y, y);
	gyro_filtered.z = LowPassFilter2p_apply(&gyro_filter_z, z);
	gyro_filtered.stamp = stamp;

	// acceleration
	// adjust coordinate
	acc_raw.x = ay;
	acc_raw.y = ((ax == -32768) ? 32767 : -ax);
	acc_raw.z = az;
	//acc_raw.x = ay;
	//acc_raw.y = ax;
	//acc_raw.z = -az;
	acc_raw.stamp = stamp;
	// read real value
	x = (acc_raw.x - acc_params.x_offset) * acc_params.range_scale * acc_params.x_scale;
	y = (acc_raw.y - acc_params.y_offset) * acc_params.range_scale * acc_params.y_scale;
	z = (acc_raw.z - acc_params.z_offset) * acc_params.range_scale * acc_params.z_scale;
	//
	// 2nd order low pass filter
	acc_filtered.x = LowPassFilter2p_apply(&acc_filter_x, x);
	acc_filtered.y = LowPassFilter2p_apply(&acc_filter_y, y);
	acc_filtered.z = LowPassFilter2p_apply(&acc_filter_z, z);
	acc_filtered.stamp = stamp;


	if(Is_RecordIMUData)
	{
		InMpuFiFo(gyro_raw, acc_raw);
	}
	
}

// Write byte
void MPU6000_WriteReg(U8 reg, U8 byte)
{
	MPU_Selected();		// select chip
	Delayms(1);
	MPU_SPI_RW(MPU_WRITE_REG | reg);	// send register address
	MPU_SPI_RW(byte);		// send byte
	MPU_Deselected();	// deselect chip
	Delayms(1);
}

// Read byte
U8 MPU6000_ReadReg(U8 reg)
{
	U8 byte;
	MPU_Selected();		// select chip
	MPU_SPI_RW(MPU_READ_REG | reg);	// send register address
	byte = MPU_SPI_RW(0xFF);		// read byte
	MPU_Deselected();	// deselect chip
	return byte;
}

// Write buffer
void MPU6000_WriteBuffer(U8 reg, U8*buffer, U8 num)
{
	MPU_Selected();		// select chip
	Delayms(1);
	MPU_SPI_RW(MPU_WRITE_REG | reg);	// send register address
	while(num > 0)
	{
		MPU_SPI_RW(*buffer);		// send byte
		buffer++;
		num --;
	}
	MPU_Deselected();	// deselect chip
	Delayms(1);
}

// Read buffer
void MPU6000_ReadBuffer(U8 reg, U8*buffer, U8 num)
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

void InMpuFiFo(struct IMU_DATA_RAW gin, struct IMU_DATA_RAW ain)
{
	gyro_buffer[MpuFiFoPi].x = gin.x;
	gyro_buffer[MpuFiFoPi].y = gin.y;
	gyro_buffer[MpuFiFoPi].z = gin.z;
	gyro_buffer[MpuFiFoPi].stamp = gin.stamp;

	acc_buffer[MpuFiFoPi].x = ain.x;
	acc_buffer[MpuFiFoPi].y = ain.y;
	acc_buffer[MpuFiFoPi].z = ain.z;
	acc_buffer[MpuFiFoPi].stamp = ain.stamp;

	MpuFiFoPi = (MpuFiFoPi + 1) % MPU_FIFO_LEN;
}

U8 OutMpuFiFo(struct IMU_DATA_RAW *gout, struct IMU_DATA_RAW *aout)
{
	if(MpuFiFoPo == MpuFiFoPi)
		return false;

	gout->x = gyro_buffer[MpuFiFoPo].x;
	gout->y = gyro_buffer[MpuFiFoPo].y;
	gout->z = gyro_buffer[MpuFiFoPo].z;
	gout->stamp = gyro_buffer[MpuFiFoPo].stamp;

	aout->x = acc_buffer[MpuFiFoPo].x;
	aout->y = acc_buffer[MpuFiFoPo].y;
	aout->z = acc_buffer[MpuFiFoPo].z;
	aout->stamp = acc_buffer[MpuFiFoPo].stamp;
	
	MpuFiFoPo = (MpuFiFoPo + 1) % MPU_FIFO_LEN;
	return true;
}

U16 GetMpuFiFoSize(void)
{
	if(MpuFiFoPi < MpuFiFoPo)
		return (MpuFiFoPi + MPU_FIFO_LEN - MpuFiFoPo);
	else
		return (MpuFiFoPi - MpuFiFoPo);
}

void ClearMpuFiFo(void)
{
	MpuFiFoPo = MpuFiFoPi;
}

U8 IsMpuFiFoEmpty(void)
{
	return (MpuFiFoPo == MpuFiFoPi);
}


