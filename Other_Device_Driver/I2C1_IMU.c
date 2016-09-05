#include "Configuration.h"

//
U8 gyro_buffer[GYRO_BUF_LEN] = {0};
U8 acc_buffer[ACC_BUF_LEN] = {0};
U8 compass_buffer[COMPASS_BUF_LEN] = {0};
//
struct XYZ_DATA gyro_raw;
struct XYZ_DATA accel_raw;
struct XYZ_DATA magnet_raw;
//
struct LowPassFilter2p gyro_filter_z;
struct XYZ_DATA gyro_filtered;
bool gyro_update = FALSE;

void IMU_Init(void)
{
	// Init filter
	LowPassFilter2p_Init(&gyro_filter_z, 50, 20);
	//
	Init_ITG3205();
	Init_ADXL345();
	Init_HMC5883L();
}

void IMU_Stop(void)
{
	I2C2_WriteByte(ITG3205_Addr, ITG3205_PWR_M, 0x40);   // standby mode
	I2C2_WriteByte(ADXL345_Addr, 0x2D,0x00);   // stop measure
	I2C2_WriteByte(HMC5883L_WriteAddr, 0x02,0x02);   // idle mode
}

// initialize ITG3205
void Init_ITG3205(void)
{
	/*
	I2C2_WriteByte(ITG3205_Addr, ITG3205_PWR_M, 0x80);	// reset device and registers
	I2C2_WriteByte(ITG3205_Addr, ITG3205_SMPL, 0x13);    	// sample divided by (19+1), 1k/20 = 50
	I2C2_WriteByte(ITG3205_Addr, ITG3205_DLPF, 0x1b);    	// ±2000°, 1kHz, DLPF 1<188Hz>
	I2C2_WriteByte(ITG3205_Addr, ITG3205_INT_C, 0xD1);  	// active low, open drain , 50us pulse, cleared any rg read, data ready int enable
	I2C2_WriteByte(ITG3205_Addr, ITG3205_PWR_M, 0x00); 	//
	*/
		
	I2C2_WriteByte(ITG3205_Addr, ITG3205_PWR_M, 0x80);	// reset device and registers
	I2C2_WriteByte(ITG3205_Addr, ITG3205_SMPL, 0x00);    	// sample divided by (0+1), 1k/1= 1k
	I2C2_WriteByte(ITG3205_Addr, ITG3205_DLPF, 0x1A);    	// ±2000°, 1kHz, DLPF 2<98Hz>
	I2C2_WriteByte(ITG3205_Addr, ITG3205_INT_C, 0x00);  	// active low, open drain , 50us pulse, cleared any rg read, no interrupt enables
	I2C2_WriteByte(ITG3205_Addr, ITG3205_PWR_M, 0x30); 	// XG\YG standby

#if defined (CONFIG_DEBUG)
	printf("Initializing ITG3205 ... Done\r\n");
	DebugPrintf();
#endif
}

	// initialize ADXL345
void Init_ADXL345(void)
{
	I2C2_WriteByte(ADXL345_Addr, 0x2D,0x00);   // stop measure
	I2C2_WriteByte(ADXL345_Addr, 0x2E,0x00);   // first disable DATA_READY interupt
	I2C2_WriteByte(ADXL345_Addr, 0x31,0x2B);   // interrupt active low, +-16g，13 bit adc
  	I2C2_WriteByte(ADXL345_Addr, 0x2C,0x0A);   // sample rate 100Hz
  	I2C2_WriteByte(ADXL345_Addr, 0x38,0x00);   // bypass mode
  	I2C2_WriteByte(ADXL345_Addr, 0x2F,0x00);   // interupt map to int1
   	I2C2_WriteByte(ADXL345_Addr, 0x2E,0x80);   // enable DATA_READY interupt
   	I2C2_WriteByte(ADXL345_Addr, 0x2D,0x08);   // measure
	//I2C2_WriteByte(ADXL345_Addr, 0x1E,0x00);   //X 偏移量 
	//I2C2_WriteByte(ADXL345_Addr, 0x1F,0x00);   //Y 偏移量 
	//I2C2_WriteByte(ADXL345_Addr, 0x20,0x05);   //Z 偏移量 
}

	// initialize HMC5883L
void Init_HMC5883L(void)
{
	// +- 1.3G, 12bit adc, -2048~2047
	I2C2_WriteByte(HMC5883L_WriteAddr, 0x02,0x02);   // idle mode
	I2C2_WriteByte(HMC5883L_WriteAddr, 0x00,0x78);   // 8 samples averaged, 75 Hz
       I2C2_WriteByte(HMC5883L_WriteAddr, 0x02,0x00);   // Continuous-Measurement mode
}


//******读取ITG3205 数据****************************************
bool Read_ITG3205(void)
{
	double stamp = getTimeStamp();
       if(!I2C2_ReadBuffer(ITG3205_Addr, ITG3205_TMP_H, gyro_buffer, GYRO_BUF_LEN)) 	// temp, x, y, z
   	{
   		return FALSE;
   	}

	gyro_raw.x = (S16)((gyro_buffer[2] << 8) | gyro_buffer[3]);		//读取计算X 轴数据
	gyro_raw.y = (S16)((gyro_buffer[4] << 8) | gyro_buffer[5]);		//读取计算Y 轴数据
	gyro_raw.z = (S16)((gyro_buffer[6] << 8) | gyro_buffer[7]);		//读取计算Z 轴数据

	gyro_filtered.z = LowPassFilter2p_apply(&gyro_filter_z, gyro_raw.z);

	return TRUE;
}

bool Read_ITG3205_Z(void)
{
	//double stamp = getTimeStamp();
       if(!I2C2_ReadBuffer(ITG3205_Addr, ITG3205_GZ_H, gyro_buffer, 2)) 	// z
   	{
   		I2C2_ReadBuffer(ITG3205_Addr, ITG3205_INT_S, gyro_buffer, 1);
		I2C2_ReadBuffer(ITG3205_Addr, ITG3205_TMP_H, gyro_buffer, GYRO_BUF_LEN);
   		//Init_ITG3205();
   		printf("ITG Err.\r\n");
		DebugPrintfDMA();
   		return FALSE;
   	}

	gyro_raw.z = (S16)((gyro_buffer[0] << 8) | gyro_buffer[1]);		//读取计算X 轴数据
	//gyro_filtered.z = LowPassFilter2p_apply(&gyro_filter_z, gyro_raw.z);

	return TRUE;
}

//******读取ADXL345 数据****************************************
bool Read_ADXL345(void)
{
	double stamp = getTimeStamp();
	if(!I2C2_ReadBuffer(ADXL345_Addr, 0x32, acc_buffer, ACC_BUF_LEN))  // x, y, z
	{
		return FALSE;
	}
	
	accel_raw.x = (S16)((acc_buffer[1] << 8) + acc_buffer[0]);  //合成数据
	accel_raw.y = (S16)((acc_buffer[3] << 8) + acc_buffer[2]);  //合成数据
	accel_raw.z = (S16)((acc_buffer[5] << 8) + acc_buffer[4]);  //合成数据
		
	return TRUE;
}

//******读取HMC5883L 数据****************************************
bool Read_HMC5883L(void)
{
	double stamp = getTimeStamp();
	if(!I2C2_ReadBuffer(HMC5883L_ReadAddr, 0x03, compass_buffer, COMPASS_BUF_LEN)) //x, z, y
	{
		return FALSE;
	}
       
       magnet_raw.x= (S16)((compass_buffer[0] << 8) | compass_buffer[1]); //Combine MSB and LSB of X Data output register
       magnet_raw.y = (S16)((compass_buffer[2] << 8) | compass_buffer[3]); //Combine MSB and LSB of Z Data output register
       magnet_raw.z = (S16)((compass_buffer[4] << 8) | compass_buffer[5]); 
	
	//calculate the angle
       //compass_angle = atan2(compass_out_y, compass_out_x) * (180 / 3.14159265) + 180; // angle in degrees
       return TRUE;
}



