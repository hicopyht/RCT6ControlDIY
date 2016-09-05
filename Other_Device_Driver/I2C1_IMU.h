#ifndef _I2C1_H
#define _I2C1_H

//定义ITG3205内部地址********************
#define	ITG3205_WHO	0x00
#define	ITG3205_SMPL	0x15
#define	ITG3205_DLPF	0x16
#define	ITG3205_INT_C	0x17
#define	ITG3205_INT_S	0x1A
#define	ITG3205_TMP_H	0x1B
#define	ITG3205_TMP_L	0x1C
#define	ITG3205_GX_H	0x1D
#define	ITG3205_GX_L	0x1E
#define	ITG3205_GY_H	0x1F
#define	ITG3205_GY_L	0x20
#define	ITG3205_GZ_H	0x21
#define	ITG3205_GZ_L	0x22
#define	ITG3205_PWR_M	0x3E
//****************************

//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define	ITG3205_Addr		0xD0	//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define	ADXL345_Addr		0xA6	//加速度传感器器件地址
#define	HMC5883L_WriteAddr	0x3C	//磁场传感器器件地址
#define	HMC5883L_ReadAddr		0x3C	//磁场传感器器件地址
#define	HMC5983L_Addr			0x3C

// imu data buffer
struct XYZ_DATA
{
	S16 x;
	S16 y;
	S16 z;
	double stamp;
};

// data buffer
#define GYRO_BUF_LEN		8
#define ACC_BUF_LEN			6
#define COMPASS_BUF_LEN	6
//
extern U8 gyro_buffer[GYRO_BUF_LEN];
extern U8 acc_buffer[ACC_BUF_LEN];
extern U8 compass_buffer[COMPASS_BUF_LEN];
//
extern struct XYZ_DATA gyro_raw;
extern struct XYZ_DATA accel_raw;
extern struct XYZ_DATA magnet_raw;
//
extern struct XYZ_DATA gyro_filtered;
extern bool gyro_update;

void IMU_Init(void);
void IMU_Stop(void);
void Init_ITG3205(void);
void Init_ADXL345(void);
void Init_HMC5883L(void);
bool Read_ITG3205(void);
bool Read_ITG3205_Z(void);
bool Read_ADXL345(void);
bool Read_HMC5883L(void);


#endif

