#ifndef _MPU9250_H
#define _MPU9250_H


#define MPU9250_REG_SELF_TEST_X	0x0D
#define MPU9250_REG_SELF_TEST_Y	0x0E
#define MPU9250_REG_SELF_TEST_Z	0x0F
#define MPU9250_REG_SELF_TEST_A	0x10
#define MPU9250_REG_SMPLRT_DIV	0x19
//
#define MPU9250_REG_CONFIG           	0x1A
#define MPU9250_REG_GYRO_CONFIG	0x1B
#define MPU9250_REG_ACCEL_CONFIG	0x1C
#define MPU9250_REG_ACCEL_CONFIG2	0x1D
#define MPU9250_REG_FIFO_EN          	0x23
#define MPU9250_REG_INT_PIN_CFG	0x37
#define MPU9250_REG_INT_ENABLE	0x38
#define MPU9250_REG_INT_STATUS 	0x3A
//
#define MPU9250_REG_ACCEL_XOUT_H	0x3B
#define MPU9250_REG_ACCEL_XOUT_L	0x3C
#define MPU9250_REG_ACCEL_YOUT_H 	0x3D
#define MPU9250_REG_ACCEL_YOUT_L	0x3E
#define MPU9250_REG_ACCEL_ZOUT_H	0x3F
#define MPU9250_REG_ACCEL_ZOUT_L	0x40
#define MPU9250_REG_TEMP_OUT_H	0x41
#define MPU9250_REG_TEMP_OUT_L 	0x42
#define MPU9250_REG_GYRO_XOUT_H	0x43
#define MPU9250_REG_GYRO_XOUT_L	0x44
#define MPU9250_REG_GYRO_YOUT_H	0x45
#define MPU9250_REG_GYRO_YOUT_L	0x46
#define MPU9250_REG_GYRO_ZOUT_H 	0x47
#define MPU9250_REG_GYRO_ZOUT_L 	0x48
//
#define MPU9250_REG_MAG_XOUT_L	0x03
#define MPU9250_REG_MAG_XOUT_H	0x04
#define MPU9250_REG_MAG_YOUT_L	0x05
#define MPU9250_REG_MAG_YOUT_H	0x06
#define MPU9250_REG_MAG_ZOUT_L	0x07
#define MPU9250_REG_MAG_ZOUT_H	0x08
//
#define MPU9250_REG_SIGNAL_PATH_RESET	0x68
#define MPU9250_REG_USER_CTRL		0x6A
#define MPU9250_REG_PWR_MGMT_1	0x6B
#define MPU9250_REG_PWR_MGMT_2	0x6C
#define MPU9250_REG_FIFO_COUNTH	0x72
#define MPU9250_REG_FIFO_COUNTL	0x73
#define MPU9250_REG_FIFO_R_W		0x74
#define MPU9250_REG_WHO_AM_I		0x75
// AK8963
#define AK8963_ST1		0x02
#define AK8963_ST2		0x09
#define AK8963_CNTL1	0x0A
#define AK8963_CNTL2	0x0B
//
#define ACCEL_ADDRESS	0xD0 
#define GYRO_ADDRESS	0xD0
#define MAG_ADDRESS	0x18


// Read/Write operation
#define MPU_WRITE_REG	0x00
#define MPU_READ_REG	0x80
#define MPU_SPI_RW(x)	SPI2_SendByte(x)
#define MPU9250_ACCEl_RESULT_FIRST	0x3B
#define MPU9250_GYRO_RESULT_FIRST		0x43
#define MPU9250_MAG_RESULT_FIRST		0x03
//
#define MPU_Selected()		SPI2_CS_LOW()	//set low
#define MPU_Deselected()		SPI2_CS_HIGH()

struct XYZ_DATA
{
	S16 x;
	S16 y;
	S16 z;
};

extern struct XYZ_DATA accel_data;
extern struct XYZ_DATA gyro_data;
extern struct XYZ_DATA magnet_data;
extern U16 mpu_temp_value;


extern U8 accel_buffer[14];
extern U8 xyz_buffer[6];
//
extern struct LowPassFilter2p gyro_filter_z;
extern struct XYZ_DATA gyro_data_filtered;


void MPU9250_Init(void);
void MPU9250_ReadResult(void);
void MPU9250_WriteReg(U8 reg, U8 byte);
U8 MPU9250_ReadReg(U8 reg);
void MPU9250_WriteBuffer(U8 reg, U8*buffer, U8 num);
void MPU9250_ReadBuffer(U8 reg, U8*buffer, U8 num);

#endif

