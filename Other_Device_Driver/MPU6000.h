#ifndef _MPU6000_H
#define _MPU6000_H

#define MPU6000_REG_SELF_TEST_X	0x0D
#define MPU6000_REG_SELF_TEST_Y	0x0E
#define MPU6000_REG_SELF_TEST_Z	0x0F
#define MPU6000_REG_SELF_TEST_A	0x10
#define MPU6000_REG_SMPLRT_DIV	0x19
//
#define MPU6000_REG_CONFIG           	0x1A
#define MPU6000_REG_GYRO_CONFIG	0x1B
#define MPU6000_REG_ACCEL_CONFIG	0x1C
#define MPU6000_REG_FIFO_EN          	0x23
#define MPU6000_REG_INT_PIN_CFG	0x37
#define MPU6000_REG_INT_ENABLE	0x38
#define MPU6000_REG_INT_STATUS 	0x3A
//
#define MPU6000_REG_ACCEL_XOUT_H	0x3B
#define MPU6000_REG_ACCEL_XOUT_L	0x3C
#define MPU6000_REG_ACCEL_YOUT_H 	0x3D
#define MPU6000_REG_ACCEL_YOUT_L	0x3E
#define MPU6000_REG_ACCEL_ZOUT_H	0x3F
#define MPU6000_REG_ACCEL_ZOUT_L	0x40
#define MPU6000_REG_TEMP_OUT_H	0x41
#define MPU6000_REG_TEMP_OUT_L 	0x42
#define MPU6000_REG_GYRO_XOUT_H	0x43
#define MPU6000_REG_GYRO_XOUT_L	0x44
#define MPU6000_REG_GYRO_YOUT_H	0x45
#define MPU6000_REG_GYRO_YOUT_L	0x46
#define MPU6000_REG_GYRO_ZOUT_H 	0x47
#define MPU6000_REG_GYRO_ZOUT_L 	0x48
//
#define MPU6000_REG_SIGNAL_PATH_RESET	0x68
#define MPU6000_REG_USER_CTRL		0x6A
#define MPU6000_REG_PWR_MGMT_1	0x6B
#define MPU6000_REG_PWR_MGMT_2	0x6C
#define MPU6000_REG_FIFO_COUNTH	0x72
#define MPU6000_REG_FIFO_COUNTL	0x73
#define MPU6000_REG_FIFO_R_W		0x74
#define MPU6000_REG_WHO_AM_I		0x75

//
#define BITS_DLPF_CFG_256HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ			0x01
#define BITS_DLPF_CFG_98HZ				0x02
#define BITS_DLPF_CFG_42HZ				0x03
#define BITS_DLPF_CFG_20HZ				0x04
#define BITS_DLPF_CFG_10HZ				0x05
#define BITS_DLPF_CFG_5HZ				0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK				0x07

// Read/Write operation
#define MPU_WRITE_REG	0x00
#define MPU_READ_REG	0x80
#define MPU_SPI_RW(x)	SPI1_SendByte(x)
#define MPU6000_REG_RESULT_FIRST	0x3B


extern U8 mpu_buffer[14];
extern U8 mpu_temperature[2];
//
#define MPU_FIFO_LEN	700
//
extern struct IMU_DATA_FILTERED gyro_filtered;
extern struct IMU_DATA_FILTERED acc_filtered;
//
//
extern U8 Is_RecordIMUData;

void MPU6000_Init(void);
void MPU6000_ReadResult(void);
void MPU6000_WriteReg(U8 reg, U8 byte);
U8 MPU6000_ReadReg(U8 reg);
void MPU6000_WriteBuffer(U8 reg, U8*buffer, U8 num);
void MPU6000_ReadBuffer(U8 reg, U8*buffer, U8 num);
//
void InMpuFiFo(struct IMU_DATA_RAW gin, struct IMU_DATA_RAW ain);
U8 OutMpuFiFo(struct IMU_DATA_RAW *gout, struct IMU_DATA_RAW *aout);
U16 GetMpuFiFoSize(void);
void ClearMpuFiFo(void);
U8 IsMpuFiFoEmpty(void);


#endif

