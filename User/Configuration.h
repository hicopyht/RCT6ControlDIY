//系统配置文件，一般所有文件均须引用

//这一段无需改动

#ifndef _TYPE_DEF
#define _TYPE_DEF
typedef unsigned char  uint8;                   /* 无符号8位整型变量         */
typedef signed   char  int8;                    /* 有符号8位整型变量         */
typedef unsigned short uint16;                  /* 无符号16位整型变量        */
typedef signed   short int16;                   /* 有符号16位整型变量        */
typedef unsigned int   uint32;                  /* 无符号32位整型变量        */
typedef signed   int   int32;                   /* 有符号32位整型变量        */
typedef float          fp32;                     /* 单精度浮点数（32位长度）  */
typedef double         fp64;                    /* 双精度浮点数（64位长度）  */

typedef uint8  U8;
typedef uint16 U16;
typedef uint32 U32;
typedef int8   S8;
typedef int16  S16;
typedef int32  S32; 
typedef fp32   F32;
typedef fp64   F64;

typedef double V4[4];
typedef double V3[3];
#endif // _TYPE_DEF

#ifndef _Configuration_H
#define _Configuration_H

#define PI			3.14159265358979323846
#define M_G 			9.80f
#define M_PI_2		1.570796
#define M_2_PI_1 	6.28318530717958647692

#define TO_RAD(x)	(x * 0.01745329252)  // *pi/180
#define TO_DEG(x)	(x * 57.2957795131)  // *180/pi

#define SYSTEM_RESET	(SCB->AIRCR = 0x05FA0000 | (u32)0x04)		//重启

#define CONFIG_DEBUG
#define DebugPrintf()			U2Printf()
#define DebugPrintfDMA()		U2PrintfDMA()

#include "stm32f10x.h"
#include "RCC_Configuration.h"
#include "NVIC_Configuration.h"
#include "SysTick_Configuration.h"
#include "GPIO_Configuration.h"
#include "USART_Configuration.h"
#include "ADC_Configuration.h"
#include "DMA_Configuration.h"
#include "EXTI_Configuration.h"
#include "SPI_Configuration.h"
#include "TIM_Configuration.h"
#include "I2C_Configuration.h"


#include "stdio.h"
#include "math.h"
#include "string.h"

#include "MPU6000.h"

#include "parameter.h"
#include "Function.h"
#include "var.h"
#include "motor.h"
#include "pid.h"
#include "LowPassFilter2p.h"


void InitDevice(void);	

#endif /* _Configuration_H */

