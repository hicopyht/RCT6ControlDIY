#include "Configuration.h"


const char * param_names[PARAM_NUM] = {
	"save",		// 0
	"odometry_frequency",	// 1, actually being controlled by MPU6500 INT
	"controller_frequency",	// 2, same as odometry frequency
	"controller_timeout",	// 3
	"pwm_max",	// 4
	"pwm_min",	// 5
	"pwm_deadzone",	// 6
	"pwm_acceleration",	// 7
	"pwm_deceleration",	// 8
	"velocity_max",	// 9
	"velocity_min",	// 10
	"velocity_deadzone",	// 11
	"velocity_acceleration",	// 12
	"velocity_deceleration",	// 13
	"velocity_pid_p",	// 14
	"velocity_pid_i",	// 15
	"velocity_pid_d",	// 16
	"velocity_pid_clamp"	// 17
};

const float default_values[PARAM_NUM] = {
	1.0,	// 1
	50,	// 50 Hz
	50,	// 50 Hz
	0.5, 	// 0.5 second
	300,
	-300,
	20,
	7,
	7,
	19.8,	// rad/s
	-19.8,	// rad/s
	0.2,		// rad/s
	0.5,
	0.5,
	1.6,
	0.2,
	0.6,
	20.0
};

struct ParamData param_vector[PARAM_NUM];
U8 flash_page_buffer[FLASH_PAGE_SIZE] = {0};

void paramsInit(void)
{
	U32 i = 0;
	U32 address = (U32)PARAM_START_ADDRESS;
	U32 saved = 0xFFFFFFFF;

	// Read saved byte
	saved = FlashReadWord( address );

#if defined (CONFIG_DEBUG)
	printf("Param saved: %s\r\n", (saved ? "true" : "false") );
	DebugPrintf();
#endif

	// Initialized data
	for( i = 0; i < PARAM_NUM; i++)
	{
		param_vector[i].value.f = default_values[i];
		param_vector[i].name = param_names[i];
	}

	// Load params
	if( !saved )
		paramsSave();
	else		// Save params
		paramsLoad();

	// Info
#if defined (CONFIG_DEBUG)
	paramsInfoPrint();
#endif
	
}

void paramsRefresh(void)
{
	// Set params from parameters server
	VelocityBuff.deadzone = paramReadFloatWithDefault( "velocity_deadzone", VELOCITY_DEADZONE );
	VelocityBuff.max = paramReadFloatWithDefault("velocity_max", VELOCITY_MAX);
	VelocityBuff.min = paramReadFloatWithDefault("velocity_min", VELOCITY_MIN);
	//
	Velocity.deadzone = VelocityBuff.deadzone;
	Velocity.max = VelocityBuff.max;
	Velocity.min = VelocityBuff.min;
	Velocity.accel = paramReadFloatWithDefault("velocity_acceleration", VELOCITY_ACCEL );
	Velocity.decel = paramReadFloatWithDefault("velocity_deceleration", VELOCITY_DECEL );

	//
	PidBuff.p_gain = paramReadFloatWithDefault( "velocity_pid_p", VELOCITY_PID_P_GAIN );
	PidBuff.i_gain = paramReadFloatWithDefault( "velocity_pid_i", VELOCITY_PID_I_GAIN );
	PidBuff.d_gain = paramReadFloatWithDefault( "velocity_pid_d", VELOCITY_PID_D_GAIN );
	PidBuff.i_max = paramReadFloatWithDefault( "velocity_pid_clamp", VELOCITY_PID_CLAMP_GAIN );
	PidBuff.i_min = -PidBuff.i_max;
	//
	VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
	VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;
	//
	MOTOR[MOTOR_LEFT].pwm_deadzone = paramReadFloatWithDefault( "pwm_deadzone", MOTOR_PWM_DEADZONE );
	MOTOR[MOTOR_LEFT].pwm_max = paramReadFloatWithDefault( "pwm_max", MOTOR_PWM_MAX );
	MOTOR[MOTOR_LEFT].pwm_min = paramReadFloatWithDefault( "pwm_min", MOTOR_PWM_MIN );
	MOTOR[MOTOR_LEFT].acceleration = paramReadFloatWithDefault( "pwm_acceleration", MOTOR_PWM_ACCEL);
	MOTOR[MOTOR_LEFT].deceleration = paramReadFloatWithDefault( "pwm_deceleration", MOTOR_PWM_DECEL );
	MOTOR[MOTOR_RIGHT].pwm_deadzone =MOTOR[MOTOR_LEFT].pwm_deadzone;
	MOTOR[MOTOR_RIGHT].pwm_max =MOTOR[MOTOR_LEFT].pwm_max;
	MOTOR[MOTOR_RIGHT].pwm_min =MOTOR[MOTOR_LEFT].pwm_min;
	MOTOR[MOTOR_RIGHT].acceleration =MOTOR[MOTOR_LEFT].acceleration;
	MOTOR[MOTOR_RIGHT].deceleration =MOTOR[MOTOR_LEFT].deceleration;
}

void paramsInfoPrint(void)
{
	U32 i = 0;

	printf("Params list, size = %d:\r\n", PARAM_NUM);
	DebugPrintf();

	for( i = 0; i < PARAM_NUM; i++ )
	{
	       printf("   -%d: %s = %f \r\n", i, param_vector[i].name, param_vector[i].value.f );
		//printf("   -name: '%s'\r\n", param_vector[i].name);
		//printf("    value: %f\r\n", param_vector[i].value.f);
		//printf("    bytes: %x %x %x %x\r\n", param_vector[i].value.bytes[0], param_vector[i].value.bytes[1], param_vector[i].value.bytes[2], param_vector[i].value.bytes[3]);
		DebugPrintf();
	}
}

void paramsLoad(void)
{
	U32 i = 0;
	U32 address = (U32)PARAM_START_ADDRESS;

#if defined (CONFIG_DEBUG)
	printf("Load params %d.\r\n", PARAM_NUM);
	DebugPrintf();
#endif
	
	for( i = 0; i < PARAM_NUM; i++ )
	{
		param_vector[i].value.word = *(U32*)address;
		address += 4;
	}
}

void paramsSave(void)
{
	U32 i = 0;
	U32 address = (U32)PARAM_START_ADDRESS;
	U32 *pWords = (U32*)flash_page_buffer;

	for( i = 0; i < FLASH_PAGE_SIZE; i++)
		flash_page_buffer[i] = 0xFF;

#if defined (CONFIG_DEBUG)
	printf("Save params %d.\r\n", PARAM_NUM);
	DebugPrintf();
#endif

	
	for( i = 0; i < PARAM_NUM; i++)
	{
		*pWords =  param_vector[i].value.word;
		pWords ++;
	}

	FlashWritePage( address,  flash_page_buffer );

}

void paramsReset( U8 save )
{
	U32 i = 0;

#if defined (CONFIG_DEBUG)
	printf("Reset params.\r\n");
	DebugPrintf();
#endif
	
	// Initialized data
	for( i = 0; i < PARAM_NUM; i++)
	{
		param_vector[i].value.f = default_values[i];
		param_vector[i].name = param_names[i];
	}

	// Save to flash
	if( save )
		paramsSave();
}


U32 paramFind( const char* name )
{
	U32 i = 0;

	while( i < PARAM_NUM )
	{
		if( strcmp( name, param_names[i] ) == 0 )
			break;
		i++;
	}

	return i;
}

U8 paramValidName( const char* name )
{
	return ( paramFind( name ) < PARAM_NUM );
}

U8 paramValidIndex( U8 index )
{
	return (index < PARAM_NUM);
}

U8 paramSetFloat( U8 index, float value )
{
	if( index >= PARAM_NUM )
		return FALSE;

	param_vector[index].value.f = value;
	return TRUE;
}

U8 paramGetFloat( U8 index, float* value )
{
	if( index >= PARAM_NUM )
		return FALSE;

	*value = param_vector[index].value.f;
	return TRUE;
}

U8 paramSetBytes4(U8 index, U8* bytes4)
{
	if( index >= PARAM_NUM )
		return FALSE;

	param_vector[index].value.bytes[0] = bytes4[0];
	param_vector[index].value.bytes[1] = bytes4[1];
	param_vector[index].value.bytes[2] = bytes4[2];
	param_vector[index].value.bytes[3] = bytes4[3];
	return TRUE;
}

U8 paramGetBytes4(U8 index, U8* bytes4)
{
	if( index >= PARAM_NUM )
		return FALSE;

	bytes4[0] = param_vector[index].value.bytes[0];
	bytes4[1] = param_vector[index].value.bytes[1];
	bytes4[2] = param_vector[index].value.bytes[2];
	bytes4[3] = param_vector[index].value.bytes[3];
	return TRUE;
}

bool paramWriteValue( const char* name, ParamValue value )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		param_vector[loc].value = value;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool paramWriteLong( const char* name, U32 data )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		param_vector[loc].value.word = data;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool paramWriteFloat(  const char* name, float data )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		param_vector[loc].value.f = data;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool paramReadValue( const char* name, ParamValue* value )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		*value = param_vector[loc].value;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool paramReadLong( const char* name, U32 *data )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		*data = param_vector[loc].value.word;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool paramReadFloat( const char* name, float *data )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		*data = param_vector[loc].value.f;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

float paramReadFloatWithDefault( const char* name, float default_value )
{
	U32 loc = paramFind( name );
	
	if( loc < PARAM_NUM )
	{
		return param_vector[loc].value.f;
	}
	else
	{
		return default_value;
	}
}

void FlashWritePage( U32 address, U8 * data  )
{
	U32 i = 0;
	U32 pAddress = (U32)address;
	U32* pData = (U32*)data;

	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();

  	/* Clear All pending flags */
  	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  	/* Erase the FLASH pages */
	FLASH_ErasePage((U32)address);

	for(i=0; i<FLASH_PAGE_SIZE; i+=4)
	{
		FLASH_ProgramWord(pAddress, *pData);
		if(*(U32*)pAddress != *pData)
		{
			printf("Flash write error, return.\r\n");
			DebugPrintfDMA();
			return;
		}
		pAddress += 4;
		pData +=1;
	}	

	FLASH_Lock();

}

void FlashReadPage( U32 address, U8 * data )
{
	U32 i = 0;
	U32 pAddress = (U32)address;
	U32* pData = (U32*)data;

	for(i=0; i<FLASH_PAGE_SIZE; i+=4)
	{
		*pData = *(U32*)pAddress;
		pAddress += 4;
		pData +=1;
	}
}

bool FlashWriteWord( U32 address, U32 data)
{
	FLASH_ProgramWord( address, data);
	if(*(U32*)address != data)
	{
		printf("Flash write error, return.\r\n");
		DebugPrintfDMA();
		return FALSE;
	}

	return TRUE;
}

U32 FlashReadWord( U32 address )
{
	return *(U32*)address;
}

/*
void testParamWord(void)
{
	U32 i = 0; 
	U8 bytes[16] = {0};
	U32 *pdata = (U32*)bytes;
	U32 start_address = PARAM_START_ADDRESS; // page 124

	// Read
	for( i = 0; i < 4; i++)
		*(pdata+i) = FlashReadWord( start_address + i*4  );
	printf("Read 4 words from flash:\r\n");
	DebugPrintf();
	for( i = 0; i < 16; i++)
	{
		printf(" %d", bytes[i] );
	}
	printf("\r\n");
	DebugPrintf();
	delayms2(500);

	// Write
	bytes[0] -= 10;
	bytes[1] -= 20;
	bytes[2] -= 30;
	bytes[3] -= 40;
	*(pdata+1) = 0x12345678;
	printf("Write 4 words to flash:\r\n");
	DebugPrintf();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	FLASH_ErasePage( start_address );
	for( i = 0; i < 4; i++)
	{
		FlashWriteWord( start_address + i*4, *(pdata+i) );
	}
	FLASH_Lock();
	// Info
	for( i = 0; i < 16; i++)
	{
		printf(" %d", bytes[i] );
	}
	printf("\r\n");
	DebugPrintf();
	delayms2(500);

	// Read
	for( i = 0; i < 4; i++)
		*(pdata+i) = FlashReadWord( start_address + i*4  );
	// Info
	printf("Read 4 words from flash:\r\n");
	DebugPrintf();
	for( i = 0; i < 16; i++)
	{
		printf(" %d", bytes[i] );
	}
	printf("\r\n");
	DebugPrintf();
	delayms2(500);
	
}

void testParamPage(void)
{
	U32 i = 0;
	U8 bytes[FLASH_PAGE_SIZE] = {0};

	// Read
	FlashReadPage( PARAM_START_ADDRESS, bytes);
	printf("Read flash data:\r\n");
	DebugPrintf();
	for( i = 0; i < FLASH_PAGE_SIZE; i++)
	{
		if( i % 50 == 49)
		{
			printf(" %X\r\n", bytes[i] );
			DebugPrintf();
		}
		else
		{
			printf(" %X", bytes[i] );
		}
	}
	printf("\r\n");
	DebugPrintf();

	bytes[0] -= 0x04;
	bytes[1] -= 0x08;
	bytes[2] -= 0x0C;
	bytes[3] -= 0x10;

	// Write
	printf("Write Flash:\r\n");
	FlashWritePage( PARAM_START_ADDRESS, bytes);
	delayms2(500);
	printf("Done.\n");
	DebugPrintf();
	printf("Write flash data:\r\n");
	for( i = 0; i < FLASH_PAGE_SIZE; i++)
	{
		if( i % 50 == 49)
		{
			printf(" %X\r\n", bytes[i] );
			DebugPrintf();
		}
		else
		{
			printf(" %X", bytes[i] );
		}
	}
	printf("\r\n");
	DebugPrintf();

	// Read
	FlashReadPage( PARAM_START_ADDRESS, bytes);
	printf("Read flash data:\r\n");
	for( i = 0; i < FLASH_PAGE_SIZE; i++)
	{
		if( i % 50 == 49)
		{
			printf(" %X\r\n", bytes[i] );
			DebugPrintf();
		}
		else
		{
			printf(" %X", bytes[i] );
		}
	}
	printf("\r\n");
	DebugPrintf();
	
}
*/

