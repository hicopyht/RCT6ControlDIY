#include "Configuration.h"

U8 encoder_cycle = ENCODER_READ_CYCLE;
const float rad_per_pulse = PI * 2.0 / ENCODER_PULSES;
U8 controller_cycle = MOTOR_CONTROL_CYCLE;	//
U8 is_control_motor = FALSE;	//
struct PidGain PidBuff;
struct SPEED_DATA VelocityBuff;
struct SPEED_DATA Velocity;
struct MOTOR_DATA MOTOR[MOTOR_NUM];	//电机加速度控制结构体
//
struct VELOCITY_PID VelocityPid[MOTOR_NUM];

// Encoder Reading
/*
U16 encoder_left_value = 0;
U16 encoder_right_value = 0;
double encoder_stamp = 0;
*/

/********************************************************************************
 *函数原型:  void Motor_Init(void)            	     
 *参数说明: 	无
 *返回值:    无                                                             
 *说明:      初始化与电机控制有关外设及变量                                   
 ********************************************************************************/
void Motor_Init(void)
{
	MotorData_Init();
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
	
	
	TB6612_Enable;
	
#if defined (CONFIG_DEBUG)
	printf("Initializing Motors ... Enable ... Done.\r\n");
	DebugPrintf();
#endif

}

/********************************************************************************
 *函数原型:  void MotorData_Init(void)                    	     
 *参数说明: 	无
 *返回值:    无                                                             
 *说明:      初始化电机速度控制数组                                   
 ********************************************************************************/
void MotorData_Init(void)
{
	U8 i;

	// Velocity buffer for UART communication
	VelocityBuff.left_buff = 0;
	VelocityBuff.right_buff = 0;
	VelocityBuff.max = VELOCITY_MAX;
	VelocityBuff.min = VELOCITY_MIN;
	VelocityBuff.deadzone = VELOCITY_DEADZONE;
	VelocityBuff.timeout = MOTOR_CMD_TIMEOUT;
	VelocityBuff.stamp.sec = 0;
	VelocityBuff.stamp.usec = 0;
	//
	Velocity.left_buff = 0;
	Velocity.right_buff = 0;
	Velocity.left_value = 0;
	Velocity.right_value = 0;
	Velocity.max = VELOCITY_MAX;
	Velocity.min = VELOCITY_MIN;
	Velocity.deadzone = VELOCITY_DEADZONE;
	Velocity.accel = VELOCITY_ACCEL;
	Velocity.decel = VELOCITY_DECEL;
	Velocity.timeout = MOTOR_CMD_TIMEOUT;
	Velocity.stamp.sec = 0;
	Velocity.stamp.usec = 0;
	
	initPid( &(VelocityPid[MOTOR_LEFT].pid), VELOCITY_PID_P_GAIN, VELOCITY_PID_I_GAIN, VELOCITY_PID_D_GAIN, VELOCITY_PID_CLAMP_GAIN, -VELOCITY_PID_CLAMP_GAIN);
	initPid( &(VelocityPid[MOTOR_RIGHT].pid), VELOCITY_PID_P_GAIN, VELOCITY_PID_I_GAIN, VELOCITY_PID_D_GAIN, VELOCITY_PID_CLAMP_GAIN, -VELOCITY_PID_CLAMP_GAIN);
	
	for(i=0;i<MOTOR_NUM;i++)
	{
		//速度输出结构体初始化
		MOTOR[i].pwm_buff= 0;
		MOTOR[i].pwm_value = 0;
		MOTOR[i].acceleration = MOTOR_PWM_ACCEL;
		MOTOR[i].deceleration = MOTOR_PWM_DECEL;
		MOTOR[i].pwm_max = MOTOR_PWM_MAX;
		MOTOR[i].pwm_min = MOTOR_PWM_MIN;
		MOTOR[i].pwm_deadzone = MOTOR_PWM_DEADZONE;
		MOTOR[i].pwm_real_output = 0;
		MOTOR[i].is_output = TRUE;

		VelocityPid[i].velocity_value = 0;
		VelocityPid[i].velocity_feed = 0;
		VelocityPid[i].pwm_output = 0;
		VelocityPid[i].pwm_deadzone = MOTOR_PWM_DEADZONE;
		VelocityPid[i].encoder = 0;
		VelocityPid[i].encoder_last = 0;
		VelocityPid[i].dt = 0;
	}

}


/********************************************************************************
 *函数原型:  void Motor_Control(void)           	     
 *参数说明: 	无
 *返回值:    无                                                         
 *说明:      电机速度控制                                      
 ********************************************************************************/
void Motor_Control(void)
{
	static struct TIME_STAMP stamp_last;
	struct TIME_STAMP stamp;
	float dt;

	// Reading Data
	//stamp = encoder_stamp;
	//VelocityPid[MOTOR_LEFT].encoder= encoder_left_value;
	//VelocityPid[MOTOR_RIGHT].encoder = encoder_left_value;
	getTimeStamp( &stamp );;
	VelocityPid[MOTOR_LEFT].encoder= readEncoderL();
	VelocityPid[MOTOR_RIGHT].encoder = readEncoderR();

	// Send encoders and gyroscope measurement
	SendEncodersAndGyro( VelocityPid[MOTOR_LEFT].encoder, VelocityPid[MOTOR_RIGHT].encoder, gyro_raw.z );

	// Delta time
	dt =  deltaTimeStampFloat(&stamp, &stamp_last);;
	VelocityPid[MOTOR_RIGHT].dt = VelocityPid[MOTOR_LEFT].dt = dt;
	copyTimeStamp( stamp, stamp_last );

	// Set speed from buffer
	if( deltaTimeStampDouble( &stamp, &(VelocityBuff.stamp)) > VelocityBuff.timeout )
	{
		Velocity.left_buff = 0;
		Velocity.right_buff = 0;
	}
	else
	{
		Velocity.left_buff = VelocityBuff.left_buff;
		Velocity.right_buff = VelocityBuff.right_buff;
	}

	if( dt > 0.2 )	// Encoder timeout
		return;

	calVelocity( &Velocity );


	// Set velocity pid target
	VelocityPid[MOTOR_LEFT].velocity_value = Velocity.left_value;
	VelocityPid[MOTOR_RIGHT].velocity_value = Velocity.right_value;

	// Compute velocity pid
	computeVelocityPidOutput( &VelocityPid[MOTOR_LEFT] );
	computeVelocityPidOutput( &VelocityPid[MOTOR_RIGHT] );

	// Set PWM value
	setSpeed( &MOTOR[MOTOR_LEFT], VelocityPid[MOTOR_LEFT].pwm_output );
	setSpeed( &MOTOR[MOTOR_RIGHT], VelocityPid[MOTOR_RIGHT].pwm_output );

	// Limit acceleration
	//calSpeed( &MOTOR[MOTOR_LEFT] );
	//calSpeed( &MOTOR[MOTOR_RIGHT] );
	MOTOR[MOTOR_LEFT].pwm_value = MOTOR[MOTOR_LEFT].pwm_buff;
	MOTOR[MOTOR_RIGHT].pwm_value = MOTOR[MOTOR_RIGHT].pwm_buff;

	
	outputSpeed();
	//printf("\r\n MS\t%d\t%d", MOTOR[0].pwm_value, MOTOR[1].pwm_value);
	//printf("Dt = %f, GZ = %d \r\n", dt, gyro_raw.z );
	//DebugPrintfDMA();
}

void calVelocity( struct SPEED_DATA * velocity )
{
	if( velocity->left_value > 0 )	// forward
	{
		if( velocity->left_buff - velocity->left_value > velocity->accel )	// acceleration
			velocity->left_value += velocity->accel;
		else if( velocity->left_value - velocity->left_buff > velocity->decel )	// deceleration
			velocity->left_value -= velocity->decel;
		else
			velocity->left_value = velocity->left_buff;
	}
	else		// backward
	{
		if( velocity->left_value - velocity->left_buff > velocity->accel )	// acceleration
			velocity->left_value -= velocity->accel;
		else if( velocity->left_buff - velocity->left_value > velocity->decel )	// deceleration
			velocity->left_value += velocity->decel;
		else
			velocity->left_value = velocity->left_buff;
	}
	
	if( velocity->right_value > 0 )	// forward
	{
		if( velocity->right_buff - velocity->right_value > velocity->accel )	// acceleration
			velocity->right_value += velocity->accel;
		else if( velocity->right_value - velocity->right_buff > velocity->decel )	// deceleration
			velocity->right_value -= velocity->decel;
		else
			velocity->right_value = velocity->right_buff;
	}
	else		// backward
	{
		if( velocity->right_value - velocity->right_buff > velocity->accel )	// acceleration
			velocity->right_value -= velocity->accel;
		else if( velocity->right_buff - velocity->right_value > velocity->decel )	// deceleration
			velocity->right_value += velocity->decel;
		else
			velocity->right_value = velocity->right_buff;
	}
	
}

void setSpeed( struct MOTOR_DATA* motor, S16 pwm )
{
	// Check speed range
	if( pwm > motor->pwm_max ) 
		motor->pwm_buff = motor->pwm_max;
	else if( pwm < motor->pwm_min )
		motor->pwm_buff = motor->pwm_min;
	else
		motor->pwm_buff = pwm;
}


/********************************************************************************
 *函数原型: void Cal_Speed(void)                  	     
 *参数说明: 	无
 *返回值:    无                                                              
 *说明:      电机输出速度计算                                    
 ********************************************************************************/
void calSpeed( struct MOTOR_DATA* motor )
{
	if( motor->pwm_value >= 0 ) // forward
	{
		if( (motor->pwm_buff - motor->pwm_value) > motor->acceleration ) 		// acceleration
			motor->pwm_value += motor->acceleration;
		else if( (motor->pwm_value - motor->pwm_buff) > motor->deceleration )	// decleration
			motor->pwm_value -= motor->deceleration;
		else
			motor->pwm_value = motor->pwm_buff;
	}
	else
	{
		if( (motor->pwm_value - motor->pwm_buff) > motor->acceleration ) 		// acceleration
			motor->pwm_value -= motor->acceleration;
		else if( (motor->pwm_buff - motor->pwm_value) > motor->deceleration )	// decleration
			motor->pwm_value += motor->deceleration;
		else
			motor->pwm_value = motor->pwm_buff;
	}

	// PWM limit
	limitMinMax( motor->pwm_value, motor->pwm_min, motor->pwm_max );
}


void outputSpeed(void)
{
	static S16 pwm_left = 0;
	static S16 pwm_right = 0;

	// LEFT
	if( MOTOR[MOTOR_LEFT].pwm_value > MOTOR[MOTOR_LEFT].pwm_deadzone)		// FORWARD
	{
		MOTOR_LEFT_PWM( MOTOR[MOTOR_LEFT].pwm_value );
		MOTOR_LEFT_FORWARD;
		//
		MOTOR[MOTOR_LEFT].pwm_real_output = MOTOR[MOTOR_LEFT].pwm_value;
	}
	else if( MOTOR[MOTOR_LEFT].pwm_value < -MOTOR[MOTOR_LEFT].pwm_deadzone)		// BACKWARD
	{
		MOTOR_LEFT_PWM( -MOTOR[MOTOR_LEFT].pwm_value );
		MOTOR_LEFT_BACKWARD;
		//
		MOTOR[MOTOR_LEFT].pwm_real_output = MOTOR[MOTOR_LEFT].pwm_value;
	}
	else		// STOP
	{
		MOTOR_LEFT_PWM(0);
		MOTOR_LEFT_STOP;
		//
		MOTOR[MOTOR_LEFT].pwm_real_output = 0;
	}

	// RIGHT
	if( MOTOR[MOTOR_RIGHT].pwm_value > MOTOR[MOTOR_RIGHT].pwm_deadzone )		// FORWARD
	{
		MOTOR_RIGHT_PWM( MOTOR[MOTOR_RIGHT].pwm_value );
		MOTOR_RIGHT_FORWARD;
		//
		MOTOR[MOTOR_RIGHT].pwm_real_output = MOTOR[MOTOR_RIGHT].pwm_value;
	}
	else if( MOTOR[MOTOR_RIGHT].pwm_value < -MOTOR[MOTOR_RIGHT].pwm_deadzone)	// BACKWARD
	{
		MOTOR_RIGHT_PWM( -MOTOR[MOTOR_RIGHT].pwm_value );
		MOTOR_RIGHT_BACKWARD;
		//
		MOTOR[MOTOR_RIGHT].pwm_real_output = MOTOR[MOTOR_RIGHT].pwm_value;
	}
	else		// STOP
	{
		MOTOR_RIGHT_PWM(0);
		MOTOR_RIGHT_STOP;
		//
		MOTOR[MOTOR_RIGHT].pwm_real_output= 0;
	}

	if( debug_motor )
	{
		if( pwm_left != MOTOR[MOTOR_LEFT].pwm_real_output|| pwm_right != MOTOR[MOTOR_RIGHT].pwm_real_output )
		{
			printf("PWM: L = %d, \tR = %d\r\n", MOTOR[MOTOR_LEFT].pwm_real_output, MOTOR[MOTOR_RIGHT].pwm_real_output);
			DebugPrintfDMA();
			pwm_left = MOTOR[MOTOR_LEFT].pwm_real_output;
			pwm_right = MOTOR[MOTOR_RIGHT].pwm_real_output;
		}
	}

}

S16 computeEncoderDelta( U16 encoder, U16 encoder_last )
{
	S16 delta = (S16)( encoder - encoder_last );
	if( delta > ENCODER_MAX_VALUE)
		delta -= ENCODER_OVERFLOW_VALUE;
	else if( delta < -ENCODER_MAX_VALUE )
		delta += ENCODER_OVERFLOW_VALUE;

	return delta;
}

void computeVelocityPidOutput( struct VELOCITY_PID *velocity_pid )
{
	S16 delta;
	float error;
	
	// Encoder delta
	delta = computeEncoderDelta( velocity_pid->encoder, velocity_pid->encoder_last );
	velocity_pid->encoder_last = velocity_pid->encoder;

	// Feedback speed
	velocity_pid->velocity_feed =delta * rad_per_pulse / velocity_pid->dt;	// rad / s

	// Error
	error = velocity_pid->velocity_value - velocity_pid->velocity_feed;

	// Compute pid command
	computeCommand( &(velocity_pid->pid), error, velocity_pid->dt );
	
	// Compute pwm output
	velocity_pid->pwm_output += velocity_pid->pid.command;

	// Limit pwm value
	if( velocity_pid->pwm_output > MOTOR_PWM_MAX )
		velocity_pid->pwm_output = MOTOR_PWM_MAX;
	if( velocity_pid->pwm_output < -MOTOR_PWM_MAX )
		velocity_pid->pwm_output = -MOTOR_PWM_MAX;
	
}

