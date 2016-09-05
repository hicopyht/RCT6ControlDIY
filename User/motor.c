#include "Configuration.h"

U8 encoder_cycle = ENCODER_READ_CYCLE;
const float rad_per_pulse = PI * 2.0 / ENCODER_PULSES;
U8 controller_cycle = MOTOR_CONTROL_CYCLE;	//
U8 is_control_motor = FALSE;	//
struct PidGain PidBuff;
struct SPEED_DATA VelocityBuff;
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
	VelocityBuff.velocity_deadzone = paramReadFloatWithDefault( "velocity_deadzone", VELOCITY_DEADZONE );
	//
	MOTOR[MOTOR_LEFT].speed_deadzone= paramReadFloatWithDefault( "pwm_deadzone", MOTOR_PWM_DEADZONE );
	MOTOR[MOTOR_RIGHT].speed_deadzone = paramReadFloatWithDefault( "pwm_deadzone", MOTOR_PWM_DEADZONE );
	//
	PidBuff.p_gain = paramReadFloatWithDefault( "velocity_pid_p", VELOCITY_PID_P_GAIN );
	PidBuff.i_gain = paramReadFloatWithDefault( "velocity_pid_i", VELOCITY_PID_I_GAIN );
	PidBuff.d_gain = paramReadFloatWithDefault( "velocity_pid_d", VELOCITY_PID_D_GAIN );
	PidBuff.i_max = paramReadFloatWithDefault( "velocity_pid_clamp", VELOCITY_PID_CLAMP_GAIN );
	PidBuff.i_min = -PidBuff.i_max;
	//
	VelocityPid[MOTOR_LEFT].pid.gains = PidBuff;
	VelocityPid[MOTOR_RIGHT].pid.gains = PidBuff;

	
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

	VelocityBuff.velocity_left = 0;
	VelocityBuff.velocity_right = 0;
	VelocityBuff.velocity_max = VELOCITY_MAX;
	VelocityBuff.velocity_min = VELOCITY_MIN;
	VelocityBuff.velocity_deadzone = VELOCITY_DEADZONE;
	VelocityBuff.stamp = 0;
	VelocityBuff.timeout = MOTOR_CMD_TIMEOUT;

	initPid( &(VelocityPid[MOTOR_LEFT].pid), VELOCITY_PID_P_GAIN, VELOCITY_PID_I_GAIN, VELOCITY_PID_D_GAIN, VELOCITY_PID_CLAMP_GAIN, -VELOCITY_PID_CLAMP_GAIN);
	initPid( &(VelocityPid[MOTOR_RIGHT].pid), VELOCITY_PID_P_GAIN, VELOCITY_PID_I_GAIN, VELOCITY_PID_D_GAIN, VELOCITY_PID_CLAMP_GAIN, -VELOCITY_PID_CLAMP_GAIN);
	
	for(i=0;i<MOTOR_NUM;i++)
	{
		//速度输出结构体初始化
		MOTOR[i].speed_buff= 0;
		MOTOR[i].speed_value = 0;
		MOTOR[i].pwm_value = 0;
		MOTOR[i].acceleration = MOTOR_ACCELERATION;
		MOTOR[i].deceleration = MOTOR_DECELERATION;
		MOTOR[i].speed_max = MOTOR_PWM_MAX;
		MOTOR[i].speed_min = MOTOR_PWM_MIN;
		MOTOR[i].speed_deadzone = MOTOR_PWM_DEADZONE;
		MOTOR[i].is_output = TRUE;

		VelocityPid[i].velocity_value = 0;
		VelocityPid[i].velocity_feed = 0;
		VelocityPid[i].pwm_output = 0;
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
	static double stamp_last = 0;
	double stamp;
	float dt;

	// Reading Data
	//stamp = encoder_stamp;
	//VelocityPid[MOTOR_LEFT].encoder= encoder_left_value;
	//VelocityPid[MOTOR_RIGHT].encoder = encoder_left_value;
	stamp = getTimeStamp();;
	VelocityPid[MOTOR_LEFT].encoder= readEncoderL();
	VelocityPid[MOTOR_RIGHT].encoder = readEncoderR();

	// Read gyro measurement
	Read_ITG3205_Z();

	// Send encoders and gyroscope measurement
	SendEncodersAndGyro( VelocityPid[MOTOR_LEFT].encoder, VelocityPid[MOTOR_RIGHT].encoder, gyro_raw.z );

	// Delta time
	dt =  stamp - stamp_last;
	VelocityPid[MOTOR_RIGHT].dt = VelocityPid[MOTOR_LEFT].dt = dt;
	stamp_last = stamp;

	// Set speed from buffer
	if( stamp - VelocityBuff.stamp > VelocityBuff.timeout )
	{
		VelocityPid[MOTOR_LEFT].velocity_value = 0;
		VelocityPid[MOTOR_RIGHT].velocity_value = 0;
	}
	else
	{
		VelocityPid[MOTOR_LEFT].velocity_value = VelocityBuff.velocity_left;
		VelocityPid[MOTOR_RIGHT].velocity_value = VelocityBuff.velocity_right;
	}

	if( dt > 0.2 )
		return;

	// Velocity pid
	computeVelocityPidOutput( &VelocityPid[MOTOR_LEFT] );
	computeVelocityPidOutput( &VelocityPid[MOTOR_RIGHT] );

	// Set PWM value
	setSpeed( &MOTOR[MOTOR_LEFT], VelocityPid[MOTOR_LEFT].pwm_output );
	setSpeed( &MOTOR[MOTOR_RIGHT], VelocityPid[MOTOR_RIGHT].pwm_output );
	
	// Limit acceleration
	calSpeed( &MOTOR[MOTOR_LEFT] );
	calSpeed( &MOTOR[MOTOR_RIGHT] );
	outputSpeed();
	
	//printf("\r\n MS\t%d\t%d", MOTOR[0].pwm_value, MOTOR[1].pwm_value);
	//printf("Dt = %f, GZ = %d \r\n", dt, gyro_raw.z );
	//DebugPrintfDMA();
}

void setSpeed( struct MOTOR_DATA* motor, S16 speed )
{
	// Check speed range
	if( speed > motor->speed_max ) 
		motor->speed_buff = motor->speed_max;
	else if( speed < motor->speed_min )
		motor->speed_buff = motor->speed_min;
	else
		motor->speed_buff = speed;
		
}


/********************************************************************************
 *函数原型: void Cal_Speed(void)                  	     
 *参数说明: 	无
 *返回值:    无                                                              
 *说明:      电机输出速度计算                                    
 ********************************************************************************/
void calSpeed( struct MOTOR_DATA* motor )
{
	if( motor->speed_value >= 0 ) // forward
	{
		if( (motor->speed_buff - motor->speed_value) > motor->acceleration ) 		// acceleration
			motor->speed_value += motor->acceleration;
		else if( (motor->speed_value - motor->speed_buff) > motor->deceleration )	// decleration
			motor->speed_value -= motor->deceleration;
		else
			motor->speed_value = motor->speed_buff;
	}
	else
	{
		if( (motor->speed_value - motor->speed_buff) > motor->acceleration ) 		// acceleration
			motor->speed_value -= motor->acceleration;
		else if( (motor->speed_buff - motor->speed_value) > motor->deceleration )	// decleration
			motor->speed_value += motor->deceleration;
		else
			motor->speed_value = motor->speed_buff;
	}

	// PWM limit
	if( motor->speed_value > motor->speed_max ) 
		motor->speed_value = motor->speed_max;
	else if( motor->speed_value < motor->speed_min )
		motor->speed_value = motor->speed_min;
}


void outputSpeed(void)
{
	static S16 pwm_left = 0;
	static S16 pwm_right = 0;

	// LEFT
	if( MOTOR[MOTOR_LEFT].speed_value > MOTOR[MOTOR_LEFT].speed_deadzone)		// FORWARD
	{
		MOTOR_LEFT_PWM( MOTOR[MOTOR_LEFT].speed_value );
		MOTOR_LEFT_FORWARD;
		//
		MOTOR[MOTOR_LEFT].pwm_value = MOTOR[MOTOR_LEFT].speed_value;
	}
	else if( MOTOR[MOTOR_LEFT].speed_value < -MOTOR[MOTOR_LEFT].speed_deadzone)		// BACKWARD
	{
		MOTOR_LEFT_PWM( -MOTOR[MOTOR_LEFT].speed_value );
		MOTOR_LEFT_BACKWARD;
		//
		MOTOR[MOTOR_LEFT].pwm_value = MOTOR[MOTOR_LEFT].speed_value;
	}
	else		// STOP
	{
		MOTOR_LEFT_PWM(0);
		MOTOR_LEFT_STOP;
		//
		MOTOR[MOTOR_LEFT].pwm_value = 0;
	}

	// RIGHT
	if( MOTOR[MOTOR_RIGHT].speed_value > MOTOR[MOTOR_RIGHT].speed_deadzone )		// FORWARD
	{
		MOTOR_RIGHT_PWM( MOTOR[MOTOR_RIGHT].speed_value );
		MOTOR_RIGHT_FORWARD;
		//
		MOTOR[MOTOR_RIGHT].pwm_value = MOTOR[MOTOR_RIGHT].speed_value;
	}
	else if( MOTOR[MOTOR_RIGHT].speed_value < -MOTOR[MOTOR_RIGHT].speed_deadzone)	// BACKWARD
	{
		MOTOR_RIGHT_PWM( -MOTOR[MOTOR_RIGHT].speed_value );
		MOTOR_RIGHT_BACKWARD;
		//
		MOTOR[MOTOR_RIGHT].pwm_value = MOTOR[MOTOR_RIGHT].speed_value;
	}
	else		// STOP
	{
		MOTOR_RIGHT_PWM(0);
		MOTOR_RIGHT_STOP;
		//
		MOTOR[MOTOR_RIGHT].pwm_value = 0;
	}
/*
#if defined (CONFIG_DEBUG)
	if( pwm_left != MOTOR[MOTOR_LEFT].pwm_value || pwm_right != MOTOR[MOTOR_RIGHT].pwm_value )
	{
		printf("PWM: L = %d, \tR = %d\n", MOTOR[MOTOR_LEFT].pwm_value, MOTOR[MOTOR_RIGHT].pwm_value);
		DebugPrintfDMA();
		pwm_left = MOTOR[MOTOR_LEFT].pwm_value;
		pwm_right = MOTOR[MOTOR_RIGHT].pwm_value;
	}
#endif
*/

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

