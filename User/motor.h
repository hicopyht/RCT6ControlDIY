#ifndef _MOTOR_H
#define _MOTOR_H

#include "pid.h"

#define MOTOR_LEFT_STOP			{PC2_LOW;PC3_LOW;}
#define MOTOR_LEFT_FORWARD		{PC2_HIGH;PC3_LOW;}
#define MOTOR_LEFT_BACKWARD		{PC2_LOW;PC3_HIGH;}
#define MOTOR_RIGHT_STOP			{PC4_LOW;PC5_LOW;}
#define MOTOR_RIGHT_FORWARD		{PC4_HIGH;PC5_LOW;}
#define MOTOR_RIGHT_BACKWARD		{PC4_LOW;PC5_HIGH;}

#define TB6612_Disable	(PD2_LOW)
#define TB6612_Enable	(PD2_HIGH)

//电机控制PWM输出定义
#define MOTOR_LEFT_PWM(x)		PWML_Output(x)
#define MOTOR_RIGHT_PWM(x)	PWMR_Output(x)
//电机速度控制相关定义
#define MOTOR_NUM			2
#define MOTOR_LEFT			0
#define MOTOR_RIGHT		1
#define MOTOR_PWM_MAX		300	//(900 - MIN_MOTOR_SPEED)
#define MOTOR_PWM_MIN		(-300)
#define MOTOR_PWM_DEADZONE	20
#define MOTOR_PWM_ACCEL		7
#define MOTOR_PWM_DECEL		7
//电机控制参数	
#define MOTOR_CONTROL_CYCLE	20
#define MOTOR_CMD_TIMEOUT		(0.5)
//
#define VELOCITY_MAX		(19.8)
#define VELOCITY_MIN		(-19.8)
#define VELOCITY_DEADZONE	(0.2)
#define VELOCITY_ACCEL	(0.5)
#define VELOCITY_DECEL	(0.5)
//
#define VELOCITY_PID_P_GAIN		(1.6)
#define VELOCITY_PID_I_GAIN		(0.2)
#define VELOCITY_PID_D_GAIN		(0.6)
#define VELOCITY_PID_CLAMP_GAIN	(20.0)
//
#define ENCODER_READ_CYCLE		20
#define ENCODER_PULSES				3120
#define ENCODER_MAX_VALUE			10000
#define ENCODER_OVERFLOW_VALUE	65536


struct SPEED_DATA
{
	float left_value;	// current
	float right_value;
	float left_buff;	// target
	float right_buff;
	float max;
	float min;
	float deadzone;
	float accel;
	float decel;
	float timeout;
	struct TIME_STAMP stamp;
};

struct MOTOR_DATA
{
	S16	pwm_value;//当前速度值
	S16 pwm_buff;	//给定速度值
	S16 acceleration;//加速度
	S16 deceleration;//减速度
	S16 pwm_min;//最小速度
	S16 pwm_max;//最大速度
	S16 pwm_deadzone;
	S16 pwm_real_output;
	U8 is_output;
};			

struct VELOCITY_PID
{
	float velocity_value;
	float velocity_feed;
	S16 pwm_output;
	S16 pwm_deadzone;
	U16 encoder;
	U16 encoder_last;
	float dt;
	struct PidData pid;
};

extern U8 encoder_cycle;
extern U8 controller_cycle;
extern U8 is_control_motor;
extern struct PidGain PidBuff;
extern struct SPEED_DATA VelocityBuff;
extern struct SPEED_DATA Velocity;
extern struct MOTOR_DATA MOTOR[MOTOR_NUM];//电机加速度控制结构体
extern struct VELOCITY_PID VelocityPid[MOTOR_NUM];
// Encoder Reading
/*
extern U16 encoder_left_value;
extern U16 encoder_right_value;
extern double encoder_stamp;
*/

void Motor_Init(void);
void MotorData_Init(void);
void Motor_Control(void);
void calVelocity( struct SPEED_DATA * velocity );
void setSpeed( struct MOTOR_DATA* motor, S16 pwm );
void calSpeed( struct MOTOR_DATA* motor );
void outputSpeed(void);
S16 computeEncoderDelta( U16 encoder, U16 encoder_last );
void computeVelocityPidOutput( struct VELOCITY_PID *velocity_pid);

#endif /* __MOTOR_H */

