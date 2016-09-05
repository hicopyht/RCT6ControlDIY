#include "pid.h"

void deInitPid( struct PidData * pid )
{
	//
	pid->gains.p_gain = 1.0;
	pid->gains.i_gain = 0;
	pid->gains.d_gain = 0;
	pid->gains.i_max = 0;
	pid->gains.i_min = 0;
	//
	pid->p_error_last = 0;
	pid->p_error = 0;
	pid->i_error = 0;
	pid->d_error = 0;
	pid->command = 0;
}

void initPid( struct PidData * pid, float p, float i, float d, float i_max, float i_min )
{
	//
	pid->gains.p_gain = p;
	pid->gains.i_gain = i;
	pid->gains.d_gain = d;
	pid->gains.i_max = i_max;
	pid->gains.i_min = i_min;
	//
	pid->p_error_last = 0;
	pid->p_error = 0;
	pid->i_error = 0;
	pid->d_error = 0;
	pid->command = 0;
}

void computeCommand( struct PidData * pid, float error, float dt )
{
	float p_term, d_term, i_term;

	if( dt <= 0 || error > 1e9 || error < -1e9 )
		return;

	pid->p_error = error;
	pid->d_error = (error - pid->p_error_last) / dt;
	pid->p_error_last = error;

	if( pid->d_error > 1e9 || pid->d_error < -1e9 )
		return;

	// Calculate proportional contribution to command
	p_term = pid->gains.p_gain * pid->p_error;

	// Calculate the integral of the position error
	pid->i_error += dt * pid->p_error;

	// Calculate integral contribution to command
	i_term = pid->gains.i_gain * pid->i_error;

	// Limit i_term so that the limit is meaningful in the output
	if( i_term > pid->gains.i_max )
		i_term = pid->gains.i_max;
	if( i_term < pid->gains.i_min )
		i_term = pid->gains.i_min;

	// Calculate derivative contribution to command
	d_term = pid->gains.d_gain * pid->d_error;

	// Compute the command
	pid->command = p_term + i_term + d_term;

}
