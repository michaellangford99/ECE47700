#include "stm32f4xx.h"
#include "string.h"
#include "systick.h"
#include "math.h"
#include "pid.h"


void init_PID(struct PID* pid)
{
	pid->D_Error=0;
	pid->kP = 0.0f;
	pid->kI = 0.0f;
	pid->kD = 0.0f;
	pid->Error = 0.0f;
	pid->Last_Error = 0.0f;
	pid->P_Error = 0.0f;
	pid->I_Error = 0.0f;
	pid->D_Error = 0.0f;
	pid->Output = 0.0f;
	pid->Last_Output = 0.0f;
	pid->Delta_Output = 0.0f;
	pid->maxP = 0.2f;
	pid->maxI = 0.2f;
	pid->maxD = 0.2f;
}

float bound(float value, float abs_max)
{
	if (value < -abs_max)
		return -abs_max;
	if (value > abs_max)
		return abs_max;
	return value;
}

float update_PID(struct PID* pid, float real, float desired)
{
	pid->Error = desired - real;

	pid->P_Error = bound( pid->Error * pid->kP, pid->maxP);
	pid->I_Error = bound( pid->I_Error + (pid->Error)*pid->kI, pid->maxI);
	pid->D_Error = bound((pid->Error - pid->Last_Error)*pid->kD, pid->maxD);

	pid->Output =  pid->P_Error;
	pid->Output += pid->I_Error;
	pid->Output += pid->D_Error;

	//Delta_Output = Output - Last_Output;

	//Delta_Output */

	pid->Last_Error = pid->Error;
	pid->Last_Output = pid->Output;

	return pid->Output;
}
/*
void PIDClass::clear_D_Error(float real, float desired)
{
	Last_Error = desired - real;
}*/

void set_PID_bounds(struct PID* pid, float P, float I, float D)
{
	pid->maxP = P;
	pid->maxI = I;
	pid->maxD = D;
}

void set_PID_constants(struct PID* pid, float KP, float KI, float KD)
{
	pid->kP = KP;
	pid->kI = KI;
	pid->kD = KD;
}

