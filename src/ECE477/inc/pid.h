
#ifndef __PID_H__
#define __PID_H__

struct PID
{
	float kP;
	float kI;
	float kD;
	float Error;
	float Last_Error;
	float P_Error;
	float I_Error;
	float D_Error;
	float Output;
	float Last_Output;
	float Delta_Output;
};

	void init_PID(struct PID* pid);
	float update_PID(struct PID* pid, float real, float desired);
	void set_PID_constants(struct PID* pid, float KP, float KI, float KD);

#endif
