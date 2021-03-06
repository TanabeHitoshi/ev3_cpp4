/******************************************************************************
 **	t@C¼ : Motor.h
 **
 ** Copyright (c) 2009-2011 MathWorks, Inc.
 ** All rights reserved.
 ******************************************************************************/
#ifndef TAIL_H
#define TAIL_H

#include "ev3api.h"
static const motor_port_t
    tail_motor      = EV3_PORT_A;

class Tail {
public:
    Tail();
	void Set_PID(void);		// PIDðZbg·é
	void Stop(void);		// öKðâ~³¹é
	void Angle_Set(signed int angle);	//öKÌpxZbg
	void Control(void);		// öKÌ§ä
private:
	float P_GAIN;				/* ®Sâ~p[^§ääáW */
	float I_GAIN;				/* ®Sâ~p[^§ääáW */
	float D_GAIN;				/* ®Sâ~p[^§ääáW */
	signed int mTailAngle;		//öKÌ§äpx
};
#endif  // EV3_UNIT_BALANCERCPP_H_
