/******************************************************************************
 **	�t�@�C���� : Motor.h
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
	void Set_PID(void);		// PID���Z�b�g����
	void Stop(void);		// ���K���~������
	void Angle_Set(signed int angle);	//���K�̊p�x�Z�b�g
	void Control(void);		// ���K�̐���
private:
	float P_GAIN;				/* ���S��~�p���[�^������W�� */
	float I_GAIN;				/* ���S��~�p���[�^������W�� */
	float D_GAIN;				/* ���S��~�p���[�^������W�� */
	signed int mTailAngle;		//���K�̐���p�x
};
#endif  // EV3_UNIT_BALANCERCPP_H_
