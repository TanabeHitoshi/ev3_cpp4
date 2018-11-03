/******************************************************************************
 **	�t�@�C���� : Motor.h
 **
 ** Copyright (c) 2009-2011 MathWorks, Inc.
 ** All rights reserved.
 ******************************************************************************/
#ifndef DRIVE_H
#define DRIVE_H

#include "ev3api.h"
static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B;

enum State {							//�o�����X���s�̗L��
		BALANCE_ON,						//�o�����X���s
		BALANCE_TURN,					//�o�����X���Ƃ�Ȃ����]
		TAIL_ON,						//���K�ő��s
		TAIL_GATE,						//�Q�[�g�ʉ�
		TAIL_TURN,						//���K�ŉ�]
		STRAIGHT						//�܂������ɐi��
	};

class Drive {
public:
    Drive();
	void run(signed char, signed char);
	void balance_line_trace(void); //�o�����X���s
	void balance_turn(void);		//�o�����X�Ń^�[��
	void tail_line_trace(void); //���K���s
	void tail_line_gate(void); //���K���s
	void tail_turn(void);			//���K�Ń^�[��
	void straight_run(void);		// �^�������ɐi��
	void trace(signed char forward, signed char turn);
	int MODE;

//	int BALANCE = 0;
//	int TAIL = 1;

	State mState;							//���s���
private:

};
#endif  // EV3_UNIT_BALANCERCPP_H_
