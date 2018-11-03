/******************************************************************************
 **	�t�@�C���� : viewer.h
 **
 ** Copyright (c) 2009-2011 MathWorks, Inc.
 ** All rights reserved.
 ******************************************************************************/
#ifndef ISPOSITION_H
#define ISPOSITION_H

#include "ev3api.h"


class Position {
public:
    Position();
	void reset(void);	/* ���[�^���[�G���R�[�_���O�ɂ��� */
	int32_t tripmeter(void);		// ��ԋ����𑪒肷��
	void tripmeter_set(void);	//��ԋ����̑���ʒu
	int32_t odometer(void);			// �T�v : �G���R�[�_�[�ő��s�����𑪒肷��
	int isTurnAngle(void);		//��]�p�x���v������

private:
	int32_t tripmeter_pre;
	int32_t count_left_pre;
	int32_t count_right_pre;
};
#endif  // EV3_UNIT_BALANCERCPP_H_
