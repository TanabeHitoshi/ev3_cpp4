/******************************************************************************
 *  BalancerCpp.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef ISSONAR_H_
#define ISSONAR_H_

#include "ev3api.h"

class Sonar {
public:
    Sonar();

	void ini(void);				//�����g�Z���T�[�̏����ݒ�
	void getDistance(void);		// �����g�Z���T�[�̒l
	int16_t Distance(void);			// �����g�Z���T�[����̋���

private:
	int16_t mSonarDistance;					// ����

};
#endif  // EV3_UNIT_BALANCERCPP_H_
