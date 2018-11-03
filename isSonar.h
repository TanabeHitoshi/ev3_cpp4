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

	void ini(void);				//超音波センサーの初期設定
	void getDistance(void);		// 超音波センサーの値
	int16_t Distance(void);			// 超音波センサーからの距離

private:
	int16_t mSonarDistance;					// 距離

};
#endif  // EV3_UNIT_BALANCERCPP_H_
