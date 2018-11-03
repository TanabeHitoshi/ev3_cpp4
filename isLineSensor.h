/******************************************************************************
 *  BalancerCpp.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef ISLINESENSOR_H_
#define ISLINESENSOR_H_

#include "ev3api.h"

//#define BALANCE 0
//#define TAIL 1

class LineSensor {
public:
    LineSensor();

    void init(void);

	int mBlack[3];         /* 黒色の光センサ値 */
	int mWhite[3];         /* 白色の光センサ値 */
	char LR;				/* どちらのエッジか */

	float normanaization( int sensor_val, int s);
	void set_pid(float kp, float ki, float kd);
	float pid_control(float target_val, int s);

	void lineCalibration( int s );

private:
	float mKp;
	float mKi;
	float mKd;
};
#endif  // EV3_UNIT_BALANCERCPP_H_
