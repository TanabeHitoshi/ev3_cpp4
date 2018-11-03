/******************************************************************************
 **	ファイル名 : Motor.h
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
	void Set_PID(void);		// PIDをセットする
	void Stop(void);		// 尾尻を停止させる
	void Angle_Set(signed int angle);	//尾尻の角度セット
	void Control(void);		// 尾尻の制御
private:
	float P_GAIN;				/* 完全停止用モータ制御比例係数 */
	float I_GAIN;				/* 完全停止用モータ制御比例係数 */
	float D_GAIN;				/* 完全停止用モータ制御比例係数 */
	signed int mTailAngle;		//尾尻の制御角度
};
#endif  // EV3_UNIT_BALANCERCPP_H_
