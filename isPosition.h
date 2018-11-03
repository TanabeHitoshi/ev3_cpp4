/******************************************************************************
 **	ファイル名 : viewer.h
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
	void reset(void);	/* ロータリーエンコーダを０にする */
	int32_t tripmeter(void);		// 区間距離を測定する
	void tripmeter_set(void);	//区間距離の測定位置
	int32_t odometer(void);			// 概要 : エンコーダーで走行距離を測定する
	int isTurnAngle(void);		//回転角度を計測する

private:
	int32_t tripmeter_pre;
	int32_t count_left_pre;
	int32_t count_right_pre;
};
#endif  // EV3_UNIT_BALANCERCPP_H_
