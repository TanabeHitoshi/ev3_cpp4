/******************************************************************************
 **	ファイル名 : Motor.h
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

enum State {							//バランス走行の有無
		BALANCE_ON,						//バランス走行
		BALANCE_TURN,					//バランスをとりながら回転
		TAIL_ON,						//尾尻で走行
		TAIL_GATE,						//ゲート通過
		TAIL_TURN,						//尾尻で回転
		STRAIGHT						//まっすぐに進む
	};

class Drive {
public:
    Drive();
	void run(signed char, signed char);
	void balance_line_trace(void); //バランス走行
	void balance_turn(void);		//バランスでターン
	void tail_line_trace(void); //尾尻走行
	void tail_line_gate(void); //尾尻走行
	void tail_turn(void);			//尾尻でターン
	void straight_run(void);		// 真っ直ぐに進む
	void trace(signed char forward, signed char turn);
	int MODE;

//	int BALANCE = 0;
//	int TAIL = 1;

	State mState;							//走行状態
private:

};
#endif  // EV3_UNIT_BALANCERCPP_H_
