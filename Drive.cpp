/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Drive.h"
#include "ev3api.h"

/**
 * コンストラクタ
 */
Drive::Drive(){
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
}

//*****************************************************************************
// 関数名 : run
// 引数 : pwm_L:左モーター  pwm_R:右モーター 
// 返り値 : 無し
// 概要 : 走行する
//*****************************************************************************
void Drive::run(signed char pwm_L, signed char pwm_R)
{
	/* 左モータの制御 */
	if (pwm_L == 0){
		ev3_motor_stop(left_motor, true);
	}else{
		if(pwm_L > 100)pwm_L = 100;
		if(pwm_L < -100)pwm_L = -100;
		ev3_motor_set_power(left_motor, (int)pwm_L);
	}
	/* 右モータの制御 */
	if (pwm_R == 0){
		ev3_motor_stop(right_motor, true);
	}else{
		if(pwm_R > 100)pwm_R = 100;
		if(pwm_R < -100)pwm_R = -100;
		ev3_motor_set_power(right_motor, (int)pwm_R);
	}
}
//*****************************************************************************
// 関数名 : Blance_line_trace
// 引数 : 無し
// 返り値 : 無し
// 概要 : バランスをとりながら走行する
//*****************************************************************************
void Drive::balance_line_trace(void)
{
	mState = BALANCE_ON;
	MODE = 0;
}
//*****************************************************************************
// 関数名 : Blance_turn
// 引数 : 無し
// 返り値 : 無し
// 概要 : バランスをとりながらターンする
//*****************************************************************************
void Drive::balance_turn(void)
{
	mState = BALANCE_TURN;
	MODE = 0;
}
//*****************************************************************************
// 関数名 : taile_line_trace
// 引数 : 無し
// 返り値 : 無し
// 概要 : バランスをとらずに走行する
//*****************************************************************************
void Drive::tail_line_trace(void)
{
	mState = TAIL_ON;
	MODE = 1;
}
//*****************************************************************************
// 関数名 : taile_line_gate
// 引数 : 無し
// 返り値 : 無し
// 概要 : バランスをとらずに走行する
//*****************************************************************************
void Drive::tail_line_gate(void)
{
	mState = TAIL_GATE;
	MODE = 2;
}
//*****************************************************************************
// 関数名 : taile_turn
// 引数 : 無し
// 返り値 : 無し
// 概要 : バランスをとらずにターンする
//*****************************************************************************
void Drive::tail_turn(void)
{
	mState = TAIL_TURN;
	MODE = 1;
}
//*****************************************************************************
// 関数名 : straight_run
// 引数 : 無し
// 返り値 : 無し
// 概要 : 真っ直ぐに進む
//*****************************************************************************
void Drive::straight_run(void)
{
	mState = STRAIGHT;
	MODE = 1;
}
//*****************************************************************************
// 関数名 : trace
// 引数 : forward (前進速度)　turn (転換速度)
// 返り値 : 無し
// 概要 : バランスをとらずに走行する
//*****************************************************************************
void Drive::trace(signed char forward, signed char turn)
{
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */
	
	pwm_L = forward - turn;
	pwm_R = forward + turn;		        
	run(pwm_L,pwm_R);
//	mState = TAIL_ON;

}

