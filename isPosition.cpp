/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Drive.h"
#include "ev3api.h"
#include "isPosition.h"

/**
 * コンストラクタ
 */
Position::Position(){
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);
}

//*****************************************************************************
// 関数名 : reset
// 引数 : なし
// 返り値 : なし
// 概要 : エンコーダーの走行距離をリセットする
//*****************************************************************************
void Position::reset(void) {
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);
}
//*****************************************************************************
// 関数名 : odometer
// 引数 : なし	EV3_PORT_C(左), EV3_PORT_B(右)
// 返り値 : 走行距離（ｍｍ）
// 概要 : エンコーダーで走行距離を測定する
//*****************************************************************************
int32_t Position::odometer(void)
{
	int32_t circumference  = 314; // 車輪円周長さ(mm)
	int32_t s = (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)); // エンコーダ左右合計
	return (((s / 360) * circumference) + (circumference * (s % 360) / 360)) / 2;
}

//*****************************************************************************
// 関数名 : tripmeter_set
// 引数 : なし	
// 返り値 : なし
// 概要 : 走行位置のセット
//*****************************************************************************
void Position::tripmeter_set(void)
{
	tripmeter_pre = odometer();
	count_left_pre = ev3_motor_get_counts(left_motor);
	count_right_pre = ev3_motor_get_counts(right_motor);
}
//*****************************************************************************
// 関数名 : tripmeter
// 引数 : なし	
// 返り値 : なし
// 概要 : 走行位置のセット
//*****************************************************************************
int32_t Position::tripmeter(void)
{
	return odometer() - tripmeter_pre;
}
//*****************************************************************************
// 関数名 : isTurnAngle
// 引数 : なし	
// 返り値 : 回転角度
// 概要 : 回転角度を計測する
//*****************************************************************************
int Position::isTurnAngle(void)
{
	int32_t circumference  = 314; // 車輪円周長さ(mm)
	float tread = 180.0;			//トレッド
	int32_t count_left_now;
	int32_t count_right_now;

	count_left_now = count_left_pre - ev3_motor_get_counts(left_motor);
	count_right_now = count_right_pre - ev3_motor_get_counts(right_motor);

	return  (circumference * (count_left_now - count_right_now)/2)/(int32_t)(tread * 3.14);
}
