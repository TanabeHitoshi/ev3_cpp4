/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Tail.h"
#include "ev3api.h"

#define PWM_ABS_MAX          100 /* 完全停止用モータ制御PWM絶対最大値 */

/**
 * コンストラクタ
 */
Tail::Tail(){
    /* モーター出力ポートの設定 */
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);
	Set_PID();
}

//*****************************************************************************
// 関数名 : Set_PID
// 引数 : pid値
// 返り値 : 無し
// 概要 : 尾尻制御用のPID値セット
//*****************************************************************************
void Tail::Set_PID(void)
{
	P_GAIN = 6.0;          /* 完全停止用モータ制御比例係数 */
	I_GAIN = 1.0;           //0.5F /* 完全停止用モータ制御比例係数 */
	D_GAIN = 11.0;           //3.0F /* 完全停止用モータ制御比例係数 */
}
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Tail::Control( void )
{
	signed int angle;

	angle = mTailAngle;
	static float iPwm = 0.0;
	static float prePwm = 0.0;
	static float dPwm = 0.0;
	float Pwm;

    Pwm = (float)(angle - ev3_motor_get_counts(EV3_PORT_A)); /* 比例 */
	iPwm += Pwm - prePwm;									/* 積分値 */
    dPwm = Pwm - prePwm;
	float pwm = Pwm*P_GAIN + iPwm*I_GAIN +dPwm*D_GAIN; 
	prePwm = Pwm;

    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
        pwm = PWM_ABS_MAX;
    else if (pwm < -PWM_ABS_MAX)
        pwm = -PWM_ABS_MAX;

	//モーターへ出力処理
    if (pwm == 0)
        ev3_motor_stop(tail_motor, true);
    else
        ev3_motor_set_power(tail_motor, (signed char)pwm);
}
//*****************************************************************************
// 関数名 : tail_Stop
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Tail::Stop( void )
{
	P_GAIN = 5.0;          /* 完全停止用モータ制御比例係数 */
	I_GAIN = 0.0;           //0.5F /* 完全停止用モータ制御比例係数 */
	D_GAIN = 10.0;           //3.0F /* 完全停止用モータ制御比例係数 */
}
//*****************************************************************************
// 関数名 : tail_angle
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 尾尻の角度制御
//*****************************************************************************
void Tail::Angle_Set(signed int angle)
{
	mTailAngle = angle;
}

