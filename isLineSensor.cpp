/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "isLineSensor.h"
#include "ev3api.h"
#include "BalancerCpp.h"

/**
 * コンストラクタ
 */
LineSensor::LineSensor(){
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
}

//*****************************************************************************
// 関数名 : normanaization
// 引数 : sensol_val:センサーの値 s;走行時のモード
// 返り値 : 標本化したセンサー値
// 概要 :読み取った値を標本化する。
//*****************************************************************************
float LineSensor::normanaization( int sensor_val,int s)
{
	float sval;

	sval = ((float)sensor_val - mBlack[s]) / (mWhite[s] - mBlack[s]);

	return sval;
}
//*****************************************************************************
// 関数名 : set_pid
// 引数 : kp:比例　ki:積分　kd:微分
// 返り値 : なし
// 概要 :PID制御の値をセットする
//*****************************************************************************
void LineSensor::set_pid(float kp, float ki, float kd)
{
	mKp = kp;
	mKi = ki;
	mKd = kd;
}

//*****************************************************************************
// 関数名 : pid_sample
// 引数 : sensor_val (センサー値), target_val(目標値)
// 返り値 : 操作量
// 概要 :PID制御サンプル（下記のところからのコピー）
// ETロボコンではじめるシステム制御（4）
// 滑らかで安定したライントレースを実現する」
// http://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html
//*****************************************************************************
float LineSensor::pid_control(float target_val,int s)
{
	float p =0, i=0, d=0;
	float pid;

	static float diff[2] = {0.0, 0.0};
	static float integral = 0.0;

	diff[0] = diff[1];
	diff[1] = normanaization(ev3_color_sensor_get_reflect(color_sensor),s) - target_val;	//偏差を取得
	integral += (diff[1] + diff[0]) / 2.0 ;

	p = mKp * diff[1];
	i = mKi * integral;
	d = mKd * (diff[1] - diff[0]);
	pid = p + i + d;

	if(pid > 100.0)pid = 100.0;
	if(pid < -100.0)pid = -100.0;
	
	if(LR == 'R')	return pid;	/* 左エッジ */
	else			return -pid;
}

//*****************************************************************************
// 関数名 : lineCalibration
// 引数 : 回数
// 返り値 : 無し
// 概要 : カラーセンサーのキャリブレーション　黒白の順に行う
//*****************************************************************************
void LineSensor::lineCalibration( int s )
{
	char buf[100];

    ev3_lcd_draw_string("calibration", 0, 20);
	while(1){
	    ev3_led_set_color(LED_GREEN); /* スタート通知 */
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){ /* 押された */
			mBlack[s] = ev3_color_sensor_get_reflect(color_sensor);
			ev3_speaker_play_tone(440,1000);
			break;
		}
    	tslp_tsk(100); /* 100msec周期起動 */
	}
	sprintf(buf,"Black %d   %d   %d",mBlack[0],mBlack[1],mBlack[2]);
	ev3_lcd_draw_string(buf, 60, 20);
    tslp_tsk(2000); /* 2sec周期起動 */
	while(1){
	    ev3_led_set_color(LED_RED); /* スタート通知 */
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){ /* 押された */
			mWhite[s] = ev3_color_sensor_get_reflect(color_sensor);
			ev3_speaker_play_tone(880,1000);
			break;
		}
    	tslp_tsk(100); /* 100msec周期起動 */
	}
	sprintf(buf,"White %d   %d   %d",mWhite[0],mWhite[1],mWhite[2]);
	ev3_lcd_draw_string(buf, 60, 30);
    tslp_tsk(2000); /* 2sec周期起動 */
}