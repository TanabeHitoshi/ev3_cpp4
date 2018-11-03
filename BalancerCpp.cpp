/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "balancer.h"
#include "BalancerCpp.h"
#include "Drive.h"

/**
 * コンストラクタ
 */
Balancer::Balancer()
    : mForward(0),
      mTurn(0),
      mOffset(0),
      mRightPwm(0),
      mLeftPwm(0) {
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
}

/**
 * バランサを初期化する
 * @param offset ジャイロセンサオフセット値
 */
void Balancer::init(void) {
//	int gyro_offset[10];
//	int i,j,tmp;
//	char buf[100];

    ev3_led_set_color(LED_OFF); /* スタート通知 */
	ev3_speaker_play_tone(440,1000);
	while(ev3_touch_sensor_is_pressed(EV3_PORT_1) == 0);/* 押されるまで */
	ev3_speaker_play_tone(440,1000);
    tslp_tsk(2000); /* 2sec周期起動 */
	while(ev3_gyro_sensor_reset(EV3_PORT_4) != E_OK);

	/* ジャイロの値を読み取る */
//	for(i = 0; i < 10; i++){
//		gyro_offset[i] = ev3_gyro_sensor_get_rate(EV3_PORT_4); 
//		tslp_tsk(250); /* 0.25sec周期起動 */
//	}
	/* 並べ替える */
//	for(i = 0; i < 10; i++){
//		for(j = 10-1; j > i; j--){
//			if(gyro_offset[j-1] > gyro_offset[j]){
//				tmp = gyro_offset[j];
//				gyro_offset[j] = gyro_offset[j-1];
//				gyro_offset[j-1] = tmp;
//			}
//		}
//	}
	/* オフセット値を計算する */
//	tmp = 0;
//	for(i = 2; i < 9; i++){
//		tmp += gyro_offset[i];
//	}
//	mOffset = (float)tmp/8.0;

//	sprintf(buf,"Gyro OFFSET %.2f",mOffset);
//	ev3_lcd_draw_string(buf, 0, 40);

	mOffset = 0;
	ev3_led_set_color(LED_RED); 
	ev3_speaker_play_tone(440,1000);

//    mOffset = offset;
    balance_init();  // 倒立振子制御初期化
}
/**
 * バックラッシュキャンセル
 * @note          直近のPWM値に応じてエンコーダ値にバックラッシュ分の値を追加します
 * @param rwEnc   右車輪エンコーダ値
 * @param lwEnc   左車輪エンコーダ値
 * @date          2017/10/24
 * @auther        Koji SHIMIZU
 */
void Balancer::cancelBacklash(int& rwEnc, int& lwEnc)
{
  const int BACKLASHHALF = 4;   // バックラッシュの半分[deg]

  if(mRightPwm < 0) rwEnc += BACKLASHHALF;
  else if(mRightPwm > 0) rwEnc -= BACKLASHHALF;

  if(mLeftPwm < 0) lwEnc += BACKLASHHALF;
  else if(mLeftPwm > 0) lwEnc -= BACKLASHHALF;
}

/**
 * バランサーの値を更新する
 * @param angle   角速度
 * @param rwEnc   右車輪エンコーダー値
 * @param lwEnc   左車輪エンコーダー値
 * @param battety バッテリー電圧値
 */
void Balancer::update(int angle, int rwEnc, int lwEnc, int battery) {
  cancelBacklash(lwEnc, rwEnc); // バックラッシュキャンセル
  // 倒立振子制御APIを呼び出し、倒立走行するための
  // 左右モーター出力値を得る
  balance_control (
     static_cast<float>(mForward),
     static_cast<float>(mTurn),
     static_cast<float>(angle),
     static_cast<float>(mOffset),
     static_cast<float>(lwEnc),
     static_cast<float>(rwEnc),
     static_cast<float>(battery),
     &mLeftPwm,
     &mRightPwm);
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Balancer::setCommand(int forward, int turn) {
    mForward = forward;
    mTurn    = turn;
}
/**
 * gyro offset値を設定する
 * @param mOffset 前進値
 * @param turn    旋回値
 */
void Balancer::setOffset(float offset) {
	mOffset = offset;
}
/**
 * 右車輪のPWM値を取得する
 * @return 右車輪のPWM値
 */
int8_t Balancer::getPwmRight() {
    return mRightPwm;
}

/**
 * 左車輪のPWM値を取得する
 * @return 左車輪のPWM値
 */
int8_t Balancer::getPwmLeft() {
    return mLeftPwm;
}
