/******************************************************************************
 *  BalancerCpp.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "isSonar.h"
#include "ev3api.h"
#include "BalancerCpp.h"

/**
 * コンストラクタ
 */
Sonar::Sonar(){
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
}
//*****************************************************************************
// 関数名 : ini
// 引数 : なし
// 返り値 : なし
// 概要 :超音波センサーの初期設定
//*****************************************************************************
void Sonar::ini(void)
{
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
}

//*****************************************************************************
// 関数名 : getDistance
// 引数 : なし
// 返り値 : 超音波センサーの値
// 概要 :超音波センサーの値
//*****************************************************************************
void Sonar::getDistance(void)
{
	mSonarDistance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
}
//*****************************************************************************
// 関数名 : Distance
// 引数 : なし
// 返り値 : 超音波センサーの値
// 概要 :超音波センサーの値
//*****************************************************************************
int16_t Sonar::Distance(void)
{
	return mSonarDistance;
}
