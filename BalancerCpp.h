/******************************************************************************
 *  BalancerCpp.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class Balancer
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCERCPP_H_
#define EV3_UNIT_BALANCERCPP_H_

#include "ev3api.h"

/** センサー、モーターの接続を定義します **/
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

class Balancer {
public:
    Balancer();

    void init(void);
	void update(int angle, int rwEnc, int lwEnc, int battery);
    void setCommand(int forward, int turn);
	void setOffset(float offset);

    int8_t getPwmRight();
    int8_t getPwmLeft();
	void balance_task(intptr_t unused);
private:
    int mForward;
    int mTurn;
    float mOffset;
    int8_t mRightPwm;
    int8_t mLeftPwm;
  void cancelBacklash(int& rwEnc, int& lwEnc);
};
#endif  // EV3_UNIT_BALANCERCPP_H_
