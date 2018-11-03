/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "Drive.h"
#include "isLineSensor.h"
#include "isPosition.h"
#include "isSonar.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#define L_COURSE

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

	
/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP	90 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE	10 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_GATE		60 /* ゲートをくぐる時の角度[度] */
#define TAIL_ANGLE_LINE		80	/* テール走行での角度 */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */
//static int sonar_alert(void);

#include "BalancerCpp.h"        // <1>
Balancer balancer;              // バランスクラス
Drive motor;					// モーター駆動クラス
LineSensor sensor;				// カラーセンサークラス
Position pos;					// トリップメータクラス
Sonar sonar;					//超音波センサークラス

int gSpeed;      /* 前後進命令 */
int gTurn;         /* 旋回命令 */
int pattern;
int gCourse;		/* Ｌコース、Ｒコース */
long cnt;			/* タイマー用 */
int16_t MaxGyro=0;
int16_t MiniGyro=0;
/* メインタスク */
void main_task(intptr_t unused)
{
	int Tail_angle = 0;
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_c4", 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
	//コース選択
	#ifdef L_COURSE
		gCourse = 1;
	#else
		gCourse = 0;
	#endif


    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

	pattern = 0;
    /* スタート待機 */
    while(1)
    {
		motor.tail_Angle_Set(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

		switch(pattern){
			case 0:	/* ジャイロセンサのキャリブレーション */
				balancer.init();
				act_tsk(TAIL_TASK);	//尾尻制御の開始
				pattern = 1;
			break;
			case 1:	/* カラーセンサーのキャリブレーション */
				motor.tail_Angle_Set(TAIL_ANGLE_STAND_UP);
				sensor.lineCalibration(0); /* カラーセンサーのキャリブレーション */
//				if(gCourse == 0){
					motor.tail_Angle_Set(TAIL_ANGLE_LINE);
					sensor.lineCalibration(1); /* カラーセンサーのキャリブレーション */
//			    }
				act_tsk(BT_TASK);		/* Bluetooth通信タスクの起動 */
				pattern = 2;
			break;
			case 2: /* スタート待ち */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),BALANCE) > 0.7) ev3_speaker_play_tone(440,10);
				else if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),BALANCE) < 0.5) ev3_speaker_play_tone(880,10);
				else ev3_speaker_play_tone(660,10);
		        if (bt_cmd == 1)
		        {
		            pattern = 5; /* リモートスタート */
		        }
		        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
		        {
		            pattern = 5; /* タッチセンサが押された */
		        }
			break;
			case 5: /* 走行準備 */
			    /* 走行モーターエンコーダーリセット */
			    ter_tsk(BT_TASK);	/* スタート用 Bluetoothを停止 */
				pos.reset();			// トリップメータのリセット
			    ev3_led_set_color(LED_GREEN); /* スタート通知 */
//				act_tsk(BALANCE_TASK);			//倒立振子タスクを開始する
				Tail_angle =TAIL_ANGLE_STAND_UP*10;
				pattern = 7;
			break;
			case 7:
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle / 10);
				if(Tail_angle == (TAIL_ANGLE_STAND_UP +5)*10){
					act_tsk(BALANCE_TASK);			//倒立振子タスクを開始する
					pattern = 8;
				}
			break;
			case 8: /* 第0区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 45;						/* 速度設定 */
				sensor.set_pid(0.0, 0.0, 0.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.odometer() > 50)pattern = 10;
			break;
			case 10: /* 第1区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 45;						/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.odometer() > 200)pattern = 15;
			break;
			case 15: /* 第2区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 65;						/* 速度設定 */
				sensor.set_pid(30.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
//				if((int)pos.odometer() > 350)pattern = 17;
//				if((int)pos.odometer() > 350)pattern = 300;
				if((int)pos.odometer() > 350)pattern = 600;
			break;
			case 17: /* 第2区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 105;						/* 速度設定 */
				sensor.set_pid(30.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.odometer() > 500)pattern = 20;
			break;
			case 20: /* 第3区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 150;						/* 速度設定 */
				sensor.set_pid(20.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.odometer() > 2000)pattern = 25;
			break;
			case 25: /* 第4区間走行 */
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.odometer() > 2000)pattern = 25;
			break;

/* ゲート処理に入ります */
			case 300: /* 超音波センサーを使用可能にする */
				act_tsk(SONAR_TASK);			//超音波タスクを開始する
				pattern = 310;
			break;
			case 310: /* ゆっくり20cmまで近づく*/
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)sonar.Distance()< 20){
					Tail_angle = TAIL_ANGLE_DRIVE*10;
					pattern = 320; /* 近づく距離 */
				}
			break;
			case 320: /* 尾尻を降ろす*/
//				ev3_speaker_play_tone(440,1000);
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(Tail_angle == TAIL_ANGLE_GATE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 340;
				}
			break;
			case 330: /* 2秒待つ*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */

				if(cnt > 2000/4){
					cnt = 0;
					pattern = 340;
				}
			break;
			case 340: /* 尾尻に着地*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 100;						/* 速度設定 */
				motor.straight_run();				/* まっぐに進む */
				if(cnt > 50/4){
					cnt = 0;
					pattern = 350;
				}
			break;
			case 350: /* 尾尻に着地*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();				/* 停止 */

				if(cnt > 100/4){
					cnt = 0;
					pattern = 360;
				}
			break;
			case 360: /* 5cmまでゲートに近づく*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				
				if((int)sonar.Distance()< 5){
					Tail_angle = TAIL_ANGLE_LINE*10;
					pattern = 370;		/* 近づく距離 */
				}
			break;
			case 370: /* 尾尻をさらに下げる*/
//				ev3_speaker_play_tone(440,1000);
				Tail_angle--;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_GATE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 380;
				}
			break;
			case 380: /* ゲートをくぐり抜ける*/
				motor.tail_Angle_Set(TAIL_ANGLE_GATE);	/* 尾尻走行用角度に制御 */
				gSpeed = 10;							/* 速度設定 */
				motor.straight_run();					/* まっぐに進む */

				if((int)pos.tripmeter()> 200){
					cnt = 0;
					Tail_angle = TAIL_ANGLE_GATE*10;
					pattern = 390;		/* 走行する距離200mm */
				}
			break;
			case 390: /* 尾尻をトレース用角度まで戻す*/
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_LINE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 400;
				}
			break;
			case 400: /* 180度回転*/
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 20;						/* 速度設定 */
				motor.tail_turn();			/* 尾尻走行で回転 */

				if((int)pos.isTurnAngle()> 180){	/* 回転角度180度 */
					pattern = 410;		
				}
			break;
			case 410: /* 5cmまでゲートに近づく*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				
				if((int)sonar.Distance()< 5){
					Tail_angle = TAIL_ANGLE_LINE*10;
					pattern = 420;		/* 近づく距離 */
				}
			break;
			case 420: /* 尾尻をさらに下げる*/
//				ev3_speaker_play_tone(440,1000);
				Tail_angle--;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_GATE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 430;
				}
			break;
			case 430: /* ゲートをくぐり抜ける*/
				motor.tail_Angle_Set(TAIL_ANGLE_GATE);	/* 尾尻走行用角度に制御 */
				gSpeed = 10;							/* 速度設定 */
				motor.straight_run();					/* まっぐに進む */

				if((int)pos.tripmeter()> 200){
					cnt = 0;
					Tail_angle = TAIL_ANGLE_GATE*10;
					pattern = 440;		/* 走行する距離200mm */
				}
			break;
			case 440: /* 尾尻をトレース用角度まで戻す*/
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_LINE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 450;
				}
			break;
			case 450: /* 180度回転*/
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 15;						/* 速度設定 */
				motor.tail_turn();			/* 尾尻走行で回転 */

				if((int)pos.isTurnAngle()> 180){	/* 回転角度180度 */
					pattern = 460;		
				}
			break;
			case 460: /* 5cmまでゲートに近づく*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				
				if((int)sonar.Distance()< 5){
					Tail_angle = TAIL_ANGLE_LINE*10;
					pattern = 470;		/* 近づく距離 */
				}
			break;
			case 470: /* 尾尻をさらに下げる*/
//				ev3_speaker_play_tone(440,1000);
				Tail_angle--;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_GATE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 480;
				}
			break;
			case 480: /* ゲートをくぐり抜ける*/
				motor.tail_Angle_Set(TAIL_ANGLE_GATE);	/* 尾尻走行用角度に制御 */
				gSpeed = 10;							/* 速度設定 */
				motor.straight_run();					/* まっぐに進む */

				if((int)pos.tripmeter()> 200){
					cnt = 0;
					Tail_angle = TAIL_ANGLE_GATE*10;
					pattern = 490;		/* 走行する距離200mm */
				}
			break;
			case 490: /* 尾尻をトレース用角度まで戻す*/
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */

				if(Tail_angle == TAIL_ANGLE_LINE*10){
					cnt = 0;
					pattern = 500;
				}
			break;
			case 500: /* 尾尻でトレース*/
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */

				if((int)pos.tripmeter()> 1000)pattern = 900;		/* 走行する距離1000mm */
			break;
/* 階段処理に入ります */
			case 600: /* 尾尻をおろす準備*/
				int16_t gyro_now;
				motor.tail_Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				sensor.set_pid(30.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				gyro_now = ev3_gyro_sensor_get_rate(gyro_sensor);
				Tail_angle = TAIL_ANGLE_DRIVE*10;
				pattern = 610; 
			break;
			case 610: /* 尾尻を降ろす*/
//				ev3_speaker_play_tone(440,1000);
				Tail_angle++;
				motor.tail_Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(Tail_angle == TAIL_ANGLE_GATE*10){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 620;
				}
			case 620: /* 2秒待つ*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */

				if(cnt > (2000/4)){
					cnt = 0;
					pattern = 630;
				}
			break;
			case 630: /* 尾尻に着地*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 100;						/* 速度設定 */
				motor.straight_run();				/* まっぐに進む */
				if(cnt > (50/4)){
					cnt = 0;
					pattern = 640;
				}
			break;
			case 640: /* 尾尻に着地*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();				/* 停止 */

				if(cnt > (1000/4)){
					cnt = 0;
					pattern = 650;
				}
			break;
			case 650: /* 尾尻に着地*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 100;						/* 速度設定 */
				motor.straight_run();				/* まっぐに進む */
				if(cnt > 50/4){
					cnt = 0;
					pattern = 660;
				}
			break;
			case 660: /* 階段に近づく*/
//				ev3_speaker_play_tone(440,1000);
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 10;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				
				if(gyro_now > MaxGyro)MaxGyro = gyro_now;
				if(gyro_now < MiniGyro)MiniGyro = gyro_now;
				if(gyro_now > 30){	/* ジャイロが大きく変化する */
					pos.tripmeter_set();		/* トリップメータをセットする */
					cnt = 0;
					Tail_angle = TAIL_ANGLE_DRIVE*10;
					pattern = 670; 
				}
			break;
			case 670: /* 加速して一気に階段に登る*/
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
//				if(pos.tripmeter() < -15)	gSpeed = 10;	/* 階段まで近ずく速度 */
//				else						gSpeed = 100;	/* 階段を登る速度 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				gSpeed = 100;
				sensor.set_pid(50.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */

				if((int)pos.tripmeter()> 10)pattern = 680;		/* 走行する距離100mm */
			break;
			case 680:
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				if((int)pos.tripmeter()> 200)gSpeed = -10;		/* 走行する距離100mm */
				else if((int)pos.tripmeter()< 100)gSpeed = 10;
				else gSpeed = 0;
			break;

/* ガレージイン処理に入ります */
			case 900: /* ガレージイン*/
				motor.tail_Angle_Set(TAIL_ANGLE_LINE);	/* 尾尻走行用角度に制御 */
				gSpeed = 0;								/* 速度設定 */
				motor.straight_run();					/* 停止 */

				pattern = 900;
			break;
		}

        tslp_tsk(4); /* 4msec周期起動 */
    }
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
	while(1){
		uint8_t c = fgetc(bt); /* 受信 */
		switch(c){
			case '1':
				bt_cmd = 1;
			break;
			default:
			break;
		}
	dly_tsk(100);/* 100msec周期起動 */
	}
}
//*****************************************************************************
// 関数名 : balance_task
// 引数 : unused
// 返り値 : 無し
// 概要 : バランス制御タスク
//*****************************************************************************
void balance_task(intptr_t unused)
{
	while(1){
		switch(motor.mState){
		case BALANCE_ON:
			gTurn = (int)sensor.pid_control(0.6,BALANCE);
	        balancer.setCommand( gSpeed, gTurn );
			balancer.update();
			motor.run(balancer.getPwmLeft(),balancer.getPwmRight());
		break;
		case BALANCE_TURN:
	        balancer.setCommand( 0, 0 );
			balancer.update();
			motor.run(balancer.getPwmLeft() - 40 ,balancer.getPwmRight() + 50);
		break;
		case TAIL_ON:
			gTurn = (int)sensor.pid_control(0.6,TAIL);
			motor.trace(gSpeed, -gTurn);
		break;
		case TAIL_TURN:
			motor.run( -gSpeed , gSpeed + 15);		
		break;
		case STRAIGHT:
			motor.run(gSpeed,gSpeed);
		break;
		}
	cnt++;
	dly_tsk(4);	/* 4msec周期起動 */
	}
}
//*****************************************************************************
// 関数名 : tail_task
// 引数 : unused
// 返り値 : 無し
// 概要 : 尾尻制御タスク
//*****************************************************************************
void tail_task(intptr_t unused)
{
	while(1){
		motor.tail_Control();
	dly_tsk(10);	/* 10msec周期起動 */
	}
}
//*****************************************************************************
// 関数名 : sonar_task
// 引数 : unused
// 返り値 : 無し
// 概要 : 超音波センサータスク
//*****************************************************************************
void sonar_task(intptr_t unused)
{
	while(1){
		sonar.getDistance();
	dly_tsk(50);/* 50msec周期起動 */
	}
}
//*****************************************************************************
// 関数名 : monitor_task
// 引数 : unused
// 返り値 : なし
// 概要 : 色々な値をＬＣＤに表示する
//*****************************************************************************
void monitor_task(intptr_t idx)
{
	char buf[100];

	while(1){
		sprintf(buf,"pattern  %d",pattern);
		ev3_lcd_draw_string(buf, 0, 10);
		sprintf(buf,"voltage %d  gyro %4d",ev3_battery_voltage_mV(),ev3_gyro_sensor_get_rate(EV3_PORT_4));
		ev3_lcd_draw_string(buf, 0, 50);
		sprintf(buf,"sonar %4d",sonar.Distance());
		ev3_lcd_draw_string(buf, 0, 60);
	dly_tsk(500);/* 500msec周期起動 */
	}
}
//*****************************************************************************
// 関数名 : logger_task
// 引数 : unused
// 返り値 : なし
// 概要 : 色々な値をPCに表示する
//*****************************************************************************
void logger_task(intptr_t idx)
{
	static int state = 0;
	while(1){
		if(pattern == 0 && state == 0){
			fprintf(bt,"Calibration of the gyro\n\r");
			state = 1;
		}else if(pattern == 1 && state == 1){
			fprintf(bt,"Calibration of the Line Sensor\n\r");
			state = 2;
		}else if(pattern == 2){	
			fprintf(bt,"Waitting start !!!  %lf\n\r",sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),BALANCE));
		}else if(pattern == 5){
			fprintf(bt,"Ready to run !!!\n\r");
		}else if(pattern > 5){
			fprintf(bt,"%d,%3d,%3d,%3d,%4ld,%6d,%d,%2.6lf,%4d,%4d\n\r",
					motor.mState,
					pattern,
					gSpeed,		
					gTurn,
					(long)pos.odometer(),
					(int)sonar.Distance(),
					ev3_battery_voltage_mV(),
					sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),BALANCE),
					MaxGyro,
					MiniGyro
//					gPwm_L,gPwm_R
//					gGyro,
//					gSonar,
//					(float)gVolt/1000.0,
//					(int)isTailAngle()
			);
		}
	dly_tsk(100);/* 100msec周期起動 */
	}
}
