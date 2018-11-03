/********************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ********************************************************************************/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "Drive.h"
#include "Tail.h"
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

#define L_COURSE	/* ゲート */
//#define R_COURSE	/* シーソー */
//#define TEST
//#define CALIBRATION	/* キャリブレーション*/

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
//#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define PLAIN  0.6		         /* トレース面での閾値 */
#define STEP  0.5          /* 階段での閾値 */
/* sample_c2マクロ */
//#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP	84 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE	5 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_GATE		55 /* ゲートをくぐる時の角度[度] */
#define TAIL_ANGLE_LINE		80	/* テール走行での角度 */
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
Tail tail;						// 尾尻クラス
LineSensor sensor;				// カラーセンサークラス
Position pos;					// トリップメータクラス
Sonar sonar;					//超音波センサークラス

/* コースデータ */
typedef struct {
	char state_t;			/* E:終了 */
	int speed_t;			/* 走行速度 */
	float kp_t;				/* PID */
	float ki_t;
	float kd_t;
	int32_t tripmeter_t;	/* 走行距離 */
	int offset_t;				/* 音程 */
} ET_COURSE_t;

int gSpeed;      /* 前後進命令 */
int gTurn;         /* 旋回命令 */
int gTurnL,gTurnR;	/* 旋回時の速度 */
int pattern;
int gCourse;		/* Ｌコース、Ｒコース */
long cnt;			/* タイマー用 */
long cntR;			/* Ｒコース用タイマー */
int16_t MaxGyro=0;
int16_t MiniGyro=0;
int16_t gGyro;
int16_t gyro_now;
int16_t gyro_old;
int16_t gSonar;
int Gate_distance;
float th;			/* トレースの閾値 */
int Course_p = 0;
int Tail_angle = 0;
int gate_N;
int seesaw_N;
/* メインタスク */
void main_task(intptr_t unused)
{
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("2018", 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
	//コース選択
	#ifdef TEST
		ET_COURSE_t course_t[10] = {
			{'S',20, 0.0, 0.0, 0.0, 50, 5},
			{'3',100, 6.5, 0.0, 40.0, 300, 0},
			{'4',80, 10.0, 0.0, 50.0, 660, 0},
			{'5',80, 5.0, 0.0, 30.0, 555, 0},
			{'6',70, 12.0, 0.0, 60.0, 670, 0},
			{'7',100, 6.5, 0.0, 40.0, 300, 0},
			{'7', 50, 6.5, 0.0, 40.0, 300, 0},
			{'E',0, 50.0, 0.0, 50.0, 250, 440},
			{'E',0, 50.0, 0.0, 50.0, 250, 440},
			{'E',0, 50.0, 0.0, 50.0, 250, 440},
		};
		#ifdef R_COURSE	/*シーソー*/
			gCourse = 1;
		#else			/* ゲート */
			gCourse = 0;
		#endif
	#else
		#ifdef R_COURSE	/*シーソー*/
			ET_COURSE_t course_t[10] = {
				{'0',20, 0.0, 0.0, 0.0, 50, 5},
				{'1',100, 7.0, 0.0, 50.0, 1995, 0},
				{'2',80, 15.0, 0.0, 50.0, 1710, 0},
				{'3',80, 7.0, 0.0, 40.0, 400, 0},
				{'4',60, 10.0, 0.0, 50.0, 1220, 0},
				{'5',100, 7.0, 0.0, 40.0, 590, 0},
				{'6',70, 7.0, 0.0, 50.0, 1285, 0},
				{'7',100, 7.0, 0.0, 50.0, 2150, 0},
				{'8', 50, 7.0, 0.0, 50.0, 300, 0},
				{'E',50, 5.0, 0.0, 10.0, 100, 0},
			};
			gCourse = 1;
		#else			/* ゲート */
			ET_COURSE_t course_t[10] = {
				{'0',20, 0.0, 0.0, 0.0, 50, 5},
				{'1',100, 6.5, 0.0, 40.0, 2335, 0},
				{'2',80, 10.0, 0.0, 40.0, 1680, 0},
				{'3',100, 6.5, 0.0, 40.0, 1655, 0},
				{'4',80, 10.0, 0.0, 50.0, 660, 0},
				{'5',80, 5.0, 0.0, 30.0, 545, 0},
				{'6',60, 12.0, 0.0, 60.0, 680, 0},
				{'7',100, 6.5, 0.0, 40.0, 2200, 0},
				{'8', 50, 6.5, 0.0, 40.0, 300, 0},
				{'E',0, 50.0, 0.0, 50.0, 250, 0},
			};
			gCourse = 0;
		#endif
	#endif

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

	pattern = 0;
    /* スタート待機 */
    while(1)
    {
//		tail.Angle_Set(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
		gSonar = (int)sonar.Distance();
		gGyro = ev3_gyro_sensor_get_rate(gyro_sensor);
		switch(pattern){
			case 0:	/* ジャイロセンサのキャリブレーション */
				balancer.init();
				act_tsk(TAIL_TASK);	//尾尻制御の開始
				cnt = 0;
				pattern = 1;
			break;
			case 1: /* コースの設定　*/	
				fprintf(bt,"gCourse = %d\n\r",gCourse);
				if(gCourse == 0){
					ev3_led_set_color(LED_GREEN); /* スタート通知 */
				    ev3_lcd_draw_string("gate", 0, CALIB_FONT_HEIGHT*1);
				}else{
					ev3_led_set_color(LED_RED); /* スタート通知 */
				    ev3_lcd_draw_string("seasue", 0, CALIB_FONT_HEIGHT*1);
				}
				sensor.LR = 'R';
				pattern = 2;
			break;
			case 2:	/* カラーセンサーのキャリブレーション */
				tail.Angle_Set(TAIL_ANGLE_STAND_UP);
	#ifdef CALIBRATION
				sensor.lineCalibration(0); /* カラーセンサーのキャリブレーション */
				if(gCourse == 0){
					tail.Angle_Set(TAIL_ANGLE_LINE);
					sensor.lineCalibration(1); /* カラーセンサーのキャリブレーション */
					tail.Angle_Set(TAIL_ANGLE_GATE);
					sensor.lineCalibration(2); /* カラーセンサーのキャリブレーション */
					tail.Angle_Set(TAIL_ANGLE_STAND_UP);
			    }
	#else
				sensor.mBlack[0] = 16;sensor.mBlack[1] = 14;sensor.mBlack[2] = 5;
				sensor.mWhite[0] = 34;sensor.mWhite[1] = 29;sensor.mWhite[2] = 9;
	#endif
				fprintf(bt,"Black %d   %d   %d\n\r",sensor.mBlack[0],sensor.mBlack[1],sensor.mBlack[2]);
				fprintf(bt,"White %d   %d   %d\n\r",sensor.mWhite[0],sensor.mWhite[1],sensor.mWhite[2]);
				act_tsk(BT_TASK);		/* Bluetooth通信タスクの起動 */
				th = PLAIN;				/* トレース面での閾値をセット */
				pattern = 3;
			break;
			case 3: /* スタート待ち */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) > 0.65) ev3_speaker_play_tone(440,10);
				else if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) < 0.35) ev3_speaker_play_tone(880,10);
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
				gSpeed = 0;						/* 速度設定 */
				sensor.set_pid(0.0, 0.0, 0.0);		/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				Tail_angle++;
				tail.Angle_Set(Tail_angle / 10);
				if(Tail_angle == (TAIL_ANGLE_STAND_UP + 5)*10){
					#ifdef TEST
						pattern = 100;				//難所のテスト用
					#else
						pattern = 100;				//通常走行
					#endif
					act_tsk(BALANCE_TASK);			//倒立振子タスクを開始する
					pos.tripmeter_set();			/* トリップメータをセットする */
				}
			break;
 /* トレース */
			case 100:
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				balancer.setOffset(course_t[Course_p].offset_t);
				gSpeed = course_t[Course_p].speed_t;						/* 速度設定 */
				sensor.set_pid(course_t[Course_p].kp_t, course_t[Course_p].ki_t, course_t[Course_p].kd_t);		/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.tripmeter() > course_t[Course_p].tripmeter_t){
					pos.tripmeter_set();			/* トリップメータをセットする */
					ev3_speaker_play_tone(440,500);
					Course_p++;
					if(course_t[Course_p].state_t == 'E'){
						th = 0.9;
						balancer.setOffset(0);
						if(gCourse == 0)pattern = 300;	/* L コース(ゲート) */
						else 			pattern = 600;	/* R コース(シーソー)	*/			
					}
				}
			break;
/* ゲート処理に入ります */
			case 300: /* 超音波センサーを使用可能にする */
				act_tsk(SONAR_TASK);			//超音波タスクを開始する
				pattern = 310;
			break;
			case 310: /* ゆっくり30cmまで近づく*/
				tail.Angle_Set(TAIL_ANGLE_GATE);/* バランス走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				sensor.set_pid(15.0, 0.0, 40.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
//				if((int)sonar.Distance()< 30){
					ev3_speaker_play_tone(440,1000);
					cnt = 0;
					Tail_angle = TAIL_ANGLE_GATE*10;
					pattern = 330; /* 近づく距離 */
//				}
			break;
			case 330: /* 1秒待つ*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;							/* 速度設定 */
				sensor.set_pid(15.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(cnt > 1000/4){
					ev3_speaker_play_tone(660,1000);
					cnt = 0;
					pattern = 340;
				}
			break;
			case 340: /* 尾尻に着地*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 100;						/* 速度設定 */
				motor.straight_run();				/* まっぐに進む */
//				if(gGyro < -10){
//					balancer.setOffset(-20);
					if(cnt > 10/4){
						cnt = 0;
						pattern = 350;
					}
//				}
			break;
			case 350: /* 尾尻に着地*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();				/* 停止 */
				if(cnt > 100/4){
					sensor.set_pid(20.0, 0.0, 50.0);	/* ＰＩＤ設定 */
					cnt = 0;
					gate_N = 1;
					pattern = 355;
				}
			break;
			case 355: /* 尾尻に着地*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 8;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				if((int)sonar.Distance()< 25){
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					Gate_distance = (int)gSonar - 10;
					pattern = 360;
				}
			break;
			case 360: /* 10cmまでゲートに近づく*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				gSpeed = 8;						/* 速度設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
				if((int)pos.tripmeter() > Gate_distance * 10 || (int)sonar.Distance()< 10){
					ev3_speaker_play_tone(880,1000);
					Tail_angle = TAIL_ANGLE_LINE*10;
					pattern = 370;		/* 近づく距離 */
				}
			break;
			case 370: /* 尾尻をさらに下げる*/
				Tail_angle--;
				tail.Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */
				if(Tail_angle == TAIL_ANGLE_GATE*10){
					ev3_speaker_play_tone(660,1000);
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					pattern = 380;
				}
			break;
			case 380: /* ゲートをくぐり抜ける*/
				tail.Angle_Set(TAIL_ANGLE_GATE);	/* 尾尻走行用角度に制御 */
				gSpeed = 12;							/* 速度設定 */
				sensor.set_pid(15.0, 0.0, 40.0);	/* ＰＩＤ設定 */
				motor.tail_line_gate();			/* 尾尻走行でライントレース */
				if((int)pos.tripmeter()> 400){
					ev3_speaker_play_tone(440,1000);
					cnt = 0;
					Tail_angle = TAIL_ANGLE_GATE*10;
					pattern = 390;		/* 走行する距離200mm */
				}
			break;
			case 390: /* 尾尻をトレース用角度まで戻す*/
				Tail_angle++;
				tail.Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = -10;						/* 速度設定 */
				motor.straight_run();			/* 停止 */
				if(Tail_angle == TAIL_ANGLE_LINE*10){
					ev3_speaker_play_tone(660,1000);
					cnt = 0;
					pos.tripmeter_set();		/* トリップメータをセットする */
					gate_N++;
					sensor.LR = 'L';
/*
					if(gate_N % 2 == 0) sensor.LR = 'L';
					else 				sensor.LR = 'R';
*/
					if(gate_N > 3) pattern = 520;
					else 			pattern = 400;		
				}
			break;
			case 400: /* 180度回転*/
				tail.Angle_Set(TAIL_ANGLE_LINE);	/* 尾尻走行用角度に制御 */
				gSpeed = 10;							/* 速度設定 */
				motor.tail_turn();						/* 尾尻走行で回転 */
				if((int)pos.isTurnAngle()> 120){		/* 回転角度180度 */
					ev3_speaker_play_tone(880,1000);
					pattern = 410;		
				}
			break;
			case 410: /* 180度回転*/
				tail.Angle_Set(TAIL_ANGLE_LINE);	/* 尾尻走行用角度に制御 */
				gSpeed = 10;							/* 速度設定 */
				motor.tail_turn();						/* 尾尻走行で回転 */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) < 0.1){		/* 回転角度180度 */
					ev3_speaker_play_tone(880,1000);
					pattern = 355;		
				}
			break;
			case 520: /* 尾尻でトレース*/
				tail.Angle_Set(TAIL_ANGLE_LINE);/* 尾尻走行用角度に制御 */
				sensor.LR = 'L';
sensor.set_pid(20.0, 0.0, 0.0);	/* ＰＩＤ設定 */
				gSpeed = 10;						/* 速度設定 */
				if((int)pos.isTurnAngle() < 10)	sensor.set_pid(20.0, 0.0, 0.0);	/* ＰＩＤ設定 */
				motor.tail_line_trace();			/* 尾尻走行でライントレース */
/*850mm*/		if((int)pos.tripmeter()> 555){/* 走行する距離8505mm */
					pattern = 530;		
					Tail_angle =0;
				}
			break;
			case 530: /* 尾尻を停止用角度まで戻す*/
				Tail_angle++;
				tail.Angle_Set(TAIL_ANGLE_GATE);/* 尾尻走行用角度に制御 */
				gSpeed = 0;						/* 速度設定 */
				motor.straight_run();			/* 停止 */
//				if(Tail_angle == TAIL_ANGLE_STAND_UP*10){
//					ev3_speaker_play_tone(440,1000);
//					cnt = 0;
//					ter_tsk(TAIL_TASK);	//尾尻制御の停止
					pattern = 900;
//				}
			break;
/* シーソー処理に入ります */
			case 600: /* ゆっくりシーソーに近づく */
				int16_t gyro_now;
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				gSpeed = 60;						/* 速度設定 */
				sensor.set_pid(15.0, 0.0, 40.0);	/* ＰＩＤ設定 */
//				balancer.setOffset(-5);
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				gyro_now = ev3_gyro_sensor_get_rate(gyro_sensor);
				gGyro = gyro_now;
				if(gyro_now > MaxGyro)MaxGyro = gyro_now;
				if(gyro_now < MiniGyro)MiniGyro = gyro_now;
				if(gyro_now > 100){	/* ジャイロが大きく変化する */
					pos.tripmeter_set();		/* トリップメータをセットする */
					seesaw_N = 1;
					balancer.setOffset(0);
					cnt = 0;
					pattern = 610; 
				}
			break;
			case 605: /* 再度ゆっくりシーソーに近づく */
				gSpeed = 0;						/* 速度設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				gyro_now = ev3_gyro_sensor_get_rate(gyro_sensor);
				gGyro = gyro_now;
				if(gyro_now > MaxGyro)MaxGyro = gyro_now;
				if(gyro_now < MiniGyro)MiniGyro = gyro_now;
				if(gyro_now > 100){	/* ジャイロが大きく変化する */
					cnt = 0;
					th = STEP;				/* 階段での閾値 */
					pattern = 610; 
				}
			break;
			case 610: /* 加速して一気にシーソーに登る */
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
//				gSpeed = 9000 * 70 /ev3_battery_voltage_mV();
				gSpeed = 100;
//				motor.straight_run();				/* まっすぐ進む */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if((int)pos.tripmeter()> 20){		/* 走行する距離20mm */
					cnt = 0;					
					Tail_angle = TAIL_ANGLE_DRIVE*10;
					pattern = 620;		
				}
			break;
			case 620: /* シーソー上でトレース */
				sensor.set_pid(10.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
//				motor.straight_run();				/* まっすぐ進む */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) < 0.1)
					gSpeed = 0;
				else
					gSpeed = 10;
				if((int)pos.tripmeter()> 550){		/* 走行する距離650mm */
					cnt = 0;
					if(seesaw_N > 2) pattern = 626;
					else 			pattern = 625;
					Tail_angle = TAIL_ANGLE_DRIVE*10;
				}
			break;
			case 625: /* シーソー上でトレース */
				sensor.set_pid(10.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) < 0.1)
					gSpeed = 0;
				else
					gSpeed = -5;	
				if((int)pos.tripmeter()> 300){		/* 走行する距離300mm */
					seesaw_N++;
					cnt = 0;
					if((int)pos.tripmeter()> 650) pattern = 626;
					else 						pattern = 620;
				}
			break;
			case 626: /* シーソー上でトレース */
				sensor.set_pid(10.0, 0.0, 50.0);	/* ＰＩＤ設定 */
				tail.Angle_Set(TAIL_ANGLE_DRIVE);/* バランス走行用角度に制御 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE) < 0.1)
					gSpeed = 0;
				else
					gSpeed = 5;
				if((int)pos.tripmeter()> 700){		/* 走行する距離650mm */
					cnt = 0;
					pattern = 630;
					Tail_angle = TAIL_ANGLE_DRIVE*10;
				}
			break;
			case 630: /* シーソー降りてバランスを取って真っ直ぐ進む */
				Tail_angle = Tail_angle + 5;
				tail.Angle_Set(Tail_angle/10);/* 尾尻走行用角度に制御 */
				gSpeed = 20;
//				motor.straight_run();				/* まっすぐ進む */
				sensor.set_pid(0.0, 0.0, 0.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
				if(Tail_angle > TAIL_ANGLE_GATE*10){
					cnt = 0;
					pattern = 640;
				}
			break;
			case 635: /* シーソー降りてバランスを取って真っ直ぐ進む */
				gSpeed = 0;
//				motor.straight_run();				/* まっすぐ進む */
				sensor.set_pid(0.0, 0.0, 0.0);	/* ＰＩＤ設定 */
				motor.balance_line_trace();			/* バランスをとりながらラインとレース */
			break;
			case 640: /* 尾っぽで真っ直ぐ進む */
				gSpeed = 20;
				tail.Angle_Set(TAIL_ANGLE_GATE);
				motor.straight_run();				/* まっすぐ進む */
				if((int)pos.tripmeter()> 850){		/* 走行する距離1000mm */
					cnt = 0;
					pattern = 660;		
				}
			break;
			case 650: /* 停止 */
				gSpeed = 0;
				tail.Angle_Set(TAIL_ANGLE_GATE);
				motor.straight_run();				/* まっすぐ進む */
				if((int)pos.tripmeter()> 900){		/* 走行する距離1000mm */
					cnt = 0;
					pattern = 660;		
				}
			break;
			case 660: /* 尾っぽで真っ直ぐ進む */
				gSpeed = -10;
				tail.Angle_Set(TAIL_ANGLE_GATE);
				motor.straight_run();				/* まっすぐ進む */
				if(cnt > 1000/4){
					cnt = 0;
					pattern = 670;		
				}
			break;
			case 670: /* 停止 */
				gSpeed = 20;
				tail.Angle_Set(TAIL_ANGLE_GATE);
				motor.straight_run();				/* まっすぐ進む */
				if(cnt > 100/4){
					cnt = 0;
					pattern = 680;		
				}
			break;
			case 680: /* 停止 */
				gSpeed = 0;
				tail.Angle_Set(TAIL_ANGLE_GATE);
				motor.straight_run();				/* まっすぐ進む */
				if(cnt > 100/4){
					cnt = 0;
					pattern = 680;		
				}
			break;
/* ガレージイン処理に入ります */
			case 900: /* ガレージイン*/
				tail.Angle_Set(TAIL_ANGLE_LINE);	/* 尾尻走行用角度に制御 */
				gSpeed = 0;								/* 速度設定 */
				motor.straight_run();					/* 停止 */
				tail.Stop();
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
	int32_t motor_ang_l, motor_ang_r;
	int gyro, volt;
	
	while(1){
		/* 倒立振子制御API に渡すパラメータを取得する */
		motor_ang_l = ev3_motor_get_counts(left_motor);
		motor_ang_r = ev3_motor_get_counts(right_motor);
		gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
		volt = ev3_battery_voltage_mV();
		switch(motor.mState){
		case BALANCE_ON:
			gTurn = (int)sensor.pid_control(th,motor.MODE);
	        balancer.setCommand( gSpeed, gTurn );
			balancer.update(gyro, motor_ang_r, motor_ang_l, volt);
			motor.run(balancer.getPwmLeft(),balancer.getPwmRight());
		break;
		case BALANCE_TURN:
	        balancer.setCommand( 0, 0 );
			balancer.update(gyro, motor_ang_r, motor_ang_l, volt);
			motor.run(balancer.getPwmLeft() - gTurnL ,balancer.getPwmRight() + gTurnR);
		break;
		case TAIL_ON:
			gTurn = (int)sensor.pid_control(0.8,motor.MODE);
			motor.trace(gSpeed, -gTurn);
		break;
		case TAIL_GATE:
			gTurn = (int)sensor.pid_control(0.6,motor.MODE);
			motor.trace(gSpeed, -gTurn);
		break;
		case TAIL_TURN:
			motor.run( -gSpeed , gSpeed );//+ 15);		
		break;
		case STRAIGHT:
			motor.run(gSpeed,gSpeed);
		break;
		}
	cnt++;
	cntR++;
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
		tail.Control();
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
			fprintf(bt,"Waitting start !!!  %lf\n\r",sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE));
		}else if(pattern == 3){	
			fprintf(bt,"Waitting start !!!  %lf\n\r",sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE));
		}else if(pattern == 5){
			fprintf(bt,"Ready to run !!!\n\r");
		}else if(pattern > 5){
			fprintf(bt,"%d,%3d,%3d,%3d,%6ld,%6d,%d,%2.6lf,%4d,%4d,%4d,%6d\n\r",
					gate_N,
//					motor.mState,
					pattern,
					gSpeed,		
					gTurn,
					(long)pos.tripmeter(),
					(int)sonar.Distance(),
					ev3_battery_voltage_mV(),
					sensor.normanaization(ev3_color_sensor_get_reflect(color_sensor),motor.MODE),
					MaxGyro,
					(int)pos.tripmeter(),
					(int)pos.isTurnAngle(),
//					cntR
					gGyro
//					gSonar
//					(float)gVolt/1000.0,
//					(int)isTailAngle()
			);
		}
	dly_tsk(100);/* 100msec周期起動 */
	}
}
