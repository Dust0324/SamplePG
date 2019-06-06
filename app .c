
/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 **
		ライントレースをPID制御に変更
		一定間隔でBluetoothにメッセージを送信
		走行中に0が入力されるとモータ停止
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "Gray.h"

//PID用定数
double KP =0.25;
double KI =0.7;			//upでラインに張り付く
double KD= 0.045;


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


/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */

static int      bt_stop = 0;     /* Bluetoothコマンド 0:走行停止 */

static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  3          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 35 /* 超音波センサによる障害物検知距離[cm] 30*/
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP  92 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"		/* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234"	/* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    	/* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

//ルックアップゲート関数内
#define TAIL_ANGLE_A 60
#define TAIL_ANGLE_B 5

/* 関数プロトタイプ宣言 */
void look_up_gate(void);
void toritsu(float forward, float turn);
static int sonar_alert(void);
static void tail_control(signed int angle);
void hyoji(int t);

int angle = 0;
int distance = 0;
int left_angle = 0, right_angle = 0;
int flg = 0;	//障害物検知フラグ
int flg2 = 1;	//尻尾降下フラグ
int flg3 = 0;	//ゴール通過フラグ
int tail = 0;

/* メインタスク */
void main_task(intptr_t unused)
{
	//メッセージ送信用カウンタ変数
	unsigned long cnt = 0;

    signed char forward = 58;      /* 前後進命令 */
    signed char turn;         /* 旋回命令 */
    signed char pwm_L, pwm_R; /* 左右モータPWM出力 */
	
	int cnt_fin = 0;			/*終了用カウンタ*/

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_c4", 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP + angle); /* 完全停止用角度に制御 */

        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* タッチセンサが押された */
        }

       tslp_tsk(7); /* 7msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* ジャイロセンサーリセット */
    tslp_tsk(7);/*7msecのウェイト*/
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init(); /* 倒立振子API初期化 */

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    
	//PID用変数
   	float midpoint = (LIGHT_WHITE - LIGHT_BLACK) / 2 + LIGHT_BLACK;
	int noge = 0;
    float lasterror = 0, integral = 0;
	forward = 80;
	tail_control(TAIL_ANGLE_DRIVE);
    while(1)
    {
		//倒立振子用変数
        int32_t motor_ang_l, motor_ang_r;
        int gyro, volt, ultra;
		
		motor_ang_l = ev3_motor_get_counts(left_motor);
		
		//BACKボタンで無限ループを抜ける
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

		//0を受信したら無限ループを抜ける
        if (bt_stop) break;
		
		if (cnt_fin >= 1000) {
				//destance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
				break;
		}
		
		if(flg2 == 1){
			//cnt_fin = 0;
        	tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
		}else{
			//cnt_fin++;
			tail_control(80);
			forward = turn = 0;
		}
		
		if(flg == 1){
			cnt_fin++;
		}

		if(cnt_fin >= 500){
			flg2 = 0;
		}
		
		if (sonar_alert() == 1 && flg3 == 1) {/* 障害物検知 */
				flg = 1;
            	forward = turn = 0; /* 障害物を検知したら停止 */
        }else if(flg != 1){
              if(motor_ang_l > 11000/*11500*/){
				//break;
                forward = 50;
                flg3 = 1;
            	KP =0.24;
				KI =1.0;
				KD =0.041;
               //midpoint = (LIGHT_WHITE - 20) / 2 + 20;
				midpoint = 20;
           }else if(motor_ang_l < 3500){
				forward = 85;
           }else if(motor_ang_l >= 3500 && motor_ang_l < 5000){
				KP =0.41;
				KI =0.7;
				KD =0.041;
				forward = 80;//80
			}else if(motor_ang_l >= 5000 && motor_ang_l < 6500){
				KP =0.2;
				KI =0.7;
				KD =0.041;
				forward = 85;
			}else if(motor_ang_l >= 6500 && motor_ang_l < 7500){
				//forward = 80;
				KP =0.25;
//				KP =0.4;
				
				KI =1.0;
				KD =0.04;
				forward = 85;
				midpoint = 17;
			}else if(motor_ang_l >= 7500 && motor_ang_l < 9250){
				//forward = 80;
				KP =0.3;
				KI =0.7;
				KD =0.04;
				forward = 85;
				ev3_led_set_color(LED_RED);
//				midpoint = 20;
			}else{
				KP =0.24;
				KI =0.7;
				KD =0.041;
				//forward = 80;
				forward = 85;
				ev3_led_set_color(LED_ORANGE);
           }
			//PID制御(操作量を計算)する
	        float error = (midpoint + 5) - ev3_color_sensor_get_reflect(color_sensor);//P制御
	        integral = error + integral * KP;
          	//                               KP												//KP...-で左戻りup?
	        //float steer = 0.07 * error + 0.3 * integral + 1 * (error - lasterror);
	       float steer =   KI * error + KP * integral + KD * (error - lasterror) / 0.004;
          	//                KI             KP             KD								//KI...+で右戻りup?
			lasterror = error;
			//倒立振子用変数に操作量を格納する
			turn = steer;
			//turn = 0;	//コース走行時コメントアウト
		}
        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

		//一定時間ごとにBluetoothへメッセージを送信する
/*		if(++cnt > 250){
			static unsigned long t = 0;
			char buf[128];

			sprintf(buf, "%ld ", t);
			fprintf(bt, "%s", buf);

            sprintf(buf, "%d ", forward);
			fprintf(bt, "%s", buf);

			//sprintf(buf, "%d ", ev3_color_sensor_get_reflect(color_sensor));
			sprintf(buf, "%d ", flg);
			fprintf(bt, "%s", buf);

			//sprintf(buf, "%d ", gyro);
			sprintf(buf, "%d ", flg2);
			fprintf(bt, "%s", buf);

			//sprintf(buf, "%d ", (int)motor_ang_l);
			sprintf(buf, "%d ", cnt_fin);
			fprintf(bt, "%s", buf); 

			//sprintf(buf, "%d ", (int)motor_ang_r);
			//fprintf(bt, "%s", buf);

			fprintf(bt, "\n\r");

			t++;
			cnt = 0;
			noge += 5;
		}
    	*/
    	
        /* 倒立振子制御APIを呼び出し、倒立走行するための */
        /* 左右モータ出力値を得る */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (signed char*)&pwm_L,
            (signed char*)&pwm_R);

        /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
        /* 出力0時に、その都度設定する */
        if (pwm_L == 0)
        {
             ev3_motor_stop(left_motor, true);
        }
        else
        {
            ev3_motor_set_power(left_motor, (int)pwm_L);
        }
        
        if (pwm_R == 0)
        {
             ev3_motor_stop(right_motor, true);
        }
        else
        {
            ev3_motor_set_power(right_motor, (int)pwm_R);
        }

        tslp_tsk(4); /* 4msec周期起動 */
    }
	//tail_control(85);
	//destance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
	ev3_motor_steer(left_motor, right_motor, 40, 10); //転倒防止の前進
    tslp_tsk(100);
    look_up_gate();
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);
	
	while(1)
    {
		char buf[128];
        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
           sprintf(buf, "%ld ", ev3_color_sensor_get_reflect(color_sensor));
			fprintf(bt, "%s", buf);
	
			fprintf(bt, "\n\r");
        }

    	tslp_tsk(7); /* 7msecウェイト */
    }
	
    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    //signed int distance;
    if (++counter == 40/8) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }
    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
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
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
		case '0':
			bt_stop = 1;
			break;
        case '1':
            bt_cmd = 1;
            break;
		case '2':
            flg2 = 0;
            break;
           case 'd':
			ev3_speaker_set_volume(50);
			ev3_speaker_play_tone(NOTE_G4, 100);
            angle--;
            break;
          case 'u':
			ev3_speaker_set_volume(50);
			ev3_speaker_play_tone(NOTE_A4, 100);
            angle++;
            break;
        default:
            break;
        }
        //fputc(c, bt); /* エコーバック */
		fprintf(bt, "%c\n\r", c);
    }
}

//*****************************************************************************
// 関数名 : hyoji
// 引数 : 表示したい値
// 返り値 : なし
// 概要 : 値を表示
//*****************************************************************************
void hyoji(int t){
	static int cnt;
	char buf[128];
	if(++cnt > 125){
		sprintf(buf, "%ld ", t);
		fprintf(bt, "%s", buf);
		
		fprintf(bt, "\n\r");
		cnt = 0;
	}
}
//*****************************************************************************
// 関数名 : hyoji2
// 引数 : 表示したい値
// 返り値 : なし
// 概要 : 値を表示
//*****************************************************************************
void hyoji2(int t){
	char buf[128];

	sprintf(buf, "%ld ", t);
	fprintf(bt, "%s", buf);
	
	fprintf(bt, "\n\r");
}
//*****************************************************************************
// 関数名 : look_up_gate
// 引数 : なし
// 返り値 : なし
// 概要 : ルックアップゲート攻略
//*****************************************************************************
void look_up_gate(void)
{
 	int motor_ang_l,motor_ang_r, i;
	float forward, turn = 0;
	int color, angle, fleg = 0;
    int count = 0,sp=10,x = -15,cnt = 0;
	int angle_back;
	int anglet[3];
    ev3_motor_reset_counts(left_motor);
  	left_angle = ev3_motor_get_counts(left_motor);
	color = ev3_color_sensor_get_reflect(color_sensor);
	ev3_motor_reset_counts(tail_motor);
	//destance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
	ev3_speaker_set_volume(50);

	//ev3_motor_rotate(tail_motor, 80, 100, false);    //一段階目の尻尾
    //tslp_tsk(50);
	//ev3_motor_steer(left_motor, right_motor, 40, 0); //転倒防止の前進
    //tslp_tsk(90);
	
    ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(2000);
    ev3_motor_rotate(tail_motor, -13, 4, true);   //二段階目の尻尾
	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(1500);
	//ev3_motor_rotate(tail_motor, -1, 4, true);   //二段階目の尻尾
	ev3_motor_reset_counts(left_motor);
	tslp_tsk(1500);
	left_angle = ev3_motor_get_counts(left_motor);
	while(90 > left_angle){					
		color = ev3_color_sensor_get_reflect(color_sensor);
		if(color > 5){
			fleg = 1;
			break;
		}
		left_angle = ev3_motor_get_counts(left_motor);
		ev3_motor_steer(left_motor, right_motor, 10, -100);
       	tslp_tsk(4);
		//hyoji(color);
   	}
	if(fleg != 1){
		ev3_motor_reset_counts(left_motor);
		while(90 > left_angle){
			color = ev3_color_sensor_get_reflect(color_sensor);
			if(color > 5){
				fleg = 1;
				break;
			}
			left_angle = ev3_motor_get_counts(left_motor);
			ev3_motor_steer(left_motor, right_motor, 10, 100);
       		tslp_tsk(4);
			//hyoji(color);
		}
   	}
	if(distance >= 30){
		distance = 15;
		ev3_speaker_play_tone(NOTE_G4, 100);
	}
	/*distance += 12.5;								//障害物との距離を計算し進む距離を出す
	angle = distance / 25;
	angle *= 360;
	angle = angle * 1.5;*/
	angle = 400;
   	ev3_motor_reset_counts(left_motor);
   	left_angle = ev3_motor_get_counts(left_motor);
	hyoji2(distance-12.5);
	hyoji2(left_angle);
	hyoji2(angle);
	anglet[0] = angle * 1.1;
	anglet[1] = angle * 1.3;
	anglet[2] = angle * 1.6;
//	anglet[1] = angle * 1.4;
//	anglet[2] = angle * 1.6;
	for(i = 0;i < 2;i++){
		ev3_motor_reset_counts(left_motor);
   		left_angle = ev3_motor_get_counts(left_motor);
		while(anglet[i] > left_angle){						//ゲートを通過する処理
			color = ev3_color_sensor_get_reflect(color_sensor);
			left_angle = ev3_motor_get_counts(left_motor);
				if(color < 7){
				turn = -60;
			}else{
				turn = 60;
			}
			ev3_motor_steer(left_motor, right_motor, 10, turn);
	       	tslp_tsk(4);
			//hyoji(left_angle);
			hyoji(color);
	   	}
		ev3_speaker_play_tone(NOTE_G4, 100);
		ev3_motor_stop(left_motor, false);
		ev3_motor_stop(right_motor, false);
		tslp_tsk(1000);
		ev3_motor_rotate(right_motor, 400/*360*/, 10, true);   //二段階目の尻尾
		ev3_motor_stop(left_motor, false);
		ev3_motor_stop(right_motor, false);
		tslp_tsk(1000);
		ev3_motor_rotate(left_motor, -250/*270*/, 10, true);   //二段階目の尻尾
		ev3_motor_stop(left_motor, false);
		ev3_motor_stop(right_motor, false);
		tslp_tsk(1000);
	}
    ev3_motor_reset_counts(left_motor);
    left_angle = ev3_motor_get_counts(left_motor);
	while(anglet[2] > left_angle){						//2回目 ゲートを通過する処理
		color = ev3_color_sensor_get_reflect(color_sensor);
		left_angle = ev3_motor_get_counts(left_motor);
		if(color < 7){
			turn = -60;
		}else{
			turn = 60;
		}
		ev3_motor_steer(left_motor, right_motor, 10, turn);
   		tslp_tsk(4);
		//hyoji(color);
   	}
   	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(1500);

   	ev3_motor_rotate(tail_motor,TAIL_ANGLE_B, 100, true);
   	tslp_tsk(1000);
	ev3_motor_rotate(tail_motor, TAIL_ANGLE_B, 80, true);
	tslp_tsk(1000);
	ev3_motor_rotate(tail_motor, 5, 3, true);
   	tslp_tsk(1200);
	ev3_motor_rotate(tail_motor, 3, 3, true);//4
    tslp_tsk(1200);
	//ev3_motor_rotate(tail_motor, 3, 3, true);
    //tslp_tsk(1200);

	ev3_motor_reset_counts(left_motor);
	fleg = 0;
		while(1){
	    	if(color < 6){
				fleg = 1;
				break;
	    	}
	    	
			left_angle = ev3_motor_get_counts(left_motor);
			if(left_angle > 300){
				break;
			}
		  	color = ev3_color_sensor_get_reflect(color_sensor);
		    ev3_motor_steer(left_motor, right_motor, 10, turn);
			if(color < 14){
				turn = -40;
			}else{
				turn = 60;
			}
		    tslp_tsk(4);
			//hyoji(color);
		}
	ev3_speaker_set_volume(50);
	ev3_speaker_play_tone(NOTE_G4, 100);
	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(1500);
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);
	while(motor_ang_l < 550){
		motor_ang_r = ev3_motor_get_counts(right_motor);
		motor_ang_l = ev3_motor_get_counts(left_motor);
		color = ev3_color_sensor_get_reflect(color_sensor);
		if(motor_ang_r > 100){
			ev3_motor_reset_counts(right_motor);
			sp--;
		}
		ev3_motor_steer(left_motor, right_motor, sp, turn);
		if(color < 16){
			turn = -60;
		}else{
			turn = 60;
		}
		//hyoji(color);
		tslp_tsk(4); 
	}
	
   	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(100000);
}
