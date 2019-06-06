
/**
 ******************************************************************************
 ** �t�@�C���� : app.c
 **
 ** �T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/HRP2�pC�T���v���v���O����
 **
 ** ���L : sample_c4 (sample_c3��Bluetooth�ʐM�����[�g�X�^�[�g�@�\��ǉ�)
 **
		���C���g���[�X��PID����ɕύX
		���Ԋu��Bluetooth�Ƀ��b�Z�[�W�𑗐M
		���s����0�����͂����ƃ��[�^��~
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "Gray.h"

//PID�p�萔
double KP =0.25;
double KI =0.7;			//up�Ń��C���ɒ���t��
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
 * �Z���T�[�A���[�^�[�̐ڑ����`���܂�
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

static int      bt_cmd = 0;     /* Bluetooth�R�}���h 1:�����[�g�X�^�[�g */

static int      bt_stop = 0;     /* Bluetooth�R�}���h 0:���s��~ */

static FILE     *bt = NULL;     /* Bluetooth�t�@�C���n���h�� */

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET  3          /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define LIGHT_WHITE  40         /* ���F�̌��Z���T�l */
#define LIGHT_BLACK  0          /* ���F�̌��Z���T�l */
/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 35 /* �����g�Z���T�ɂ���Q�����m����[cm] 30*/
/* sample_c3�}�N�� */
#define TAIL_ANGLE_STAND_UP  92 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^����PWM��΍ő�l */
/* sample_c4�}�N�� */
//#define DEVICE_NAME     "ET0"		/* Bluetooth�� hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME�Őݒ� */
//#define PASS_KEY        "1234"	/* �p�X�L�[    hrp2/target/ev3.h BLUETOOTH_PIN_CODE�Őݒ� */
#define CMD_START         '1'    	/* �����[�g�X�^�[�g�R�}���h */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

//���b�N�A�b�v�Q�[�g�֐���
#define TAIL_ANGLE_A 60
#define TAIL_ANGLE_B 5

/* �֐��v���g�^�C�v�錾 */
void look_up_gate(void);
void toritsu(float forward, float turn);
static int sonar_alert(void);
static void tail_control(signed int angle);
void hyoji(int t);

int angle = 0;
int distance = 0;
int left_angle = 0, right_angle = 0;
int flg = 0;	//��Q�����m�t���O
int flg2 = 1;	//�K���~���t���O
int flg3 = 0;	//�S�[���ʉ߃t���O
int tail = 0;

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
	//���b�Z�[�W���M�p�J�E���^�ϐ�
	unsigned long cnt = 0;

    signed char forward = 58;      /* �O��i���� */
    signed char turn;         /* ���񖽗� */
    signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */
	
	int cnt_fin = 0;			/*�I���p�J�E���^*/

    /* LCD��ʕ\�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_c4", 0, CALIB_FONT_HEIGHT*1);

    /* �Z���T�[���̓|�[�g�̐ݒ� */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* ���˗����[�h */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* ���[�^�[�o�̓|�[�g�̐ݒ� */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth�ʐM�^�X�N�̋N�� */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* �����������ʒm */

    /* �X�^�[�g�ҋ@ */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP + angle); /* ���S��~�p�p�x�ɐ��� */

        if (bt_cmd == 1)
        {
            break; /* �����[�g�X�^�[�g */
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* �^�b�`�Z���T�������ꂽ */
        }

       tslp_tsk(7); /* 7msec�E�F�C�g */
    }

    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* �W���C���Z���T�[���Z�b�g */
    tslp_tsk(7);/*7msec�̃E�F�C�g*/
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init(); /* �|���U�qAPI������ */

    ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */

    
	//PID�p�ϐ�
   	float midpoint = (LIGHT_WHITE - LIGHT_BLACK) / 2 + LIGHT_BLACK;
	int noge = 0;
    float lasterror = 0, integral = 0;
	forward = 80;
	tail_control(TAIL_ANGLE_DRIVE);
    while(1)
    {
		//�|���U�q�p�ϐ�
        int32_t motor_ang_l, motor_ang_r;
        int gyro, volt, ultra;
		
		motor_ang_l = ev3_motor_get_counts(left_motor);
		
		//BACK�{�^���Ŗ������[�v�𔲂���
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

		//0����M�����疳�����[�v�𔲂���
        if (bt_stop) break;
		
		if (cnt_fin >= 1000) {
				//destance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
				break;
		}
		
		if(flg2 == 1){
			//cnt_fin = 0;
        	tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
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
		
		if (sonar_alert() == 1 && flg3 == 1) {/* ��Q�����m */
				flg = 1;
            	forward = turn = 0; /* ��Q�������m�������~ */
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
			//PID����(����ʂ��v�Z)����
	        float error = (midpoint + 5) - ev3_color_sensor_get_reflect(color_sensor);//P����
	        integral = error + integral * KP;
          	//                               KP												//KP...-�ō��߂�up?
	        //float steer = 0.07 * error + 0.3 * integral + 1 * (error - lasterror);
	       float steer =   KI * error + KP * integral + KD * (error - lasterror) / 0.004;
          	//                KI             KP             KD								//KI...+�ŉE�߂�up?
			lasterror = error;
			//�|���U�q�p�ϐ��ɑ���ʂ��i�[����
			turn = steer;
			//turn = 0;	//�R�[�X���s���R�����g�A�E�g
		}
        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

		//��莞�Ԃ��Ƃ�Bluetooth�փ��b�Z�[�W�𑗐M����
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
    	
        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
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

        /* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
        /* �o��0���ɁA���̓s�x�ݒ肷�� */
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

        tslp_tsk(4); /* 4msec�����N�� */
    }
	//tail_control(85);
	//destance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
	ev3_motor_steer(left_motor, right_motor, 40, 10); //�]�|�h�~�̑O�i
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

    	tslp_tsk(7); /* 7msec�E�F�C�g */
    }
	
    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    //signed int distance;
    if (++counter == 40/8) /* ��40msec�������ɏ�Q�����m  */
    {
        /*
         * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
         * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
         * EV3�̏ꍇ�́A�v�m�F
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* ��Q�������m */
        }
        else
        {
            alert = 0; /* ��Q������ */
        }
        counter = 0;
    }
    return alert;
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
static void tail_control(signed int angle)
{
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* ��ᐧ�� */
    /* PWM�o�͖O�a���� */
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
// �֐��� : bt_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ��M */
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
        //fputc(c, bt); /* �G�R�[�o�b�N */
		fprintf(bt, "%c\n\r", c);
    }
}

//*****************************************************************************
// �֐��� : hyoji
// ���� : �\���������l
// �Ԃ�l : �Ȃ�
// �T�v : �l��\��
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
// �֐��� : hyoji2
// ���� : �\���������l
// �Ԃ�l : �Ȃ�
// �T�v : �l��\��
//*****************************************************************************
void hyoji2(int t){
	char buf[128];

	sprintf(buf, "%ld ", t);
	fprintf(bt, "%s", buf);
	
	fprintf(bt, "\n\r");
}
//*****************************************************************************
// �֐��� : look_up_gate
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���b�N�A�b�v�Q�[�g�U��
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

	//ev3_motor_rotate(tail_motor, 80, 100, false);    //��i�K�ڂ̐K��
    //tslp_tsk(50);
	//ev3_motor_steer(left_motor, right_motor, 40, 0); //�]�|�h�~�̑O�i
    //tslp_tsk(90);
	
    ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(2000);
    ev3_motor_rotate(tail_motor, -13, 4, true);   //��i�K�ڂ̐K��
	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	tslp_tsk(1500);
	//ev3_motor_rotate(tail_motor, -1, 4, true);   //��i�K�ڂ̐K��
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
	/*distance += 12.5;								//��Q���Ƃ̋������v�Z���i�ދ������o��
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
		while(anglet[i] > left_angle){						//�Q�[�g��ʉ߂��鏈��
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
		ev3_motor_rotate(right_motor, 400/*360*/, 10, true);   //��i�K�ڂ̐K��
		ev3_motor_stop(left_motor, false);
		ev3_motor_stop(right_motor, false);
		tslp_tsk(1000);
		ev3_motor_rotate(left_motor, -250/*270*/, 10, true);   //��i�K�ڂ̐K��
		ev3_motor_stop(left_motor, false);
		ev3_motor_stop(right_motor, false);
		tslp_tsk(1000);
	}
    ev3_motor_reset_counts(left_motor);
    left_angle = ev3_motor_get_counts(left_motor);
	while(anglet[2] > left_angle){						//2��� �Q�[�g��ʉ߂��鏈��
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
