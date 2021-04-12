/*
  Copyright (C) 2009 Sung Ho Park
  Contact: ubinos.org@gmail.com

  This file is part of the exe_helloworld component of the Ubinos.

  GNU General Public License Usage
  This file may be used under the terms of the GNU
  General Public License version 3.0 as published by the Free Software
  Foundation and appearing in the file license_gpl3.txt included in the
  packaging of this file. Please review the following information to
  ensure the GNU General Public License version 3.0 requirements will be
  met: http://www.gnu.org/copyleft/gpl.html.

  GNU Lesser General Public License Usage
  Alternatively, this file may be used under the terms of the GNU Lesser
  General Public License version 2.1 as published by the Free Software
  Foundation and appearing in the file license_lgpl.txt included in the
  packaging of this file. Please review the following information to
  ensure the GNU Lesser General Public License version 2.1 requirements
  will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.

  Commercial Usage
  Alternatively, licensees holding valid commercial licenses may
  use this file in accordance with the commercial license agreement
  provided with the software or, alternatively, in accordance with the
  terms contained in a written agreement between you and rightful owner.
*/

/* -------------------------------------------------------------------------
	Include
 ------------------------------------------------------------------------- */
#include "../ubiconfig.h"

 // standard c library include
#include <stdio.h>
#include <sam4e.h>

// ubinos library include
#include "itf_ubinos/itf/bsp.h"
#include "itf_ubinos/itf/ubinos.h"
#include "itf_ubinos/itf/bsp_fpu.h"


// chipset driver include
#include "ioport.h"
#include "pio/pio.h"

// new estk driver include
#include "lib_new_estk_api/itf/new_estk_led.h"
#include "lib_new_estk_api/itf/new_estk_glcd.h"
#include "lib_sensorcalib/itf/lib_sensorcalib.h"
#include "lib_EV3_sensor/itf/lib_EV3_sensor.h"
#include "lib_switch/itf/lib_switch.h"
#include "lib_sensor/itf/lib_sensor.h"
#include "lib_i2c/itf/lib_i2c.h"
#include "lib_motor_driver/itf/lib_motor_driver.h"
#include "lib_ubi_pio/itf/lib_ubi_pio.h"

// custom library header file include
//#include "../../lib_default/itf/lib_default.h"

// user header file include


/* define */
#define R_MOTOR_PORT	0	//오른쪽 모터 포트 번호
#define L_MOTOR_PORT	1	//왼쪽 모터 포트 번호

#define LIGHT_SENSOR_PORT	0 	//광센서 포트 번호
#define ULTRA_SENSOR_PORT	1	//초음파센서 포트 번호
#define	SOUND_SENSOR_PORT	3	//피치센서 포트 번호

/* 센서 값 */
#define SOUND_MIN_VALUE		150	//피치센서 경계 값

/* 모터 속도 */
#define MAX_SPEED	850	//최대 모터 스피드  750
#define DEFAULT_SPEED	200	//기본 모터 스피드 500
#define BLACK_SPEED 	300
#define MIN_SPEED		100

/* 현재 로봇의 상태 값 */
#define STATE_ESCAPE	0	//로봇이 레벨 0에 있을 때
#define	STATE_CORRECT	1	//로봇이 목표지점에 가까워질 때
#define STATE_WRONG	2	//로봇이 목표지점에서 멀어질 때
#define STATE_GOAL	3	//로봇이 목표지점에 도달했을 때
#define STATE_YESCAPE	4
#define STATE_YWRONG	5
#define STATE_GWRONG	6


/* 현재 로봇의 위치 */
ev3_color_t ev3_color;

/* 컬러센서 모드 */
color_mode_t sensor_mode = COL_COLOR;
#define MAC_ARRAY_SIZE	5

/* 경기 관련 상수 */
#define LOCK	1
#define	UNLOCK	0

#define	PLAY_TIME	60	//경기 시간
#define TIME_OUT	700	//회전 함수 타임 아웃



/* -------------------------------------------------------------------------
	Global variables
 ------------------------------------------------------------------------- */
int flag = LOCK;
int current_state = STATE_CORRECT;
int current_color = WHITE;
int previous_color = WHITE;

int Gain1 = 5; //회전 함수 제어 계수(회전 각도)
int Gain2 = 7; //회전 함수 제어 계수(회전 속도) //7

/* 태스크 */
void start_end_task(void);
void move_robot_task(void);
void check_state_task(void);

/* 함수 */
void waiting(void);
void turn_right(int degree);
void turn_left(int degree);
void random_turn(void);
void go_forward(int speed);
void go_backward(int speed);


/* 인터럽트 서비스 루틴 */
void sw1_isr(void);
void sw2_isr(void);
//void sound_isr(void);

int usrmain(int argc, char* argv[]) {
	int r;

	printf("\n\n\n\r");
	printf("================================================================================\n\r");
	printf("exe_hw2_test (build time: %s %s)\n\r", __TIME__, __DATE__);
	printf("================================================================================\n\r");

	motor_init();
	encoder_init();
	sensor_init(0, NXT_DIGITAL_SENSOR_SONA, 0, NXT_SENSOR_SOUND);
	ev3_sensor_init(LIGHT_SENSOR_PORT, sensor_mode);
	glcd_init();
	switch_init(sw1_isr, sw2_isr);

	/* 태스크 생성 */

	r = task_create(NULL, start_end_task, NULL, task_gethighestpriority(), 500, NULL);

	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	r = task_create(NULL, check_state_task, NULL, task_gethighestpriority() - 1, 500, NULL);

	if (0 != r) {
		logme("fail at task_create\r\n");
	}


	r = task_create(NULL, move_robot_task, NULL, task_gethighestpriority() - 1, 500, NULL);

	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	printf("\nStart Robot\r\n");
	ubik_comp_start();
	return 0;
}


/* 피치센서 값을 체크하여 경기를 시작하고 1분 뒤에 정지시킨다.
 * 로봇의 초기 위치(레벨0)에서 빠져
 * 나온다. */
void start_end_task(void) {
	flag = LOCK;
	waiting();


	while (sensor_get(SOUND_SENSOR_PORT) > SOUND_MIN_VALUE) {
		glcdGotoChar(0, 0);
		glcd_printf("val : %d\n", ev3_sensor_get(0));
		glcd_printf("sound : %d\n", sensor_get(3));
		printf("sound : %d\n\r color: %s\n\r", sensor_get(3), current_color);
	}



	turn_right(80);
	//go_forward(0);
	go_backward(MAX_SPEED); //max
	bsp_busywaitms(10);
	//*go_forward(0);
	task_sleep(PLAY_TIME * 1000);
	go_forward(0);
	while (1);
}



/* 로봇의 현재 상태에 따른 움직임을 정의한다.
 * 다양한 상태에 따른 로봇의 움직임을 정의할 수 있다. */
void move_robot_task(void) {
	//	go_forward(0);
	while (1) {
		switch (current_state) {
		case STATE_ESCAPE:
			go_forward(500);
			bsp_busywaitms(5);
			//go_backward(DEFAULT_SPEED);
			turn_right(80);
			bsp_busywaitms(5);
			go_backward(DEFAULT_SPEED);  //MAX
			//bsp_busywaitms(5);  //**

			break;
		case STATE_WRONG:  //멀어질때
		//	go_backward(0);
		//	bsp_busywaitms(5);
			turn_right(170);
			bsp_busywaitms(5);
			go_backward(DEFAULT_SPEED);
			break;

		case STATE_GWRONG:
			go_forward(DEFAULT_SPEED);
			bsp_busywaitms(5);
			turn_right(80);

			//go_backward(MIN_SPEED);
			break;

		case STATE_GOAL:
			go_backward(BLACK_SPEED);
			bsp_busywaitms(5);  //**
			go_forward(BLACK_SPEED);
			bsp_busywaitms(5);  //**

		//	go_forward(0);
		//	turn_left(180);
		//	go_backward(500);
			//bsp_busywaitms(50);
			break;
		case STATE_YESCAPE:
			go_backward(BLACK_SPEED);
			/*
			else {
				random_turn();
				//turn_right(180);
				//bsp_busywaitms(5);
			//	go_backward(DEFAULT_SPEED);
			}
			*/
			break;
		case STATE_YWRONG:
			go_forward(DEFAULT_SPEED);
			bsp_busywaitms(5);
			turn_right(80);
			go_backward(DEFAULT_SPEED);
			break;
		default:   //가까워질때
			go_backward(500);
			break;
		}
		task_sleep(5);
	}
}



/* 광센서 값을 읽어들여 현재의 로봇 위치 및 상태를 체크한다.
  로봇의 위치에 따라 다양한 상태를 정의할 수 있다. */

void check_state_task(void) {
	int i, j, cnt, state = 0;
	int color_array[MAC_ARRAY_SIZE];

	/* 경기장 바닥 색의 경계선 부분에서
	  흰색 값으로 인식하므로 필터링을 거친다. */
	while (1) {
		for (i = 0; i < MAC_ARRAY_SIZE; i++) {
			color_array[i] = ev3_sensor_get(0);
			task_sleep(5);
		}

		for (i = 0; i < (MAC_ARRAY_SIZE - 1); i++) {
			cnt = 1;

			for (j = 0; j < MAC_ARRAY_SIZE; j++) {
				if (color_array[i] == color_array[j])
					cnt++;
			}

			if (cnt > (MAC_ARRAY_SIZE / 2)) {
				current_color = color_array[i];
				break;
			}
		}

		state = current_color - previous_color;

		switch (current_color) {
		case WHITE:
			current_state = STATE_ESCAPE;
			break;

		case YELLOW:
		case RED:
			if (state < 0) {
				current_state = STATE_CORRECT;
			}
			else {
				if ((previous_color == 5) || (previous_color == 4)) {
					current_state = STATE_YESCAPE;
				}
				else {
					current_state = STATE_YWRONG;
				}
			}
			//current_state = (state < 0 ) ? STATE_CORRECT : STATE_YESCAPE;
			break;
		case GREEN:
			current_state = (state <= 0) ? STATE_CORRECT : STATE_GWRONG;
			break;
		case BLUE:
			current_state = (state <= 0) ? STATE_CORRECT : STATE_GWRONG; //
			break;
		case BLACK:
			current_state = STATE_GOAL;
			break;
		default:
			break;
		}
		previous_color = current_color;
		task_sleep(10); //

	}
}


/* 스모로봇을 오른쪽으로 입력한 각도만큼 회전시킨다.
  원하는 각도만큼 스모로봇이 회전하지 못했을 때 timeout 값에 의해서 빠져나온다.
  Gain1 과 Gain2의 제어 계수를 갖는다. */

void turn_right(int degree) {
	int speed = 300;
	int diff_degree = 0;
	int timeout = 0;

	encoder_reset(R_MOTOR_PORT);
	encoder_reset(L_MOTOR_PORT);
	timeout = TIME_OUT;

	do {
		diff_degree = (encoder_get(R_MOTOR_PORT) - encoder_get(L_MOTOR_PORT)) - (degree * Gain1);
		speed = Gain2 * diff_degree;
		motor_set(R_MOTOR_PORT, (-speed));
		motor_set(L_MOTOR_PORT, speed);
		timeout--;
	} while ((diff_degree != 0) && (timeout != 0));
}

/*스모로봇을 왼쪽으로 입력한 각도만큼 회전시킨다.
 원하는 각도만큼 스모로봇이 회전하지 못했을 때 timeout 값에 의해서 빠져나온다.
  Gain1 과 Gain2의 제어 계수를 갖는다. */

void turn_left(int degree) {
	int speed = 0;
	int diff_degree = 0;
	int timeout = 0;

	encoder_reset(R_MOTOR_PORT);
	encoder_reset(L_MOTOR_PORT);
	timeout = TIME_OUT;

	do {
		diff_degree = (encoder_get(R_MOTOR_PORT) - encoder_get(L_MOTOR_PORT)) - (degree * Gain1);
		speed = Gain2 * diff_degree;
		motor_set(R_MOTOR_PORT, speed);
		motor_set(L_MOTOR_PORT, (-speed));
		timeout--;
	} while ((diff_degree != 0) && (timeout != 0));
}

/*랜덤하게 스모로봇을 회전시킨다.
  1~180도의 각도 범위를 갖는다 */

void random_turn(void) {
	int ran_num;
	ran_num = (rand() % 120 + 1);

	if (ran_num % 2) {
		turn_right(ran_num);
	}
	else {
		turn_left(ran_num);
	}
}

/*로봇을 뒤로 움직이게 한다. */
void go_forward(int speed) {
	motor_set(R_MOTOR_PORT, speed);
	motor_set(L_MOTOR_PORT, speed);
}

/*로봇을 앞으로 움직이게 한다.*/
void go_backward(int speed) {
	motor_set(R_MOTOR_PORT, -speed);
	motor_set(L_MOTOR_PORT, -speed);
}


/*버튼 1을 누르기 전까지 대기한다. */
void waiting(void) {
	while (flag == LOCK) {
	}
	flag = LOCK;
}


/* 버튼 1과 2의 인터럽트 서비스 루틴*/
void sw1_isr(void) {
	task_sleep(1000);
	flag = UNLOCK;
}

void sw2_isr(void) {
}

/*
void sound_isr(void){
	if(sensor_get(3)<SOUND_MIN_VALUE){
		task_sleep(1000);
		flag = UNLOCK;
	}

}

*/
