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
#define R_MOTOR_PORT	0	//������ ���� ��Ʈ ��ȣ
#define L_MOTOR_PORT	1	//���� ���� ��Ʈ ��ȣ

#define LIGHT_SENSOR_PORT	0 	//������ ��Ʈ ��ȣ
#define ULTRA_SENSOR_PORT	1	//�����ļ��� ��Ʈ ��ȣ
#define	SOUND_SENSOR_PORT	3	//��ġ���� ��Ʈ ��ȣ

/* ���� �� */
#define SOUND_MIN_VALUE		150	//��ġ���� ��� ��

/* ���� �ӵ� */
#define MAX_SPEED	850	//�ִ� ���� ���ǵ�  750
#define DEFAULT_SPEED	200	//�⺻ ���� ���ǵ� 500
#define BLACK_SPEED 	300
#define MIN_SPEED		100

/* ���� �κ��� ���� �� */
#define STATE_ESCAPE	0	//�κ��� ���� 0�� ���� ��
#define	STATE_CORRECT	1	//�κ��� ��ǥ������ ������� ��
#define STATE_WRONG	2	//�κ��� ��ǥ�������� �־��� ��
#define STATE_GOAL	3	//�κ��� ��ǥ������ �������� ��
#define STATE_YESCAPE	4
#define STATE_YWRONG	5
#define STATE_GWRONG	6


/* ���� �κ��� ��ġ */
ev3_color_t ev3_color;

/* �÷����� ��� */
color_mode_t sensor_mode = COL_COLOR;
#define MAC_ARRAY_SIZE	5

/* ��� ���� ��� */
#define LOCK	1
#define	UNLOCK	0

#define	PLAY_TIME	60	//��� �ð�
#define TIME_OUT	700	//ȸ�� �Լ� Ÿ�� �ƿ�



/* -------------------------------------------------------------------------
	Global variables
 ------------------------------------------------------------------------- */
int flag = LOCK;
int current_state = STATE_CORRECT;
int current_color = WHITE;
int previous_color = WHITE;

int Gain1 = 5; //ȸ�� �Լ� ���� ���(ȸ�� ����)
int Gain2 = 7; //ȸ�� �Լ� ���� ���(ȸ�� �ӵ�) //7

/* �½�ũ */
void start_end_task(void);
void move_robot_task(void);
void check_state_task(void);

/* �Լ� */
void waiting(void);
void turn_right(int degree);
void turn_left(int degree);
void random_turn(void);
void go_forward(int speed);
void go_backward(int speed);


/* ���ͷ�Ʈ ���� ��ƾ */
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

	/* �½�ũ ���� */

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


/* ��ġ���� ���� üũ�Ͽ� ��⸦ �����ϰ� 1�� �ڿ� ������Ų��.
 * �κ��� �ʱ� ��ġ(����0)���� ����
 * ���´�. */
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



/* �κ��� ���� ���¿� ���� �������� �����Ѵ�.
 * �پ��� ���¿� ���� �κ��� �������� ������ �� �ִ�. */
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
		case STATE_WRONG:  //�־�����
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
		default:   //���������
			go_backward(500);
			break;
		}
		task_sleep(5);
	}
}



/* ������ ���� �о�鿩 ������ �κ� ��ġ �� ���¸� üũ�Ѵ�.
  �κ��� ��ġ�� ���� �پ��� ���¸� ������ �� �ִ�. */

void check_state_task(void) {
	int i, j, cnt, state = 0;
	int color_array[MAC_ARRAY_SIZE];

	/* ����� �ٴ� ���� ��輱 �κп���
	  ��� ������ �ν��ϹǷ� ���͸��� ��ģ��. */
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


/* ����κ��� ���������� �Է��� ������ŭ ȸ����Ų��.
  ���ϴ� ������ŭ ����κ��� ȸ������ ������ �� timeout ���� ���ؼ� �������´�.
  Gain1 �� Gain2�� ���� ����� ���´�. */

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

/*����κ��� �������� �Է��� ������ŭ ȸ����Ų��.
 ���ϴ� ������ŭ ����κ��� ȸ������ ������ �� timeout ���� ���ؼ� �������´�.
  Gain1 �� Gain2�� ���� ����� ���´�. */

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

/*�����ϰ� ����κ��� ȸ����Ų��.
  1~180���� ���� ������ ���´� */

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

/*�κ��� �ڷ� �����̰� �Ѵ�. */
void go_forward(int speed) {
	motor_set(R_MOTOR_PORT, speed);
	motor_set(L_MOTOR_PORT, speed);
}

/*�κ��� ������ �����̰� �Ѵ�.*/
void go_backward(int speed) {
	motor_set(R_MOTOR_PORT, -speed);
	motor_set(L_MOTOR_PORT, -speed);
}


/*��ư 1�� ������ ������ ����Ѵ�. */
void waiting(void) {
	while (flag == LOCK) {
	}
	flag = LOCK;
}


/* ��ư 1�� 2�� ���ͷ�Ʈ ���� ��ƾ*/
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
