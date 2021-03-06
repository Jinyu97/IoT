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
#include <stdlib.h>
#include <sam4e.h>

// ubinos library include
#include "itf_ubinos/itf/bsp.h"
#include "itf_ubinos/itf/ubinos.h"
#include "itf_ubinos/itf/bsp_fpu.h"
#include "itf_ubinos/itf/ubik_mutex.h"
#include "itf_ubinos/itf/bsp_intr.h"


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
#include "lib_new_estk_api/itf/new_estk_ioport_set.h"

#include "lib_bluetooth/itf/BT_Module_Interface.h"
#include "lib_bluetooth/itf/lib_BT.h"
// custom library header file include
//#include "../../lib_default/itf/lib_default.h"

// user header file include

/* -------------------------------------------------------------------------
	Global variables
 ------------------------------------------------------------------------- */
#define BLE_MODULE_ID_0	0X01
#define BLE_MODULE_ID_1	0X02
#define BLE_MODULE_ID_2	0X03
#define BLE_MODULE_ID_3	0X60

#define C_MOTOR_PORT 0
#define S_MOTOR_PORT 1
#define B_MOTOR_PORT 2


#define LIGHT_PORT 0
#define ULTRA_PORT 1
#define SOUND_PORT 3


 //print_packet is a 19 bytes without Command
uint8_t print_packet[DATA_SEND_BUFFER_SIZE] = { 0, };

//Main Task Massage Queue
msgq_pt BT_user_event_queue;
//Mutex for print_packet
mutex_pt _g_mutex;
sem_pt _gsem;
int count = 0;
int distance = 0;
int input = 0;
int sugar, prima = 0;
int start = 0;



/* -------------------------------------------------------------------------
	Prototypes
 ------------------------------------------------------------------------- */
 //static void ultratask(void * arg);
static void BT_peripheraltask(void* arg);
static void coffeetask(void* arg);
static void cuptask(void* arg);
static void sugarprima(void* arg);
static void stop(void* arg);

static void timer_isr(void);
static void print_lcd(void* arg);

/* -------------------------------------------------------------------------
	Function Definitions
 ------------------------------------------------------------------------- */
int usrmain(int argc, char* argv[]) {
	led_init();
	glcd_init();
	motor_init();
	sensor_init(0, NXT_DIGITAL_SENSOR_SONA, 0, NXT_SENSOR_SOUND);
	int r;

	printf("\n\n\n\r");
	printf("================================================================================\n\r");
	printf("exe_ultrasonic_test (build time: %s %s)\n\r", __TIME__, __DATE__);
	printf("================================================================================\n\r");

	/*	r = task_create(NULL, rootfunc, NULL, task_getmiddlepriority(), 256, "root");
		if (0 != r) {
			logme("fail at task_create\r\n");

		}*/

	r = sem_create(&_gsem);
	if (0 != r) {
		logme("fail at sem_create\r\n");
	}

	r = task_create(NULL, print_lcd, NULL, task_getmiddlepriority() + 1, 256, "print_lcd");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	/*
		r = task_create(NULL, ultratask, NULL, task_getmiddlepriority()+2, 256, "ultra");
			if (0 != r) {
				logme("fail at task_create\r\n");
		}
	*/
	r = task_create(NULL, BT_peripheraltask, NULL, task_getmiddlepriority(), 256, "bt");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	r = task_create(NULL, coffeetask, NULL, task_getmiddlepriority() - 1, 256, "coffee");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	r = task_create(NULL, sugarprima, NULL, task_getmiddlepriority() - 2, 256, "sugar");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}
	/*
		r = task_create(NULL, cuptask, NULL, task_getmiddlepriority(), 256, "cup");
			if (0 != r) {
				logme("fail at task_create\r\n");
		}*/
	r = task_create(NULL, stop, NULL, task_getmiddlepriority() - 3, 256, "stop");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}

	r = msgq_create(&BT_user_event_queue, sizeof(BT_Evt_t), MAIN_MSGQ_MAX_COUNT);
	if (0 != r) {
		logme("fail at msgq_create\r\n");
	}



	PMC->PMC_PCER0 = 1 << ID_TC3;

	TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
	TC1->TC_CHANNEL[0].TC_IDR = 0xFFFFFFFF;

	TC1->TC_CHANNEL[0].TC_CMR = (TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_CPCTRG);
	TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;

	intr_connectisr(TC3_IRQn, timer_isr, 0x40, INTR_OPT__LEVEL);

	intr_enable(TC3_IRQn);

	TC1->TC_CHANNEL[0].TC_RC = 32768;

	TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;


	ubik_comp_start();

	return 0;
}


static void timer_isr(void) {
	unsigned int reg;

	sem_give(_gsem);
	count++;

	reg = TC1->TC_CHANNEL[0].TC_SR;
	printf("HW_TIMER [TC:%d] \r\n", reg);

}

static void print_lcd(void* arg) {
	ev3_sensor_init(LIGHT_PORT, COL_COLOR);

	for (;;) {
		sem_take(_gsem);
		glcdGotoChar(0, 0);

		if (ev3_sensor_get(LIGHT_PORT) != 6) {
			glcd_clear();

			glcd_printf("PAUSE\n");
			motor_set(C_MOTOR_PORT, 300);
			bsp_busywaitms(80);
			motor_set(C_MOTOR_PORT, 0);
			motor_set(S_MOTOR_PORT, -100);
			bsp_busywaitms(15);
			motor_set(S_MOTOR_PORT, 0);


			glcd_clear();
			glcdGotoChar(0, 0);
			glcd_printf("THE END");
			bsp_busywaitms(40);
			end();
			glcd_clear();

			for (int i = 0; i < 3; i++) {
				led_on(LED1);
				bsp_busywaitms(5);
				led_off(LED1);
				//	bsp_busywaitms(5);
				led_on(LED2);
				bsp_busywaitms(5);
				led_off(LED2);
				//	bsp_busywaitms(5);
				led_on(LED3);
				bsp_busywaitms(5);
				led_off(LED3);

			}

			bsp_busywaitms(10000);
			glcd_clear();
			end();
			//bsp_busywait(10000);


		}

		if (sensor_get(SOUND_PORT) < 430) {
			motor_set(B_MOTOR_PORT, 0);
			glcd_clear();
			glcdGotoChar(0, 0);
			led_on(LED1);
			led_on(LED2);
			led_on(LED3);
			//glcd_printf("%d\n",sensor_get(SOUND_PORT));

			glcd_printf("Thank YOU\nTake it!!\n");
			bsp_busywaitms(100);
			end();
			glcd_clear();
			led_off(LED1);
			led_off(LED2);
			led_off(LED3);
			bsp_busywaitms(10000);
		}


		//	glcd_printf("HW_TIMER : %3d", count);
		task_sleep(100);
	}
}

static void sugarprima(void* arg) {

	// prima

		//glcd_printf("sugar prima start\n");

	switch (prima) {
	case 1:
		motor_set(C_MOTOR_PORT, -250);
		bsp_busywaitms(10);
		motor_set(C_MOTOR_PORT, 0);
		bsp_busywaitms(50);
		motor_set(C_MOTOR_PORT, 300);
		bsp_busywaitms(70);//80
		break;
	case 2:
		motor_set(C_MOTOR_PORT, -250);
		bsp_busywaitms(16);
		motor_set(C_MOTOR_PORT, 0);
		bsp_busywaitms(50);
		motor_set(C_MOTOR_PORT, 300);
		bsp_busywaitms(80);
		break;
	case 3:
		motor_set(C_MOTOR_PORT, -300);
		bsp_busywaitms(20);
		motor_set(C_MOTOR_PORT, 0);
		bsp_busywaitms(50);
		motor_set(C_MOTOR_PORT, 300);
		bsp_busywaitms(90);
		break;

	default:
		motor_set(C_MOTOR_PORT, 300);
		bsp_busywaitms(50);
		break;
	}
	//	bsp_busywaitms(5000);
	//	motor_set(0,-150);
	//	task_sleep(1000);
	//	motor_set(0,150);
	//	task_sleep(2000);
	motor_set(0, 0);


	//sugar task

//	glcd_init();
	//glcdGotoChar(0,0);
//	motor_init();



	for (int i = 0; i < sugar; i++) {
		motor_set(S_MOTOR_PORT, 150);
		bsp_busywaitms(15);//20
		motor_set(S_MOTOR_PORT, 0);
		bsp_busywaitms(10);
	}




	task_sleep(100);


}



static void coffeetask(void* arg) {


	//coffee task
	glcd_clear();
	bsp_busywaitms(10);
	glcdGotoChar(0, 0);
	bsp_busywaitms(5);
	glcd_printf("Wait a second\n");
	bsp_busywaitms(10);



	motor_set(C_MOTOR_PORT, -300);
	bsp_busywaitms(19);
	motor_set(C_MOTOR_PORT, 0);
	bsp_busywaitms(80);
	motor_set(C_MOTOR_PORT, -300);
	bsp_busywaitms(12);
	motor_set(C_MOTOR_PORT, 0);
	bsp_busywaitms(80);


	//	bsp_busywaitms(1000);

	bsp_busywaitms(150);
	task_sleep(1000);

}





static void stop(void* arg) {
	for (;;) {
		motor_set(B_MOTOR_PORT, 500);
		bsp_busywaitms(2);
		motor_set(B_MOTOR_PORT, 0);
		bsp_busywaitms(1);
		motor_set(B_MOTOR_PORT, -500);
		bsp_busywaitms(2);
		motor_set(B_MOTOR_PORT, 0);
		bsp_busywaitms(1);
	}

	glcd_clear();

	task_sleep(1000);
}



static void BT_peripheraltask(void* arg) {

	int dis;
	glcd_clear();
	glcdGotoChar(0, 0);
	while (1) {
		dis = sensor_get(ULTRA_PORT);
		if (dis < 30) {
			glcd_printf("Open Your\n");
			bsp_busywaitms(5);
			glcdGotoChar(0, 2);
			glcd_printf("Smart phone\n");
			bsp_busywaitms(5);
			glcdGotoChar(0, 4);
			glcd_printf("Application\n");
			bsp_busywaitms(5);
			start = 1;
			break;
		}

	}

	bsp_busywaitms(50);



	glcd_clear();
	bsp_busywaitms(20);//
	glcdGotoChar(0, 0);
	bsp_busywaitms(15);
	glcd_printf("Enter the\n");
	bsp_busywaitms(5);
	glcdGotoChar(0, 2);
	glcd_printf("amount of\n");
	bsp_busywaitms(5);
	glcdGotoChar(0, 4);
	glcd_printf("sugar&prima\n");
	bsp_busywaitms(60);

	int r = 0;
	module_id_st BT_ID;
	BT_Evt_t BT_usr_msgRXBuffer = { 0, };

	//set BT_module_ID
	BT_ID.module_id[0] = BLE_MODULE_ID_0;
	BT_ID.module_id[1] = BLE_MODULE_ID_1;
	BT_ID.module_id[2] = BLE_MODULE_ID_2;
	BT_ID.module_id[3] = BLE_MODULE_ID_3;

	//bsp_busywaitms(200);

	//task_sleep(200);

	BT_INIT(INIT_ROLE_PERIPHERAL, BT_ID, BT_user_event_queue);

	BT_ADV_START();

	//glcd_printf("ble\n");

	for (;;) {
		//bsp_busywaitms(100);
		r = msgq_receive(BT_user_event_queue, (unsigned char*)&BT_usr_msgRXBuffer);

		if (0 != r) {
			logme("fail at msgq_receive\r\n");
		}
		else {
			switch (BT_usr_msgRXBuffer.status) {
			case BT_EVT_PE_DATA_READ: {
				//motor setting
				mutex_lock(_g_mutex);
				input = (int)((BT_usr_msgRXBuffer.msg[1] << 8) | (BT_usr_msgRXBuffer.msg[0]));


				for (int i = 0; i < 20; i++) {
					print_packet[i] = BT_usr_msgRXBuffer.msg[i];

				}


				mutex_unlock(_g_mutex);

				sugar = input / 10;
				prima = input % 10;
				glcdGotoChar(0, 3);
				glcd_printf("sugar : %d\n", sugar);
				bsp_busywaitms(5);
				glcdGotoChar(0, 4);
				glcd_printf("prima : %d\n", prima);
				bsp_busywaitms(15);
				//glcd_printf("before\n");
				//bsp_busywaitms(20);//100
				//glcd_printf("after\n");
				//send same msg
			//	BT_DATA_SEND(INIT_ROLE_PERIPHERAL,print_packet);

				task_sleep(500);
			}
									break;

			case BT_EVT_DISCONNECTED:

				break;

			case BT_EVT_CONNECTED:

				break;
			}
		}
	}
}

/*
static void lcd_outputtask()
{
	task_sleep(300);
	//printf_packet is a 19 bytes without CMD
	while(1)
	{
		mutex_lock(_g_mutex);
		//2nd line
		glcdGotoChar(1,2);
			glcd_printf("%d",motorset);
		mutex_unlock(_g_mutex);

		task_sleep(200);
	}
}
*/
