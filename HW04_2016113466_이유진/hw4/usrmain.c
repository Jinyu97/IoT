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

// custom library header file include
//#include "../../lib_default/itf/lib_default.h"

// user header file include

/* -------------------------------------------------------------------------
	Global variables
 ------------------------------------------------------------------------- */
#define IDLE_PRIORITY 0



task_pt task_1_handle;
task_pt task_2_handle;

/* -------------------------------------------------------------------------
	Prototypes
 ------------------------------------------------------------------------- */

static void task_1(void * arg);
static void task_2(void * arg);




/* -------------------------------------------------------------------------
	Function Definitions
 ------------------------------------------------------------------------- */

int MOTOR_COLOR;


int usrmain(int argc, char * argv[]) {
	int r;


	printf("\n\n\n\r");
	printf("================================================================================\n\r");
	printf("exe_hw4_test (build time: %s %s)\n\r", __TIME__, __DATE__);
	printf("================================================================================\n\r");

	r = task_create(&task_1_handle, task_1, NULL, IDLE_PRIORITY+2, 256, "task_1");
			if (0 != r) {
				logme("fail at task_create\r\n");
			}

	r = task_create(&task_2_handle, task_2, NULL, IDLE_PRIORITY+1, 256, "task_2");
			if (0 != r) {
				logme("fail at task_create\r\n");
			}

	ubik_comp_start();

	return 0;
}

static void task_1(void * arg) {
	glcd_init();
	ev3_sensor_init(0, COL_COLOR);
	printf("task0\n\r");

	/* calibration sensor : port0, light sensor */
	calibEV3Sensor(0, MAX_COLOR_LEVEL, color_value);





	for(;;){
		glcd_clear();
		printf("task1\n\r");
		/* check sensor level */
		switch(get_level(ev3_sensor_get(0), MAX_COLOR_LEVEL, color_value))
		{
		case COLOR_RED:
			glcd_printf("RED VAL : %d\n", ev3_sensor_get(0));
			MOTOR_COLOR = 1;
			break;

		case COLOR_YELLOW:
			glcd_printf("YELLOW VAL : %d\n", ev3_sensor_get(0));
			MOTOR_COLOR = 2;
			break;

		case COLOR_BLUE:
			glcd_printf("BLUE VAL : %d\n", ev3_sensor_get(0));
			MOTOR_COLOR = 3;
			break;

		case COLOR_GREEN:
			glcd_printf("GREEN VAL : %d\n", ev3_sensor_get(0));
			MOTOR_COLOR = 4;
			break;

		default:
			glcd_printf("UNDEFINE\n");
			break;
		}
		task_sleep(1000);		// 1s delay
	}
}


static void task_2(void * arg) {

	motor_init();
	for(;;){
		printf("task2\n\r");
		switch(MOTOR_COLOR)
		{
		case 1:
			motor_set(0,100);
			break;

		case 2:
			motor_set(0,300);
			break;

		case 3:
			motor_set(0,500);
			break;

		case 4:
			motor_set(0,700);
			break;

		default:
			break;
				}
			glcd_clear();
			task_sleep(500);		// 500ms delay

			}
		}





