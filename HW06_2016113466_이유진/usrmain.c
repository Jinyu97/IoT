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
#include "lib_motor_driver/itf/lib_motor_driver.h"
#include "lib_can/itf/lib_can.h"

// custom library header file include
//#include "../../lib_default/itf/lib_default.h"

// user header file include

/* -------------------------------------------------------------------------
	Global variables
 ------------------------------------------------------------------------- */

#define CAN_DEV_ID 0
static char _g_str_lcd[9] = "STOP\0";

/* -------------------------------------------------------------------------
	Prototypes
 ------------------------------------------------------------------------- */
static void lcd_outputtask(void);
static void can_slavetask(void);

/* -------------------------------------------------------------------------
	Function Definitions
 ------------------------------------------------------------------------- */
int usrmain(int argc, char * argv[]) {
	int r;

	printf("\n\n\n\r");
	printf("================================================================================\n\r");
	printf("exe_can_test (build time: %s %s)\n\r", __TIME__, __DATE__);
	printf("================================================================================\n\r");


	glcd_init();

	r = task_create(NULL, lcd_outputtask, NULL, task_getmiddlepriority(), 256, "outputtask");
	if (0 != r) {
		logme("fail at task_create\r\n");
	}
	r = task_create(NULL, can_slavetask, NULL, task_getmiddlepriority(), 256, "can_slavetask");
		if (0 != r) {
			logme("fail at task_create\r\n");
	}

	ubik_comp_start();

	return 0;
}


static void lcd_outputtask(){
	glcd_clear();

//1st line
	glcdGotoChar(1,1);
	glcd_printf("CAN Slave");

	while(1){
//2nd line
		glcdGotoChar(1,2);
		glcd_printf("%s", _g_str_lcd);

		task_sleep(500);
	}
}




static void can_slavetask() {

	uint8_t tx_buf[8] = "slave \0";
	uint8_t rx_buf[9];
	//char tx_buf[8] = "slave \0";
	//char rx_buf[9];

	uint8_t rx_length;
	int count = 0;
	CAN_PORT CAN_PORT1;
	CAN_PORT1.cbox_num = 0;

//open the CAN device
	can_open(CAN_DEV_ID, CAN_BPS_500K);

//open the CAN port
	can_port_open(CAN_DEV_ID, &CAN_PORT1, 1, true, 255);
	can_port_set_protocol(CAN_DEV_ID, &CAN_PORT1, CAN_PROTOCOL_2_0_A);
	can_port_set_recvid(CAN_DEV_ID, &CAN_PORT1, 0);
	can_port_set_recvidmask(CAN_DEV_ID, &CAN_PORT1, 0);

//set the CAN port
	can_port_set(CAN_DEV_ID, &CAN_PORT1);

	task_sleep(1000);

	motor_init();

	for (;;) {

// send CAN data to the CAN slave
	//can_port_send(CAN_DEV_ID, &CAN_PORT1, tx_buf, sizeof(tx_buf), 0);

// receive CAN data from the CAN slave("MASTER ..")
	rx_length = can_port_recv(CAN_DEV_ID, &CAN_PORT1, rx_buf, 8, 0);

	if(rx_length > 0) {
//setup the LCD output data
		_g_str_lcd[8] = '\0';
		memcpy(_g_str_lcd, rx_buf, rx_length);
	}


	if(rx_buf[7] == '5') motor_set(0,500);
	else if(rx_buf[7] == '3') motor_set(0,300);
	else if(rx_buf[7] == '2') motor_set(0,200);
	else motor_set(0,0);

	printf("%s", _g_str_lcd);
	task_sleep(1000);

	}
}

