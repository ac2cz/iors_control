/*
 * config.h
 *
 *  Created on: Sep 28, 2022
 *      Author: g0kla
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "debug.h"

#define VERSION __DATE__ " iors_control - Version 0.1"
#define DEBUG 1
#define true 1
#define false 0
#define MAX_CALLSIGN_LEN 10
#define MAX_FILE_PATH_LEN 256
#define AX25_MAX_DATA_LEN 2048
#define AGW_PORT 8000

/* These global variables are not in the config file */
extern int g_run_self_test;    /* true when the self test is running */
extern int g_verbose;          /* print verbose output when set */

/* Define paramaters for config file */
#define MAX_CONFIG_LINE_LENGTH 128
#define BIT_RATE "bit_rate"
#define CALLSIGN "callsign"
#define MAX_FRAMES_IN_TX_BUFFER "max_frames_in_tx_buffer"
#define SERIAL_DEV "serial_device"

extern int g_bit_rate; /* the bit rate of the TNC - 1200 4800 9600. Change actual value in DireWolf) */
extern char g_callsign[MAX_CALLSIGN_LEN];
extern int g_max_frames_in_tx_buffer;
extern char g_serial_dev[MAX_FILE_PATH_LEN]; // device name for the serial port for Rig control

void load_config();

#endif /* CONFIG_H_ */
