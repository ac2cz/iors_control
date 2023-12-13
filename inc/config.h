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

#include <stdbool.h>
#include "debug.h"
#include "common_config.h"

#define VERSION __DATE__ " iors_control - Version 0.1"

#define TRUE true
#define FALSE false

/* These global variables are not in the config file */
extern int g_run_self_test;    /* true when the self test is running */
extern int g_verbose;          /* print verbose output when set */
extern int g_iors_control_state; /* The state of the IORS control state machine */
extern int g_radio_pm;
extern int g_radio_cross_band_repeater;

/* Define paramaters for config file */
#define MAX_CONFIG_LINE_LENGTH 128
#define CONFIG_BIT_RATE "bit_rate"
#define CONFIG_CALLSIGN "callsign"
#define CONFIG_MAX_FRAMES_IN_TX_BUFFER "max_frames_in_tx_buffer"
#define CONFIG_SERIAL_DEV "serial_device"
#define CONFIG_PTT_SERIAL_DEV "ptt_serial_device"
#define CONFIG_RADIO_ID "radio_id_str"
#define CONFIG_RADIO_TYPE "radio_type_str"
#define CONFIG_RADIO_MAIN_FIRMWARE "radio_main_firmware_str"
#define CONFIG_RADIO_PANEL_FIRMWARE "radio_panel_firmware_str"

#define CONFIG_DIREWOLF_PATH "direwolf_path"
#define CONFIG_DIREWOLF_LOGFILE_PATH "direwolf_logfile_path"
#define CONFIG_DIREWOLF_CONFIG_PATH "direwolf_config_path"

#define CONFIG_SSTV_PATH "sstv_path"
#define CONFIG_SSTV_QUEUE_PATH "sstv_queue_path"
#define CONFIG_SSTV_LOGFILE_PATH "sstv_logfile_path"
#define CONFIG_SSTV_SENT "sstv_sent"

#define CONFIG_IORS_LAST_COMMAND_TIME_PATH "iors_last_command_time_path"

#define PID_COMMAND 0xBC
#define PID_FILE 0xBB

extern int g_bit_rate; /* the bit rate of the TNC - 1200 4800 9600. Change actual value in DireWolf) */
extern char g_callsign[MAX_CALLSIGN_LEN];
extern int g_max_frames_in_tx_buffer;
extern char g_serial_dev[MAX_FILE_PATH_LEN]; // device name for the serial port for Rig control
extern char g_ptt_serial_dev[MAX_FILE_PATH_LEN]; // device name for the serial port for Rig control
extern char g_radio_id[15]; // The ID that the radio returns through the serial port
extern char g_radio_type[15]; // The type that the TY command returns through the serial port
extern char g_radio_main_firmware[25]; // The firmware that the FV 0 command returns through the serial port
extern char g_radio_panel_firmware[25]; // The firmware that the FV 1 command returns through the serial port

extern char g_direwolf_path[MAX_FILE_PATH_LEN];
extern char g_direwolf_logfile_path[MAX_FILE_PATH_LEN];
extern char g_direwolf_config_path[MAX_FILE_PATH_LEN];

extern char g_sstv_path[MAX_FILE_PATH_LEN];
extern char g_sstv_queue_path[MAX_FILE_PATH_LEN];
extern char g_sstv_logfile_path[MAX_FILE_PATH_LEN];

extern char g_iors_last_command_time_path[MAX_FILE_PATH_LEN];

void load_config();

#endif /* CONFIG_H_ */
