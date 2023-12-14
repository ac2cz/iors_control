/*
 * iors_controller.c
 *
 *  Created on: Dec 1, 2023
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
 * IORS_CONTROLLER
 * ===============
 * This state machine manages the radio.
 */

/* System include files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <pthread.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

/* Program includes */
#include "config.h"
#include "radio.h"
#include "iors_controller.h"
#include "agw_tnc.h"
#include "str_util.h"
#include "ax25_tools.h"
#include "iors_command.h"

#define PROCESS_KILLED -9
#define PROCESS_RUNNING 2

#define PID_NOT_RUNNING -99

/* Forward declarations */
void iors_process_event(struct t_iors_event *iors_event);
int sstv_send();
int sstv_stop();
int next_sstv_file(char * filename);
int valid_command(char *from_callsign, unsigned char *data, int len, struct t_iors_event *command_event);
int send_ok(char *from_callsign);
int send_err(char *from_callsign, int err);
int start_program(char * command, char *argv[], char * logfile);
int program_stopped(char * program_name, pid_t pid);
int start_tnc();
int stop_program(char * program_name, pid_t pid);
int connect_tnc();

/* Local variables */
int g_iors_control_state = STATE_INIT;
int sstv_state = STATE_SSTV_INIT;
pthread_t tnc_listen_pthread = 0;
struct t_iors_event command_event;
int tnc_connected = false;
int radio_connected = false;
int cross_band_repeater = false;
int sstv_loop = 0;
int sstv_loop_repeat = 0;
int frame_num = 0;
pid_t tnc_pid = PID_NOT_RUNNING, sstv_pid = PID_NOT_RUNNING;
int ptt_serial = 0; /* Remember the serial file descriptor for the PTT as SSTV keeps it open across states */
int timer_t1_limit = TIMER_T1_DEFAULT_LIMIT;
int timer_t2_limit = TIMER_T2_DEFAULT_LIMIT;
int timer_t3_limit = TIMER_T3_DEFAULT_LIMIT;
int timer_t4_limit = TIMER_T4_DEFAULT_LIMIT;
int TIMER_T1 = 0;
int TIMER_T2 = 0;
int TIMER_T3 = 0;
int TIMER_T4 = 0;
int retries_N = 0;

time_t last_time_checked_radio = 0;
time_t last_time_checked_tnc = 0;
time_t last_time_checked_programs = 0;

char * TEST_SSTV_FILE = "ariss.wav";

/*****
 * iors_control_loop()
 * Loop that monitors for events and sends them to the iors_control_state_machine
 *
 * At boot we need to connect the radio and set the time.
 * We then check the mode we are in.
 *
 *
 */

void iors_control_loop() {
	struct t_iors_event iors_event;
	struct t_agw_frame_ptr frame;

	while(1) {
		time_t now = time(0);
		int rc = EXIT_FAILURE;
		if (last_time_checked_radio == 0) {
			/* Startup - is the right radio connected */
			if (radio_check_initial_connection(g_serial_dev) == EXIT_SUCCESS) {
				// Set the time on the PI from the radio RTC.  NB: This will fail if not run as root
				struct timeval tval;
				if (radio_panel_get_time(g_ptt_serial_dev, &tval.tv_sec) == EXIT_SUCCESS) {
					tval.tv_usec = 0;
					if (settimeofday(&tval, 0) != 0) {
						error_print ("error %d setting time to %d\n", errno, tval.tv_sec);
					}
				}
				iors_event.event = EVENT_RADIO_CONNECTED;
				radio_connected = true;
				last_time_checked_radio = now;
				iors_process_event(&iors_event);
			} else {
				/* Nothing to do here but wait for the radio to become available */
				sleep(10);
			}
		} else if ((now - last_time_checked_radio) > PERIOD_TO_CHECK_RADIO_CONNECTED) {
			/* Check if the radio is connected */
			if (radio_check_connection(g_serial_dev) == EXIT_FAILURE) {
				if (radio_connected) {
					radio_connected = false;
					iors_event.event = EVENT_RADIO_DISCONNECTED;
					iors_process_event(&iors_event);
				}
			} else {
				if (!radio_connected) {
					radio_connected = true;
					iors_event.event = EVENT_RADIO_CONNECTED;
					iors_process_event(&iors_event);
				}
			}

			last_time_checked_radio = now;
		}

		/* The TNC is connected when we need it.  It is checked every time we try to send a packet
		 * This is an extra check in case there is no activity  */
		if (last_time_checked_tnc == 0 || (now - last_time_checked_tnc) > PERIOD_TO_CHECK_TNC_CONNECTED) {
			last_time_checked_tnc = now;
		}

		if (TIMER_T1 != 0 && (now - TIMER_T1) > timer_t1_limit) {
			/* Timer T1 was running and expired */
			iors_event.event = TIMER_T1_EXPIRED;
			TIMER_T1 = 0; // Stop T1
			timer_t1_limit = TIMER_T1_DEFAULT_LIMIT;
			iors_process_event(&iors_event);
		}
		if (TIMER_T2 != 0 && (now - TIMER_T2) > timer_t2_limit) {
			/* Timer T2 was running and expired */
			iors_event.event = TIMER_T2_EXPIRED;
			TIMER_T2 = 0; // Stop T2
			timer_t2_limit = TIMER_T2_DEFAULT_LIMIT;
			iors_process_event(&iors_event);
		}
		if (TIMER_T3 != 0 && (now - TIMER_T3) > timer_t3_limit) {
			/* Timer T3 was running and expired */
			iors_event.event = TIMER_T3_EXPIRED;
			TIMER_T3 = 0; // Stop T3
			timer_t3_limit = TIMER_T3_DEFAULT_LIMIT;
			iors_process_event(&iors_event);
		}
		if (TIMER_T4 != 0 && (now - TIMER_T4) > timer_t4_limit) {
			/* Timer T4 was running and expired */
			iors_event.event = TIMER_T4_EXPIRED;
			TIMER_T4 = 0; // Stop T4
			timer_t4_limit = TIMER_T4_DEFAULT_LIMIT;
			iors_process_event(&iors_event);
		}

		if (last_time_checked_programs == 0 || (now - last_time_checked_programs) > PERIOD_TO_CHECK_PROGRAMS) {
			last_time_checked_programs = now;

			/* Check if the programs are running */
			//if (sstv_pid != PID_NOT_RUNNING) {
			if (program_stopped("SSTV", sstv_pid)) {
				iors_event.event = EVENT_SSTV_EXITED;
				//EVENT_SSTV_FAILED;
				iors_process_event(&iors_event);
			}

			if (program_stopped("TNC", tnc_pid)) {
				exit_tnc_listen_process();
				iors_event.event = EVENT_TNC_EXITED;
				iors_process_event(&iors_event);
			}

		}
		rc = get_next_frame(frame_num, &frame);
		if (rc == EXIT_SUCCESS) {
			frame_num++;
			if (frame_num == MAX_RX_QUEUE_LEN)
				frame_num=0;

			switch (frame.header->data_kind) {
			case 'K': // Monitored frame
				//			debug_print("FRM:%d:",frame_num);
				//			print_header(frame.header);
				//			print_data(frame.data, frame.header->data_len);
				//			debug_print("| %d bytes\n", frame.header->data_len);
				if (strncasecmp(frame.header->call_to, g_callsign, MAX_CALLSIGN_LEN) == 0) { // this was sent to our Callsign
					if (valid_command(frame.header->call_from, frame.data, frame.header->data_len, &command_event) == EXIT_SUCCESS) {
						iors_process_event(&command_event);
					}
				}
				break;
			}
		} else {
			usleep(1000); // sleep 1ms
		}
	}
}

void iors_process_event(struct t_iors_event *iors_event) {
	switch (g_iors_control_state) {
	case STATE_INIT: // Initialization state
		debug_print("STATE: INIT\n");
		if (iors_event->event == EVENT_RADIO_CONNECTED) {
			g_iors_control_state = STATE_RADIO_CONNECTED;
			if (start_tnc() == EXIT_SUCCESS) {
				g_iors_control_state = STATE_TNC_CONNECTED;
			} else {
				TIMER_T1 = time(0); // Start T1
			}
		}
		break;
	case STATE_RADIO_CONNECTED: {
		debug_print("STATE: RADIO CONNECTED\n");
		switch (iors_event->event) {
		case EVENT_RADIO_DISCONNECTED:
			g_iors_control_state = STATE_INIT;
			break;

		case TIMER_T1_EXPIRED:
			retries_N++;
		case EVENT_TNC_DISCONNECTED:
			if (start_tnc() == EXIT_SUCCESS) {
				g_iors_control_state = STATE_TNC_CONNECTED;
				retries_N = 0;
			} else {
				TIMER_T1 = time(0); // Start T1
			}
			break;
		default:
			break;
		}

		break;
	}

	case STATE_TNC_CONNECTED:
		debug_print("STATE: TNC CONNECTED\n");
		switch (iors_event->event) {
		case EVENT_RADIO_DISCONNECTED:
			g_iors_control_state = STATE_INIT;
			break;
		case EVENT_TNC_EXITED:
		case EVENT_TNC_DISCONNECTED:
			if (start_tnc() == EXIT_SUCCESS) {
				g_iors_control_state = STATE_TNC_CONNECTED;
			} else {
				g_iors_control_state = STATE_RADIO_CONNECTED;
				TIMER_T1 = time(0); // Start T1
			}
			break;

		case COMMAND_RX: {
			//debug_print("COMMAND WHEN TNC CONNECTED: ns %d cmd %d\n",iors_event->nameSpace, iors_event->comarg.command);
			switch (iors_event->comarg.command) {
			case SWCmdOpsPM1:
				debug_print("Command: PM1\n");
				if (radio_program_pm(g_serial_dev, RADIO_PM1, RADIO_XBAND_RPT_OFF) != EXIT_SUCCESS) {
					debug_print("ERROR: Can not program the radio\n");
				////	send_err
				}
				break;
			case SWCmdOpsXBandRepeaterMode:
				debug_print("Command: Cross band Repeater mode\n");
				if (radio_set_cross_band_mode() == EXIT_SUCCESS)
					g_iors_control_state = STATE_CROSS_BAND_REPEATER;
				break;
			case SWCmdOpsSSTVSend: {
				debug_print("Command: Enable SSTV\n");
				sstv_loop = 0;
				sstv_loop_repeat = 0;
				sstv_send();

				break;
			}
			case SWCmdOpsSSTVLoop: {
				debug_print("Command: Enable SSTV LOOP\n");
				sstv_loop = 1;
				sstv_loop_repeat = 0;
				sstv_send();

				break;
			}
			case SWCmdOpsAPRSMode: {
				debug_print("Command: APRS mode\n");
				if (radio_set_aprs_mode() == EXIT_SUCCESS)
					g_iors_control_state = STATE_APRS;
				break;
			}
			case SWCmdOpsTime: {
				time_t t = iors_event->comarg.arguments[0] + (iors_event->comarg.arguments[1] << 16);
				//debug_print("Command: Set time to %ld\n", t);
				if (radio_panel_set_time(g_ptt_serial_dev,&t) == EXIT_SUCCESS) {
					struct timeval tval;
					tval.tv_usec = 0;
					tval.tv_sec = t;
					/* Set the time on the PI.  This will fail if not root */
					if (settimeofday(&tval, 0) != 0) {
						error_print ("error %d setting time to %d\n", errno, tval.tv_sec);
					}
				}

				break;
			}
			default:
				break;
			}
			break;
		}

		default:
			break;
		}
		break;

		case STATE_APRS: {
			debug_print("STATE: APRS\n");
			switch (iors_event->event) {
			case EVENT_RADIO_DISCONNECTED:
				g_iors_control_state = STATE_INIT;
				break;
			case EVENT_TNC_EXITED:
			case EVENT_TNC_DISCONNECTED:
				if (start_tnc() == EXIT_SUCCESS) {
					g_iors_control_state = STATE_TNC_CONNECTED;
				} else {
					g_iors_control_state = STATE_RADIO_CONNECTED;
					TIMER_T1 = time(0); // Start T1
				}
				break;
			case COMMAND_RX: {
				switch (iors_event->comarg.command) {
				case SWCmdOpsXBandRepeaterMode:
					debug_print("Command: Cross band Repeater mode\n");
					if (radio_set_cross_band_mode() == EXIT_SUCCESS)
						g_iors_control_state = STATE_CROSS_BAND_REPEATER;
					break;
				case SWCmdOpsSSTVSend: {
					debug_print("Command: Enable SSTV\n");
					sstv_loop = 0;
					sstv_loop_repeat = 0;
					sstv_send();
					break;
				}
				case SWCmdOpsSSTVLoop: {
					debug_print("Command: Enable SSTV Loop\n");
					sstv_loop = 1;
					sstv_loop_repeat = 0;
					sstv_send();
					break;
				}
				default:
					break;
				}
				break;
			}

			default:
				break;
			}
		}
		break;


	case STATE_SSTV: {
		debug_print("STATE: SSTV\n");
		switch (iors_event->event) {
		case EVENT_RADIO_DISCONNECTED:
			sstv_stop();
			TIMER_T1 = time(0);
			g_iors_control_state = STATE_INIT;
			break;

			// TODO - how to handle TNC disconnected.  We dont want to stop SSTV transmit if it is working.  Remember it and process in sstv_stop?

		case EVENT_SSTV_EXITED:
			sstv_stop();
			break;
		case TIMER_T1_EXPIRED:
			sstv_send();
			break;
		case TIMER_T3_EXPIRED:
			sstv_send();
			break;
		case TIMER_T4_EXPIRED:
			sstv_loop = 0;
			sstv_stop();
			break;

		/* We only receive commands between SSTV images because TNC uses the same sound card. */
		case COMMAND_RX: {
			//debug_print("COMMAND WHEN TNC CONNECTED: ns %d cmd %d\n",iors_event->nameSpace, iors_event->comarg.command);
			switch (iors_event->comarg.command) {
			case SWCmdOpsSSTVStop:
				sstv_loop = 0;
				TIMER_T1 = 0;
				timer_t1_limit = TIMER_T1_DEFAULT_LIMIT;
				g_iors_control_state = STATE_TNC_CONNECTED;
				break;

			default:
				break;
			}
			break;
		}

		default:
			break;
		}
		break;
	}
	case STATE_CROSS_BAND_REPEATER:
		debug_print("STATE: CROSS BAND REPEATER\n");
		switch (iors_event->event) {
		case EVENT_RADIO_DISCONNECTED:
			g_iors_control_state = STATE_INIT;
			break;
		case EVENT_TNC_EXITED:
		case EVENT_TNC_DISCONNECTED:
			if (start_tnc() == EXIT_SUCCESS) {
				g_iors_control_state = STATE_TNC_CONNECTED;
			} else {
				// TODO - we leave the cross band repeater as is?
				// So we return to this state if the TNC reconnects?
				g_iors_control_state = STATE_RADIO_CONNECTED;
				TIMER_T1 = time(0); // Start T1
			}
			break;
		case COMMAND_RX: {
			switch (iors_event->comarg.command) {
			case SWCmdOpsSSTVSend: {
				debug_print("Command: Enable SSTV\n");
				sstv_loop = 0;
				sstv_loop_repeat = 0;
				sstv_send();
				break;
			}
			case SWCmdOpsSSTVLoop: {
				debug_print("Command: Enable SSTV Loop\n");
				sstv_loop = 1;
				sstv_loop_repeat = 0;
				sstv_send();
				break;
			}
			case SWCmdOpsAPRSMode:
				debug_print("Command: APRS mode\n");
				if (radio_set_aprs_mode() == EXIT_SUCCESS)
					g_iors_control_state = STATE_APRS;
				break;
			default:
				break;
			}
		    break;
		}

		default:
			break;
		}
		break;

	default :
		break;
	}
}

/**
 * sstv_send()
 * Sub routine to start SSTV sending
 * aplay will be started in the background
 * If it can not be started then T1 begins for retries
 * If it is started then T4 runs as a timeout
 *
 */
int sstv_send() {
	debug_print("SSTV Send\n");
	char filename[MAX_FILE_PATH_LEN];
	if (next_sstv_file(filename) == EXIT_FAILURE) {
		// No file
		if (sstv_loop_repeat) {
			// TODO - reset to start of queue
		} else {
			debug_print("SSTV - No file to send\n");
			TIMER_T1 = 0;
			timer_t1_limit = TIMER_T1_DEFAULT_LIMIT;
			TIMER_T3 = 0;
			TIMER_T4 = 0;
			sstv_loop = 0;
			g_iors_control_state = STATE_TNC_CONNECTED;
			return EXIT_FAILURE;
		}
	} else {
		debug_print("SSTV - sending %s\n",filename);
		/* Then there is a file in the queue to send */
		sleep(1); // give some time for OK to be sent
		exit_tnc_listen_process();
		tnc_close();
		if (stop_program("Direwolf", tnc_pid) != EXIT_SUCCESS) {
			// TODO - PANIC.  CANT KILL IT - REBOOT??
			error_print ("ERROR: CANT KILL DIREWOLF!\n");
		}
		if (radio_set_sstv_mode() == EXIT_SUCCESS) {
			timer_t3_limit = TIMER_T3_DEFAULT_LIMIT;
			TIMER_T3 = time(0); // start T3 - timeout
			/* Start APLAY in the background */
			ptt_serial = open_rts_serial(g_ptt_serial_dev);
			set_rts(ptt_serial, true);
			//				char *argv[]={"pysstv","--chan 2","--rate 48000","--resize","--mode PD120",
			char *argv[]={"aplay","-c2", "-r48000", "-Dhw:2,0", filename,(char *)NULL};
			sstv_pid = start_program(g_sstv_path,argv, g_sstv_logfile_path);

			if (sstv_pid == -1) {
				set_rts(ptt_serial, false);
				error_print("Can not start sstv\n");
				TIMER_T3 = 0;
				TIMER_T1 = time(0); // start T1
				g_iors_control_state = STATE_SSTV;
			} else {
				debug_print("SSTV playing\n");
				g_iors_control_state = STATE_SSTV;
			}
		}

	}
	return EXIT_SUCCESS;
}

int sstv_stop() {
	TIMER_T1 = 0;
	timer_t1_limit = TIMER_T1_DEFAULT_LIMIT;
	TIMER_T3 = 0;
	TIMER_T4 = 0;

	/* Make sure we have collected any status from child process */
	stop_program("SSTV", sstv_pid);
	sstv_pid = PID_NOT_RUNNING;
	debug_print ("SSTV finished\n");
	set_rts(ptt_serial, false);
	close_rts_serial(ptt_serial);

	sleep(2); // Allow audio port to become available

	if (start_tnc() == EXIT_SUCCESS) {
		g_iors_control_state = STATE_TNC_CONNECTED;
	} else {
		TIMER_T1 = time(0); // start T1
		g_iors_control_state = STATE_RADIO_CONNECTED;
		return EXIT_FAILURE;
	}

	if (sstv_loop) {
		/* Then we listen for commands while remaining in SSTV state, then transmit next picture if no command */
		timer_t1_limit = 10; // 10 second pause
		TIMER_T1 = time(0); // start T1
		g_iors_control_state = STATE_SSTV;
	}

	return EXIT_SUCCESS;
}

/**
 * next_sstv_file()
 * Return the path to the next file in the sstv queue
 *
 */
int next_sstv_file(char * filename) {


	DIR * d = opendir(g_sstv_queue_path);
	if (d == NULL) { printf("** Could not open sstv queue %s\n",g_sstv_queue_path); return EXIT_FAILURE; }
	struct dirent *de;
	for (de = readdir(d); de != NULL; de = readdir(d)) {
		//debug_print("CHECKING: %s\n",de->d_name);
		if ((strcmp(de->d_name, ".") != 0) && (strcmp(de->d_name, "..") != 0)) {
			//if (str_ends_with(de->d_name, WAV_FILE_EXT)) {
			strlcpy(filename, g_sstv_queue_path, MAX_FILE_PATH_LEN);
			strlcat(filename, "/", MAX_FILE_PATH_LEN);
			strlcat(filename, de->d_name, MAX_FILE_PATH_LEN);
			//debug_print("Found: %s\n",filename);
			closedir(d);
			return EXIT_SUCCESS;
			//}
		}
	}
	closedir(d);


	return EXIT_SUCCESS;
}

int valid_command(char *from_callsign, unsigned char *data, int len, struct t_iors_event *command_event) {
	struct t_ax25_header *ax25_header;
	ax25_header = (struct t_ax25_header *)data;
	if ((ax25_header->pid & 0xff) != PID_COMMAND) {
		return EXIT_FAILURE;
	}
	SWCmdUplink *sw_command;
	sw_command = (SWCmdUplink *)(data + sizeof(AX25_HEADER));

	debug_print("Received Command %04x addr: %d names: %d cmd %d from %s length %d\n",(sw_command->dateTime),
			sw_command->address, sw_command->namespaceNumber, (sw_command->comArg.command), from_callsign, len);
	int i;
//	for (i=0; i<4; i++)
//		debug_print("arg:%d %d\n",i,sw_command->comArg.arguments[i]);
	/* Pass the data to the command processor */
	if(AuthenticateSoftwareCommand(sw_command)){
		//debug_print("Command Authenticated!\n");
		int rc = send_ok(from_callsign);
		if (rc != EXIT_SUCCESS) {
			debug_print("\n Error : Could not send OK Response to TNC \n");
		}
		// usleep(1000*1000);
//		if(DispatchSoftwareCommand(sw_command,true))
//			return EXIT_SUCCESS;
//		else
//			return EXIT_FAILURE;
		command_event->event = COMMAND_RX;
		command_event->nameSpace = sw_command->namespaceNumber;
		command_event->comarg = sw_command->comArg;
//		int i;
//		for (i=0; i<4; i++) {
//			comarg->arguments[i] = sw_command->comArg.arguments[i];
//		}

		return EXIT_SUCCESS;
	} else {
		int r = send_err(from_callsign, 5);
		if (r != EXIT_SUCCESS) {
			debug_print("\n Error : Could not send ERR Response to TNC \n");
		}
	}
	return EXIT_FAILURE;
}

/**
 * send_ok()
 *
 * Send a UI frame from the broadcast callsign to the station with PID BB and the
 * text OK <callsign>0x0Drequest_list
 */
int send_ok(char *from_callsign) {
	int rc = EXIT_SUCCESS;
	char buffer[4 + strlen(from_callsign)]; // OK + 10 char for callsign with SSID
	strlcpy(buffer,"OK ", sizeof(buffer));
	strlcat(buffer, from_callsign, sizeof(buffer));
	rc = send_raw_packet(g_callsign, from_callsign, PID_FILE, (unsigned char *)buffer, sizeof(buffer));

	return rc;
}

/**
 * send_err()
 *
 * Send a UI frame to the station containing an error response.  The error values are defined in
 * the header file
 *
 * returns EXIT_SUCCESS unless it is unable to send the data to the TNC
 *
 */
int send_err(char *from_callsign, int err) {
	int rc = EXIT_SUCCESS;
	char err_str[2];
	snprintf(err_str, 3, "%d",err);
	char buffer[6 + strlen(err_str)+ strlen(from_callsign)]; // NO -XX + 10 char for callsign with SSID
	char CR = 0x0d;
	strlcpy(buffer,"NO -", sizeof(buffer));
	strlcat(buffer, err_str, sizeof(buffer));
	strlcat(buffer," ", sizeof(buffer));
	strlcat(buffer, from_callsign, sizeof(buffer));
	strncat(buffer,&CR,1); // very specifically add just one char to the end of the string for the CR
	rc = send_raw_packet(g_callsign, from_callsign, PID_FILE, (unsigned char *)buffer, sizeof(buffer));

	return rc;
}

/**
 * Monitor a running child process based on its pid.  We only care if
 * the program was running and it exited.
 *
 * Returns:
 *   true if the program changed state and exited
 *
 */
int program_stopped(char * program_name, pid_t pid) {
	if (kill(pid,0) == 0) {
		/* Need to run waitpid to gather up any defunc process */
		int wstatus;
		int prog_state = waitpid(pid, &wstatus,WNOHANG);
		if (prog_state == pid) {
			if (WIFEXITED(wstatus)) {
				debug_print("%s exited, status=%d\n", program_name, WEXITSTATUS(wstatus));
			} else {
				debug_print("%s stopped running abnormally\n",program_name);
			}
			pid = PID_NOT_RUNNING;
			return true;
		} else if (prog_state == 0) {
			//debug_print("%s Running - PID did not change state\n",program_name);
			return false;
		} else {
			/* This pid was not valid and the program was not running*/
			return false;
		}
	} else {
		//debug_print("%s Not running\n",program_name);
	}

//	} else if (WIFSIGNALED(status)) {
//		printf("killed by signal %d\n", WTERMSIG(status));
//		return(PROCESS_KILLED);
//	} else if (WIFSTOPPED(status)) {
//		printf("stopped by signal %d\n", WSTOPSIG(status));
//		return(PROCESS_KILLED);
//	} else if (WIFCONTINUED(status)) {
//		printf("continued\n");
//		return PROCESS_RUNNING;
//	}

	return false;
}

/**
 * start_program()

 * Start the program given in the command
 *
 * Returns the PID of the started program or 0 of there was an error
 */
int start_program(char * command, char *argv[], char * logfile) {
	/*Spawn a child to run the program.*/
	pid_t pid = fork();
	if (pid == -1) {
		// fork failed
		return -1;
	}
	if (pid==0) { /* child process */
		printf("CHILD: Running %s with PID %ld\n", argv[0], (long) getpid());
		int fd = open(logfile, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
		dup2(fd, 1);   // make stdout go to file
		dup2(fd, 2);   // make stderr go to file
		close(fd);     // fd no longer needed - the dup'ed handles are sufficient
		execv(command,argv);
		exit(127); /* only if execv fails, otherwise this is all replaced */
	} else { /* pid!=0; parent process */

	}
	return pid;
}

/**
 * stop_program()
 * If program is running then shutdown program with a signal
 */
int stop_program(char * program_name, pid_t program_pid) {
	if (program_pid == PID_NOT_RUNNING) return EXIT_SUCCESS;

	if (kill(program_pid, 0) == 0) { // Signal 0 tests if the pid is running
		//debug_print("%s running. Stopping with SIGINT....\n", program_name);
		kill(program_pid, SIGINT);
		int wstatus;
		int dw_prog_state = waitpid(program_pid, &wstatus,0);
		if (dw_prog_state != program_pid) {
			debug_print("%s: Stopping with SIGKILL....\n", program_name);
			kill(program_pid, SIGKILL);
			usleep(100*1000); // wait 100ms to exit
			dw_prog_state = waitpid(program_pid, &wstatus,WNOHANG);
			if (dw_prog_state != program_pid)
				return EXIT_FAILURE; // could not stop direwolf.  May need to reboot!
		} else {
			debug_print("%s: Stopped with SIGINT\n", program_name);
		}
	}

	program_pid = PID_NOT_RUNNING;
	return EXIT_SUCCESS;
}

/**
 * start_tnc()
 * Run direwolf and store the PID in direwolf_pid
 */
int start_tnc() {
    int running = false;
	if (kill(tnc_pid,0) == 0) {// pid valid, could be running or a defunct pid waiting to report
		int wstatus;
		int prog_state = waitpid(tnc_pid, &wstatus,WNOHANG);
		if (prog_state == 0) {
			/* Already running */
			debug_print("start_tnc(): DIREWOLF ALREADY RUNNING\n");
			running = true;
		} else if (prog_state == tnc_pid){
			debug_print("start_tnc(): DEFUNCT DIREWOLF pid %d Exited\n",tnc_pid);
		}
	}

	if (! running) {
		char *argv[]={"direwolf","-q d","-r 48000",(char *)NULL};
		tnc_pid = start_program(g_direwolf_path,argv, g_direwolf_logfile_path);
		if (tnc_pid == -1) {
			error_print("start_tnc(): Can not start direwolf\n");
			tnc_pid = PID_NOT_RUNNING;
			return EXIT_FAILURE;
		}
	}
	sleep(2); // give TNC time to start
	if (connect_tnc() == EXIT_SUCCESS) {
		return EXIT_SUCCESS;
	} else {
		return EXIT_FAILURE;
	}
}

int connect_tnc() {
	int rc = EXIT_FAILURE;

	/* Connect to direwolf*/
	rc = tnc_connect("127.0.0.1", AGW_PORT, g_bit_rate, g_max_frames_in_tx_buffer);
	if (rc == EXIT_SUCCESS) {
		rc = tnc_start_monitoring('k'); // k monitors raw frames, required to process UI frames
		if (rc != EXIT_SUCCESS) {
			error_print("\n Error : Could not monitor TNC k frames \n");
		}
		rc = tnc_start_monitoring('m'); // monitors connected frames, also required to monitor T frames to manage the TX frame queue
		if (rc != EXIT_SUCCESS) {
			error_print("\n Error : Could not monitor TNC m frames\n");
		}

		/**
		 * Start a thread to listen to the TNC.  This will write all received frames into
		 * a circular buffer.  This thread runs in the background and is always ready to
		 * receive data from the TNC.
		 *
		 * The receive loop reads frames from the buffer and processes
		 * them when we have time.
		 */
		char *name = "TNC Listen Thread";
		rc = pthread_create( &tnc_listen_pthread, NULL, tnc_listen_process, (void*) name);
		if (rc != EXIT_SUCCESS) {
			error_print("FATAL. Could not start the TNC listen thread.\n");
			exit(rc);
		}

		//debug_print ("Connected to tnc\n");
		return EXIT_SUCCESS;
	} else {
		error_print ("Could not connect to tnc\n");
	}
	return EXIT_FAILURE;
}
