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

/* Program includes */
#include "config.h"
#include "radio.h"
#include "iors_controller.h"
#include "agw_tnc.h"
#include "str_util.h"
#include "ax25_tools.h"
#include "iors_command.h"

#define PROCESS_KILLED -9
#define PROCESS_RUNNING -1

#define PID_NOT_RUNNING -99

#define TIMER_T1_LIMIT 3
#define TIMER_T3_LIMIT 60

/* Forward declarations */
int state_sstv_mode();
int monitor_tnc_frames();
int handle_command(char *from_callsign, unsigned char *data, int len);
int send_ok(char *from_callsign);
int send_err(char *from_callsign, int err);
int start_program(char * command, char *argv[], char * logfile);
int monitor_program(pid_t pid);

/* Local variables */
int g_iors_control_state = STATE_INIT;
int sstv_state = STATE_SSTV_INIT;
pthread_t tnc_listen_pthread;
int frame_num = 0;
pid_t direwolf_pid = PID_NOT_RUNNING, sstv_pid = PID_NOT_RUNNING;
int ptt_serial;
int TIMER_T1 = 0;
int TIMER_T3 = 0;


/*****
 * TODO:
 * Update this to be event driven.
 * It should receive Commands as the events
 * The while loop should be a level above that monitors for packets and sends any received
 * commands here
 * At boot the first "command" is an internal command: Connect_radio
 * If successful it can send itself a command "Connect TNC" etc.
 * But, how do retries work?  That suggests events from timers that are fired in the background
 * So if we are in state INIT and execute Connect_radio but fail, then we start timer and when
 * done it sends a connect_radio event again.  If #retries exceeded then something happens like
 * reboot radio/cpu.
 *
 * We can then draw out the whole state machine/model and study it.
 *
 */

void iors_control_loop() {
	int rc = 0;
	/* Enter the IORS CONTROL State machine */
	while(1) {
		switch (g_iors_control_state) {
		case STATE_INIT: // Initialization state
			debug_print("Init .. Trying to connect to radio\n");
			/* Make sure we can talk to the radio */
			if (radio_check_connection(g_serial_dev) == EXIT_SUCCESS) {
				g_iors_control_state = STATE_RADIO_CONNECTED;
			} else {
				//TODO - what other trouble shooting can happen here?  The radio may be off
				sleep(60); /* Wait for a minute and try again */
			}
			break;
		case STATE_RADIO_CONNECTED:
			/* Start Direwolf if it is not already running */
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

				g_iors_control_state = STATE_TNC_CONNECTED;
			} else {
				char *argv[]={"direwolf","-q d","-r 48000",(char *)NULL};
				direwolf_pid = start_program(g_direwolf_path,argv, g_direwolf_logfile_path);
				if (direwolf_pid == -1) {
					error_print("Can not start direwolf\n");
					sleep(60);
				} else {
					sleep(3); // give tnc time to start
				}
			}
			break;

		case STATE_TNC_CONNECTED:
			monitor_tnc_frames();
			//monitor_program(direwolf_pid);
			break;

		case STATE_PACKET_MODE:
			monitor_tnc_frames();
			break;

		case STATE_SSTV_MODE:
			//monitor_tnc_frames();
			state_sstv_mode();
			break;

		case STATE_X_BAND_REPEATER_MODE:
			monitor_tnc_frames();
			break;

		default :
			break;
		}
	}
}

int state_sstv_mode() {

	switch (sstv_state) {
			case STATE_SSTV_INIT: // Initialization state
				sleep(3);
				debug_print("Init .. Starting SSTV\n");
				//int prog_state = monitor_program(direwolf_pid);
				if (direwolf_pid != PID_NOT_RUNNING) {
					int dw_prog_state = kill(direwolf_pid, 0);
					if (dw_prog_state == 0) {
						debug_print("Direwolf running. Stopping with SIGINT....\n");
						kill(direwolf_pid, SIGINT);
						int wstatus;
						dw_prog_state = waitpid(direwolf_pid, &wstatus,0);
						if (dw_prog_state != direwolf_pid) {
							debug_print("Direwolf: Stopping with SIGKILL....\n");
							kill(direwolf_pid, SIGKILL);
						} else {
							debug_print("Direwolf: Stopped with SIGINT\n");
						}
					}
					direwolf_pid = PID_NOT_RUNNING;
				}
				sstv_state = STATE_SSTV_SEND;
				break;
			case STATE_SSTV_SEND:

				ptt_serial = open_rts_serial(g_ptt_serial_dev);
				set_rts(ptt_serial, true);

//				char *argv[]={"pysstv","--chan 2","--rate 48000","--resize","--mode PD120",
//						"/home/g0kla/sstv_images/ariss-tim-peak.jpg","/home/g0kla/sstv_images/ariss-tim-peak.wav",(char *)NULL};
				char *argv[]={"aplay","/home/g0kla/sstv_images/ariss.wav",(char *)NULL};
//				char *argv[]={"aplay","-c2", "-r48000", "-D hw:1,0", "/home/g0kla/sstv_images/ariss-tim-peak.wav",(char *)NULL};
				sstv_pid = start_program(g_sstv_path,argv, g_sstv_logfile_path);

				if (sstv_pid == -1) {
					set_rts(ptt_serial, false);
					error_print("Can not start sstv\n");
					sleep(60);
				} else {
					debug_print("SSTV playing\n");
					sstv_state = STATE_SSTV_MONITOR;
				}
				break;
			case STATE_SSTV_MONITOR:
				int wstatus;
				int prog_state = waitpid(sstv_pid, &wstatus,0);
//
//				if (sstv_pid != PID_NOT_RUNNING)
//					prog_state = kill(sstv_pid, 0); // check if it is running
//				if (prog_state != 0) {
					// no longer running, clean up and change state
					debug_print ("SSTV finished\n");
					set_rts(ptt_serial, false);
					close_rts_serial(ptt_serial);
					sstv_pid = PID_NOT_RUNNING;
					sstv_state = STATE_SSTV_INIT;
					g_iors_control_state = STATE_RADIO_CONNECTED;
//				} else {
//					//debug_print(".");
//					usleep(100*1000);
//				}
				//monitor_tnc_frames();
				break;
	}
	return EXIT_SUCCESS;
}

int monitor_tnc_frames() {
	struct t_agw_frame_ptr frame;
	int rc = get_next_frame(frame_num, &frame);
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

			if (strncasecmp(frame.header->call_to, g_callsign, MAX_CALLSIGN_LEN) == 0) {
				// this was sent to our Callsign

				struct t_ax25_header *ax25_header;
				ax25_header = (struct t_ax25_header *)frame.data;
				//					debug_print("IORS Packet: pid: %02x \n", ax25_header->pid & 0xff);
				if ((ax25_header->pid & 0xff) == PID_COMMAND) {
					handle_command(frame.header->call_from, frame.data, frame.header->data_len);

				}
			}

			break;


		}
	} else {
		usleep(1000); // sleep 1ms
	}
	return EXIT_SUCCESS;
}

int handle_command(char *from_callsign, unsigned char *data, int len) {
	//debug_print("IORS COMMAND: len: %d \n", len);
	SWCmdUplink *sw_command;
	sw_command = (SWCmdUplink *)(data + sizeof(AX25_HEADER));

	debug_print("Received Command %04x addr: %d names: %d cmd %d from %s length %d\n",(sw_command->dateTime),
			sw_command->address, sw_command->namespaceNumber, (sw_command->comArg.command), from_callsign, len);

	/* Pass the data to the command processor */
	if(AuthenticateSoftwareCommand(sw_command)){
		//debug_print("Command Authenticated!\n");
		int rc = send_ok(from_callsign);
		if (rc != EXIT_SUCCESS) {
			debug_print("\n Error : Could not send OK Response to TNC \n");
		}
		// usleep(1000*1000);
		if(DispatchSoftwareCommand(sw_command,true))
			return EXIT_SUCCESS;
		else
			return EXIT_FAILURE;
	} else {
		int r = send_err(from_callsign, 5);
		if (r != EXIT_SUCCESS) {
			debug_print("\n Error : Could not send ERR Response to TNC \n");
		}
	}
	return EXIT_SUCCESS;
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
 * Monitor a running child process based on its pid
 * Returns:
 *   The exit code of the process if it is finished
 *   -9 if it was killed or the pid was invalid
 *   -1 if it is still running
 *
 */
int monitor_program(pid_t pid) {
	int status;
	pid_t w;

//	w = waitpid(pid, &status, WNOHANG);
	w = waitpid(pid, &status, WUNTRACED | WCONTINUED);
	if (w == -1) {
		perror("waitpid");
		return(EXIT_FAILURE);
	}

	if (WIFEXITED(status)) {
		printf("exited, status=%d\n", WEXITSTATUS(status));
		return(WEXITSTATUS(status));
	} else if (WIFSIGNALED(status)) {
		printf("killed by signal %d\n", WTERMSIG(status));
		return(PROCESS_KILLED);
	} else if (WIFSTOPPED(status)) {
		printf("stopped by signal %d\n", WSTOPSIG(status));
		return(PROCESS_KILLED);
	} else if (WIFCONTINUED(status)) {
		printf("continued\n");
		return PROCESS_RUNNING;
	}

	// If we get here we did not recognize the response.  Assume an error
	return EXIT_FAILURE;
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

