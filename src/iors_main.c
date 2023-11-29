/*
 * main.c
 *
 *  Created on: Oct 2, 2022
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

/* System include files */
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>

/* Project include files */
#include "radio.h"
#include "config.h"
#include "agw_tnc.h"
#include "str_util.h"
#include "ax25_tools.h"
#include "iors_command.h"

/* Forward declarations */
void help(void);
void signal_exit (int sig);
void signal_load_config (int sig);
int monitor_tnc_frames();
int handle_command(char *from_callsign, unsigned char *data, int len);
int send_ok(char *from_callsign);
int send_err(char *from_callsign, int err);
int start_program(char * command, char *argv[], char * logfile);
int monitor_program(pid_t pid);

/*
 *  GLOBAL VARIABLES defined here.  They are declared in config.h
 *  These are the default values.  Many can be updated with a value
 *  in pacsat.config or can be overridden on the command line.
 *
 */
int g_verbose = false;
int g_frames_queued = 0;
int g_iors_control_state = STATE_INIT;
char pid_file_path[MAX_FILE_PATH_LEN] = "iors_control.pid";

/* These global variables are in the iors_control.config file.  This defines their default value
 * but it is almost certainly overwritten in the config files.  It should be
 * changed there. */
int g_bit_rate = 1200;
char g_callsign[MAX_CALLSIGN_LEN] = "NA1ISS-12";
char g_radio_id[15] = "ID TM-D710G";
char g_radio_type[15] = "TY K,0,0,1,0";
char g_radio_main_firmware[25] = "FV 0,1.00,1.00,A,1";
char g_radio_panel_firmware[25] = "FV 1,1.00,1.01,A,1";
char g_serial_dev[MAX_FILE_PATH_LEN] = "/dev/ttyUSB0";
int g_max_frames_in_tx_buffer = 2;
char g_direwolf_path[MAX_FILE_PATH_LEN] = "/usr/local/bin/direwolf";
char g_direwolf_logfile[MAX_FILE_PATH_LEN] = "/tmp/direwolf.log";
char g_direwolf_config_path[MAX_FILE_PATH_LEN] = "~/direwolf.conf";
char g_iors_last_command_time_path[MAX_FILE_PATH_LEN] = "iors_last_command_time.dat";


/* Local variables */
pthread_t tnc_listen_pthread;
int frame_num = 0;
pid_t direwolf_pid;

/**
 * Print this help if the -h or --help command line options are used
 */
void help(void) {
	printf(
			"Usage: pacsat [OPTION]... \n"
			"-h,--help                        help\n"
			"-t,--test                        Run self test functions and exit\n"
			"-v,--verbose                     print additional status and progress messages\n"

	);
	exit(EXIT_SUCCESS);
}

void cleanup() {
	int tnc_close();
	if (remove(pid_file_path) != 0) {
		error_print("Could not remove the pid file at exit: %s\n", pid_file_path);
	}
}

void signal_exit (int sig) {
	debug_print (" Signal received, exiting ...\n");
	cleanup();
	exit (0);
}

void signal_load_config (int sig) {
	error_print (" Signal received, updating config not yet implemented...\n");
	// TODO SIHUP should reload the config perhaps
}

void exit_if_running() {
	int my_pid = getpid();
	FILE * fd = fopen(pid_file_path, "r");
	if (fd == NULL) {
		// no pid file, make a new one
		fd = fopen(pid_file_path, "w");
		if (fd == NULL) {
			error_print("Could not create the pid file\n");
			// This is not fatal, but we might we running twice..
		}
		fprintf(fd, "%d\n", my_pid);
	} else {
		char line [ MAX_CONFIG_LINE_LENGTH ]; /* or other suitable maximum line size */
		fgets ( line, sizeof line, fd ); /* read a line */
		line[strcspn(line,"\n")] = 0; // Move the nul termination to get rid of the new line
		int pid = atoi(line);
		//int pid = strtol(line, NULL, 0);
		debug_print("pid file contains: %d\n",pid);
		if (kill(pid,0) == -1) {
			debug_print("But it was not running, making new pid file\n");
			fd = fopen(pid_file_path, "w");
			if (fd == NULL) {
				error_print("Could not create the pid file\n");
				// This is not fatal, but we might we running twice..
			}
			fprintf(fd, "%d\n", my_pid);
		} else {
			debug_print("Already running..\n");
			fclose(fd);
			exit(1);
		}
	}
	fclose(fd);

}

int main(int argc, char *argv[])  {
	exit_if_running();
	signal (SIGQUIT, signal_exit);
	signal (SIGTERM, signal_exit);
	signal (SIGHUP, signal_load_config);
	signal (SIGINT, signal_exit);

	struct option long_option[] = {
			{"help", 0, NULL, 'h'},
			{"test", 0, NULL, 't'},
			{"verbose", 0, NULL, 'v'},
			{NULL, 0, NULL, 0},
	};

	int more_help = false;
	while (1) {
		int c;
		if ((c = getopt_long(argc, argv, "htv:", long_option, NULL)) < 0)
			break;
		switch (c) {
		case 'h': // help
			more_help = true;
			break;
//		case 't': // self test
//			g_run_self_test = true;
//			break;
		case 'v': // verbose
			g_verbose = true;
			break;
		}
	}

	if (more_help) {
		help();
		return 0;
	}
    int rc = EXIT_SUCCESS;

	printf("ARISS IORS Control Program\n");
	printf("Build: %s\n", VERSION);

	/* Load the config from disk */
    load_config();

    /* Load the last command time */
    load_last_commmand_time();

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
				direwolf_pid = start_program(g_direwolf_path,argv, g_direwolf_logfile);
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
			monitor_tnc_frames();
			break;

		case STATE_X_BAND_REPEATER_MODE:
			monitor_tnc_frames();
			break;

		default :
			break;
		}
	}

#ifdef JUNK
    //	char * data = "MS BILL\r";
    //	char * data = "CC 0\r";
    //	char * data = "FO 0\r"; // Read VFO channel - 0 is A 1 is B
    //	char * data = "DL 0\r"; // Set Dual band 0, single band 1
    //	char * data = "VM 0\r"; // Returns VFO mode
        //    char * data_set = "CC 1,0144000000,0,0,0,0,0,0,08,08,000,00600000,0,0000000000,0\r";
    //    radio_send_command(data_set, strlen(data_set));

    //radio_set_aprs_mode();
    //radio_set_sstv_mode();
//    radio_set_cross_band_mode();



    if (radio_program_pm_and_data_band(g_serial_dev, RADIO_PM0, RADIO_XBAND_RPT_OFF, RADIO_EXT_DATA_BAND_TXA_RXB, RADIO_EXT_DATA_SPEED_9600) != EXIT_SUCCESS) {
    	error_print("FATAL ERROR: Can not setup the radio\n");
    	return EXIT_FAILURE;
    }


    if (radio_set_channel(g_serial_dev, RADIO_RPT01_TX_CHANNEL, RADIO_RPT01_RX_CHANNEL) != EXIT_SUCCESS) {
    	debug_print("ERROR: Can't change channels\n");
    	return EXIT_FAILURE;
	}

//    char response[25];
//    int rlen = 10;
//	int n = radio_send_command(g_serial_dev, RADIO_CMD_SET_TNC_MODE, strlen(RADIO_CMD_SET_TNC_MODE), response, rlen);
//	if (n < 0) {
//		debug_print("Can not send TNC MODE command to radio\n");
//		return EXIT_FAILURE;
//	}
//	response[n] = 0;
//	debug_print("TNC MODE: %s",response);

#endif
    cleanup();
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

int monitor_program(pid_t pid) {
	int status;
	pid_t w;

	w = waitpid(pid, &status, WUNTRACED | WCONTINUED);
	if (w == -1) {
		perror("waitpid");
		return(EXIT_FAILURE);
	}

	if (WIFEXITED(status)) {
		printf("exited, status=%d\n", WEXITSTATUS(status));
		return(EXIT_FAILURE);
	} else if (WIFSIGNALED(status)) {
		printf("killed by signal %d\n", WTERMSIG(status));
		return(EXIT_FAILURE);
	} else if (WIFSTOPPED(status)) {
		printf("stopped by signal %d\n", WSTOPSIG(status));
		return(EXIT_FAILURE);
	} else if (WIFCONTINUED(status)) {
		printf("continued\n");
	}

	return EXIT_SUCCESS;
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
	    	printf("Starting %s PID is %ld\n", argv[0], (long) getpid());
	    	int fd = open(logfile, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);

	    	dup2(fd, 1);   // make stdout go to file
	    	dup2(fd, 2);   // make stderr go to file
	    	close(fd);     // fd no longer needed - the dup'ed handles are sufficient

	        //static char *argv[]={"echo","Foo is my name.",NULL};
	        execv(command,argv);
	        exit(127); /* only if execv fails */
	    } else { /* pid!=0; parent process */

	    }
	    return pid;
}
