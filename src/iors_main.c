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
#include <string.h>
#include <signal.h>


/* Project include files */
#include "config.h"
#include "str_util.h"
#include "iors_command.h"
#include "iors_controller.h"

/* Forward declarations */
void help(void);
void signal_exit (int sig);
void signal_load_config (int sig);

/*
 *  GLOBAL VARIABLES defined here.  They are declared in config.h
 *  These are the default values.  Many can be updated with a value
 *  in pacsat.config or can be overridden on the command line.
 *
 */
int g_verbose = false;
int g_frames_queued = 0;
int g_radio_pm = -1;
int g_radio_cross_band_repeater = -1;

/* These global variables are in the iors_control.config file.  This defines their default value
 * but it is overwritten if it is contained in the config file.  */
int g_bit_rate = 1200;
char g_callsign[MAX_CALLSIGN_LEN] = "NA1ISS-12";
char g_radio_id[15] = "ID TM-D710G";
char g_radio_type[15] = "TY K,0,0,1,0";
char g_radio_main_firmware[25] = "FV 0,1.00,1.00,A,1";
char g_radio_panel_firmware[25] = "FV 1,1.00,1.01,A,1";
char g_serial_dev[MAX_FILE_PATH_LEN] = "";
char g_ptt_serial_dev[MAX_FILE_PATH_LEN] = "";
int g_max_frames_in_tx_buffer = 2;

char g_direwolf_path[MAX_FILE_PATH_LEN] = "/usr/local/bin/direwolf";
char g_direwolf_logfile_path[MAX_FILE_PATH_LEN] = "/tmp/direwolf.log";
char g_direwolf_config_path[MAX_FILE_PATH_LEN] = "~/direwolf.conf";

char g_sstv_path[MAX_FILE_PATH_LEN] = "sstv.sh";
char g_sstv_logfile_path[MAX_FILE_PATH_LEN] = "/tmp/pysstv.log";

char g_iors_last_command_time_path[MAX_FILE_PATH_LEN] = "iors_last_command_time.dat";


/* Local variables */
char pid_file_path[MAX_FILE_PATH_LEN] = "iors_control.pid";
char config_file_name[MAX_FILE_PATH_LEN] = "iors_control.config";

/**
 * Print this help if the -h or --help command line options are used
 */
void help(void) {
	printf(
			"Usage: pacsat [OPTION]... \n"
			"-h,--help                        help\n"
			"-c,--config                      use config file specified\n"
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
			{"help", no_argument, NULL, 'h'},
			{"config", required_argument, NULL, 'c'},
			{"verbose", no_argument, NULL, 'v'},
			{NULL, 0, NULL, 0},
	};

	int more_help = false;
	while (1) {
		int c;
		if ((c = getopt_long(argc, argv, "hc:v", long_option, NULL)) < 0)
			break;
		switch (c) {
		case 'h': // help
			more_help = true;
			break;
		case 'c': // config file name
//			config_file_name = optarg;
			strlcpy(config_file_name, optarg, sizeof(config_file_name));
			break;
		case 'v': // verbose
			g_verbose = true;
			break;
		}
	}

	if (more_help) {
		help();
		return 0;
	}

	printf("ARISS IORS Control Program\n");
	printf("Build: %s\n", VERSION);

	/* Load the config from disk */
    load_config(config_file_name);

    /* Load the last command time */
    load_last_commmand_time();

    iors_control_loop();

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
