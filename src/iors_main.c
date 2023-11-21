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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "radio.h"
#include "config.h"

/*
 *  GLOBAL VARIABLES defined here.  They are declared in config.h
 *  These are the default values.  Many can be updated with a value
 *  in pacsat.config or can be overridden on the command line.
 *
 */
int g_verbose = false;
int g_frames_queued = 0;

/* These global variables are in the config file */
int g_bit_rate = 1200;
char g_callsign[10] = "NA1ISS-12";
char g_serial_dev[MAX_FILE_PATH_LEN] = "/dev/ttyUSBX";
int g_max_frames_in_tx_buffer = 2;

int main() {
	/* Load the config from disk */
    load_config();
//    radio_program_pm(g_serial_dev, RADIO_PM0, RADIO_XBAND_RPT_OFF);

    //	char * data = "MS BILL\r";
    //	char * data = "CC 0\r";
    //	char * data = "FO 0\r"; // Read VFO channel - 0 is A 1 is B
    //	char * data = "DL 0\r"; // Set Dual band 0, single band 1
    //	char * data = "ID\r"; // Get id
    	int rlen = 25;
    //	char * data = "TY\r"; // Get type
    //	char * data = "VM 0\r"; // Returns VFO mode
    	char response[rlen];
        radio_send_command(g_serial_dev, RADIO_CMD_ID, strlen(RADIO_CMD_ID), response, rlen);
        radio_send_command(g_serial_dev, RADIO_CMD_TYPE, strlen(RADIO_CMD_TYPE), response, rlen);

        //    char * data_set = "CC 1,0144000000,0,0,0,0,0,0,08,08,000,00600000,0,0000000000,0\r";
    //    radio_send_command(data_set, strlen(data_set));


    return 0;
}
