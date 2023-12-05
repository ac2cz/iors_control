/*
 * radio.h
 *
 *  Created on: Nov 21, 2023
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

#ifndef RADIO_H_
#define RADIO_H_

#define RADIO_XBAND_RPT_ON 1
#define RADIO_XBAND_RPT_OFF 0

#define RADIO_PM0 0
#define RADIO_PM1 1
#define RADIO_PM2 2
#define RADIO_PM3 3
#define RADIO_PM4 4
#define RADIO_PM5 5

#define RADIO_CMD_GET_ID "ID\r"
#define RADIO_CMD_GET_TYPE "TY\r"
#define RADIO_CMD_GET_MAIN_FIRMWARE "FV 0\r"
#define RADIO_CMD_GET_PANEL_FIRMWARE "FV 1\r"

#define RADIO_CMD_GET_MEM_VFO_MODE_A "VM 0\r"
#define RADIO_CMD_GET_MEM_VFO_MODE_B "VM 1\r"
#define RADIO_CMD_SET_MEM_MODE_A "VM 0,1\r"
#define RADIO_CMD_SET_VFO_MODE_A "VM 0,0\r"
#define RADIO_CMD_SET_MEM_MODE_B "VM 1,1\r"
#define RADIO_CMD_SET_VFO_MODE_B "VM 1,0\r"

#define RADIO_CMD_GET_CHANNEL_MODE "CD\r"
#define RADIO_CMD_SET_CHANNEL_MODE "CD 1\r"
#define RADIO_CMD_SET_FREQ_MODE "CD 0\r"

#define RADIO_CMD_GET_TNC_MODE "TN\r"
#define RADIO_CMD_SET_TNC_MODE "TN 0,2\r"


#define RADIO_CMD_MEM_CHANNEL "MR "

#define RADIO_EXT_DATA_BAND_A 0
#define RADIO_EXT_DATA_BAND_B 1
#define RADIO_EXT_DATA_BAND_TXA_RXB 2
#define RADIO_EXT_DATA_BAND_TXB_RXA 3
#define RADIO_EXT_DATA_SPEED_1200 0
#define RADIO_EXT_DATA_SPEED_9600 1


#define RADIO_RPT01_TX_CHANNEL 201
#define RADIO_RPT01_RX_CHANNEL 202
#define RADIO_SSTV_TX_CHANNEL 300
#define RADIO_SSTV_RX_CHANNEL 300

void radio_closeserial(int fd);
int radio_openserial(char *devicename);
int setRTS(int fd, int level);
int radio_check_initial_connection(char *serialdev);
int radio_set_cross_band_mode();
int radio_set_aprs_mode();
int radio_set_sstv_mode();
int radio_program_pm(char *serialdev, int pm, int cross_band_repeater);
int radio_program_pm_and_data_band(char *serialdev, int pm, int cross_band_repeater, int ext_data_band, int ext_data_speed);
int radio_send_command(char *serialdev, char * data, int len, char *response, int rlen);
int radio_set_channel(char *serialdev, int band_a_channel, int band_b_channel);

void close_rts_serial(int fd);
int open_rts_serial(char *devicename);
void set_rts(int fd, int state);

#endif /* RADIO_H_ */
