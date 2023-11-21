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

#define RADIO_CMD_ID "ID\r"
#define RADIO_CMD_TYPE "TY\r"


void radio_closeserial(int fd);
int radio_openserial(char *devicename);
int setRTS(int fd, int level);
int radio_program_pm(char *serialdev, int pm, int cross_band_repeater);
int radio_send_command(char *serialdev, char * data, int len, char *response, int rlen);

#endif /* RADIO_H_ */
