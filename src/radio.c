/*
 * radio.c
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

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string.h>

#include <errno.h>
#include "config.h"
#include "radio.h"
#include "str_util.h"

/* Forward declarations */
int radio_reset_speed(int fd);

/* Local vars */
static struct termios oldterminfo;

/**
 * radio_closeserial()
 * Restore the serial device settings and close the serial connection
 */
void radio_closeserial(int fd) {
	tcflush(fd,TCIOFLUSH );
    tcsetattr(fd, TCSANOW, &oldterminfo);
    if (close(fd) < 0)
        perror("closeserial()");
}

/**
 * radio_openserial()
 * Open a serial connection to the radio.
 * Cache the existing settings so they can be restored when we close the device
 * Setup a 9600 8N1 connection
 * Send and receive raw bytes
 * Use hardware flow control
 *
 */
int radio_openserial(char *devicename) {
    int fd;
    struct termios options;

    /* NOCTTY means we are not a terminal and dont want control codes.  NDELAY means we dont care about the state of the DCD line */
    if ((fd = open(devicename, O_RDWR | O_NOCTTY | O_NDELAY)) == -1) {
        perror("openserial(): open()");
        return 0;
    }

    if (tcgetattr(fd, &oldterminfo) == -1) {
            perror("openserial(): tcgetattr()");
            return 0;
    }

    options = oldterminfo;

    /*
    * Set the baud rates to 9600...
    */
    if (cfsetispeed(&options, B9600) == -1) {
        perror("openserial(): cfsetispeed()");
        return 0;
    }
    if (cfsetospeed(&options, B9600) == -1) {
        perror("openserial(): cfsetospeed()");
        return 0;
    }
    /*
    * Enable the receiver and set local mode... The c_cflag member contains two options that should always be enabled,
    * CLOCAL and CREAD. These will ensure that your program does not become the 'owner' of the port subject to sporatic
    * job control and hangup signals, and also that the serial interface driver will read incoming data bytes.
    */
    options.c_cflag |= (CLOCAL | CREAD);

    /* 8N1 */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= CRTSCTS; /* Enable hardware flow control */
//    options.c_cflag &= ~CRTSCTS; /* disable hardware flow contol */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN); /* RAW Input */
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
            |INLCR|IGNCR|ICRNL|IXON);
    options.c_oflag &= ~OPOST; /* Raw output */

    /*
    * Set the new options for the port...
    */
    if (tcsetattr(fd, TCSANOW, &options) == -1) { //TCSANOW constant specifies that all changes should occur immediatel
    	printf("init serial error: tcsetattr()\n");
    	radio_closeserial(fd);
    	return 0;
    }

    return fd;
}

int radio_reset_speed(int fd) {
    struct termios attr;

	if (tcgetattr(fd, &attr) == -1) {
	            perror("openserial(): tcgetattr()");
	            return -1;
	    }
    if (cfsetispeed(&attr, B57600) == -1) {
        perror("openserial(): cfsetispeed()");
        return 0;
    }
    if (cfsetospeed(&attr, B57600) == -1) {
        perror("openserial(): cfsetospeed()");
        return 0;
    }
    if (tcsetattr (fd, TCSANOW, &attr) != 0) {
      	 printf ("error %d resetting term speed", errno);
      	 return -1;
    }
    return EXIT_SUCCESS;
}


int setRTS(int fd, int level) {
    int status;

    if (ioctl(fd, TIOCMGET, &status) == -1) {
        perror("setRTS(): TIOCMGET");
        return 0;
    }
    if (level)
        status |= TIOCM_RTS;
    else
        status &= ~TIOCM_RTS;
    if (ioctl(fd, TIOCMSET, &status) == -1) {
        perror("setRTS(): TIOCMSET");
        return 0;
    }
    return 1;
}

//int program_read(int fd, char * buf, int len) {
//	int i, n;
//	int total = 0;
//	while ((n = read(fd, buf, len)) != -1 && (total < len)) {
//		total += n;
//		for (i=0; i< n; i++) {
//			//				debug_print("%c",buf[i]);
//			printf(" %0x",buf[i] & 0xff);
//			if ((i % 16 == 0) && (i != 0))
//				printf("\n");
//		}
//		printf("|\n");
//	}
//	return total;
//
//}

int radio_set_cross_band_mode() {
    if (radio_program_pm(g_serial_dev, RADIO_PM0, RADIO_XBAND_RPT_ON) != EXIT_SUCCESS) {
    	debug_print("FATAL ERROR: Can not setup the radio\n");
    	return EXIT_FAILURE;
    }

    if (radio_set_channel(g_serial_dev, RADIO_RPT01_TX_CHANNEL, RADIO_RPT01_RX_CHANNEL) != EXIT_SUCCESS) {
    	debug_print("ERROR: Can't change channels\n");
    	return EXIT_FAILURE;
	}
    return EXIT_SUCCESS;
}

int radio_set_aprs_mode() {
    if (radio_program_pm(g_serial_dev, RADIO_PM3, RADIO_XBAND_RPT_OFF) != EXIT_SUCCESS) {
    	debug_print("FATAL ERROR: Can not setup the radio\n");
    	return EXIT_FAILURE;
    }

    if (radio_set_channel(g_serial_dev, RADIO_SSTV_TX_CHANNEL, RADIO_SSTV_RX_CHANNEL) != EXIT_SUCCESS) {
    	debug_print("ERROR: Can't change channels\n");
    	return EXIT_FAILURE;
	}
    return EXIT_SUCCESS;
}

int radio_set_sstv_mode() {
    if (radio_program_pm(g_serial_dev, RADIO_PM0, RADIO_XBAND_RPT_OFF) != EXIT_SUCCESS) {
    	debug_print("FATAL ERROR: Can not setup the radio\n");
    	return EXIT_FAILURE;
    }

    if (radio_set_channel(g_serial_dev, RADIO_SSTV_TX_CHANNEL, RADIO_SSTV_RX_CHANNEL) != EXIT_SUCCESS) {
    	debug_print("ERROR: Can't change channels\n");
    	return EXIT_FAILURE;
	}
    return EXIT_SUCCESS;
}

/**
 * radio_check_connection()
 * Confirm that that we are connected to the ARISS radio.
 * To support debugging, which may use a different radio, the ID and TYPE
 * strings are contained in the config file
 *
 */
int radio_check_connection(char *serialdev) {
	int rlen = 25;
	char response[rlen];
	int n = radio_send_command(serialdev, RADIO_CMD_GET_ID, strlen(RADIO_CMD_GET_ID), response, rlen);
	if (n < 0) {
		debug_print("Can not send command to radio\n");
		return EXIT_FAILURE;
	}
	response[n] = 0; // Terminate the string
	if (strncmp(response, g_radio_id, strlen(g_radio_id)) != 0) {
		debug_print("Can not connect to radio: %s.  Got id %s\n", g_radio_id, response);
		return EXIT_FAILURE;
	}
	debug_print("CONNECTED TO RADIO: %s\n",response);

	n = radio_send_command(serialdev, RADIO_CMD_GET_TYPE, strlen(RADIO_CMD_GET_TYPE), response, rlen);
	if (n < 0) {
		debug_print("Can not send command to radio\n");
		return EXIT_FAILURE;
	}
	response[n] = 0; // Terminate the string
	if (strncmp(response, g_radio_type, strlen(g_radio_type)) != 0) {
		debug_print("Wrong radio type: %s Got type %s\n", g_radio_type, response);
		return EXIT_FAILURE;
	}
	//debug_print("RADIO TYPE: %s",response);

	return EXIT_SUCCESS;
}

/**
 * Change the channel for band A and band B
 */
int radio_set_channel(char *serialdev, int band_a_channel, int band_b_channel) {
	char response[25];
	/* Make sure we are in channel mode */
	int rlen = 10;
	int n = radio_send_command(serialdev, RADIO_CMD_GET_MEM_VFO_MODE_A, strlen(RADIO_CMD_GET_MEM_VFO_MODE_A), response, rlen);
	if (n < 5) {
		debug_print("Can not send GET MEM A command to radio\n");
		return EXIT_FAILURE;
	}
	response[n] = 0;
	if (strncmp(RADIO_CMD_SET_MEM_MODE_A, response, strlen(RADIO_CMD_SET_MEM_MODE_A)) != 0) {
		debug_print("A band Not in Mem mode, switching\n");
		n = radio_send_command(serialdev, RADIO_CMD_SET_MEM_MODE_A, strlen(RADIO_CMD_SET_MEM_MODE_A), response, rlen);
		response[n] = 0;
//		debug_print("MEM MODE A: %s\n",response);
	}
	n = radio_send_command(serialdev, RADIO_CMD_GET_MEM_VFO_MODE_B, strlen(RADIO_CMD_GET_MEM_VFO_MODE_B), response, rlen);
	if (n < 5) {
		debug_print("Can not send GET MEM B command to radio\n");
		return EXIT_FAILURE;
	}
	response[n] = 0;
	if (strncmp(RADIO_CMD_SET_MEM_MODE_B, response, strlen(RADIO_CMD_SET_MEM_MODE_B)) != 0) {
		debug_print("B band Not in Mem mode, switching\n");
		n = radio_send_command(serialdev, RADIO_CMD_SET_MEM_MODE_B, strlen(RADIO_CMD_SET_MEM_MODE_B), response, rlen);
		response[n] = 0;
//		debug_print("MEM MODE B: %s\n",response);
	}

	/* Set the requested channels */
	char command[25];
	char channel_str[4];
	strlcpy(command, RADIO_CMD_MEM_CHANNEL, 4);
	strlcat(command, "0,", sizeof(command));
	snprintf(channel_str, 5, "%d\r",band_a_channel);
	strlcat(command, channel_str,sizeof(command));
	n = radio_send_command(serialdev, command, strlen(command), response, rlen);
	if (n < 5) {
		debug_print("Can not send command %s to radio\n", command);
		return EXIT_FAILURE;
	}
	response[n] = 0;
	debug_print("MR: %s",response);

	// TODO = check the channel was set or return error

	strlcpy(command, RADIO_CMD_MEM_CHANNEL, 4);
	strlcat(command, "1,", sizeof(command));
	snprintf(channel_str, 5, "%d\r",band_b_channel);
	strlcat(command, channel_str,sizeof(command));
	n = radio_send_command(serialdev, command, strlen(command), response, rlen);
	if (n < 5) {
		debug_print("Can not send command %s to radio\n", command);
		return EXIT_FAILURE;
	}
	response[n] = 0;
	debug_print("MR: %s",response);

	return EXIT_SUCCESS;
}

/*
 * radio_program_pm()
 * Put the radio into programming mode so that we can set the PM Channel
 * and toggle the radio in and out of cross band repeater
 * Pass in the desired PM channel and the boolean state of the
 * cross band repeater
 *
 * Confirms that the correct radio is connected and that the serial port is
 * setup as expected before programming the radio.
 *
 * TODO - If a command fails we still want to try to leave program mode.
 *
 * Returns EXIT_SUCCESS or EXIT_FAILURE
 *
 */
int radio_program_pm(char *serialdev, int pm, int cross_band_repeater) {
	char buf[1024];

	if (radio_check_connection(serialdev) != EXIT_SUCCESS) {
		return EXIT_FAILURE;
	}

    int fd = radio_openserial(serialdev);
    if (!fd) {
        fprintf(stderr, "Error while initializing %s.\n", serialdev);
        return EXIT_FAILURE;
    }

    tcflush(fd,TCIOFLUSH ); // clear any existing data

    char * prog_mode = "0M PROGRAM\r";
    int len = strlen(prog_mode);
    int p = write(fd, prog_mode, len);
    if (p != len) {
    	fprintf(stderr, "Error writing data.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

	usleep(100 * 1000);

    /* Wait for 3 bytes */
    int n = read(fd, buf, 3);
    if (strncmp("0M\r",buf,3) != 0) {
    int i;	    for (i=0; i< n; i++) {
    	    	//				debug_print("%c",buf[i]);
    	    	printf(" %0x",buf[i] & 0xff);
    	    	if ((i % 16 == 0) && (i != 0))
    	    		printf("\n");
    	    }
    	    printf("|\n");
    	fprintf(stderr, "ERROR: Could not set radio into program mode. Mising 3 byte response\n");
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    if (radio_reset_speed(fd) != EXIT_SUCCESS) {
    	fprintf(stderr, "ERROR: Could not set radio program mode baud rate\n");
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    tcflush(fd,TCIOFLUSH ); // clear any existing data and give terminal time to settle

    /* Send a request to read 0x17 bytes of data.  This will include the state of the
     * X-band repeater and the PM channel number */
    char dataR1[5] = {0x52, 0x00, 0x00, 0X00, 0x17};
    len = sizeof(dataR1);
    p = write(fd, dataR1, len);
    if (p != len) {
    	fprintf(stderr, "Error writing dataR1.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    usleep(50 * 1000);

    n = read(fd, buf, 0x17+5);
//    for (i=0; i< n; i++) {
//    	//				debug_print("%c",buf[i]);
//    	printf(" %0x",buf[i] & 0xff);
//    	if ((i % 16 == 0) && (i != 0))
//    		printf("\n");
//    }
//    printf("|\n");
    if (n != 0x17+5) {
    	fprintf(stderr, "Error reading data.  Requested %d but %d returned\n",0x17+5, n);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
//    printf("TRANSPONDER: %d\n", buf[0x10 + 5]);
//    printf("PM CHANNEL: %d\n", buf[0x16 + 5]);

    /* Now write the same data back but set the bytes for PM channel and x-band repeater*/
    buf[0x10 + 5] = cross_band_repeater; // cross band repeater bit
    buf[0x16 + 5] = pm; // PM Channel.  Only 0 can have the cross band repeater

    len = 0x17 + 5;
    p = write(fd, buf, len);
    if (p != len) {
    	fprintf(stderr, "Error writing dataW1.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

    /* Check that we received the 0x06 ack */
    usleep(50 * 1000);
    n = read(fd, buf, 1);
    if (n!=1 || buf[0] != 0x06) {
    	fprintf(stderr, "Error reading data.  Requested %d but %d returned\n",1, n);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

    /* Now check the external data band settings.  There are 6 memory regions starting at 0x200
     * and each 512 bytes long.  The databand is further offset at 0x175 and the speed is at 0x176 */
    /* Send a request to read 0x10 bytes of data.  */
    char dataR2[5] = {0x52, 0x00, 0x03, 0x75, 0x2};
    len = sizeof(dataR2);
    p = write(fd, dataR2, len);
    if (p != len) {
    	fprintf(stderr, "Error writing dataR1.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    usleep(50 * 1000);

    n = read(fd, buf, 0x2+5);
    int i;
    for (i=0; i< n; i++) {
    	//				debug_print("%c",buf[i]);
    	printf(" %0x",buf[i] & 0xff);
    	if ((i % 16 == 0) && (i != 0))
    		printf("\n");
    }
    printf("|\n");
    if (n != 0x2+5) {
    	fprintf(stderr, "Error reading data.  Requested %d but %d returned\n",0x2+5, n);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    printf("DATA BAND: %d\n", buf[5]);
    printf("SPEED: %d\n", buf[6]);

    /* Now write the same data back but set the bytes for data band and speed for this PM */
    buf[5] = 2; // external data band.  1 = band B, 2 = band A TX, band B RX
    buf[6] = 1; // speed

    len = 0x2 + 5;
    p = write(fd, buf, len);
    if (p != len) {
    	fprintf(stderr, "Error writing TNC mode and speed.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

    /* Check that we received the 0x06 ack */
    usleep(50 * 1000);
    n = read(fd, buf, 1);
    if (n!=1 || buf[0] != 0x06) {
    	fprintf(stderr, "Error reading data.  Requested %d but %d returned\n",1, n);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

    /* Now write an E to exit programming mode*/
    char data[] = {0x45};
    len = sizeof(data);
    p = write(fd, data, len);
    if (p != len) {
    	fprintf(stderr, "Error writing data.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }

    /* Check that we received the 0x06 ack */
	usleep(50 * 1000);
	n = read(fd, buf, 1);
    if (n!=1 || buf[0] != 0x06) {
    	fprintf(stderr, "Error reading data.  Requested %d but %d returned\n",1, n);
    	radio_closeserial(fd);
    	return EXIT_FAILURE;
    }
    radio_closeserial(fd);

    /* Wait for radio to come back up*/
    usleep(250*1000);
	while (radio_check_connection(serialdev) != EXIT_SUCCESS) {
		usleep(250*1000);
	}


    return EXIT_SUCCESS;
}

/*
 * radio_send_command()
 * Open the serial connection and send a command to the radio.
 * Pass in a buffer containing the command and its length
 * Pass in a buffer large enough to hold the response and its expected length
 *
 * Returns the number of bytes in the response or -1 for a failure
 */
int radio_send_command(char *serialdev, char * data, int len, char *response, int rlen) {

		int fd = radio_openserial(serialdev);
		if (!fd) {
			fprintf(stderr, "Error while initializing %s.\n", serialdev);
			return -1;
		}
		tcflush(fd,TCIOFLUSH );

		int p = write(fd, data, len);
		tcdrain(fd);
		if (p != len) {
			debug_print("Error writing data.  Sent %d but %d written\n",len, p);
			radio_closeserial(fd);
			return -1;
		}
//		else {
//			debug_print("Wrote %d bytes\n",p);
//		}
		usleep(50*1000);
		debug_print("Response:");
		int n = read(fd, response, rlen);
		if (response[0] == '?') {
			debug_print("Radio Command failed\n");
			radio_closeserial(fd);
			return -1;
		}

		int i;
		for (i=0; i< n; i++) {
//			debug_print("%c",response[i]);
								debug_print(" %0x",response[i] & 0xff);
		}
		debug_print("\n");
		if (n < 0) {
			printf ("error %d reading data\n", errno);
			radio_closeserial(fd);
			return 1;
		}
		//		set_rts(g_serial_fd, 0);
		radio_closeserial(fd);
		return n;
}
