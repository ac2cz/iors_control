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

/* Forward declarations */
int radio_reset_speed(int fd);

/* Local vars */
static struct termios oldterminfo;

void radio_closeserial(int fd) {
    tcsetattr(fd, TCSANOW, &oldterminfo);
    if (close(fd) < 0)
        perror("closeserial()");
}


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
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* RAW Input */
    options.c_oflag &= ~OPOST; /* Raw output */

    /*
    * Set the new options for the port...
    */
    if (tcsetattr(fd, TCSANOW, &options) == -1) { //TCSANOW constant specifies that all changes should occur immediatel
    	printf("init serial error: tcsetattr()\n");
    	radio_closeserial(fd);
    	return 0;
    }

//
//    fcntl(fd, F_SETFL, FNDELAY);
//    fcntl(fd, F_SETFL, 0);

//    attr = oldterminfo;
//
//        /*
//        * Set the baud rates to 9600...
//        */
//        if (cfsetispeed(&attr, B9600) == -1) {
//            perror("openserial(): cfsetispeed()");
//            return 0;
//        }
//        if (cfsetospeed(&attr, B9600) == -1) {
//            perror("openserial(): cfsetospeed()");
//            return 0;
//        }
//        /*
//         * Enable the receiver and set local mode... The c_cflag member contains two options that should always be enabled,
//         * CLOCAL and CREAD. These will ensure that your program does not become the 'owner' of the port subject to sporatic
//         * job control and hangup signals, and also that the serial interface driver will read incoming data bytes.
//         */
//        attr.c_cflag |= (CLOCAL | CREAD);
//
//    /* This uses noncanonical input, and turns off most processing to give an unmodified
// channel to the terminal. It does exactly this:
//           termios-p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
//                                         |INLCR|IGNCR|ICRNL|IXON);
//           termios-p->c_oflag &= ~OPOST;
//           termios-p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
//           termios-p->c_cflag &= ~(CSIZE|PARENB);
//           termios-p->c_cflag |= CS8;
//     */
//    cfmakeraw(&attr);
//
//    attr.c_cc[VMIN] = (cc_t)1024; // wait for max 1024 bytes
//    attr.c_cc[VTIME] = 1;
//
//    attr.c_cflag |= CRTSCTS; /* Enable hardware flow control */
////    attr.c_cflag &= ~CRTSCTS; /* disable hardware flow contol */
//
////    attr.c_iflag |= (IXON | IXOFF | IXANY); /* Enable software flow control */
//    attr.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable software flow control */
//
//    if (tcsetattr (fd, TCSANOW, &attr) != 0)
//     	 printf ("error %d setting term attributes at open", errno);
//
//     /* FD in blocking mode so we wait for some data when reading */
//     fcntl(fd, F_SETFL, 0);

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
    return 0;
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

int program_read(int fd, char * buf, int len) {
	int i, n;
	int total = 0;
	while ((n = read(fd, buf, len)) != -1 && (total < len)) {
		total += n;
		for (i=0; i< n; i++) {
			//				debug_print("%c",buf[i]);
			printf(" %0x",buf[i] & 0xff);
			if ((i % 16 == 0) && (i != 0))
				printf("\n");
		}
		printf("|\n");
	}
	return total;

}

int radio_program_pm(char *serialdev, int pm, int cross_band_repeater) {
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
    	return 1;
    }

	usleep(50 * 1000);

    char buf[1024];
    int i;
    /* Wait for 3 bytes */
    int n = read(fd, buf, 3);
    radio_reset_speed(fd);
    tcflush(fd,TCIOFLUSH ); // clear any existing data and give terminal time to settle

    char dataR1[5] = {0x52, 0x00, 0x00, 0X00, 0x17};
//    char dataW1[] = {0x57, 0, 0, 0, 0x11, 0, 0x4b, 0x1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
//    		 0xff, 0xff, 0xff, 0xff, 0x00};

    len = sizeof(dataR1);
    p = write(fd, dataR1, len);
    if (p != len) {
    	fprintf(stderr, "Error writing dataR1.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return 1;
    }
    usleep(50 * 1000);

    n = read(fd, buf, 0x17+5);
    for (i=0; i< n; i++) {
    	//				debug_print("%c",buf[i]);
    	printf(" %0x",buf[i] & 0xff);
    	if ((i % 16 == 0) && (i != 0))
    		printf("\n");
    }
    printf("|\n");

    printf("TRANSPONDER: %d\n", buf[0x10 + 5]);
    printf("PM CHANNEL: %d\n", buf[0x16 + 5]);

    buf[0x10 + 5] = cross_band_repeater; // cross band repeater bit
    buf[0x16 + 5] = pm; // PM Channel.  Only 0 can have the cross band repeater

    //len = sizeof(dataW1);
    len = 0x17 + 5;
    p = write(fd, buf, len);
    if (p != len) {
    	fprintf(stderr, "Error writing dataW1.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return 1;
    }
    usleep(50 * 1000);

    n = read(fd, buf, 1);
    for (i=0; i< n; i++) {
    	//				debug_print("%c",buf[i]);
    	printf(" %0x",buf[i] & 0xff);
    	if ((i % 16 == 0) && (i != 0))
    		printf("\n");
    }
    printf("|\n");



    char data[] = {0x45};
    len = sizeof(data);
    p = write(fd, data, len);
    if (p != len) {
    	fprintf(stderr, "Error writing data.  Sent %d but %d written\n",len, p);
    	radio_closeserial(fd);
    	return 1;
    }
	usleep(50 * 1000);

	n = read(fd, buf, 1);
	for (i=0; i< n; i++) {
		//				debug_print("%c",buf[i]);
		printf(" %0x",buf[i] & 0xff);
		if ((i % 16 == 0) && (i != 0))
			printf("\n");
	}
	printf("|\n");
    radio_closeserial(fd);

    return EXIT_SUCCESS;
}

int radio_send_command(char *serialdev, char * data, int len, char *response, int rlen) {

		int fd = radio_openserial(serialdev);
		if (!fd) {
			fprintf(stderr, "Error while initializing %s.\n", serialdev);
			return 1;
		}
		tcflush(fd,TCIOFLUSH );

		int p = write(fd, data, len);
		tcdrain(fd);
		if (p != len) {
			debug_print("Error writing data.  Sent %d but %d written\n",len, p);
			radio_closeserial(fd);
			return 1;
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
			return 1;
		}

		int i;
		for (i=0; i< n; i++) {
			debug_print("%c",response[i]);
			//					debug_print(" %0x",buf[i] & 0xff);
		}

		if (n < 0) {
			printf ("error %d reading data\n", errno);
			radio_closeserial(fd);
			return 1;
		}
		tcflush(fd,TCIOFLUSH );
		//		set_rts(g_serial_fd, 0);
		radio_closeserial(fd);
		return EXIT_SUCCESS;
}
