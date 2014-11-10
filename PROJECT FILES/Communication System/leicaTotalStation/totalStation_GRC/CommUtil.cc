#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "CommUtil.h"

//#define DEBUG 1

int open_serial(const char *port, const int speed, const int parity, struct termios *saved_tty_parameters) {
	int fd;

	signal(SIGIO, SIG_IGN);

	/* open port */
	if ((fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY)) < 0) {
		return fd;
	}

	return conf_serial(fd, speed, parity, saved_tty_parameters);
}

int conf_serial(int fd, int speed, int parity, struct termios *saved_tty_parameters) {
	struct termios tio;

	/* save olds settings port */
	if (saved_tty_parameters)
		if (tcgetattr(fd, saved_tty_parameters) < 0) {
#ifdef DEBUG
			perror("Can't get terminal parameters ");
#endif
			close(fd);
			return -1 ;
		}

	/* settings port */
	memset(&tio, 0, sizeof(tio));

	switch (speed) {
		case 0:
			tio.c_cflag = B0;
			break;
		case 50:
			tio.c_cflag = B50;
			break;
		case 75:
			tio.c_cflag = B75;
			break;
		case 110:
			tio.c_cflag = B110;
			break;
		case 134:
			tio.c_cflag = B134;
			break;
		case 150:
			tio.c_cflag = B150;
			break;
		case 200:
			tio.c_cflag = B200;
			break;
		case 300:
			tio.c_cflag = B300;
			break;
		case 600:
			tio.c_cflag = B600;
			break;
		case 1200:
			tio.c_cflag = B1200;
			break;
		case 1800:
			tio.c_cflag = B1800;
			break;
		case 2400:
			tio.c_cflag = B2400;
			break;
		case 4800:
			tio.c_cflag = B4800;
			break;
		case 9600:
			tio.c_cflag = B9600;
			break;
		case 19200:
			tio.c_cflag = B19200;
			break;
		case 38400:
			tio.c_cflag = B38400;
			break;
		case 57600:
			tio.c_cflag = B57600;
			break;
		case 115200:
			tio.c_cflag = B115200;
			break;
		case 230400:
			tio.c_cflag = B230400;
			break;
		case 460800:
			tio.c_cflag = B460800;
			break;
		case 500000:
			tio.c_cflag = B500000;
			break;
		default:
			tio.c_cflag = B9600;
	}

	tio.c_cflag = tio.c_cflag | CS8;

	if (parity == 0) {
		tio.c_iflag = IGNPAR;
	} else if (parity & 1) {
		tio.c_cflag = tio.c_cflag | PARENB | PARODD;
		tio.c_iflag = INPCK;
	} else {
		tio.c_cflag = tio.c_cflag | PARENB;
		tio.c_iflag = INPCK;
	}

	tio.c_cflag = tio.c_cflag | CLOCAL | CREAD;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;

	/* clean port */
	tcflush(fd, TCIFLUSH);

	fcntl(fd, F_SETFL, FASYNC);
	/* activate the settings port */
	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
#ifdef DEBUG
		perror("Can't set terminal parameters ");
#endif
		close(fd);
		return -1 ;
	}

	/* clean I & O device */
	tcflush(fd, TCIOFLUSH);

#ifdef DEBUG
	printf("setting ok:\n");
	printf("device	%s\n", port);
	printf("speed	%d\n", speed);
#endif

	return fd;
}

void close_serial(int device, struct termios *saved_tty_parameters) {
	if (saved_tty_parameters) {
		if (tcsetattr(device, TCSANOW, saved_tty_parameters) < 0) {
#ifdef DEBUG
			perror("Can't restore terminal parameters ");
#endif
		}

		tcflush(device, TCIOFLUSH);
	}

	close(device);
}

