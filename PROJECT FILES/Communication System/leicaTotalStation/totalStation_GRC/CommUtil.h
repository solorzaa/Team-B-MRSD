#ifndef CommUtil_H
#define CommUtil_H

#include <stdlib.h>
#include <termios.h>

#define COMMUTIL_SERIAL_PARITY_NONE 0
#define COMMUTIL_SERIAL_PARITY_ODD 1
#define COMMUTIL_SERIAL_PARITY_EVEN 2

int open_serial(const char *port, const int speed, const int parity, struct termios *saved_tty_parameters = NULL);
int conf_serial(int fd, int speed, int parity, struct termios *saved_tty_parameters = NULL);
void close_serial(int device, struct termios *saved_tty_parameters = NULL);

#endif
