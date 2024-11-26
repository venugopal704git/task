#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
int main() 
{
	int serial_port=open("/dev/ttyACM3", O_RDWR | O_NOCTTY | O_SYNC);
	if (serial_port==-1) 
	{
		perror("Error opening serial port");
		return 0;
	}

	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) {
		perror("Error getting terminal attributes");
		close(serial_port);
		return 0;
	}
	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_lflag = 0;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 10;

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
	{
		perror("Error setting terminal attributes");
		close(serial_port);
		return 0;
	}
	char write_buf[1024]={'\0'};int n=0;
	while(1)
	{ 
		if (n=read(fd, &write_buf,sizeof(write_buf)) == -1) 
		{
			perror("Error writing to serial port");
			return 0;
		}

		//char read_buf;
		//int num_bytes = read(fd, &read_buf, sizeof(read_buf));
		printf("%s",write_buf);
		if (n < 0) 
		{
			perror("Error reading from serial port");
		}
	}
	close(serial_port);
}

