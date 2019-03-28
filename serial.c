#include <stdio.h>	// Standard input/output definitions
#include <string.h>	// String function definitions
#include <unistd.h>	// UNIX standard function definitions
#include <fcntl.h>	// File control definitions
#include <errno.h>	// Error number definitions
#include <termios.h>	// POSIX terminal control definitions

int open_port(void)
{
	int fd; // File descriptor for the port

	fd = open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_SYNC);
	if (fd == -1)
	{
		perror("open_port: Unable to open /dev/ttyS4 - ");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
		perror("Port");
	}

	return (fd);
}

int main(void)
{
	int fd = open_port();

	struct termios options;

	tcgetattr(fd, &options);

	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
	options.c_iflag &= ~IGNBRK;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;

	options.c_oflag = 0;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 5;

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~(PARENB | PARODD);
	options.c_cflag |= 0;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CRTSCTS;

	tcsetattr(fd, TCSANOW, &options);

	char buf[128];
		int n = read (fd, buf, sizeof (buf));

	for (int i = 0; i < 10; ++i)
	{
		int n = read (fd, buf, sizeof (buf));
		buf[n] = '\0';
		printf("%i bytes recivied ", n);
		printf("%s\n", buf);
	}

	return 0;
}
