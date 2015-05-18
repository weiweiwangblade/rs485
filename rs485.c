#include "stdio.h"
#include "linux/serial.h"
#include "linux/fcntl.h"

#define TIOCGRS485	0x542E
#define TIOCSRS485	0x542F

#define MY_DEVICE	"/dev/ttyUSB0"

void main(void)
{
	int fd = open(MY_DEVICE,O_RDWR);
	if(fd < 0)
	{
		printf("Open device failed!\n");
	}
	else
	{
		printf("Open ttyUSB0 sucessfully\n");
	}
	struct serial_rs485 rs485conf;
	rs485conf.flags |= SER_RS485_ENABLED;

	rs485conf.flags |= SER_RS485_RTS_ON_SEND;
//	rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);
	rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;
//	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	rs485conf.delay_rts_before_send = 100;
	rs485conf.delay_rts_after_send = 100;
	rs485conf.flags |= SER_RS485_RX_DURING_TX;

	if(ioctl (fd, TIOCSRS485, &rs485conf) < 0)
	{
		printf("Error!\n");
	}
	if(close(fd) < 0)
	{
		printf("Close device error!\n");
	}
	else
	{
		printf("Close device sucessfully\n");
	}
}
