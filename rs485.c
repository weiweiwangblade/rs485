#include "stdio.h"
#include "linux/serial.h"
#include "linux/fcntl.h"
#include "termios.h"
#include "sys/types.h"
#include "sys/stat.h"

#define TIOCGRS485	0x542E
#define TIOCSRS485	0x542F
#define FALSE		0x00
#define TRUE		0x01

#define MY_DEVICE	"/dev/ttyUSB0"
#define BAUDRATE	B115200

int speed_arr[] = {B115200,B19200,B9600};
int name_arr[] = {115200,19200,9600};


////////////////////////////////////////////////////////////////////////////////  
/** 
*@brief  设置串口通信速率 
*@param  fd     类型 int  打开串口的文件句柄 
*@param  speed  类型 int  串口速度 
*@return  void 
*/ 
void set_speed(int fd, int speed)
{
	int i;
	int status;
	struct termios opt;
	tcgetattr(fd, &opt);
	for(i= 0; i< sizeof(speed_arr) / sizeof(int); i++){
		if(speed == name_arr[i]){
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&opt, speed_arr[i]);
			cfsetospeed(&opt, speed_arr[i]);
			status = tcsetattr(fd,TCSANOW, &opt);
			if(status != 0)
				{
					perror("tcsetattr fd1\n");
					return;
				}	
			tcflush(fd,TCIOFLUSH);
		}
	}
}


////////////////////////////////////////////////////////////////////////////////  
/** 
*@brief   设置串口数据位，停止位和效验位 
*@param  fd     类型  int  打开的串口文件句柄 
*@param  databits 类型  int 数据位   取值 为 7 或者8 
*@param  stopbits 类型  int 停止位   取值为 1 或者2 
*@param  parity  类型  int  效验类型 取值为N,E,O,,S 
*/  
int set_Parity(int fd,int databits,int stopbits,int parity)  
{   
    struct termios options;   
    if  ( tcgetattr( fd,&options)  !=  0) {   
        perror("SetupSerial 1");       
        return(FALSE);    
    }  
    options.c_cflag &= ~CSIZE;   
    switch (databits) /*设置数据位数*/  
    {     
    case 7:       
        options.c_cflag |= CS7;   
        break;  
    case 8:       
        options.c_cflag |= CS8;  
        break;     
    default:      
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
    }  
    switch (parity)   
    {     
        case 'n':  
        case 'N':      
            options.c_cflag &= ~PARENB;   /* Clear parity enable */  
            options.c_iflag &= ~INPCK;     /* Enable parity checking */   
            break;    
        case 'o':     
        case 'O':       
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/    
            options.c_iflag |= INPCK;             /* Disnable parity checking */   
            break;    
        case 'e':    
        case 'E':     
            options.c_cflag |= PARENB;     /* Enable parity */      
            options.c_cflag &= ~PARODD;   /* 转换为偶效验*/       
            options.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':   
        case 's':  /*as no parity*/     
            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;break;    
        default:     
            fprintf(stderr,"Unsupported parity\n");      
            return (FALSE);    
        }    
    /* 设置停止位*/    
    switch (stopbits)  
    {     
        case 1:      
            options.c_cflag &= ~CSTOPB;    
            break;    
        case 2:      
            options.c_cflag |= CSTOPB;    
           break;  
        default:      
             fprintf(stderr,"Unsupported stop bits\n");    
             return (FALSE);   
    }   
    /* Set input parity option */   
    if (parity != 'n')     
        options.c_iflag |= INPCK;   
    tcflush(fd,TCIFLUSH);  
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/     
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
    if (tcsetattr(fd,TCSANOW,&options) != 0)     
    {   
        perror("SetupSerial 3");     
        return (FALSE);    
    }   
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/  
    options.c_oflag  &= ~OPOST;   /*Output*/  
    return (TRUE);    
}

void main(void)
{
	int i, fd, c=0, res;  
  
	char  buf[2];  
  
	printf("Start...\n");
	fd = open(MY_DEVICE,O_RDWR);
	if(fd < 0)
	{
		printf("Open device failed!\n");
	}
	else
	{
		printf("Open ttyUSB0 sucessfully\n");
	}
	printf("Open...\n");  
	set_speed(fd,115200);  
	if (set_Parity(fd,8,1,'N') == FALSE)  {  
		printf("Set Parity Error\n");  
        exit (0);  
	}
	printf("Reading...\n");  
    	while(1) {  
        res = read(fd, buf, 1); 
        if(res==0)  
            continue;  
        buf[res]=0;	
        printf("%s", buf);  
          
        if (buf[0] == 0x0d)  
            printf("\n");  
          
        if (buf[0] == '@') break;  
    }  
//	struct serial_rs485 rs485conf;
//	rs485conf.flags |= SER_RS485_ENABLED;

//	rs485conf.flags |= SER_RS485_RTS_ON_SEND;
//	rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);
//	rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;
//	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

//	rs485conf.delay_rts_before_send = 100;
//	rs485conf.delay_rts_after_send = 100;
//	rs485conf.flags |= SER_RS485_RX_DURING_TX;

//	if(ioctl (fd, TIOCSRS485, &rs485conf) < 0)
//	{
		printf("Error!\n");
//	}
	if(close(fd) < 0)
	{
		printf("Close device error!\n");
	}
	else
	{
		printf("Close device sucessfully\n");
	}
}
