
#include <string.h>
#include <termio.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>        
#include <stdlib.h>   
#include <fcntl.h>

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	/* 获取fd串口对应的termios结构体，这步主要是查询串口是否启动正常 */
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1");
		return -1;
	}
	//清空
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;	//配置成本地模式(本地连接、不改变端口所有者)、可读
	newtio.c_cflag &= ~CSIZE;		//清空数据位设置
	/* 选择数据位 */
	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;	
		break;
	}
	/* 选择校验位 */
	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);	//启用输入奇偶检测、去掉第八位
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		break;
	}
	/* 选择波特率 */
	switch( nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	/* 选择停止位，貌似linux下不能设置(1.5 0.5)停止位 */
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |=  CSTOPB;
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	/* 设置新配置 */
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
//	printf("set done!\n\r");
	return 0;
}

int main()
{
	int fd,wr_static,i=10;
	char *uart3 = "/dev/ttyUSB0";		//ttySAC代表开发板上的串口，对应iTop4412开发板上位uart3,可通过原理图查找序号
	//char *buffer = "";		
	unsigned char buffer[10]={0x55,0x55,0x01,0x07,0x01,0x20,0x03,0xE8,0x03,0xE8};
	printf("\r\nitop4412 uart3 writetest start\r\n");
	/* 打开串口，可读写，不将该设备作为此进程的控制终端，非阻塞方式操作 */
	if((fd = open(uart3, O_RDWR|O_NOCTTY|O_NDELAY))<0){
		printf("open %s is failed",uart3);
	}
	else{
		printf("open %s is success\n",uart3);
		set_opt(fd, 115200, 8, 'N', 1); //设置串口
		while(i--)
		{
			wr_static = write(fd,buffer,10);   //发送串口流数据
			if(wr_static<0)
				printf("write failed\n");
			else{
				printf("wr_static is %d\n",wr_static);
			}
			sleep(1);		
		}
	}
	close(fd);	//释放串口设备资源
}