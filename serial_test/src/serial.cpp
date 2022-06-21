
#include <string.h>
#include <termio.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>        
#include <stdlib.h>   
#include <fcntl.h>

#include "serial.h"
#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>

using namespace std;
uint16_t position = 0;
//uint8_t id;
//uint16_t time;

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
uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

void sendCMD(const Servo &frame)
{
	int fd,wr_static,i=5;
	char *uart3 = "/dev/ttyUSB0";		
	//char *buffer = "";		
	//unsigned char buffer[10]={0x55,0x55,0x01,0x07,0x01,0x20,0x03,0xE8,0x03,0xE8};
	printf("\r\nitop4412 uart3 writetest start\r\n");
	unsigned char data[10];

	for (int i=0; i<10;i++)
	data[i]=frame.data[i];

	if((fd = open(uart3, O_RDWR|O_NOCTTY|O_NDELAY))<0){
		printf("open %s is failed",uart3);
	}
	else{
		printf("open %s is success\n",uart3);
		set_opt(fd, 115200, 8, 'N', 1); //设置串口
		while(i--)
		{
			wr_static = write(fd,data,10);   //发送串口流数据
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

void LobotSerialServoMove(const uint8_t id,const uint16_t position,const uint16_t time)
{
	ROS_INFO("in it ");
  uint16_t pos = position;  
  Servo frame;
  unsigned char  buf[10];
  if(pos < 0)
    pos = 0;
  if(pos > 1000)
  pos = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(pos);
  buf[6] = GET_HIGH_BYTE(pos);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  for (int i=0; i<10;i++){frame.data[i]=buf[i];}
  sendCMD(frame);
  ROS_INFO("send it ");
}

void servo_1CB (const std_msgs::UInt8& msg)
{	
	ROS_INFO("get 1");
	position = (msg.data/240.0)*1000;
    //position = 1000;
    LobotSerialServoMove(1,position,750);
	
}

void servo_2CB (const std_msgs::UInt8& msg)
{	
	ROS_INFO("get 2");
	position = (msg.data/240.0)*1000;
    //position = 1000;
    LobotSerialServoMove(2,position,750);
	
}

int main(int argc, char **argv)
{	
    ros::init(argc, argv, "servo_cmd");               
    ros::NodeHandle nh;                                          
    ros::Subscriber servo_sub_1 = nh.subscribe("/servo_cmd_1",1, servo_1CB);  
	ros::Subscriber servo_sub_2 = nh.subscribe("/servo_cmd_2",1, servo_2CB);   
	
	//cd servoCB(x);
    ros::spin();
ROS_INFO("get ");


}