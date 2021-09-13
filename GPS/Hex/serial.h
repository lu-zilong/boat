#ifndef _SERIAL_H
#define _SERIAL_H 1  //_SERIAL_H=1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

/* serial.c *///打开   串口号，         波特率，        数据位，           停止位，        校验
INT32 OpenComPort (INT32 ComPort, INT32 baudrate, INT32 databit,const char *stopbit, char parity);
void CloseComPort (void);//关闭串口

// INT32 ReadComPort (void *data, INT32 datalength);//读
// INT32 WriteComPort (UINT8 * data, INT32 datalength);//写

INT32 ReadComPort (int fd, void *data, INT32 datalength);//读
INT32 WriteComPort (int fd, UINT8 * data, INT32 datalength);//写

int getPortFd();//得到端口文件?

#endif /* serial.c */ //?

