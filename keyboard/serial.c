#include <termios.h>            /* tcgetattr, tcsetattr */
#include <stdio.h>              /* perror, printf, puts, fprintf, fputs */
#include <unistd.h>             /* read, write, close */
#include <fcntl.h>              /* open */
#include <sys/signal.h>
#include <sys/types.h>
#include <string.h>             /* bzero, memcpy */
#include <limits.h>             /* CHAR_MAX */
//多的
#include<stdlib.h>
#include<sys/stat.h>
#include<errno.h>
//dev_name2




#include "serial.h"
//这俩啥意思，超时时间？
#define TIMEOUT_SEC(buflen,baud) (buflen*20/baud+2)
#define TIMEOUT_USEC 0 

#define CH_TO_WAIT 5//CH_TO_WAIT=5
#define CH_BITS 11//CH_BITS=11

#define BUFFER_LEN  1024    /* sendfile() */   //BUFFER_LEN=1024

INT32    fd;             //File descriptor for the port
//static定义的变量仅本文件或函数体能访问**
static struct termios termios_old, termios_new;//为何两个？？？
static fd_set   fs_read, fs_write;//fd_set  fs_read 可读文件描述符集合；fs_write 可读文件描述符集合
static struct timeval tv_timeout;//这个时间内需要监视的描述符没事件发生则返回0。

static void     set_baudrate (INT32);//设置波特率（32位）
static INT32    get_baudrate ();//得到波特率（32位）
static void     set_data_bit (INT32 databit);//设置数据位
static INT32    baudrate2Bxx (INT32 baudrate);//波特率转比特？？？
static INT32    Bxx2baudrate (INT32 _baudrate);//比特转波特率？？？
static INT32    set_port_attr (//设置端口属性
                             INT32 baudrate,//波特率 		
                             INT32 databit,//数据位
                             const char *stopbit, //停止位
                             char parity);//奇偶校验位
static void     set_stopbit (const char *stopbit);//设置停止位
static void     set_parity (char parity);//设置奇偶校验位

/* Open serial port ComPort at baudrate baud rate. */
INT32 OpenComPort (INT32 ComPort, INT32 baudrate, INT32 databit,
                   const char *stopbit, char parity)//打开串口函数。ComPort=n，打开/dev/ttyOn。默认打开ttyO0
{
    char           *pComPort;
    INT32           retval;

    switch (ComPort) {//判断端口号
    case 0:
        pComPort = "/dev/ttyS0";
        break;
    case 1:
        pComPort = "/dev/ttyS1";
        break;
    case 2:
        pComPort = "/dev/ttyS4";
        break;
    case 3:
        pComPort = "/dev/ttyS5";
        break;
    case 4:
        pComPort = "/dev/ttyS6";
        break;
    case 5:
        pComPort = "/dev/ttyS7";
        break;
    case 6:
        pComPort = "/dev/ttyUSB1";
        break;
    default:
        pComPort = "/dev/ttyS0";//原O0
        break;
    }

    fd = open (pComPort, O_RDWR | O_NOCTTY);//打开串口设备,串口设备文件描述符，文件描述符与包括相关信息（如文件的打开模式、文件的位置类型、文件的初始类型等）的文件对象相关联，这些信息被称作文件的上下文。O_NOCTTY:本程序是不作为串口端口的“控制终端”。如果不作这样特别指出,会有一些输入字符(如一些产生中断信号的键盘输入字符等)影响进程（linux）。
    if (-1 == fd) {//打开失败
        fprintf (stderr, "cannot open port %s\n", pComPort);
        return (-1);
    }

   // printf("comport fd = %d\n", fd);

    tcgetattr (fd, &termios_old);       /* save old termios value 设置和获得 termios 结构函数*/  //设置旧的
    /* 0 on success, -1 on failure */
    retval = set_port_attr (baudrate, databit, stopbit, parity);//设置端口属性   ？？？ 
    if (-1 == retval) {//设置失败
        fprintf (stderr, "\nport %s cannot set baudrate at %d\n", pComPort,
                 baudrate);
    }
    return (retval);//返回它的值。成功时是多少？
}

/* close serial port by use of file descriptor fd */  //关闭端口
void CloseComPort ()
{
    /* flush output data before close and restore old attribute */
    tcsetattr (fd, TCSADRAIN, &termios_old);//设置和获得 termios 结构函数 （旧的）  TCSADRAIN:发送了所有输出后更改才发生。若更改输出参数则应用此选项。 ？？
    close (fd);
}

int getPortFd(){//得到端口文件符
    return fd;
}

INT32 ReadComPort (void *data, INT32 datalength)//读端口
{
    INT32           retval = 0; //局部变量？
    //宏
    FD_ZERO (&fs_read);//清空集合
    FD_SET (fd, &fs_read);//文件描述符集合中增加一个新的文件描述符
    tv_timeout.tv_sec = TIMEOUT_SEC (datalength, get_baudrate ());
    tv_timeout.tv_usec = TIMEOUT_USEC;
    retval = select (fd + 1, &fs_read, NULL, NULL, &tv_timeout);//返回对应位仍然为1的fd的总数。select:用于非阻塞中，当一个/组套接字有信号时通知，实现多路复用输入/输出模型。
//（监视文件描述符变化情况。如果集合中有一个文件可读，select返回一个大于0的值，表示有文件可读。没有则根据timeout判断是否超时。超时select返回0。）     
// fd+1  检查的个数比三个的最大值多一     
//tv_timeout:若为NULL，则为阻塞；若为0,则纯非阻塞；为普通值，则为等待超时时间
    
    if (retval > 0) {
//成功返回读取的字节数，出错返回-1并设置errno，如果在调read之前已到达文件末尾，则这次read返回0
        retval = read (fd, data, datalength);//read()会把参数fd 所指的文件传送datalength个字节到data指针所指的内存中。
        return (retval);
    }
    else {
        if (0 == retval ) {
            return (0);
        }else{
            return (-1);
        }
    }


}

INT32 ReadComPortA (void *data, INT32 datalength)//读啥？？？跟上一个有何区别？？？
{
tcflush (fd, TCIOFLUSH);//TCIOFLUSH：刷新收到的数据但是不读并且刷新写入的数据但是不传送（将缓冲器清空）。
    INT32           retval = 0;
    int bytes_read;//读的字节？
    int readlen;//读的长度？



    /**
     * caculate the time of 5 characters and get the maxim
     * with 3ms and 5 ch's time
    */
    tv_timeout.tv_sec = 0;
    tv_timeout.tv_usec = ( (CH_TO_WAIT * CH_BITS) * (1000000/get_baudrate()));//微秒？
    //printf("port read timeout:%dus\n",tv_timeout.tv_usec);


    //求解释......

    bytes_read = 0;
    while(bytes_read<datalength){//读到的小于数据长度
        tv_timeout.tv_sec = 0;
        tv_timeout.tv_usec = ( (CH_TO_WAIT * CH_BITS) * (1000000/get_baudrate()));
        FD_ZERO (&fs_read);
        FD_SET (fd, &fs_read);
        retval = select (fd + 1, &fs_read, NULL, NULL, &tv_timeout);
        if ( retval >0 ) {
            readlen = read (fd, (data+bytes_read), datalength);//？？？
            bytes_read += readlen;
        } else
            return (bytes_read==0?-1:bytes_read);
    }

    return -1;

}
/* 
 * Write datalength bytes in buffer given by UINT8 *data,
 * return value: bytes written
 * Nonblock mode
*/
INT32 WriteComPort (UINT8 * data, INT32 datalength)//写端口
{
    INT32           retval, len = 0, total_len = 0;

    FD_ZERO (&fs_write);
    FD_SET (fd, &fs_write);
    tv_timeout.tv_sec = TIMEOUT_SEC (datalength, get_baudrate ());
    tv_timeout.tv_usec = TIMEOUT_USEC;

    for (total_len = 0, len = 0; total_len < datalength;) {
        retval = select (fd + 1, NULL, &fs_write, NULL, &tv_timeout);
        if (retval) {
            len = write (fd, &data[total_len], datalength - total_len);//write函数将buf中的nbytes字节内容写入文件描述符fd.成功时返回写的字节数.失败时返回-1. 并设置errno变量.
            if (len > 0) {
                total_len += len;
            }
        }
        else {
            tcflush (fd, TCOFLUSH);     /* flush all output data */
            break;
        }
    }

    return (total_len);
}

/* get serial port baudrate */
static INT32 get_baudrate ()
{
    return (Bxx2baudrate (cfgetospeed (&termios_new)));
}



/* set serial port baudrate by use of file descriptor fd */
static void set_baudrate (INT32 baudrate)
{
    termios_new.c_cflag = baudrate2Bxx (baudrate);  /* set baudrate */
}

static void set_data_bit (INT32 databit)
{
    termios_new.c_cflag &= ~CSIZE;
    switch (databit) {
    case 8:
        termios_new.c_cflag |= CS8;
        break;
    case 7:
        termios_new.c_cflag |= CS7;
        break;
    case 6:
        termios_new.c_cflag |= CS6;
        break;
    case 5:
        termios_new.c_cflag |= CS5;
        break;
    default:
        termios_new.c_cflag |= CS8;
        break;
    }
}

static void set_stopbit (const char *stopbit)
{
    if (0 == strcmp (stopbit, "1")) {
        termios_new.c_cflag &= ~CSTOPB; /* 1 stop bit */
    }
    else if (0 == strcmp (stopbit, "1.5")) {
        termios_new.c_cflag &= ~CSTOPB; /* 1.5 stop bits */
    }
    else if (0 == strcmp (stopbit, "2")) {
        termios_new.c_cflag |= CSTOPB;  /* 2 stop bits */
    }
    else {
        termios_new.c_cflag &= ~CSTOPB; /* 1 stop bit */
    }
}

static void set_parity (char parity)
{
    switch (parity) {
    case 'N':                  /* no parity check */
        termios_new.c_cflag &= ~PARENB;
        break;
    case 'E':                  /* even */
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag &= ~PARODD;
        break;
    case 'O':                  /* odd */
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag |= ~PARODD;
        break;
    default:                   /* no parity check */
        termios_new.c_cflag &= ~PARENB;
        break;
    }
}



static INT32 set_port_attr (
                          INT32 baudrate,        // 1200 2400 4800 9600 .. 115200
                          INT32 databit,           // 5, 6, 7, 8
                          const char *stopbit,  //  "1", "1.5", "2"
                          char parity)              // N(o), O(dd), E(ven)
{
    bzero(&termios_new, sizeof (termios_new));
	cfmakeraw (&termios_new);

	set_baudrate (baudrate);
    termios_new.c_cflag |= CLOCAL | CREAD;      /* | CRTSCTS */
    set_data_bit (databit);
    set_parity (parity);
    set_stopbit (stopbit);
    termios_new.c_oflag = 0;
    termios_new.c_lflag |= 0;
    termios_new.c_oflag &= ~OPOST;
    termios_new.c_cc[VTIME] = 1;        /* unit: 1/10 second. */
    termios_new.c_cc[VMIN] = 255; /* minimal characters for reading */
    tcflush (fd, TCIOFLUSH);

	return (tcsetattr (fd, TCSANOW, &termios_new));
}




/**
 * baudrate xxx to Bxxx
 * 
 * @@param baudrate xxx
 * 
 * @@return 
 */
static INT32 baudrate2Bxx (INT32 baudrate)
{
    switch (baudrate) {
    case 0:
        return (B0);
    case 50:
        return (B50);
    case 75:
        return (B75);
    case 110:
        return (B110);
    case 134:
        return (B134);
    case 150:
        return (B150);
    case 200:
        return (B200);
    case 300:
        return (B300);
    case 600:
        return (B600);
    case 1200:
        return (B1200);
    case 2400:
        return (B2400);
    case 9600:
        return (B9600);
    case 19200:
        return (B19200);
    case 38400:
        return (B38400);
    case 57600:
        return (B57600);
    case 115200:
        return (B115200);
    default:
        return (B38400);//原9600
    }
}

/**
 * get boundrate from Bxxx
 * 
 * @@param baudrate Bxxx refers to bound rate
 * 
 * @@return 
 */
static INT32 Bxx2baudrate (INT32 _baudrate)
{
/* reverse baudrate */
    switch (_baudrate) {
    case B0:
        return (0);
    case B50:
        return (50);
    case B75:
        return (75);
    case B110:
        return (110);
    case B134:
        return (134);
    case B150:
        return (150);
    case B200:
        return (200);
    case B300:
        return (300);
    case B600:
        return (600);
    case B1200:
        return (1200);
    case B2400:
        return (2400);
    case B9600:
        return (9600);
    case B19200:
        return (19200);
    case B38400:
        return (38400);
    case B57600:
        return (57600);
    case B115200:
        return (115200);
    default:
        return (38400);//原9600
    }
}


