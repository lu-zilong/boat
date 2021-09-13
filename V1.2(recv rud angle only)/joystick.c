#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> // 基本系统数据类型  dev_t设备号，size_t内存中对象大小，time_t以秒为单位计时
#include <termios.h>   // 串口编程头文件，初始化，给tty一个特定端口
#include <unistd.h>    // 定义更多函数原型，close... 对POSIX操作系统API访问功能
#include <errno.h>     // 错误代码提示，error值不同意义不同
#include <fcntl.h>     // 根据文件描述词来操作文件的特性，open,fclose...
#include <sys/ioctl.h>
// 引用其他文件封装的函数
#include "serial.h"
#include "joystick.h"

// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <math.h>

// 工控器件相关
#define LENGTH_JOYSTICK_R 9

UINT32 Joystick_Recv(INT32 fd, UINT8 recv_joystick[]) // 操纵杆数据接收
{
    INT32 res, retval, i;
    UINT32 rev = 0;

    tcflush(fd, TCIOFLUSH);

    retval = ReadComPort(fd, recv_joystick, LENGTH_JOYSTICK_R); 
    if (retval < 0)
    {
        perror("read 'write recv_joystick' error\n"); 
    }

    printf("recv joystick = ");
    for(i = 0;i < LENGTH_JOYSTICK_R;i++)
    {
        printf("%d ", recv_joystick[i]);
    }
    printf("\n");

    return rev;
}

