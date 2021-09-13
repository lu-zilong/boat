#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> // 基本系统数据类型  dev_t设备号，size_t内存中对象大小，time_t以秒为单位计时
#include <termios.h>   // 串口编程头文件，初始化，给tty一个特定端口
#include <unistd.h>    // 定义更多函数原型，close... 对POSIX操作系统API访问功能
#include <errno.h>     // 错误代码提示，error值不同意义不同
#include <fcntl.h>     // 根据文件描述词来操作文件的特性，open,fclose...
#include <sys/ioctl.h>
#include <math.h>
// 引用其他文件封装的函数
#include "serial.h"
#include "dirrev.h"

// 工控器件相关
#define LENGTH_DIRREV_W 5
#define LENGTH_DIRREV_R 5

// INT8 recv_rev = 0;

UINT8 CalcuCRC_DirRev(UINT8 array[], INT8 length) //CRC校验,原理见说明书
{
    INT8 i = 0;
    UINT8 check_bit = array[0];

    check_bit ^= array[1];
    check_bit ^= (array[3] & 0x80) << 8;
    check_bit ^= array[3] & 0x7F;
    check_bit &= 0x7F;

    return check_bit;
}

// INT8 Send_Normal(INT32 fd) // send the normal array from industrial PC to thruster 推进器正常
INT16 Send_Normal(INT32 fd) // send the normal array from industrial PC to thruster 推进器正常
{
    INT32 res, retval, i;
    INT16 rev;
    // INT8 normal_bit;

    UINT8 send_normal_array[LENGTH_DIRREV_W] = {0xC5, 0x23, 0x00, 0x00, 0x66},  // the array which confirms that the thruster is normal
                    recv_array[LENGTH_DIRREV_R] = {0x00}; 

    tcflush(fd, TCIOFLUSH);

    res = WriteComPort(fd, send_normal_array, LENGTH_DIRREV_W); // send normal
    // printf("send_normal_array: ");
    // for(i = 0;i < LENGTH_DIRREV_W;i++)
    // {
    //     printf("%X ", send_normal_array[i]);
    // }
    // printf("\n");
    if (res < 0)
    {
        perror("@@@@@@@@@@@@  write send_normal_array error  @@@@@@@@@@@@\n"); 
    }

    retval = ReadComPort(fd, recv_array, LENGTH_DIRREV_R); 
    printf("%d recv_normal_array: ", fd);
    for(i = 0;i < LENGTH_DIRREV_R;i++)
    {
        printf("%X ", recv_array[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("@@@@@@@@@@@@  read 'write send_normal_array' error  @@@@@@@@@@@@\n"); 
    }

    // if(recv_array[1] == 0x1D) // only print normal_bit
    // {
    //     normal_bit = recv_array[1];
    // }

    if(recv_array[1] == 0x20) // only print rev 只打印转速.视具体情况而定
    {
        rev = (recv_array[2] << 8) + (recv_array[3]);
        printf("%d recv_rev = %d\n ", fd, rev);
    }

    // return normal_bit;
    return rev;
}

INT8 Send_DirRev(INT32 fd, INT8 rev) // 后推执行
{
    INT32 res, retval, i;
    UINT8 check_bit = 0x00;
    INT8 recv_rev = 0x1C;

    UINT8 send_array[LENGTH_DIRREV_W] = {0xC5, 0x25, 0x00, 0x00, 0x60}; // 1:头,2:功能,3方向,4:数值,5校验
    UINT8 recv_array[LENGTH_DIRREV_R] = {0x00};

    tcflush(fd, TCIOFLUSH);

    // 正反转
    if(rev >= 0)
    {
        if(rev >= 127)
        {
            rev = 127;
        }
        send_array[LENGTH_DIRREV_W - 2] = (rev | 0x80);  
    }
    else
    {
        if(rev <= -127)
        {
            rev = -127;
        }
        send_array[LENGTH_DIRREV_W - 2] = (((~rev) + 1) & 0x7F); // (rev & 0x7F)  (((!rev) + 1) & 0x7F)
    }

    // 校验
    check_bit = CalcuCRC_DirRev(send_array, LENGTH_DIRREV_W);
    send_array[LENGTH_DIRREV_W - 1] = check_bit;

    // 执行
    res = WriteComPort(fd, send_array, LENGTH_DIRREV_W); 
    printf("%d Send_dirrev: ", fd);
    for(i = 0;i < LENGTH_DIRREV_W;i++)
    {
        printf("%X ", send_array[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("@@@@@@@@@@@@  write Send_dirrev error  @@@@@@@@@@@@\n"); 
    }

    retval = ReadComPort(fd, recv_array, LENGTH_DIRREV_R); 
    printf("%d dirrev_recv: ", fd);
    for(i = 0;i < LENGTH_DIRREV_R;i++)
    {
        printf("%X ", recv_array[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("@@@@@@@@@@@@  read 'write Send_dirrev' error  @@@@@@@@@@@@\n"); 
    }

    if(recv_array[1] == 0x20) // only print rev 只打印转速.视具体情况而定
    {
        recv_rev = (recv_array[2] << 8) + (recv_array[3]);
        printf("%d recv_rev = %d\n ", fd, recv_rev);
    }

    recv_rev = recv_array[1];

    return recv_rev;
}

