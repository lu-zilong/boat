#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> // 基本系统数据类型  dev_t设备号，size_t内存中对象大小，time_t以秒为单位计时
#include <termios.h>   // 串口编程头文件，初始化，给tty一个特定端口
#include <unistd.h>    // 定义更多函数原型，close... 对POSIX操作系统API访问功能
#include <errno.h>     // 错误代码提示，error值不同意义不同
#include <fcntl.h>     // 根据文件描述词来操作文件的特性，open,fclose...
#include <sys/ioctl.h>
// TCP/IP 服务器
#include <arpa/inet.h>  // 客户端信息转换为字符串信息,IP？？
#include <netinet/in.h> // 定义IP,端口等
#include <sys/shm.h>    // 共享内存？？
#include <sys/socket.h>
// 定时器
#include <signal.h>
#include <sys/time.h> // linux系统时间日期头文件
#include <time.h>     // 具有一写处理日期和时间的类型的函数
// 线程
#include <pthread.h>
// 引用其他文件封装的函数
#include "serial.h"
// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <stdbool.h>  // bool,true,false...
#include <sys/stat.h> // 通过文件名filename获取文件信息  1.是否需要？
#include <math.h>

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

static struct itimerval oldtv;
bool timer_sign = 0; // 标志位

// 函数声明
void Set_Timer(INT32 interv, INT32 val); // 设定定时器
void Signal_Handler(); // 定时执行的函数
int GPS_weeksec_BeiJing(double gpst[2]);

// 主函数
int main()
{
    INT32 i = 0, j = 0, n = 0, ret = 0;

    double t[2] = {2173.0, 545690.914};

    // // 读文件参数
    // char recvfromFile[100];
    // fp = fopen("/home/zmh/nfs", "r");
    // if (fp == NULL)
    // {
    //     printf("read file error\n");
    // }

    // memset(recvfromFile, 0, sizeof(recvfromFile));

    // set up timer and start
	signal(SIGALRM, Signal_Handler); // 定时执行函数
	Set_Timer(100, 1); // 定时时间

    printf("enter\n");

    //主体循环函数
    while (1)
    {
        if(timer_sign == 1) // 定时标志,每次循环等待信号
        {
            GPS_weeksec_BeiJing(t);
            timer_sign = 0;
        }
    } // while

    return 0;
}

void Set_Timer(INT32 interv, INT32 val)
{
	struct itimerval itv;
	itv.it_interval.tv_sec = 0;
	itv.it_interval.tv_usec = interv * 1000; // 100ms per
	itv.it_value.tv_sec = val; // wait for 1s , then start interval
	itv.it_value.tv_usec = 0;
	setitimer(ITIMER_REAL, &itv, &oldtv);
}
 
void Signal_Handler()
{
    timer_sign = 1; // 定时标志,每次循环等待信号
}

///*********************************************************
///Summary：GPS时间（周秒）转化为公历时间(+8北京时间)
///输入：GPS 周 秒
///输出：公历北京时间
///*********************************************************
int GPS_weeksec_BeiJing(double gpst[2])
{
    double mjd,J,j0,n1,n2,n3,dd,ad;
    int BC;
    //GPS从MJD44244开始
    ad=(gpst[0] * 86400 * 7 + gpst[1]) / 86400;
    mjd = 44244 + ad;
    // 从简化儒略日计算公历年月日时分秒
    // 返回的cal是年月日时分秒 列表
    // 公元1582年10月4日24:00点之前使用儒略历，公元1582年10月15日00:00点之后使用公历
    J = mjd + 2400000.5;
    if (J < 1721423.5)
    {
     // 公元1月1日0时
        BC = 1;
    } else
    {
        BC = 0;
    }

    if (J < 2299160.5)
    {
    // 1582.10.4. 24:00 前
    j0 = floor(J + 0.5);
    dd = J + 0.5 - j0;
    } else
    {
    // 不是闰年的年数
    n1 = floor((J - 2342031.5) / 36524.25 / 4) + 1;  // 1700.3.1.0
    n2 = floor((J - 2378555.5) / 36524.25 / 4) + 1; // 1800.3.1.0
    n3 = floor((J - 2415079.5) / 36524.25 / 4) + 1;  // 1900.3.1.0
    j0 = n1 + n2 + n3 + J + 10;
    dd = j0 + 0.5 - floor(j0 + 0.5);
    j0 = floor(j0 + 0.5);
    }

    j0 = j0 + 32083;
    int year0,year,day,month,sec,hour,minute;
    year0 = ceil(j0 / 365.25) - 1;
    year = year0 - 4800;
    day = j0 - floor(year0 * 365.25);
    month = floor((day - 0.6) / 30.6) + 3;
    day = day - round((month - 3) * 30.6);

    if (month > 12)
    {
      month = month - 12;
      year = year + 1;
    }
    year = year - BC;
    sec = round(dd * 86400);
    hour = floor(sec / 3600);
    sec = sec - hour * 3600;
    minute = floor(sec / 60);
    sec = sec - minute * 60;
    //转北京时间
    hour=hour+8;
    printf("公历时间:%d:%d:%d %d:%d:%d",year, month, day, hour, minute, sec);
}

