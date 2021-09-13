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
#include "gps.h"
#include "serial.h"
// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <stdbool.h>  // bool,true,false...
#include <sys/stat.h> // 通过文件名filename获取文件信息  1.是否需要？
#include <math.h>

// GPS
#define RECV_SIZE 68  // 接收上位机的数组长度






