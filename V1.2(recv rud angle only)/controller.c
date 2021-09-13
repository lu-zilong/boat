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
#include "controller.h"

// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <math.h>

INT16 Speed_Control(double del_speed[], INT8 goa_jst[], double K, double K1, double T, double T1, double Ts) // send the array from industrial PC to thruster
{
    // delta_speed[0] = ((goal_speed[0] + (1 + exp(-1/T2)) * goal_speed[1] + exp(-1/T2) * goal_speed[2] + K2 * (exp(-1/T2) + (T1 - T2)/T2) * delta_speed[1])) * T2/(K2 * T1);
    // goa_jst[0] = (1 + exp((-1 * T)/T2)) * goa_jst[1] - exp((-1 * T)/T2) * goa_jst[2] + ((K2 * T1)/T2) * del_speed[0] - K2 * (exp((-1 * T)/T2) + (T1 - T2)/T2) * del_speed[1];
    goa_jst[0] = (1 + exp((-1 * Ts)/T)) * goa_jst[1] - exp((-1 * Ts)/T) * goa_jst[2] + (K * (1 - exp((-1 * Ts)/T)) * T1)/K1 * del_speed[1] - exp((-1 * Ts)/T1) * (K * (1 - exp((-1 * Ts)/T)) * T1)/K1 * del_speed[2];

    // goa_jst[0] = 1.8618 * goa_jst[1] - 0.8618 * goa_jst[2] + 127.0 * del_speed[0] - 123.0524 * del_speed[1];
    // if(goa_jst[0] >= 127) goa_jst[0] = 127;
    // if(goa_jst[0] <= 0) goa_jst[0] = 0;

    return goa_jst[0];
}

double Speed_Model(INT8 goa_jst, double curr_v[], double K1, double T1, double Ts)
{
    curr_v[0] = exp((-1 * Ts/T1)) * curr_v[1] + (K1/T1) * goa_jst;

    // curr_v[0] = 0.007874 * goa_jst + 0.967216 * curr_v[1];

    return curr_v[0];
}

double Heading_Control(double del_heading[], double last_goa_rud, double K2, double T1, double T2, double T) // send the array from industrial PC to thruster
{
    double goa_rud = 0.0;

    // goa_rud = exp((-1 * T)/T2) * last_goa_rud + K2 * ((T1 * T2 + T2 - T1)/(T2 * T2)) * del_heading[0] - (K2 * T1 * exp(-1 * T/T2))/T2 * del_heading[1];
    
    goa_rud = 0.86792 * last_goa_rud + 2.848575 * del_heading[0] - 2.75517 * del_heading[1];

    if(goa_rud >= 35.0) goa_rud = 35.0;
    if(goa_rud <= -35.0) goa_rud = -35.0;

    // if(goa_rud <= 0.0) goa_rud = 0.0;

    return goa_rud;
}

double Heading_Model(double goa_rud[], double curr_h[], double K1, double T1, double T)
{
    // curr_h[0] = (1 + exp((-1 * T/T1))) * curr_h[1] - exp(-1 * T/T1) * curr_h[2] + K1 * (1 - exp(-1 * T/T1)) * goa_rud;

    curr_h[0] = 1.967213 * curr_h[1] - 0.967213 * curr_h[2] + 0.000819672 * goa_rud[0] + 0.001639344 * goa_rud[1] + 0.000819672 * goa_rud[2];

    return curr_h[0];
}

