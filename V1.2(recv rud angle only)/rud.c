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
#include "rud.h"

// 工控器件相关
#define LENGTH_DI_SEND 8
#define LENGTH_DI_RECV 7
#define LENGTH_DO_SEND 8
#define LENGTH_DO_RECV 6
#define LENGTH_SETTING_SEND 17
#define LENGTH_SETTING_RECV 8
#define LENGTH_RUD_SEND 13
#define LENGTH_RUD_RECV 8

//CRC校验
UINT16 CalcuCRC16(UINT8 *Array, UINT32 length)
{
	UINT16 Reg_CRC=0xffff;//寄存器内容

	UINT8 i,j;
	for(i=0;i<length;i++)
	{
	Reg_CRC^=Array[i];
	for(j=0;j<8;j++)
	{
	if(Reg_CRC &0x0001)
		Reg_CRC=Reg_CRC>>1^0xA001;
	else
	Reg_CRC>>=1;
	}
	}
	//printf("%X \n",Reg_CRC);
	return Reg_CRC;
}

INT16 Send_DI(INT32 fd, UINT8 sendbuf[]) // 读限位
{
    INT32 res, retval, i;
    INT16 mode = 0;

    UINT8 recvbuf[LENGTH_DI_RECV] = {0x00};

    tcflush(fd, TCIOFLUSH);

    res = WriteComPort(fd, sendbuf, LENGTH_DI_SEND); 
    printf("Send_DI: ");
    for(i = 0;i < LENGTH_DI_SEND;i++)
    {
        printf("%X ", sendbuf[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("write Send_DI error\n"); 
    }

    // usleep(100);

    retval = ReadComPort(fd, recvbuf, LENGTH_DI_RECV); 
    printf("read Send_DI: ");
    for(i = 0;i < LENGTH_DI_RECV ;i++)
    {
        printf("%X ", recvbuf[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("read 'write Send_DI' error\n"); 
    }

    // usleep(100);

    mode = recvbuf[3] + (recvbuf[4] << 8);

    return mode;
}

INT8 Send_DO(INT32 fd, UINT8 sendbuf[]) // 读限位
{
    INT32 res, retval, i;

    UINT8 recvbuf[LENGTH_DO_RECV] = {0x00};

    tcflush(fd, TCIOFLUSH);

    res = WriteComPort(fd, sendbuf, LENGTH_DO_SEND); 
    // printf("Send_DO: ");
    // for(i = 0;i < LENGTH_DO_SEND;i++)
    // {
    //     printf("%X ", sendbuf[i]);
    // }
    // printf("\n");
    if (res < 0)
    {
        perror("write Send_DO error\n"); 
    }

    usleep(1*1000);

    retval = ReadComPort(fd, recvbuf, LENGTH_DO_RECV); 
    // printf("read Send_DO: ");
    // for(i = 0;i < LENGTH_DO_RECV;i++)
    // {
    //     printf("%X ", recvbuf[i]);
    // }
    // printf("\n");
    if (retval < 0)
    {
        perror("read 'write Send_DO' error\n"); 
    }

    // usleep(100);

    return recvbuf[3];
}

void Initial_Motor(INT32 fd, INT8 motor)
{
    INT32 res, retval, i;
    UINT16 reg_crc = 0;

    UINT8 initial_motor_send[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x0F, 0x00, 0x00}, 
                    initial_motor_recv[8] = {0x00};

    tcflush(fd, TCIOFLUSH);

    switch(motor)
    {
        case 1: // 左推
            initial_motor_send[0] = 0x03;
            break;
        case 2: // 右推
            initial_motor_send[0] = 0x04;
            break;
    }

    // 校验
    reg_crc = CalcuCRC16(initial_motor_send, 6); 
    initial_motor_send[6] = reg_crc & 0xFF;
    initial_motor_send[7] = (reg_crc & 0xFF00) >> 8;

    // 方向 执行
    res = WriteComPort(fd, initial_motor_send, 8); 
    printf("motor %d, initial_motor_send: ", motor);
    for(i = 0;i < 8;i++)
    {
        printf("%X ", initial_motor_send[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("write initial_motor_send error\n"); 
    }

    usleep(1*1000);

    retval = ReadComPort(fd, initial_motor_recv, 8); 
    printf("motor %d, initial_motor_recv: ", motor);
    for(i = 0;i < 8;i++)
    {
        printf("%X ", initial_motor_recv[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("read initial_motor_recv error\n"); 
    }

    usleep(1*1000);
}

void Set_Motor_Mode(INT32 fd, INT8 motor, INT8 mode)
{
    INT32 res, retval, i;
    UINT16 reg_crc = 0;

    UINT8 setting_mode_send[8] = {0x00, 0x06, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00}, 
                    setting_mode_recv[8] = {0x00};

    tcflush(fd, TCIOFLUSH);

    switch(motor)
    {
        case 1: // 左推
            setting_mode_send[0] = 0x03;
            break;
        case 2: // 右推
            setting_mode_send[0] = 0x04;
            break;
    }

    setting_mode_send[5] = mode;

    // 校验
    reg_crc = CalcuCRC16(setting_mode_send, 6); 
    setting_mode_send[6] = reg_crc & 0xFF;
    setting_mode_send[7] = (reg_crc & 0xFF00) >> 8;

    // 方向 执行
    res = WriteComPort(fd, setting_mode_send, 8); 
    printf("motor %d, setting_mode_send: ", motor);
    for(i = 0;i < 8;i++)
    {
        printf("%X ", setting_mode_send[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("write setting_mode_send error\n"); 
    }

    usleep(1*1000);

    retval = ReadComPort(fd, setting_mode_recv, 8); 
    printf("motor %d, setting_mode_recv: ", motor);
    for(i = 0;i < 8;i++)
    {
        printf("%X ", setting_mode_recv[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("read setting_mode_recv error\n"); 
    }

    usleep(1*1000);
}

void Set_Rudder_Argument(INT32 fd, INT8 motor, INT8 speed, UINT8 acceleration, UINT8 deceleration)
{
    INT32 res, retval, i;
    UINT16 reg_crc = 0;

    UINT8 setting_argument_send[LENGTH_SETTING_SEND] = {0x00, 0x10, 0x60, 0x81, 0x00, 0x04, 0x08, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00},
                    setting_argument_recv[LENGTH_SETTING_RECV] = {0x00};

    tcflush(fd, TCIOFLUSH);

    switch(motor)
    {
        case 1: // 左推
            setting_argument_send[0] = 0x03;
            break;
        case 2: // 右推
            setting_argument_send[0] = 0x04;
            break;
    }

    setting_argument_send[LENGTH_SETTING_SEND - 7] = speed;
    setting_argument_send[LENGTH_SETTING_SEND - 6] = (acceleration & 0xFF00) >> 8;
    setting_argument_send[LENGTH_SETTING_SEND - 5] = acceleration & 0xFF;
    setting_argument_send[LENGTH_SETTING_SEND - 4] = (deceleration & 0xFF00) >> 8;
    setting_argument_send[LENGTH_SETTING_SEND - 3] = deceleration & 0xFF;

    // 校验
    reg_crc = CalcuCRC16(setting_argument_send, LENGTH_SETTING_SEND - 2); 
    setting_argument_send[LENGTH_SETTING_SEND - 2] = reg_crc & 0xFF;
    setting_argument_send[LENGTH_SETTING_SEND - 1] = (reg_crc & 0xFF00) >> 8;

    // 方向 执行
    res = WriteComPort(fd, setting_argument_send, LENGTH_SETTING_SEND); 
    printf("motor %d, setting_argument_send: ", motor);
    for(i = 0;i < LENGTH_SETTING_SEND;i++)
    {
        printf("%X ", setting_argument_send[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("write setting_argument_send error\n"); 
    }

    usleep(1*1000);

    retval = ReadComPort(fd, setting_argument_recv, LENGTH_SETTING_RECV); 
    printf("motor %d, setting_argument_recv: ", motor);
    for(i = 0;i < LENGTH_SETTING_RECV;i++)
    {
        printf("%X ", setting_argument_recv[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("read setting_send error\n"); 
    }

    usleep(1*1000);
}

void Send_Rudder(INT32 fd, INT8 motor, double rudder) // 发送后推舵角
{
    INT32 res, retval, i;
    INT32 temp = 0; // 舵角值与系数
    UINT16 reg_crc = 0; // 校验位
    double  k = 35.0;

    UINT8 goal_send[LENGTH_RUD_SEND] = {0x00, 0x10, 0x60, 0x7A, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // 目标值发送
                    running_send_left[8] = {0x03, 0x06, 0x60, 0x40, 0x00, 0x3F, 0xD7, 0xEC},  // 新位置确认并执行
                    setting_send_left[8] = {0x03, 0x06, 0x60, 0x40, 0x00, 0x0F, 0xD7, 0xF8}, // 将running_send 控制位置0，便于下一次采集
                    running_send_right[8] = {0x04, 0x06, 0x60, 0x40, 0x00, 0x3F, 0xD6, 0x5B},  // 新位置确认并执行
                    setting_send_right[8] = {0x04, 0x06, 0x60, 0x40, 0x00, 0x0F, 0xD6, 0x4F}, // 将running_send 控制位置0，便于下一次采集
                    goal_recv[LENGTH_RUD_RECV] = {0x00}, // 目标值发送后接收的返回数组
                    setting_recv[8] = {0x00}; // 设定数组发送后接收的返回数组

    tcflush(fd, TCIOFLUSH);

    switch(motor)
    {
        case 1: // 左推
            goal_send[0] = 0x03;
            break;
        case 2: // 右推
            goal_send[0] = 0x04;
            break;
    }

    temp = rudder * k; // 参数可调
    goal_send[LENGTH_RUD_SEND - 6] = (temp & 0xFF000000) >> 24;
    goal_send[LENGTH_RUD_SEND - 5] = (temp & 0xFF0000) >> 16;
    goal_send[LENGTH_RUD_SEND - 4] = (temp & 0xFF00) >> 8;
    goal_send[LENGTH_RUD_SEND - 3] = temp & 0xFF;

    // 校验
    reg_crc = CalcuCRC16(goal_send, LENGTH_RUD_SEND - 2); 
    goal_send[LENGTH_RUD_SEND - 2] = reg_crc & 0xFF;
    goal_send[LENGTH_RUD_SEND - 1] = (reg_crc & 0xFF00) >> 8;

    // 方向 执行
    res = WriteComPort(fd, goal_send, LENGTH_RUD_SEND); 
    printf("motor %d, goal_send: ", motor);
    for(i = 0;i < LENGTH_RUD_SEND;i++)
    {
        printf("%X ", goal_send[i]);
    }
    printf("\n");
    if (res < 0)
    {
        perror("********************write goal_send error********************\n"); 
    }

	usleep(50);

    retval = ReadComPort(fd, goal_recv, LENGTH_RUD_RECV); 
    printf("motor %d, goal_recv: ", motor);
    for(i = 0;i < LENGTH_RUD_RECV;i++)
    {
        printf("%X ", goal_recv[i]);
    }
    printf("\n");
    if (retval < 0)
    {
        perror("********************read goal_recv error********************\n"); 
    }

	usleep(50);

    switch(motor)
    {
        case 1: // 左推
            // 新位置确认并执行
            res = WriteComPort(fd, running_send_left, 8); 
            printf("motor %d, running_send_left: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", running_send_left[i]);
            }
            printf("\n");
            if (res < 0)
            {
                perror("********************write running_send_left error********************\n"); 
            }

		    usleep(50);

            retval = ReadComPort(fd, setting_recv, 8); 
            printf("motor %d, setting_recv: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_recv[i]);
            }
            printf("\n");
            if (retval < 0)
            {
                perror("********************read 'write running_send_left' error********************\n"); 
            }            

		    usleep(50);

            // 将running_send 控制位置0，便于下一次采集
            res = WriteComPort(fd, setting_send_left, 8); 
            printf("motor %d, setting_send_left: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_send_left[i]);
            }
            printf("\n");
            if (res < 0)
            {
                perror("********************write setting_send_left error********************\n"); 
            }

		    usleep(50);

            retval = ReadComPort(fd, setting_recv, 8); 
            printf("motor %d, setting_recv: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_recv[i]);
            }
            printf("\n");
            if (retval < 0)
            {
                perror("*********************read 'write setting_send_left' error********************\n"); 
            }            
		        
            usleep(1*1000);

            break;
        case 2: // 右推
            // 新位置确认并执行
            res = WriteComPort(fd, running_send_right, 8); 
            printf("motor %d, running_send_right: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", running_send_right[i]);
            }
            printf("\n");
            if (res < 0)
            {
                perror("********************write running_send_right error********************\n"); 
            }

		    usleep(50);

            retval = ReadComPort(fd, setting_recv, 8); 
            printf("motor %d, setting_recv: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_recv[i]);
            }
            printf("\n");
            if (retval < 0)
            {
                perror("********************read 'write running_send_right' error********************\n"); 
            }            

		    usleep(50);

            // 将running_send 控制位置0，便于下一次采集
            res = WriteComPort(fd, setting_send_right, 8); 
            printf("motor %d, setting_send_right: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_send_right[i]);
            }
            printf("\n");
            if (res < 0)
            {
                perror("********************write setting_send_right error********************\n"); 
            }

		    usleep(50);

            retval = ReadComPort(fd, setting_recv, 8); 
            printf("motor %d, setting_recv: ", motor);
            for(i = 0;i < 8;i++)
            {
                printf("%X ", setting_recv[i]);
            }
            printf("\n");
            if (retval < 0)
            {
                perror("********************read 'write setting_send_right' error********************\n"); 
            }       

            usleep(1*1000);

            break;
    }
}

// INT16 Return(INT32 fd)
// {

// }
