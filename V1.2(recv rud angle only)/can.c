#include <string.h>
#include <stdio.h>
#include "unistd.h"
#include "time.h"
#include "ECanVci.h"
#include "can.h"

#define OS_UNIX

/*  本文件具体解析请见 ''ECanVCI动态库手册'' 资料  */

INIT_CONFIG pInit; // INIT_CONFIG结构体定义了初始化CAN的配置。结构体将在InitCan函数中被填充。在初始化之前,要先填好这个结构体变量。
ERR_INFO ErrInfo;
CAN_OBJ pSend[2];
CAN_OBJ rx[2];
DWORD pD;

INT32 res;
INT32 rev = 0;

INT32 Open_CANPort()
{
    pInit.AccCode = 0; // 验收码。SJA1000的帧过滤验收码。
    pInit.AccMask = 0xffffff; // 屏蔽码。SJA1000的帧过滤屏蔽码。屏蔽码推荐设置为0xFFFF FFFF, 即全部接收。
    pInit.Reserved = 0; // 保留。
    pInit.Filter = 0; // 滤波使能。0=不使能,1=使能。使能时,请参照SJA1000验收滤波器设置验收码和屏蔽码。
    pInit.Timing0 = 0x01; // 波特率定时器0(BTR0)。设置值见表。
    pInit.Timing1 = 0x1c; // 波特率定时器1(BTR1)。设置值见表。
    pInit.Mode = 0; // 模式。=0为正常模式, =1为只听模式, =2为自发自收模式。

    // res = OpenDevice(4, 0, 0);
    // printf("opendevice:%d\r\n", res);
    // if(res == 0) return 1;

    while(res == 0)
    {
        res = CloseDevice(4, 0);
        res = OpenDevice(4, 0, 0); // 打开设备
        printf("opendevice:%d\r\n",res);
    }

    res = InitCAN(4, 0, 0, &pInit); // 此函数用以初始化指定的CAN通道。
    printf("InitCAN 1 value: %d\n", res);

    res = StartCAN(4, 0, 0); // 此函数用以启动USBCAN设备的某一个CAN通道。如有多个CAN通道时,需要多次调用。
    printf("StartCAN 1 value: %d\n", res);

    res = InitCAN(4, 0, 1, &pInit);
    printf("InitCAN 2 value: %d\n", res);

    res = StartCAN(4, 0, 1);
    printf("StartCAN 2 value: %d\n", res);

    return 0;
}

INT32 Send_Battery(INT32 frame_mode, INT32 func) // 侧推执行
{
    INT32 res, i, j, sendnum=0;

    // for(i = 0;i < 8;i++) // clear
    // {
    //     pSend[0].Data[i] = 0;
    // }

    if(frame_mode == 1) // frame of control
    {
        // define frame 定义帧
        pSend[0].ID=0x0C01F1A7;

        pSend[0].DataLen = 8;
        pSend[0].ExternFlag = 1; // 扩展帧
        pSend[0].RemoteFlag = 0;

        pSend[0].Data[0] = (func & 0xFF);
        pSend[0].Data[1] = (func & 0xFF00) >> 8;
        pSend[0].Data[2] = (func & 0xFF0000) >> 16;
    }

    if(frame_mode == 2)
    {
        // define frame 定义帧
        pSend[0].ID=0x1801F1A7;

        pSend[0].DataLen = 0;
        pSend[0].ExternFlag = 1; // 扩展帧
        pSend[0].RemoteFlag = 0;
    }

    printf("battery send = ");
    for(i = 0;i < 8;i++)
        printf("%X ", pSend[0].Data[i]);
    printf("\n");

    for(i = 0;i < 1;i++) // number of frame 1次发10帧
    {
        res = Transmit(4, 0, 0, &pSend[0], 1); // 返回实际发送成功的帧数量。
        sendnum = sendnum + res;
    }

    // printf("sendnum:%d  \r\n", sendnum);

    return 0;
}

INT32 Recv_Battery()
{
    INT32 i, j, len = 0, resnum=0;

    // usleep(1000);
    for (i = 0;i < 4;i++) // 此函数从指定的设备CAN通道的缓冲区里读取数据。
    {      
        len = Receive(4, 0, 0, &rx[0], 1, 0);
        if (len == 1)
        {
            resnum++;
        }
        printf("battery recv = ");
        for(j = 0;j < 8;j++)
            printf("%X ", rx[0].Data[j]);
        printf("\n");
    }

    // printf("resnum:%d\r\n", resnum);
}

INT32 Send_SideDir(INT32 rev) // 侧推执行
{
    INT32 res, i, j, sendnum=0;

    // define frame 定义帧
    pSend[1].ID = 0x08FF0000;

    pSend[1].DataLen = 8;
    pSend[1].ExternFlag = 1; // 扩展帧
    pSend[1].RemoteFlag = 0;
    pSend[1].Data[0] = 0x32;
    pSend[1].Data[1] = 0x99;
    // 正反转
    if(rev == 0)
    {
        pSend[1].Data[2] = 0x41;
        pSend[1].Data[3] = 0; //  
        pSend[1].Data[4] = 0; //
    }
    else
    {
        if(rev < 0)
        {
            rev = -rev;
            pSend[1].Data[2] = 0x51;
        }
        else
            pSend[1].Data[2] = 0x61;  
        
        if(rev >= 65535)
            rev = 65535;
        pSend[1].Data[3] = (rev) & 0xff; //  
        pSend[1].Data[4] = ((rev) & 0xff00) >> 8; //
    }

    pSend[1].Data[5] = 0;
    pSend[1].Data[6] = 0;
    pSend[1].Data[7] = 0;
    pSend[1].SendType = 0; // mode

    printf("side send = ");
    for(i = 0;i < 8;i++)
        printf("%X ", pSend[1].Data[i]);
    printf("\n");
    
    for(i = 0;i < 20;i++) // number of frame 1次发10帧
    {
        // pSend[1].ID=0x08FF0000;
        res = Transmit(4, 0, 1, &pSend[1],1); // 返回实际发送成功的帧数量。
        sendnum = sendnum + res;
    }

    return 0;
}

INT32 Recv_SideDir()
{
    INT32 i, j, len=0, resnum=0;

    // usleep(1000);
    for (i = 0;i < 20;i++) // 此函数从指定的设备CAN通道的缓冲区里读取数据。
    {      
        len = Receive(4, 0, 1, &rx[1], 1, 0);
        if (len == 1)
        {
            resnum++;
        }
    printf("side recv = ");
    for(j = 0;j < 8;j++)
        printf("%X ", rx[1].Data[j]);
    printf("\n");
    }

    // printf("sendnum:%d   resnum:%d\r\n", sendnum,resnum);
}

INT32 Close_CANDevice()
{
    INT32 res = 0;

    res = CloseDevice(4, 0);
    printf("CloseDevice: %d\n", res);

    return res;
}
