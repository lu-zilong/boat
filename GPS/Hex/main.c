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
// #include "gps.h"
#include "serial.h"
// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <stdbool.h>  // bool,true,false...
#include <sys/stat.h> // 通过文件名filename获取文件信息  1.是否需要？
#include <math.h>

// GPS
#define GPS_RECV_SIZE 68  // 接收上位机的数组长度
// net
INT8 *CLIENT_IP = "127.0.0.1"; // 上位机IP 17 137 192.168.142.132
#define MYPORT 2000
#define CLIENT_PORT 8000 // 8005 8888
#define CLI_SEND_SIZE 3       // 发送给上位机的数组长度

typedef long int     INT64;
typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

//标准GI定位定姿消息集 
typedef struct 
{
    UINT32 gps_week;//GPS周
    float gps_sec; //GPS秒
    float heading; //偏航
    float pitch;
    float roll;
    double latitude;//纬度
    double longitude;//经度
    float altitude;//高度
    float ve;  //东向速度
    float vn;  //北向速度
    float vu;  //天向速度
    UINT16 NSV1; //天线 1 卫星数
    UINT16 NSV2; //天线 2 卫星数
    UINT16 PT1; //定位类型 bestpos 中 pos type
    UINT16 PT2; //定位类型 heading 中 pos type
    UINT32 checksum; // 校验和
}bdfpdb;

//遥测数据帧 字节对齐 UDP发送
typedef struct sendbuf  //GPFPD
{
    INT8 head; //$ GPFPD
    UINT32 gps_week;//GPS周
    float gps_sec; //GPS秒
    float heading; //偏航
    float pitch;
    float roll;
    double y_latitude; //纬度_y单位
    double x_longitude;//经度_x单位
    float altitude;//高度
    float ve;  //东向速度
    float vn;  //北向速度
    float vu;  //天向速度
    UINT8 tail;  //报尾('*') 
    UINT8 Xor;   //校验
}bdfpdb_send;

static struct itimerval oldtv;
bool timer_sign = 0; // 标志位

//文件
FILE *fp; 

// 函数声明
INT32 OpenPort(INT32 comport, INT32 baud);  // 打开端口
void Set_Timer(INT32 interv, INT32 val); // 设定定时器
void Signal_Handler(); // 定时执行的函数
INT8 Analyse_GPS(bdfpdb bdfpdb_stru, UINT8 recvbuf[]);

// 主函数
int main()
{
    bdfpdb bdfpdb_recv;

    INT64 temp_lint = 0;
    INT32 fd485_1, fd485_2, fd485_3, fd485_4, fd485_5, fd485_6, fd485_7;
    INT32 i = 0, j = 0, temp_int = 0;
    double temp_double = 0.0;
    float temp_float = 0;

    UINT8 recv_bdfpdb[GPS_RECV_SIZE] = {0x00};

    // // 读文件参数
    // char recvfromFile[100];
    // fp = fopen("/home/zmh/nfs", "r");
    // if (fp == NULL)
    // {
    //     printf("read file error\n");
    // }

    // memset(recvfromFile, 0, sizeof(recvfromFile));

    // 串口
    INT32 comport_A = 0, comport_B = 1, comport_C = 2, comport_D = 3, comport_E = 4, comport_F = 5, comport_G = 6; 
    INT32 baud_A = 38400, baud_B = 38400, baud_C = 38400, baud_D = 38400, baud_E = 38400, baud_F = 9600, baud_G = 115200; // baudrate
    // fd485_1 = OpenPort(comport_A, baud_A); // 后推1 
    // fd485_2 = OpenPort(comport_B, baud_B); // 后推2 
    // fd485_3 = OpenPort(comport_C, baud_C); // 
    // fd485_4 = OpenPort(comport_D, baud_D); // 
    // fd485_5 = OpenPort(comport_E, baud_E); // 左右舵 
    // fd485_6 = OpenPort(comport_F, baud_F); // 操纵杆 
    fd485_7 = OpenPort(comport_G, baud_G); // 后推1 

    // 网络（UDP，服务器）
    int recv_len = 0, send_len = 0;

    socklen_t len_client, len_server;
    struct sockaddr_in server_sockaddr;
    struct sockaddr_in client_sockaddr;
    int server_sockfd; // server_sockfd:服务器句柄; len:接收返回值; num:发送返回值;

    char sendtoclient[CLI_SEND_SIZE] = {0x00}; // unsigned char

    struct timeval read_timeout; // 用于设置超时结束UDP通信中recv函数的结构体
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;

    server_sockfd = socket(AF_INET, SOCK_DGRAM, 0);  //定义sockfd
    setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout); // 设置该socket接收时，超时则结束阻塞
    len_client = sizeof(client_sockaddr);
    len_server = sizeof(server_sockaddr);

    // IPV4 ,字节流,如果套接字类型不是原始套接字，那么这个参数就为0
    // 定义sockaddr_in 处理网络通信地址
    server_sockaddr.sin_family = AF_INET;                //IPV4，地址族
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY); //本机IP地址
    server_sockaddr.sin_port = htons(MYPORT);            //端口，16位
    client_sockaddr.sin_family = AF_INET;
    client_sockaddr.sin_addr.s_addr = inet_addr(CLIENT_IP); // 自定义ip
    client_sockaddr.sin_port = htons(CLIENT_PORT);          // 端口，16位

    // bind，成功返回0，出错返回-1
    if (bind(server_sockfd, (struct sockaddr *)&server_sockaddr, sizeof(server_sockaddr)) == -1)
    //                          指向本地IP地址的结构体指针          ，长度
    {
        perror("bind error");
        exit(1);
    }

    // set up timer and start
	signal(SIGALRM, Signal_Handler); // 定时执行函数
	Set_Timer(100, 1); // 定时时间

    printf("enter\n");

    //主体循环函数
    while (1)
    {
        if(timer_sign == 1) // 定时标志,每次循环等待信号
        {
            // rizhi
            // fp = fopen("msg.txt", "a+"); 
            // stdout = fp; 

            tcflush(fd485_7, TCIOFLUSH); 

            ReadComPort(fd485_7, recv_bdfpdb, GPS_RECV_SIZE);

            printf("recv from GPS: \n");
            for(i = 0;i <= GPS_RECV_SIZE/10;i++)
            {
                if(i == GPS_RECV_SIZE/10)
                {
                    for(j = 10 * i;j < 10 * i + GPS_RECV_SIZE % 10;j++)
                        printf("%X ", recv_bdfpdb[j]);
                    printf("\n");
                }
                else
                {
                    for(j = 10 * i;j < 10 * (i + 1);j++)
                        printf("%X ", recv_bdfpdb[j]);
                    printf("\n");
                }
            }

            if(recv_bdfpdb[0] == 0xAA && recv_bdfpdb[1] == 0x44 && recv_bdfpdb[2] == 0x10)
            {
                // INT8 Analyse_GPS(bdfpdb_recv, recv_bdfpdb);

                bdfpdb_recv.gps_week = recv_bdfpdb[4] + (recv_bdfpdb[5] << 8) + (recv_bdfpdb[6] << 16) + (recv_bdfpdb[7] << 24); // GNSS 周
                temp_int = recv_bdfpdb[8] + (recv_bdfpdb[9] << 8) + (recv_bdfpdb[10] << 16) + (recv_bdfpdb[11] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.gps_sec = temp_float;
                temp_int = recv_bdfpdb[12] + (recv_bdfpdb[13] << 8) + (recv_bdfpdb[14] << 16) + (recv_bdfpdb[15] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.heading = temp_float;
                temp_int = recv_bdfpdb[16] + (recv_bdfpdb[17] << 8) + (recv_bdfpdb[18] << 16) + (recv_bdfpdb[19] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.pitch = temp_float;
                temp_int = recv_bdfpdb[20] + (recv_bdfpdb[21] << 8) + (recv_bdfpdb[22] << 16) + (recv_bdfpdb[23] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.roll = temp_float;
                temp_lint = recv_bdfpdb[24] + (recv_bdfpdb[25] << 8) + (recv_bdfpdb[26] << 16) + (recv_bdfpdb[27] << 24) + (recv_bdfpdb[28] << 32) + (recv_bdfpdb[29] << 40) + (recv_bdfpdb[30] << 48) + (recv_bdfpdb[31] << 56);
                temp_double = *(double *)&temp_lint;
                bdfpdb_recv.latitude = temp_double;
                temp_lint = recv_bdfpdb[32] + (recv_bdfpdb[33] << 8) + (recv_bdfpdb[34] << 16) + (recv_bdfpdb[35] << 24) + (recv_bdfpdb[36] << 32) + (recv_bdfpdb[37] << 40) + (recv_bdfpdb[38] << 48) + (recv_bdfpdb[39] << 56);
                temp_double = *(double *)&temp_lint;
                bdfpdb_recv.longitude = temp_double;
                temp_int = recv_bdfpdb[40] + (recv_bdfpdb[41] << 8) + (recv_bdfpdb[42] << 16) + (recv_bdfpdb[43] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.altitude = temp_float;
                temp_int = recv_bdfpdb[44] + (recv_bdfpdb[45] << 8) + (recv_bdfpdb[46] << 16) + (recv_bdfpdb[47] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.ve = temp_float;
                temp_int = recv_bdfpdb[48] + (recv_bdfpdb[49] << 8) + (recv_bdfpdb[50] << 16) + (recv_bdfpdb[51] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.vn = temp_float;
                temp_int = recv_bdfpdb[52] + (recv_bdfpdb[53] << 8) + (recv_bdfpdb[54] << 16) + (recv_bdfpdb[55] << 24);
                temp_float = *(float *)&temp_int;
                bdfpdb_recv.vu = temp_float;
                // bdfpdb_recv.gps_sec = *(float *)&(recv_bdfpdb[8] + (recv_bdfpdb[9] << 8) + (recv_bdfpdb[10] << 16) + (recv_bdfpdb[11] << 24)); // GPS 周秒
                // bdfpdb_recv.heading = *(float *)&(recv_bdfpdb[12] + (recv_bdfpdb[13] << 8) + (recv_bdfpdb[14] << 16) + (recv_bdfpdb[15] << 24));
                // bdfpdb_recv.pitch = *(float *)&(recv_bdfpdb[16] + (recv_bdfpdb[17] << 8) + (recv_bdfpdb[18] << 16) + (recv_bdfpdb[19] << 24));
                // bdfpdb_recv.roll = *(float *)&(recv_bdfpdb[20] + (recv_bdfpdb[21] << 8) + (recv_bdfpdb[22] << 16) + (recv_bdfpdb[23] << 24));
                // bdfpdb_recv.latitude = *(double *)&(recv_bdfpdb[24] + (recv_bdfpdb[25] << 8) + (recv_bdfpdb[26] << 16) + (recv_bdfpdb[27] << 24) + (recv_bdfpdb[28] << 32) + (recv_bdfpdb[29] << 40) + (recv_bdfpdb[30] << 48) + (recv_bdfpdb[31] << 56));
                // bdfpdb_recv.longitude = *(double *)&(recv_bdfpdb[32] + (recv_bdfpdb[33] << 8) + (recv_bdfpdb[34] << 16) + (recv_bdfpdb[35] << 24) + (recv_bdfpdb[36] << 32) + (recv_bdfpdb[37] << 40) + (recv_bdfpdb[38] << 48) + (recv_bdfpdb[39] << 56));
                // bdfpdb_recv.altitude = *(float *)&(recv_bdfpdb[40] + (recv_bdfpdb[41] << 8) + (recv_bdfpdb[42] << 16) + (recv_bdfpdb[43] << 24));
                // bdfpdb_recv.ve = *(float *)&(recv_bdfpdb[44] + (recv_bdfpdb[45] << 8) + (recv_bdfpdb[46] << 16) + (recv_bdfpdb[47] << 24));
                // bdfpdb_recv.vn = *(float *)&(recv_bdfpdb[48] + (recv_bdfpdb[49] << 8) + (recv_bdfpdb[50] << 16) + (recv_bdfpdb[51] << 24));
                // bdfpdb_recv.vu = *(float *)&(recv_bdfpdb[52] + (recv_bdfpdb[53] << 8) + (recv_bdfpdb[54] << 16) + (recv_bdfpdb[55] << 24));
                bdfpdb_recv.NSV1 = recv_bdfpdb[56] + (recv_bdfpdb[57] << 8);
                bdfpdb_recv.NSV2 = recv_bdfpdb[58] + (recv_bdfpdb[59] << 8);
                bdfpdb_recv.PT1 = recv_bdfpdb[60] + (recv_bdfpdb[61] << 8);
                bdfpdb_recv.PT2 = recv_bdfpdb[62] + (recv_bdfpdb[63] << 8);
                bdfpdb_recv.checksum = (recv_bdfpdb[64] << 24) + (recv_bdfpdb[65] << 16) + (recv_bdfpdb[66] << 8) + recv_bdfpdb[67]; 

                printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d, %X\n", bdfpdb_recv.gps_week, bdfpdb_recv.gps_sec, 
                                bdfpdb_recv.heading, bdfpdb_recv.pitch, bdfpdb_recv.roll, 
                                bdfpdb_recv.latitude, bdfpdb_recv.longitude, bdfpdb_recv.altitude,
                                bdfpdb_recv.ve, bdfpdb_recv.vn, bdfpdb_recv.vu,
                                bdfpdb_recv.NSV1, bdfpdb_recv.NSV2, bdfpdb_recv.PT1, bdfpdb_recv.PT2, 
                                bdfpdb_recv.checksum);
            }

            // fflush(stdout);
            // fclose(fp); 

            memset(recv_bdfpdb, 0, sizeof(recv_bdfpdb));

            timer_sign = 0;
        }
    } // while

    close(fd485_1);
    // close(fd485_2);
    // close(fd485_3);
    // close(fd485_4);
    // close(fd485_5);
    // close(fd485_6);

    return 0;
}

INT32 OpenPort(INT32 comport, INT32 baud) 
{
    INT32 fd, ret;
    if ((ret = OpenComPort(comport, baud, 8, "1", 'N')) < 0) // 打开comX, 没有|O_Nsi_side_delay
    {
        perror("Can't Open the Serial Port"); // Can't Open the Serial Port
        return -1;
    }
    fd = getPortFd();
    return fd;
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

// INT8 Analyse_GPS(bdfpdb bdfpdb_stru, UINT8 recvbuf[])
// {
//     INT64 temp_lint = 0;
//     INT32 temp_int = 0;
//     double temp_double = 0.0;
//     float temp_float = 0;

//     bdfpdb_stru.gps_week = recvbuf[4] + (recvbuf[5] << 8) + (recvbuf[6] << 16) + (recvbuf[7] << 24); // GNSS 周
//     temp_int = recvbuf[8] + (recvbuf[9] << 8) + (recvbuf[10] << 16) + (recvbuf[11] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.gps_sec = temp_float;
//     temp_int = recvbuf[12] + (recvbuf[13] << 8) + (recvbuf[14] << 16) + (recvbuf[15] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.heading = temp_float;
//     temp_int = recvbuf[16] + (recvbuf[17] << 8) + (recvbuf[18] << 16) + (recvbuf[19] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.pitch = temp_float;
//     temp_int = recvbuf[20] + (recvbuf[21] << 8) + (recvbuf[22] << 16) + (recvbuf[23] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.roll = temp_float;
//     temp_lint = recvbuf[24] + (recvbuf[25] << 8) + (recvbuf[26] << 16) + (recvbuf[27] << 24) + (recvbuf[28] << 32) + (recvbuf[29] << 40) + (recvbuf[30] << 48) + (recvbuf[31] << 56);
//     temp_double = *(double *)&temp_lint;
//     bdfpdb_stru.latitude = temp_double;
//     temp_lint = recvbuf[32] + (recvbuf[33] << 8) + (recvbuf[34] << 16) + (recvbuf[35] << 24) + (recvbuf[36] << 32) + (recvbuf[37] << 40) + (recvbuf[38] << 48) + (recvbuf[39] << 56);
//     temp_double = *(double *)&temp_lint;
//     bdfpdb_stru.longitude = temp_double;
//     temp_int = recvbuf[40] + (recvbuf[41] << 8) + (recvbuf[42] << 16) + (recvbuf[43] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.altitude = temp_float;
//     temp_int = recvbuf[44] + (recvbuf[45] << 8) + (recvbuf[46] << 16) + (recvbuf[47] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.ve = temp_float;
//     temp_int = recvbuf[48] + (recvbuf[49] << 8) + (recvbuf[50] << 16) + (recvbuf[51] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.vn = temp_float;
//     temp_int = recvbuf[52] + (recvbuf[53] << 8) + (recvbuf[54] << 16) + (recvbuf[55] << 24);
//     temp_float = *(float *)&temp_int;
//     bdfpdb_stru.vu = temp_float;
//     // bdfpdb_stru.gps_sec = *(float *)&(recvbuf[8] + (recvbuf[9] << 8) + (recvbuf[10] << 16) + (recvbuf[11] << 24)); // GPS 周秒
//     // bdfpdb_stru.heading = *(float *)&(recvbuf[12] + (recvbuf[13] << 8) + (recvbuf[14] << 16) + (recvbuf[15] << 24));
//     // bdfpdb_stru.pitch = *(float *)&(recvbuf[16] + (recvbuf[17] << 8) + (recvbuf[18] << 16) + (recvbuf[19] << 24));
//     // bdfpdb_stru.roll = *(float *)&(recvbuf[20] + (recvbuf[21] << 8) + (recvbuf[22] << 16) + (recvbuf[23] << 24));
//     // bdfpdb_stru.latitude = *(double *)&(recvbuf[24] + (recvbuf[25] << 8) + (recvbuf[26] << 16) + (recvbuf[27] << 24) + (recvbuf[28] << 32) + (recvbuf[29] << 40) + (recvbuf[30] << 48) + (recvbuf[31] << 56));
//     // bdfpdb_stru.longitude = *(double *)&(recvbuf[32] + (recvbuf[33] << 8) + (recvbuf[34] << 16) + (recvbuf[35] << 24) + (recvbuf[36] << 32) + (recvbuf[37] << 40) + (recvbuf[38] << 48) + (recvbuf[39] << 56));
//     // bdfpdb_stru.altitude = *(float *)&(recvbuf[40] + (recvbuf[41] << 8) + (recvbuf[42] << 16) + (recvbuf[43] << 24));
//     // bdfpdb_stru.ve = *(float *)&(recvbuf[44] + (recvbuf[45] << 8) + (recvbuf[46] << 16) + (recvbuf[47] << 24));
//     // bdfpdb_stru.vn = *(float *)&(recvbuf[48] + (recvbuf[49] << 8) + (recvbuf[50] << 16) + (recvbuf[51] << 24));
//     // bdfpdb_stru.vu = *(float *)&(recvbuf[52] + (recvbuf[53] << 8) + (recvbuf[54] << 16) + (recvbuf[55] << 24));
//     bdfpdb_stru.NSV1 = recvbuf[56] + (recvbuf[57] << 8);
//     bdfpdb_stru.NSV2 = recvbuf[58] + (recvbuf[59] << 8);
//     bdfpdb_stru.PT1 = recvbuf[60] + (recvbuf[61] << 8);
//     bdfpdb_stru.PT2 = recvbuf[62] + (recvbuf[63] << 8);
//     bdfpdb_stru.checksum = (recvbuf[64] << 24) + (recvbuf[65] << 16) + (recvbuf[66] << 8) + recvbuf[67]; 

//     printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d, %X\n", bdfpdb_stru.gps_week, bdfpdb_stru.gps_sec, 
//                     bdfpdb_stru.heading, bdfpdb_stru.pitch, bdfpdb_stru.roll, 
//                     bdfpdb_stru.latitude, bdfpdb_stru.longitude, bdfpdb_stru.altitude,
//                     bdfpdb_stru.ve, bdfpdb_stru.vn, bdfpdb_stru.vu,
//                     bdfpdb_stru.NSV1, bdfpdb_stru.NSV2, bdfpdb_stru.PT1, bdfpdb_stru.PT2, 
//                     bdfpdb_stru.checksum);
    
//     return 0;
// }


