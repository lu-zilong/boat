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
// #include "can.h"
#include "serial.h"
#include "rud.h"
#include "dirrev.h"
#include "joystick.h"
#include "controller.h"
// 以下头文件是否有用？
#include <limits.h>   // ???对数据类型限制
#include <stdbool.h>  // bool,true,false...
#include <sys/stat.h> // 通过文件名filename获取文件信息  1.是否需要？
#include <math.h>

//网络
#define MYPORT 10010 // 8005 7000
#define CLI_SEND_BUFSIZE 15 //发送给上位机数组长度
#define CLI_RECV_BUFSIZE 30 //jieshou上位机数组长度
// 工控器件相关
#define LENGTH_DELTA 9
#define LENGTH_DI 8
#define LENGTH_DO 8
#define LENGTH_JOYSTICK_R 9

//自动下接收的结构体
// typedef struct __autorecv__  //GPFPD
// {
//     double goal_v;
//     double goal_r;
// }auto_recv;

// 函数声明
INT32 OpenPort(INT32 comport, INT32 baud);  // 打开端口
void Set_Timer(INT32 interv, INT32 val); // 设定定时器
void Signal_Handler(); // 定时执行的函数
UINT32 Analyse_Joystick(INT8 func, UINT8 recv[]); // 操纵杆数据解析
// INT32 recvUDP_Analyse(auto_recv *net_data, INT8 *read_buff);

static struct itimerval oldtv; // 定时相关设置
INT32 j_bit = 512; // 操纵杆x,y,z分量各类值设定
UINT32 jx_bit = 512, jx_minbit = 32, jx_midbit = 512, jx_maxbit = 992, jx_minspace = 49,  jx_midspace = 50,  jx_maxspace = 49, 
            jy_bit = 512, jy_minbit = 32, jy_midbit = 512, jy_maxbit = 992, jy_minspace_side = 49, jy_minspace_rud = 120, jy_midspace_side = 50,  jy_midspace_rud = 60, jy_maxspace_side = 49, jy_maxspace_rud = 120, jy_lastbit = 512, 
            jz_bit = 512, jz_minbit = 32, jz_midbit = 512, jz_maxbit = 992, jz_minspace = 49,  jz_midspace = 50,  jz_maxspace = 49, 
            j_button = 0; // 操纵杆范围,侧推只考虑X轴

bool timer_sign = 0; // 标志位

//文件
FILE *fp; 

// 主函数
int main()
{
    INT32 fd485_1, fd485_2, fd485_3, fd485_4, fd485_5, fd485_6, fd485_7; // 文件描述符
    INT32 i = 0, n = 0;
    INT32 rud_min = -30, rud_mid = 0, rud_max = 30;
    INT32 siderev_minbit = -255, siderev_midbit = 0, siderev_maxbit = 255, siderev_send = 0, siderev_delta = 0; // 要发送的侧推数值范围
    INT32 si_dir_delay_L = 0, si_dir_delay_R = 0, si_side_delay_L = 0, si_side_delay_R = 0, t_dir_delay_L = 0, t_dir_delay_R = 0, t_side_delay_L = 0, t_side_delay_R = 0, st_dir_delay = 0; // 各种延迟设定
    INT32 siDirLDelayTime = 0, siDirRDelayTime = 0, siSideLDelayTime = 0, siSideRDelayTime = 0, tDirLDelayTime = 0, tDirRDelayTime = 0, tSideLDelayTime = 0, tSideRDelayTime = 0, stDirDelayTime = 10; // 读取的延迟时间参数
    INT16 middle_bit = 0; // 两电机是否回零的标志
    INT16 button_mode = 0; // 架控台按钮检测标志
    INT16 current_ldirrev = 0, current_rdirrev = 0; // 返回的转速
    INT8 j = 0, k = 0;
    INT8 sailing_mode = 0, manual_mode = 3, shut_sign = 0; // 航行模式:0 手动 1自动 3 即停 ;手动模式选择 1侧推2转弯3直线
    INT8 si_once_sign = 0, t_once_sign = 0, st_once_sign = 0; // 三种模式只执行一次的指令
    // INT8 rud_send = 0, rud_delta = 0; // 舵角发送
    INT8 dirrev_normal_L = 0x1C, dirrev_normal_R = 0x1C, dirrev_minbit = -127, dirrev_midbit = 0, dirrev_maxbit = 127, ldirrev_send = 0, ldirrev_delta = 0, rdirrev_send = 0, rdirrev_delta = 0, dirrev_counter = 0; // 要发送的后推数值范围
    INT8 k_side = 0, k_ldirrev = 0, k_rdirrev = 0, k_rud = 0; // 微调相关参数
    INT8 auto_goalh_num = 50, auto_rud_num = 3;
    _Bool dirrev_sign = 0; // 直线档位速度模式相关参数
    double rud_send = 0.0, rud_delta = 0.0; // 舵角发送
    double v_K = 0.707107, v_K1 = 1.0, v_T = 0.707107, v_T1 = 2.0, v_Ts = 0.1;
    double h_K1 = 2, h_K2 = 0.5, h_T1 = 3.0, h_T2 = 0.707107, h_Ts = 0.1;
    double temp = 0.0;
    double goal_v = 0.0;
    double goal_r = 0.0;
    // auto_recv auto_recv_buff;

    UINT8 recv_joystick[LENGTH_JOYSTICK_R] = {0xFF, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x06}; // 操纵杆数据接收
    UINT8 button_mode_buf[LENGTH_DI] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x0F, 0x7D, 0x46}; // 访问按钮状态
    UINT8 side_DI[LENGTH_DI] = {0x09, 0x01, 0x00, 0x02, 0x00, 0x01, 0x5D, 0x42},  
                    turn_DI[LENGTH_DI] = {0x09, 0x01, 0x00, 0x03, 0x00, 0x01, 0x0C, 0x82},  
                    stra_DI[LENGTH_DI] = {0x09, 0x01, 0x00, 0x01, 0x00, 0x01, 0xAD, 0x42}, 
                    shut_DI[LENGTH_DI] = {0x09, 0x01, 0x00, 0x0B, 0x00, 0x01, 0x8D, 0x40};  
    UINT8 side_DO_on[LENGTH_DO] = {0x08, 0x05, 0x00, 0x13, 0xFF, 0x00, 0x7D, 0x66},  // 开按钮的灯
                    turn_DO_on[LENGTH_DO] = {0x08, 0x05, 0x00, 0x14, 0xFF, 0x00, 0x3C, 0x96},  
                    stra_DO_on[LENGTH_DO] = {0x08, 0x05, 0x00, 0x15, 0xFF, 0x00, 0xCC, 0xA7},
                    side_DO_off[LENGTH_DO] = {0x08, 0x05, 0x00, 0x13, 0x00, 0x00, 0x8D, 0x57},  
                    turn_DO_off[LENGTH_DO] = {0x08, 0x05, 0x00, 0x14, 0x00, 0x00, 0x9D, 0x67},  
                    stra_DO_off[LENGTH_DO] = {0x08, 0x05, 0x00, 0x15, 0x00, 0x00, 0xDC, 0x97};
    INT8 keyboard_recv[LENGTH_DELTA] = {0}; 
    INT8 goal_jst[3] = {0x00};
    double delta_speed[3] = {0.0}, current_speed[2] = {0.0};
    double delta_heading[2] = {0.0, 0.0}, goal_rud[3] = {0.0, 0.0, 0.0}, current_heading[3] = {0.0, 0.0, 0.0};

    // 读文件参数
    // char recvfromFile[100];
    // fp = fopen("/home/zmh/nfs", "r");
    // if (fp == NULL)
    // {
    //     printf("read file error\n");
    // }

    // memset(recvfromFile, 0, sizeof(recvfromFile));

    // atoi/atof:字符串转i:int,f:float
    // fgets(recvfromFile, 100, fp); // side 
    // siderev_minbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // siderev_midbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // siderev_maxbit = atoi(recvfromFile);
    // 舵
    // fgets(recvfromFile, 100, fp);// rud
    // rud_min = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // rud_mid = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // rud_max = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // 操纵杆 x,y,z
    // jx_minbit = atoi(recvfromFile); // jx
    // fgets(recvfromFile, 100, fp);
    // jx_midbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jx_maxbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jx space
    // jx_minspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jx_midspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jx_maxspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jy
    // jy_minbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_midbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_maxbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jy space side
    // jy_minspace_side = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_midspace_side = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_maxspace_side = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jy space rud
    // jy_minspace_rud = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_midspace_rud = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jy_maxspace_rud = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jz
    // jz_minbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jz_midbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jz_maxbit = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);// jz space
    // jz_minspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jz_midspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // jz_maxspace = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // 侧推
    // siDirLDelayTime = atoi(recvfromFile); // 左推延迟时间
    // fgets(recvfromFile, 100, fp);
    // siDirRDelayTime = atoi(recvfromFile); // 右推延迟时间
    // fgets(recvfromFile, 100, fp);
    // siSideLDelayTime = atoi(recvfromFile); // 侧推左延迟时间
    // fgets(recvfromFile, 100, fp);
    // siSideRDelayTime = atoi(recvfromFile); // 侧推右延迟时间
    // fgets(recvfromFile, 100, fp);
    // 转弯
    // tDirLDelayTime = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // tDirRDelayTime = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // tSideLDelayTime = atoi(recvfromFile);
    // fgets(recvfromFile, 100, fp);
    // tSideRDelayTime = atoi(recvfromFile);
    // 直线
    // fgets(recvfromFile, 100, fp);
    // stDirDelayTime = atoi(recvfromFile);

    // 串口
    INT32 comport_A = 0, comport_B = 1, comport_C = 2, comport_D = 3, comport_E = 4, comport_F = 5, comport_G = 6; 
    INT32 baud_A = 38400, baud_B = 38400, baud_C = 38400, baud_D = 38400, baud_E = 38400, baud_F = 9600, baud_G = 38400; // baudrate
    //fd485_1 = OpenPort(comport_A, baud_A); // 后推1 
    //fd485_2 = OpenPort(comport_B, baud_B); // 后推2 
    // fd485_3 = OpenPort(comport_C, baud_C); // GPS
    fd485_4 = OpenPort(comport_D, baud_D); // 左右舵
    fd485_5 = OpenPort(comport_E, baud_E); // 架空台
    fd485_6 = OpenPort(comport_F, baud_F); // 操纵杆 
    // fd485_7 = OpenPort(comport_G, baud_G); // 

    // Open_CANPort(); // 侧推初始化

    //服务器
    socklen_t len_client, len_server,len_client1,len_client2, len_client3;
    struct sockaddr_in server_sockaddr;
    //recfrom存连接服务器的用户地址
    struct sockaddr_in client_sockaddr;
    struct sockaddr_in client_sockaddr1;
    struct sockaddr_in client_sockaddr2;
    struct sockaddr_in client_sockaddr3;
    memset(&server_sockaddr,0,sizeof(server_sockaddr));
    memset(&client_sockaddr,0,sizeof(client_sockaddr));

    memset(&client_sockaddr1,0,sizeof(client_sockaddr1));
    memset(&client_sockaddr2,0,sizeof(client_sockaddr2));
    memset(&client_sockaddr3,0,sizeof(client_sockaddr3));

    int server_sockfd; //server_sockfd:服务器句柄; len:接收返回值; num:发送返回值;
   
    char sendtoClient[CLI_SEND_BUFSIZE];
    char receivefromClient[CLI_RECV_BUFSIZE];

     /*网络通信 -------- UDP  初始化*/
    // 用于设置超时结束UDP通信中recv函数的结构体
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;

    //服务器
    ///定义sockfd
    server_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_sockfd < 0)
      {
        printf("服务端Socket创建失败\n");
        return -1;
      }
    printf("服务端Socket创建成功\n");

    setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout); // 设置该socket接收时，超时则结束阻塞

    len_client = sizeof(client_sockaddr);
    len_server = sizeof(server_sockaddr);
    len_client1 = sizeof(client_sockaddr1);
    len_client2 = sizeof(client_sockaddr2);
    len_client3 = sizeof(client_sockaddr3);
     //IPV4 ,字节流,如果套接字类型不是原始套接字，那么这个参数就为0
    ///定义sockaddr_in 处理网络通信地址
    server_sockaddr.sin_family = AF_INET;                //IPV4，地址族
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY); //本机IP地址
    server_sockaddr.sin_port = htons(MYPORT);            //端口，16位

    //定义2个客户端
    client_sockaddr1.sin_family=AF_INET;
    client_sockaddr1.sin_addr.s_addr=inet_addr("192.168.1.12"); // 136 137 192.168.1.31
    client_sockaddr1.sin_port=htons(8080); // 1005 8888

    client_sockaddr2.sin_family=AF_INET;
    client_sockaddr2.sin_addr.s_addr=inet_addr("127.0.0.1");
    client_sockaddr2.sin_port=htons(10000);

    client_sockaddr3.sin_family=AF_INET;
    client_sockaddr3.sin_addr.s_addr=inet_addr("127.0.0.1");
    client_sockaddr3.sin_port=htons(10020);

    //异常终止
    int mw_optval=1;
    setsockopt(server_sockfd,SOL_SOCKET,SO_REUSEADDR,(char*)&mw_optval,sizeof(mw_optval));
  
    ///bind，成功返回0，出错返回-1
    if (bind(server_sockfd, (struct sockaddr *)&server_sockaddr, sizeof(server_sockaddr)) == -1)    
    {
        perror("bind error");
        exit(1);
    }

    // 驱动器&电机 初始化
    UINT8 initial_motor_send1[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x01, 0x00, 0x00};
    Initial_Motor(fd485_4, 1,initial_motor_send1);
     UINT8 initial_motor_send2[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x03, 0x00, 0x00};
    Initial_Motor(fd485_4, 1,initial_motor_send2);
    UINT8 initial_motor_send3[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x0F, 0x00, 0x00};
    Initial_Motor(fd485_4, 1,initial_motor_send3);
    Set_Motor_Mode(fd485_4, 1, 1); // 设置电机模式，一般用位置和回零模式，视情况是否用速度模式
    Set_Rudder_Argument(fd485_4, 1, 10, 60, 60);

    // UINT8 initial_motor_send[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x0F, 0x00, 0x00};
    // Initial_Motor(fd485_5, 2,initial_motor_send);
    UINT8 initial_motor_send4[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x01, 0x00, 0x00};
    Initial_Motor(fd485_5, 2,initial_motor_send4);
     UINT8 initial_motor_send5[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x03, 0x00, 0x00};
    Initial_Motor(fd485_5, 2,initial_motor_send5);
    UINT8 initial_motor_send6[8] = {0x00, 0x06, 0x60, 0x40, 0x00, 0x0F, 0x00, 0x00};
    Initial_Motor(fd485_5, 2,initial_motor_send6);
    Set_Motor_Mode(fd485_5, 2, 1);
     // 设置速度、加减速度，可调
    Set_Rudder_Argument(fd485_5, 2, 10, 60, 60);

    // while(middle_bit != 0x101)
    // {
    //     middle_bit = Return(fd485_5); // 回零
    // }
    
    // dirrev_normal_R = Send_DirRev(fd485_1, ldirrev_send); // 后推1执行
    // dirrev_normal_L = Send_DirRev(fd485_2, rdirrev_send); // 后推2执行

    // 日志写入文档
    // fp = fopen("msg.txt", "a+"); 
    // stdout = fp; 

    // set up timer and start
	signal(SIGALRM, Signal_Handler); // 定时执行函数
	Set_Timer(100, 1); // 定时时间

    printf("enter\n");

    //主体循环函数
    while (1)
    {
        if(timer_sign == 1) // 定时标志,每次循环等待信号
        {
            // 键盘读航行模式与微调值，到试验现场检验
            n = recvfrom(server_sockfd, keyboard_recv, sizeof(keyboard_recv), 0, (struct sockaddr *)&server_sockaddr, &len_server);  // 接收上位机指令 keyboard_recv
            // n = ReadComPort(fd485_7, keyboard_recv, LENGTH_DELTA); // receivefromClient
            if(keyboard_recv[0] == 0x7F) // analyse net && keyboard_recv[LENGTH_DELTA - 1] == 0x7E
            {
                printf("keyboard: ");
                for(i = 0;i < LENGTH_DELTA;i++)
                {
                    printf("%d ", keyboard_recv[i]);
                }
                printf("\n");
                if (n < 0)
                {
                    perror("^^^^^^^^^^^^^^^^^^ read 'read keyboard' error ^^^^^^^^^^^^^^^^^^\n"); 
                }
                button_mode = keyboard_recv[1];
                // if(button_mode & 0x00 == 0x00) 
                // {
                    // sailing_mode = 0;
                    if(button_mode == 1) sailing_mode = 1; // manual
                // }
                if(button_mode == 2) sailing_mode = 2; // auto 
                if(button_mode == 3) sailing_mode = 3; // shut 
                k_ldirrev = keyboard_recv[2];
                k_rdirrev = keyboard_recv[3];
                k_side = keyboard_recv[4]; // 
                k_rud = keyboard_recv[6];

                memset(keyboard_recv, 0, sizeof(keyboard_recv));
            }

            // 架空台按钮模式，暂时保留
            // shut_sign = Send_DI(fd485_5, shut_DI);
            // if((shut_sign & 0x01) == 1) sailing_mode = 3;

            // 推进器接收正常指令
            // if(dirrev_normal_R == 0x1C)
            // {
            //     current_ldirrev = Send_Normal(fd485_1); // 推进器1正常接收
            // }
            // if(dirrev_normal_L == 0x1C)
            // {
            //     current_rdirrev = Send_Normal(fd485_2); // 推进器2正常接收
            // }

            // if(sailing_mode == 0) 
            // {
            //     // 架空台按钮模式，暂时保留 
            //     button_mode = Send_DI(fd485_5, button_mode_buf); 
            //     if((button_mode & 0x01) == 1) sailing_mode = 2; 
            //     if((button_mode & 0x02) == 2) sailing_mode = 1; // straight 
            //     if((button_mode & 0x80) == 0x80) sailing_mode = 3; // shut 
            //     printf("button_mode = %d, sailing_mode = %d\n", button_mode, sailing_mode); 
            // } 

            if(sailing_mode == 1) // manual
            {
                // fp = fopen("msg.txt", "a+"); 
                // stdout = fp; 
                    if(st_once_sign == 0)
                    {
                        // 架空灯
                        // Send_DO(fd485_5, side_DO_off);
                        // Send_DO(fd485_5, turn_DO_off);
                        // Send_DO(fd485_5, stra_DO_on);

                        // 微调量重新清零
                        ldirrev_delta = 0;
                        rdirrev_delta = 0;

                        st_dir_delay = 0;

                        si_once_sign = 0;
                        t_once_sign = 0;
                        st_once_sign = 1;
                    }

                    // 读取操纵杆数据
                    Joystick_Recv(fd485_6, recv_joystick); // 接收操纵杆数据
                    j_bit = Analyse_Joystick(manual_mode, recv_joystick); // 解析操纵杆指令
                    jx_bit = j_bit & 0xFFFF; // 转速信息
                    jy_bit = (j_bit & 0xFFFF0000) >> 16; // 舵角信息
                    printf("jx_bit = %d, jy_bit = %d, j_button = %d\n", jx_bit, jy_bit, j_button);

                    // rud 设定在操纵杆什么范围内打舵多少范围，操纵杆最值与中间值之间必须有提前饱和或死区，不能全区间线性对应
                    if(jy_bit >= jy_maxbit - jy_maxspace_rud)
                    {
                        jy_bit = jy_maxbit - jy_maxspace_rud - jy_midspace_rud;
                        rud_send = rud_min;
                    }
                    else if(jy_bit <= jy_minbit + jy_minspace_rud)
                    {
                        jy_bit = jy_minbit + jy_minspace_rud + jy_midspace_rud; 
                        rud_send = rud_max;
                    }
                    else if(jy_bit >= jy_midbit + jy_midspace_rud ) //&& jy_bit < jy_maxbit - jy_maxspace_rud)
                    {
                        jy_bit = jy_bit + jy_midspace_rud;
                        rud_send =( (jy_midbit - jy_bit) *(rud_max - rud_mid) )/(jy_maxbit - jy_maxspace_rud - jy_midspace_rud - jy_midbit) + rud_mid;
                    }
                    else if(jy_bit < jy_midbit - jy_midspace_rud)  //&& y_bit >= jy_minbit + jy_minspace_rud &&
                    {
                        jy_bit = jy_bit - jy_midspace_rud;
                        rud_send =((jy_midbit - jy_bit)*(rud_min - rud_mid))/(jy_minbit + jy_minspace_rud + jy_midspace_rud - jy_midbit) + rud_mid;
                    }
                    // if(jy_bit > jy_midbit - jy_midspace_rud && jy_bit < (jy_midbit + jy_midspace_rud) 
                    else
                    {
                        jy_bit = jy_midbit;
                        rud_send = rud_mid;
                    }
                    printf("REAL jy_bit = %d, j_button = %d\n", jy_bit, j_button);
                    // if(st_dir_delay >= stDirDelayTime) // 这里设定延迟时间主要考虑在其他状态到直线后应等各推进器速度降为0时再旋转后推
                    // {
                        Send_Rudder(fd485_4, 1, rud_send); // rud left
                        Send_Rudder(fd485_5, 2, rud_send); // rud right
                    //     st_dir_delay = stDirDelayTime;
                    // }
                    // st_dir_delay++;

                    // thruster 两种功能，微调功能最好在实验室试好
                    // 此注释处为速度随操纵杆变化与速度随键盘微调之功能，因实际调试很可能用到，故不删
                    // if(jx_bit >= jx_maxbit - jx_maxspace)
                    // {
                    //     ldirrev_send = dirrev_maxbit;
                    //     rdirrev_send = dirrev_maxbit;
                    //     ldirrev_send = -ldirrev_send;
                    //     rdirrev_send = -rdirrev_send;

                        // if(k_ldirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_ldirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_rdirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_rdirrev = 0;
                        // }
                        // if(k_rdirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_rdirrev = 0;
                        // }

                    //     printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                    //         dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send + ldirrev_delta); // 后推1执行
                    //         dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send + rdirrev_delta);  // 后推2执行
                    // }
                    // else if((jx_bit >= jx_midbit + jx_midspace) && (jx_bit < jx_maxbit - jx_maxspace))
                    // {
                    //     // ldirrev_send = (jx_bit * (dirrev_maxbit - dirrev_minbit) - dirrev_maxbit * jx_minbit + dirrev_minbit * jx_maxbit)/(jx_maxbit - jx_minbit);
                    //     ldirrev_send = (dirrev_maxbit - dirrev_midbit) * (jx_bit - (jx_midbit + jx_midspace))/((jx_maxbit - jx_maxspace) - (jx_midbit + jx_midspace)) + dirrev_midbit;
                    //     rdirrev_send = (dirrev_maxbit - dirrev_midbit) * (jx_bit - (jx_midbit + jx_midspace))/((jx_maxbit - jx_maxspace) - (jx_midbit + jx_midspace)) + dirrev_midbit;
                    //     ldirrev_send = -ldirrev_send;
                    //     rdirrev_send = -rdirrev_send;
                    //     // printf("ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send, rdirrev_send);

                        // if(k_ldirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_ldirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_rdirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_rdirrev = 0;
                        // }
                        // if(k_rdirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_rdirrev = 0;
                        // }

                    //     // printf("rud_send = %d\n", rud_send);
                    //     printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                    //         dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send + ldirrev_delta); // 后推1执行
                    //         dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send + rdirrev_delta);  // 后推2执行
                    // }
                    // else if(jx_bit <= jx_midbit - jx_midspace && jx_bit > jx_minbit + jx_minspace)
                    // {
                    //     ldirrev_send = (dirrev_midbit - dirrev_minbit) * (jx_bit - (jx_minbit + jx_minspace))/((jx_midbit - jx_midspace) - (jx_minbit + jx_minspace)) + dirrev_minbit;
                    //     rdirrev_send = (dirrev_midbit - dirrev_minbit) * (jx_bit - (jx_minbit + jx_minspace))/((jx_midbit - jx_midspace) - (jx_minbit + jx_minspace)) + dirrev_minbit;
                    //     ldirrev_send = -ldirrev_send;
                    //     rdirrev_send = -rdirrev_send;
                    //     // printf("ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send, rdirrev_send);

                        // if(k_ldirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_ldirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_rdirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_rdirrev = 0;
                        // }
                        // if(k_rdirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_rdirrev = 0;
                        // }

                    //     // printf("rud_send = %d\n", rud_send);
                    //     printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                    //         dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send + ldirrev_delta); // 后推1执行
                    //         dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send + rdirrev_delta);  // 后推2执行
                    // }
                    // else if(jx_bit <=  jx_minbit + jx_minspace)
                    // {
                    //     ldirrev_send = dirrev_minbit;
                    //     rdirrev_send = dirrev_minbit;
                    //     ldirrev_send = -ldirrev_send;
                    //     rdirrev_send = -rdirrev_send;

                        // if(k_ldirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_ldirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_ldirrev = 0;
                        // }
                        // if(k_rdirrev == 1)
                        // {
                        //     if(ldirrev_delta < 127)
                        //         ldirrev_delta += 1;
                        //     k_rdirrev = 0;
                        // }
                        // if(k_rdirrev == 2)
                        // {
                        //     if(ldirrev_delta > -127)
                        //         ldirrev_delta -= 1;
                        //     k_rdirrev = 0;
                        // }

                    //     // printf("rud_send = %d\n", rud_send);
                    //     printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                    //         dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send + ldirrev_delta); // 后推1执行
                    //         dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send + rdirrev_delta);  // 后推2执行
                    // }
                    // else
                    // {
                    //     ldirrev_send = 0;
                    //     rdirrev_send = 0;
                    //     // ldirrev_send = -ldirrev_send;
                    //     // rdirrev_send = -rdirrev_send;

                    // dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send); // 后推1执行
                    // dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send);  // 后推2执行

                    //     // printf("rud_send = %d\n", rud_send);
                    //     printf("ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send, rdirrev_send);
                
                    // }
/* 后推取消
                    // 此处为档位功能，目前前后各1挡127，应加入微调之功能
                    if(jx_bit > (jx_midbit + jx_midspace))
                    {
                        if(dirrev_sign == 0)
                        {
                            dirrev_sign = 1;
                            dirrev_counter--;
                            if(dirrev_counter <= -1) dirrev_counter = -1;
                        }
                    }
                    else if(jx_bit < (jx_midbit - jx_midspace))
                    {
                        if(dirrev_sign == 0)
                        {
                            dirrev_sign = 1;
                            dirrev_counter++;
                            if(dirrev_counter >= 2) dirrev_counter = 2;
                        }
                    }
                    else dirrev_sign = 0;
                    switch(dirrev_counter)
                    {
                        case 2:
                            ldirrev_send = dirrev_maxbit;
                            rdirrev_send = dirrev_maxbit;
                            break;
                        case 1:
                            ldirrev_send = 77;
                            rdirrev_send = 77;
                            break;
                        case 0:
                            ldirrev_send = dirrev_midbit;
                            rdirrev_send = dirrev_midbit;
                            break;
                        case -1:
                            ldirrev_send = dirrev_minbit;
                            rdirrev_send = dirrev_minbit;
                            break;
                    }
                    if(ldirrev_send == dirrev_midbit && rdirrev_send == dirrev_midbit)
                    {
                        printf("ldirrev_delta = %d, rdirrev_delta = %d, ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_delta, rdirrev_delta, ldirrev_send, rdirrev_send);
                        printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send, rdirrev_send);
                        // printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                        // dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send); // 后推1执行
                        // dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send);  // 后推2执行
                        // ldirrev_delta = 0;
                        // rdirrev_delta = 0;
                    }
                    else
                    {
                        // 微调
                        if(k_ldirrev == 1)
                        {
                            if(ldirrev_delta < 127)
                                ldirrev_delta += 1;
                            k_ldirrev = 0;
                        }
                        if(k_ldirrev == 2)
                        {
                            if(ldirrev_delta > -127)
                                ldirrev_delta -= 1;
                            k_ldirrev = 0;
                        }
                        if(k_rdirrev == 1)
                        {
                            if(ldirrev_delta < 127)
                                ldirrev_delta += 1;
                            k_rdirrev = 0;
                        }
                        if(k_rdirrev == 2)
                        {
                            if(ldirrev_delta > -127)
                                ldirrev_delta -= 1;
                            k_rdirrev = 0;
                        }

                        printf("ldirrev_delta = %d, rdirrev_delta = %d, ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_delta, rdirrev_delta, ldirrev_send, rdirrev_send);
                        printf("sum: ldirrev_send = %d, rdirrev_send = %d\n", ldirrev_send + ldirrev_delta, rdirrev_send + rdirrev_delta);
                        // dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send + ldirrev_delta); // 后推1执行
                        // dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send + rdirrev_delta);  // 后推2执行   
                    }
*/
            }

            if(sailing_mode == 2) // auto
            {
                // fp = fopen("msg.txt", "a+"); 
                // stdout = fp; 

                // 网络接收
                n = recvfrom(server_sockfd, receivefromClient, sizeof(receivefromClient), 0, (struct sockaddr *)&server_sockaddr, &len_server);  // 接收上位机指令
                printf("n = %d, recv_buf = %s\n", n, receivefromClient);
                //协议 ASCII字符转化为二进制数据
                // n = recvUDP_Analyse(&auto_recv_buff, receivefromClient);
                if(n > 0)
                {
                    goal_r = atof(receivefromClient); // 
                }
                // printf("goal_rud = %.2f, goal_rud = %.2f\n", auto_recv_buff.goal_v, auto_recv_buff.goal_r);
                printf("goal_rud = %.2f, goal_rud = %.2f\n", goal_v, goal_r);

                if(auto_rud_num == 3) // 0.3s per rud
                {
                    rud_send = goal_r;
                    Send_Rudder(fd485_4, 1, rud_send); // rud left
                    Send_Rudder(fd485_5, 2, rud_send); // rud right
                    auto_rud_num = 0;
                }

                /************speed control************/
                // delta_speed[1] = delta_speed[0];
                // delta_speed[0] = goal_v - current_speed[0];
                // goal_jst[2] = goal_jst[1];
                // goal_jst[1] = goal_jst[0];
                // goal_jst[0] = Speed_Control(delta_speed, goal_jst, v_K2, v_T1, v_T2, v_Ts);
                // current_speed[1] = current_speed[0];
                // current_speed[0] = Speed_Model(goal_jst[0], current_speed, v_K1, v_T1, v_Ts);
                // printf("################delta_speed = %f %f\n", delta_speed[0], delta_speed[1]);
                // printf("################goal_jst = %d %d %d\n", goal_jst[0], goal_jst[1], goal_jst[2]);
                // printf("################current_speed = %f %f\n", current_speed[0], current_speed[1]);
                // printf("%f\n", current_speed[0]);

                ldirrev_send = 127; // 这里可调
                rdirrev_send = 127;
                // dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send); // 后推1执行
                // dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send);  // 后推2执行

                // auto_goalh_num++;
                auto_rud_num++;

                memset(receivefromClient, 0, sizeof(receivefromClient));

                // fflush(stdout);
                // fclose(fp); 
            }

            if(sailing_mode == 3) // 急停功能，至少所有推进器速度降为0
            {
                siderev_send = 0;
                ldirrev_send = 0;
                rdirrev_send = 0;
                ldirrev_delta = 0;
                rdirrev_delta = 0;
                siderev_delta = 0;
                // Send_SideDir(siderev_send); // 侧推执行 
                // dirrev_normal_L = Send_DirRev(fd485_1, ldirrev_send); // 后推1执行
                // dirrev_normal_R = Send_DirRev(fd485_2, rdirrev_send);  // 后推2执行     
            }

            // 信息发送给1.接收GPS的程序2.
            sendtoClient[0] = 0x7F;
            sendtoClient[1] = sailing_mode;
            sendtoClient[2] = (int)(rud_send * 10);
            sendtoClient[3] = ldirrev_send + ldirrev_delta;
            sendtoClient[4] = rdirrev_send + rdirrev_delta;
            // sendtoClient[5] = ((siderev_send + siderev_delta) & 0xFF00) >> 8;
            // sendtoClient[6] = (siderev_send + siderev_delta) & 0xFF;
            if(recv_joystick[0] == 0xFF)
            {
                sendtoClient[7] = recv_joystick[3]; // x
                sendtoClient[8] = recv_joystick[4];
                sendtoClient[9] = recv_joystick[1]; // y
                sendtoClient[10] = recv_joystick[2];
                sendtoClient[11] = recv_joystick[5]; // z
                sendtoClient[12] = recv_joystick[6];
                sendtoClient[13] = recv_joystick[7]; // button
            }
            if(recv_joystick[1] == 0xFF)
            {
                sendtoClient[7] = recv_joystick[4]; // x
                sendtoClient[8] = recv_joystick[5];
                sendtoClient[9] = recv_joystick[2]; // y
                sendtoClient[10] = recv_joystick[3];
                sendtoClient[11] = recv_joystick[6]; // z
                sendtoClient[12] = recv_joystick[7];
                sendtoClient[13] = recv_joystick[8]; // button
            }
            sendtoClient[14] = 0x7E;

            // n = sendto(server_sockfd, sendtoClient, sizeof(sendtoClient), 0, (struct sockaddr *)& client_sockaddr1, len_client1); // cli1
            n = sendto(server_sockfd, sendtoClient, sizeof(sendtoClient), 0, (struct sockaddr*)&client_sockaddr2, len_client2); // cli2

            // printf("send length = %d, ",n);
            // printf("sendtoClient = \n");
            // for(i = 0;i < CLI_SEND_BUFSIZE;i++) // 
            // {
            // printf("%d ", sendtoClient[i]);
            // }         
            // printf("\n");

            memset(sendtoClient, 0, sizeof(sendtoClient));

            timer_sign = 0;
        }
        // fflush(stdout);
        // fclose(fp); 
    } // while

    // close(fd485_1);
    // close(fd485_2);
    // close(fd485_3);
    close(fd485_4);
    close(fd485_5);
    close(fd485_6);
    // close(fd485_7);

    close(server_sockfd);

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

            // 架空台按钮模式，暂时保留
            // button_mode = Send_DI(fd485_5, button_mode_buf);
            // if((button_mode & 0x01) == 1) 
            // {
            //     if(j == 0)
            //     {
            //         k++;
            //         j = 1;
            //     }
            //     if(k % 2 == 0) sailing_mode = 0; // manual
            //     else sailing_mode = 1; // auto
            // }
            // else j = 0;
            // if((button_mode & 0x02) == 2) manual_mode = 3; // straight
            // if((button_mode & 0x04) == 4) manual_mode = 1; // side
            // if((button_mode & 0x08) == 8) manual_mode = 2; // turn
            // // if(recv_shutDI == 1) manual_mode = 5;
            // printf("button_mode = %d, sailing_mode = %d, manual_mode = %d\n", button_mode, sailing_mode, manual_mode);

}

UINT32 Analyse_Joystick(INT8 func, UINT8 recv[])
{
    if(recv[1] == 0xFF)
    {
        if(func == 1)
        {
            j_bit = (recv[2] << 8) + recv[3]; // 解析 Y
        }
        if(func == 2)
        {
            j_bit = (recv[6] << 8) + recv[7]; // 解析Z
        }
        if(func == 3)
        {
            jx_bit = (recv[4] << 8) + recv[5]; // 解析 X
            jy_bit = (recv[2] << 8) + recv[3];
            j_bit = (jy_bit << 16) + (jx_bit);
            j_button = recv[8];
        }
    }

    if(recv[0] == 0xFF)
    {
        if(func == 1)
        {
            j_bit = (recv[1] << 8) + recv[2]; // 解析 Y
        }
        if(func == 2)
        {
            j_bit = (recv[5] << 8) + recv[6]; // 解析Z
        }
        if(func == 3)
        {
            jx_bit = (recv[3] << 8) + recv[4]; // 解析 X
            jy_bit = (recv[1] << 8) + recv[2];
            j_bit = (jy_bit << 16) + (jx_bit);
            j_button = recv[7];
        }
    }

    return j_bit;
}

// INT32 recvUDP_Analyse(auto_recv *net_data, INT8 *read_buff)
// {

//     // INT8 *ptr = NULL;

//     memset(net_data,0,sizeof(net_data));
       
//     // if(NULL==(ptr=strstr(read_buff,"$BDFPD")))
//     // {      
//     //     return -1;
//     // }
//     //字符数组名本身就是地址，无需添加&(net_data.Status)
//      sscanf(read_buff,"%lf,%lf,", &((*net_data).goal_v), &((*net_data).goal_r));

//     // printf("gps_week=%lf gps_status=%s latitude=%lf\n",(*net_data).gps_week,(*net_data).Status,(*net_data).latitude);
  
//     return 1;
// }
