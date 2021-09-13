#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> //基本系统数据类型  dev_t设备号，size_t内存中对象大小，time_t以秒为单位计时
#include <termios.h>   //串口编程头文件，初始化，给tty一个特定端口
#include <unistd.h>    //定义更多函数原型，close... 对POSIX操作系统API访问功能
#include <errno.h>     //错误代码提示，error值不同意义不同
#include <fcntl.h>     //根据文件描述词来操作文件的特性，open,fclose...
#include <sys/ioctl.h>
#include <math.h>

//定时器
#include <signal.h>
#include <sys/time.h> //linux系统时间日期头文件
#include <time.h>     //具有一写处理日期和时间的类型的函数
//TCP/IP 服务器
#include <arpa/inet.h>  //客户端信息转换为字符串信息
#include <netinet/in.h> //定义IP,端口等
#include <sys/shm.h>
#include <sys/socket.h>
//
#include "serial.h"

#include <limits.h>   //数据类型限制
#include <stdbool.h>  //bool,true,false...
#include <sys/stat.h> //通过文件名filename获取文件信息

//网络
#define MYPORT 8000 // 8005 7000
#define CLI_SEND_BUFSIZE 9 //发送给上位机数组长度
#define CLI_RECV_BUFSIZE 3 //发送给上位机数组长度

INT32 OpenPort(INT32 comport, INT32 baud);

char getch()
{
    char c;
    system("stty -echo");
    system("stty -icanon");
    c=getchar();
    system("stty icanon");
    system("stty echo");
    return c;
}
char getche()
{
    char c;
    system("stty -icanon");
    c=getchar();
    system("stty icanon");
    return c; 
}

// 主函数
int main(int argc, char **argv)
{
    int fd_485;
    int i = 0, n = 0;
    int ch;     // 输入的字符
    int col = 0; // 换行控制

    char send_buf[CLI_SEND_BUFSIZE] = {0};

    //服务器
    socklen_t len_client, len_server, len_client1, len_client2;
    struct sockaddr_in server_sockaddr;
    //recfrom存连接服务器的用户地址
    struct sockaddr_in client_sockaddr;
    struct sockaddr_in client_sockaddr1;
    struct sockaddr_in client_sockaddr2;
    memset(&server_sockaddr, 0, sizeof(server_sockaddr));
    memset(&client_sockaddr, 0, sizeof(client_sockaddr));

    memset(&client_sockaddr1, 0, sizeof(client_sockaddr1));
    memset(&client_sockaddr2, 0, sizeof(client_sockaddr2));
    int server_sockfd; //server_sockfd:服务器句柄; len:接收返回值; num:发送返回值;
   
    unsigned char sendtoClient[CLI_SEND_BUFSIZE] = {0x00};
    unsigned char receivefromClient[CLI_RECV_BUFSIZE] = {0x00};

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
     //IPV4 ,字节流,如果套接字类型不是原始套接字，那么这个参数就为0
    ///定义sockaddr_in 处理网络通信地址
    server_sockaddr.sin_family = AF_INET;                //IPV4，地址族
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY); //本机IP地址
    server_sockaddr.sin_port = htons(MYPORT);            //端口，16位

    //定义2个客户端
    client_sockaddr1.sin_family=AF_INET;
    client_sockaddr1.sin_addr.s_addr=inet_addr("127.0.0.1"); // 136 137
    client_sockaddr1.sin_port=htons(10020); // 1005 8888

    client_sockaddr2.sin_family=AF_INET;
    client_sockaddr2.sin_addr.s_addr=inet_addr("127.0.0.1");
    client_sockaddr2.sin_port=htons(10010);

    //异常终止
    int mw_optval = 1;
    setsockopt(server_sockfd,SOL_SOCKET,SO_REUSEADDR,(char*)&mw_optval,sizeof(mw_optval));
  
    ///bind，成功返回0，出错返回-1
    if (bind(server_sockfd, (struct sockaddr *)&server_sockaddr, sizeof(server_sockaddr)) == -1)    
    {
        perror("bind error");
        exit(1);
    }

    // 串口
    INT32 comport_A = 0, comport_B = 1, comport_C = 2, comport_D = 3, comport_E = 4, comport_F = 5, comport_G = 6; 
    INT32 baud_A = 38400, baud_B = 38400, baud_C = 38400, baud_D = 38400, baud_E = 38400, baud_F = 9600, baud_G = 38400; // baudrate
    // fd_485 = OpenPort(comport_G, baud_G); // 操纵杆 

    int func = 0, siderev_L = 0, siderev_H = 0, ldirrev = 0, rdirrev = 0, shut_sign = 0;
    // int lrud = 0, rrud = 0, lrud_p = 0, rrud_p = 0;
    int rud = 0, rud_p = 0;
    bool send_sign = 0;

    // unsigned char siderev = 0;
    // send_buf[0] = 0x7F;
    // send_buf[CLI_SEND_BUFSIZE - 1] = 0x7E;

  printf("enter\n");

    while (1) 
    {
        ch = getch(); // 这个就是非缓冲输入用的输入函数，不同的平台可能不一样
                          // getche()也是非缓冲输入，但这个函数会实时回显，getch()不回显
        switch (ch)
        {
            case '1':
            case '!':
                send_buf[1] = 1;
                send_sign = 1;
                break;
            case '2':
            case '@':
                send_buf[1] = 2;
                send_sign = 1;
                break;
            case 'p':
            case 'P':
                send_buf[1] = 3;
                send_sign = 1;
                break;
            // case 'A':
            // case 'a':
            //     // siderev_L += 1;
            //     // if(siderev_L > 127)
            //     // {
            //     //     siderev_L = 0;
            //     //     siderev_H += 1;
            //     // }
            //     siderev_L = 1;
            //     send_sign = 1;
            //     break;
            // case 'Z':
            // case 'z':
            //     // siderev_L -= 1;
            //     // if(siderev_L < -127)
            //     // {
            //     //     siderev_L = 0;
            //     //     siderev_H -= 1;
            //     // }
            //     siderev_L = 2;
            //     send_sign = 1;
            //     break;
            case 'S':
            case 's':
                // ldirrev += 1;
                ldirrev = 1;
                send_sign = 1;
                break;
            case 'X':
            case 'x':
                // ldirrev -= 1;
                ldirrev = 2;
                send_sign = 1;
                break;
            case 'D':
            case 'd':
                // rdirrev += 1;
                rdirrev = 1;
                send_sign = 1;
                break;
            case 'C':
            case 'c':
                // rdirrev -= 1;
                rdirrev = 2;
                send_sign = 1;
                break;
            // case 'R':
            // case 'r':
            //     lrud -= 1;
                    // send_sign = 1;
            //     break;
            // case 'T':
            // case 't':
            //     lrud += 1;
                    // send_sign = 1;
            //     break;
            // case 'Y':
            // case 'y':
            //     lrud_p -= 1;
                    // send_sign = 1;
            //     break;
            // case 'U':
            // case 'u':
            //     lrud_p += 1;
                    // send_sign = 1;
            //     break;
            // case 'F':
            // case 'f':
            //     rrud -= 1;
                    // send_sign = 1;
            //     break;
            // case 'G':
            // case 'g':
            //     rrud += 1;
                    // send_sign = 1;
            //     break;
            // case 'H':
            // case 'h':
            //     rrud_p -= 1;
                    // send_sign = 1;
            //     break;
            // case 'J':
            // case 'j':
            //     rrud_p += 1;
                    // send_sign = 1;
            //     break;
            case 'F':
            case 'f':
                rud += -1;
                send_sign = 1;
                break;
            case 'G':
            case 'g':
                rud += 1;
                send_sign = 1;
                break;
            // case 'V':
            // case 'v':
            //     rud_p = -1;
            //     send_sign = 1;
            //     break;
            // case 'B':
            // case 'b':
            //     rud_p = 1;
            //     send_sign = 1;
            //     break;
            default:
                send_sign = 0;
                break;
        }
        
       if(send_sign == 0)
       {
            send_buf[0] = 0;
            send_buf[CLI_SEND_BUFSIZE - 1] = 0;
       }
       else
       {
            send_buf[0] = 0x7F;
            send_buf[CLI_SEND_BUFSIZE - 1] = 0x7E;
       }

        // scanf("%d %d %d %d %d", &func, &ldirrev, &rdirrev, &siderev_L, &rud);
        if((send_buf[0] == 0x7F) && (send_buf[CLI_SEND_BUFSIZE - 1] == 0x7E))
        {
            // send_buf[1] = func;
            send_buf[2] = ldirrev;
            send_buf[3] = rdirrev;
            // send_buf[4] = siderev_L;
            //   send_buf[5] = siderev_H;
            send_buf[6] = rud;
            send_buf[7] = rud_p;

            //   n = sendto(server_sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)& client_sockaddr1, len_client1);
            n = sendto(server_sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr*)&client_sockaddr2, len_client2);

            // n = WriteComPort(send_buf, CLI_SEND_BUFSIZE); 
            printf("send length = %d, ", n);
            // 打印输入数组
            printf("send_buf = \n");
            for(i = 0;i < CLI_SEND_BUFSIZE;i++) // 
                printf("%d ", send_buf[i]);   
            printf("\n");
            if (n < 0)
            {
                perror("write sendbuf error\n"); 
            }
        }

        // recv_len = recvfrom(server_sockfd, receivefromClient, sizeof(receivefromClient), 0, (struct sockaddr *)&server_sockaddr, &len_server);  // 接收上位机指令

        // send_buf[1] = 0;
        send_buf[2] = 0;
        send_buf[3] = 0;
        send_buf[4] = 0;
        //   send_buf[5] = siderev_H;
        send_buf[6] = 0;
        send_buf[7] = 0;
        ldirrev = 0;
        rdirrev = 0;
        siderev_L = 0;
        rud = 0;
        shut_sign = 0;

        ch = 'm';

        memset(send_buf, 0, sizeof(send_buf));
        // memset(receivefromClient, 0, sizeof(receivefromClient));
    }

    //关闭UDP
    close(fd_485);

    // close(server_sockfd);

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
