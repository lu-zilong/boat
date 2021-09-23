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

// GPS
#define GPS_COM1RECV_SIZE 200  // 接收上位机的数组长度
#define GPS_COM2RECV_SIZE 100  // 接收上位机的数组长度
// 网络
#define MYPORT 10000
#define CLIENT_PORT 8080 // 8005 8888
char *CLIENT_IP = "192.168.1.12"; // 上位机IP 17 137 192.168.142.132
#define CLI_SEND_BUFSIZE 200       // 发送给上位机的数组长度
#define CLI_RECV_BUFSIZE 15       // 接收上位机的数组长度
#define TEMPBUFSIZE 20

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

//标准GI定位定姿消息集 
typedef struct __bdfpd__
{
    UINT32 gps_week;//GPS周
    double gps_sec; //GPS秒
    double heading; //偏航
    double pitch;
    double roll;
    double latitude;//纬度
    double longitude;//经度
    double altitude;//高度
    double ve;  //东向速度
    double vn;  //北向速度
    double vu;  //天向速度
    UINT16 NSV1; //天线 1 卫星数
    UINT16 NSV2; //天线 2 卫星数
    UINT16 PT1; //定位类型 bestpos 中 pos type
    UINT16 PT2; //定位类型 heading 中 pos type
    UINT8 checksum; // 校验和
    INT8 tail[2]; // 包尾
}bdfpd;

//遥测数据帧 字节对齐 UDP发送
typedef struct sendbuf  //GPFPD
{
    INT8 head; //$ bdfpd
    UINT32 gps_week;//GPS周
    double gps_sec; //GPS秒
    double heading; //偏航
    double pitch;
    double roll;
    double y_latitude; //纬度_y单位
    double x_longitude;//经度_x单位
    double altitude;//高度
    double ve;  //东向速度
    double vn;  //北向速度
    double vu;  //天向速度
    UINT8 tail;  //报尾('*') 
    UINT8 Xor;   //校验
}bdfpd_send;


static struct itimerval oldtv;
bool timer_sign = 0; // 标志位

UINT32 rud_send=0;

//文件
FILE *fp; 

// 函数声明
INT32 OpenPort(INT32 comport, INT32 baud);  // 打开端口
void Set_Timer(INT32 interv, INT32 val); // 设定定时器
void Signal_Handler(); // 定时执行的函数
INT32 HextoDec(INT8 a, INT8 b);
INT32 readline(INT32 fd, void *read_buff, INT32 maxlen);
_Bool GPS_XOR_checkout(INT8 *buff);
int GPS_Analyse(bdfpd *gps_data, char *read_buff);
int IMU_Analyse(int32_t *imu_data, uint8_t *read_buff);
INT8 LongLat2XY(double longitude, double latitude, double *X, double *Y);
int GPS_weeksec_BeiJing(double gpst[2]);

// 主函数
int main()
{
    bdfpd gps_data;

    // INT32 fd485_1, fd485_2, fd485_3, fd485_4, fd485_5, fd485_6;
    INT32 fd485_7, fd485_8;
    INT32 i = 0, j = 0, n = 0, ret = 0;
    INT32 clock_time = 0.0;
    INT32 hour, minute, second;
    INT32 side_rev = 0;
    INT32 jx, jy, jz;
    INT32 gps_analcheck = 0;
    INT32 imu_analcheck = 0;
    int32_t  imu_data[6];
    _Bool gps_recvcheck = 0;
    double rud = 0.0, w_x = 0.0,w_y=0,w_z=0,a_x=0,a_y=0,a_z=0;

    double temp_send[12] = {30.8309785, 120.8324900, 30.8308536, 120.8323451, 30.8316199, 120.8314938, 30.8312223, 120.8304572, 30.8308058, 120.8317033, 30.8308465, 120.8312374};
    double time_buff[2] = {0.0};
    INT8 recv_bdfpd[GPS_COM1RECV_SIZE] = {0x00};
    uint8_t recv_rawimusb[GPS_COM2RECV_SIZE] = {0x00};
    char boat_status[300];

    // // 读文件参数
    // char recvfromFile[100];
    // fp = fopen("/home/zmh/nfs", "r");
    // if (fp == NULL)
    // {
    //     printf("read file error\n");
    // }

    // memset(recvfromFile, 0, sizeof(recvfromFile));

    // 串口
    INT32 comport_A = 0, comport_B = 1, comport_C = 2, comport_D = 3, comport_E = 4, comport_F = 5, comport_G = 6, comport_H = 7; 
    INT32 baud_A = 38400, baud_B = 38400, baud_C = 38400, baud_D = 38400, baud_E = 38400, baud_F = 9600, baud_G = 115200, baud_H = 115200; // baudrate
    // fd485_1 = OpenPort(comport_A, baud_A); // 后推1 
    // fd485_2 = OpenPort(comport_B, baud_B); // 后推2 
    // fd485_3 = OpenPort(comport_C, baud_C); // 
    // fd485_4 = OpenPort(comport_D, baud_D); // 
    // fd485_5 = OpenPort(comport_E, baud_E); // 左右舵 
    // fd485_6 = OpenPort(comport_F, baud_F); // 操纵杆 
    fd485_7 = OpenPort(comport_G, baud_G); // GPS COM1
    fd485_8 = OpenPort(comport_H, baud_H); // GPS COM2

    // 网络（UDP，服务器）
    int recv_len = 0, send_len = 0;

    socklen_t len_client, len_server;
    struct sockaddr_in server_sockaddr;
    struct sockaddr_in client_sockaddr;
    int server_sockfd; // server_sockfd:服务器句柄; len:接收返回值; num:发送返回值;
    char sendtoclient[CLI_SEND_BUFSIZE], receivefromClient[CLI_RECV_BUFSIZE] = {0x00}; // unsigned char

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

    // UDP 模式下建立连接，不会阻塞程序
    // if (connect(server_sockfd, (struct sockaddr *)&client_sockaddr, sizeof(client_sockaddr)) < 0)
    // {
    //     perror("connect error!\n");
    //     exit(1);
    // }
    // //printf("connect sucessfully!\n");

    // set up timer and start
	signal(SIGALRM, Signal_Handler); // 定时执行函数
	Set_Timer(100, 1); // 定时时间

    // 写进日志里
    fp = fopen("msg.txt", "a+"); 
    stdout = fp; 

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
            tcflush(fd485_8, TCIOFLUSH); 

            memset(recv_bdfpd, 0, sizeof(recv_bdfpd));
            memset(recv_rawimusb, 0, sizeof(recv_rawimusb));
            memset(receivefromClient,0,sizeof(receivefromClient));
            memset(boat_status,0,sizeof(boat_status));
            memset(sendtoclient,0,sizeof(sendtoclient));

            // recv from boat
            recv_len = recvfrom(server_sockfd, receivefromClient, sizeof(receivefromClient), 0, 
                                (struct sockaddr *)&server_sockaddr, &len_server);  // 接收上位机指令
            // 接收推进器状态。协议6位，第一位识别，第二位确定哪个推进器，第三位执行什么动作，第四位组合动作确定状态，第五位朝向哪个方向。

            // printf("recv length = %d, ", n);
            // // 打印输入数组
            // printf("receivefromClient =");
            // for(i = 0;i < CLI_RECV_BUFSIZE;i++) // 
            // {
            // printf("%d ", receivefromClient[i]);
            // }         
            // printf("\n");
             rud_send = receivefromClient[5]<<8 + (receivefromClient[6] & 0xFF);
            rud = rud_send/10.0;
            // side_rev = (receivefromClient[5] << 8) + receivefromClient[6]; // 侧推位置
            jx = (receivefromClient[7] << 8) + receivefromClient[8]; // 操纵杆x轴
            jy = (receivefromClient[9] << 8) + receivefromClient[10]; // 操纵杆y轴
            jz = (receivefromClient[11] << 8) + receivefromClient[12]; // 操纵杆z轴

            // printf("%s \n", boat_status);

            // GPS COM1
            //读到换行符 写入缓冲区，后期队列修改
            ret = readline(fd485_7, recv_bdfpd, sizeof(recv_bdfpd));
            // fputs(boat_status, fp);

            //这样读，不定长接受
            // nread_GPS = ReadComPort(fd485_7 ,recv_bdfpd,sizeof(recv_bdfpd));
            //printf("nread_GPS=%d",nread_GPS);

            //异或校验
            gps_recvcheck = GPS_XOR_checkout(recv_bdfpd);

            //字符数组名本身就是地址，无需添加&(gps_data.Status)
            // gps_recvcheck = GPS_XOR_checkout(recv_bdfpd);
            if(gps_recvcheck == 1)
            {
                //协议 ASCII字符转化为二进制数据
                gps_analcheck = GPS_Analyse(&gps_data, recv_bdfpd);
             }

            // GPS COM2
            n = ReadComPort(fd485_8, recv_rawimusb, GPS_COM2RECV_SIZE); 
            imu_analcheck = IMU_Analyse(imu_data,recv_rawimusb);
            // z_w = recv_rawimusb[42] + (recv_rawimusb[43] << 8);
            w_z = (double)imu_data[0]*200*720/2147483648;
            w_y = (double)imu_data[1]*200*720/2147483648;
            w_x= (double)imu_data[2]*200*720/2147483648;
            a_z = (double)imu_data[3]*200*200/2147483648;
            a_y = (double)imu_data[4]*200*200/2147483648;
            a_x = (double)imu_data[5]*200*200/2147483648;
            // printf("%f %f %f %f %f %f\n", w_z,w_y,w_x,a_z,a_y,a_x);

            // if(i % 10 == 0)
            // {
                // sprintf(sendtoclient, "%.3f,%.7lf,%.7lf,%.2f,%.2f,%.2f", gps_data.gps_sec, temp_send[j],  temp_send[j + 1], gps_data.ve,  gps_data.vn,  gps_data.heading); 
                // sprintf(sendtoclient, "%.3f,%.7lf,%.7lf,%.2f,%.2f,%.2f,", gps_data.gps_sec, gps_data.latitude,  gps_data.longitude,  gps_data.ve,  gps_data.vn,  gps_data.heading); 
            
            //     j = j + 2;
            // }
            // i++;
            // if(i > 9) i = 0;
            // if(j >= 12) j = 0;

            // time
            time_buff[0] = gps_data.gps_week;
            time_buff[1] = gps_data.gps_sec;
            clock_time = GPS_weeksec_BeiJing(time_buff);
            hour = (clock_time & 0xFF0000) >> 16;
            minute = (clock_time & 0xFF00) >> 8;
            second = clock_time & 0xFF;     
            sprintf(sendtoclient, "%.7lf,%.7lf,%.2f,%.2f,%.2f,%.2f,%d,%d,%u,%lf,", gps_data.latitude,  gps_data.longitude,  gps_data.ve,  gps_data.vn,  gps_data.heading, w_z,minute,second,jy,rud); 
                // printf("sendtoclient = %s\n", sendtoclient);
            n = sendto(server_sockfd, sendtoclient, sizeof(sendtoclient), 0, (struct sockaddr *)& client_sockaddr, len_client); // 扔给上
            sprintf(boat_status, "%d,%d,%d,%d,%lf,%d,%d,%d,%d,%d,%d,%.2f", hour, minute, second, receivefromClient[1], rud, receivefromClient[3], 
                            receivefromClient[4], side_rev, jx, jy, jz, w_z); 

            // printf("%s,%s,%s", sendtoclient, boat_status, recv_bdfpd);
            printf("%s,%s", boat_status, recv_bdfpd);

            // // if(i <= 10)
            // // {
            //     sprintf(sendtoclient, "%.7lf,%.7lf,", temp_send[i], temp_send[i + 1]); 
            // // sprintf(sendtoclient, "%.3f,%.7lf,%.7lf,%.2f,%.2f,%.2f,", gps_data.gps_sec, temp_send[i],  temp_send[i + 1],  gps_data.ve,  gps_data.vn,  gps_data.heading); 
            //     printf("sendtoclient = %s\n", sendtoclient);
            //     // printf("%s,%s,%s", sendtoclient, boat_status, recv_bdfpd);
            //     n = sendto(server_sockfd, sendtoclient, sizeof(sendtoclient), 0, (struct sockaddr *)& client_sockaddr, len_client); // 扔给上
            //     i = i + 2;
            //     if(i >= 12) i = 0;
            // // }

            fflush(stdout);
            // fclose(fp); 

            timer_sign = 0;
        }
    } // while

    // close(fd485_1);
    // close(fd485_2);
    // close(fd485_3);
    // close(fd485_4);
    // close(fd485_5);
    // close(fd485_6);
    close(fd485_7);
    close(fd485_8);

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

INT32 HextoDec(INT8 a, INT8 b) 
{
    INT32 n1, n2;

    if (a >= 'A'&& a <= 'F')
    {
        n1 = 10 + (a - 'A');
    }
    else 
    {
        n1 = a - '0';
    }
    if (b >= 'A'&& b <= 'F') 
    {
        n2 = 10 + (b - 'A');
    }
    else 
    {
        n2 = b - '0';
    }
    
    return ((n1 << 4) + n2); //n1 * 16
}

INT32 readline(INT32 fd, void *read_buff, INT32 maxlen)
{
    INT32 n,rc;
    INT8 c,*ptr;
    
    ptr=read_buff;
    for(n=1;n<maxlen;n++)
    {

        if(rc=read(fd,&c,1)==1)
        {
            *ptr++=c;
            if(c=='\n')
            break;

        }else if(rc==0)
        {
            *ptr=0;
            return n-1;
        }
    }
    *ptr=0;
    return n;
}

int GPS_Analyse(bdfpd *gps_data, char *read_buff)
{

     char *ptr = NULL;

     memset(gps_data,0,sizeof(gps_data));
       
    if(NULL==(ptr=strstr(read_buff,"$BDFPD")))
    {      
        return -1;
    }
    //字符数组名本身就是地址，无需添加&(gps_data.Status)
     sscanf(read_buff,"$BDFPD,%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%hd,%hd,%hd,%hd*",
     &((*gps_data).gps_week),&((*gps_data).gps_sec),&((*gps_data).heading),&((*gps_data).pitch),&((*gps_data).roll),
     &((*gps_data).latitude),&((*gps_data).longitude),&((*gps_data).altitude),&((*gps_data).ve),&((*gps_data).vn),&((*gps_data).vu),
     &((*gps_data).NSV1),&((*gps_data).NSV2),&((*gps_data).PT1),&((*gps_data).PT2));

    // printf("gps_week=%lf gps_status=%s latitude=%lf\n",(*gps_data).gps_week,(*gps_data).Status,(*gps_data).latitude);
  
    return 1;
}

int IMU_Analyse(int32_t *imu_data, uint8_t *read_buff)
{
     for(int i=0;i<GPS_COM2RECV_SIZE-56;i++)
     {
        if(read_buff[i]==0xaa && read_buff[i+1]==0x44 &&read_buff[i+2]==0x13)
        {
            // printf("%x %x %x %x ", read_buff[i+24],read_buff[i+25],read_buff[i+26],read_buff[i+27]);
            imu_data[0] =(read_buff[i+43]<<24) + (read_buff[i+42]<<16) + (read_buff[i+41]<<8) + read_buff[i+40];
            imu_data [1]=(read_buff[i+47]<<24) + (read_buff[i+46]<<16) + (read_buff[i+45]<<8) + read_buff[i+44];
            imu_data [2]=(read_buff[i+51]<<24) + (read_buff[i+50]<<16) + (read_buff[i+49]<<8) + read_buff[i+48];
            imu_data [3]=(read_buff[i+31]<<24) + (read_buff[i+30]<<16) + (read_buff[i+29]<<8) + read_buff[i+28];
            imu_data [4]=(read_buff[i+35]<<24) + (read_buff[i+34]<<16) + (read_buff[i+33]<<8) + read_buff[i+32];
            imu_data [5]=(read_buff[i+39]<<24) + (read_buff[i+38]<<16) + (read_buff[i+37]<<8) + read_buff[i+36];
            //  printf("%d ", imu_data);
            break;
        }
     }
    return 1;
}

_Bool GPS_XOR_checkout(INT8 *buff)
{
    INT32 i = 3;
    INT8 tmp = buff[1] ^ buff[2];

    if ((buff[0] == '$') && (buff[1] == 'B') && (buff[2] == 'D') && (buff[3] == 'F') && (buff[4] == 'P') && (buff[5] == 'D')) 
    {       
		//$与*所有字符位异或校验
	    while (buff[i] != '*' && buff[i] != '\0')
        {
            tmp = tmp ^ buff[i];
            //printf("%c",buff[i]);
            i++;
		}
		i++;
        //校验位2个字符是校验和字符8位的底4位和高4位的16进制数字字符组成     
        INT32 x;
		x = HextoDec(buff[i], buff[i + 1]);
		//返回十进制便于比较
		if (tmp == x)
        {
            // printf("GPS接收指令验证合格\n");
			return 1; 
		}
        else
        {
            // printf("GPS接收指令验证不合格\n");
            return 0;
        }
	}
}

///Summary：WGS84大地坐标系转高斯平面坐标系
///输入：经纬度
///输出：平面坐标x,y
///*********************************************************
INT8 LongLat2XY(double longitude, double latitude, double *X, double *Y)
{
    INT32 ProjNo=0; 
    INT32 ZoneWide; //带宽
    double longitude1,latitude1, longitude0,latitude0, X0,Y0, xval,yval;
    double a,f, e2,ee, NN, T,C,A, M, iPI;
    iPI = 0.0174532925199433; //3.1415926535898/180.0;
    
    //3度带宽
    ZoneWide = 3;
    //6度带宽
    //ZoneWide = 6;
    
    //大地坐标系选择
    //a=6378245.0; f=1.0/298.3; //54年北京坐标系参数
    //a=6378140.0; f=1/298.257; //80年西安坐标系参数
    a = 6378137.0; f = 1.0/298.257223563;//WGS84坐标系参数
    
    //6度带参数
    //ProjNo = (INT32)(longitude / ZoneWide) ;
    //longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
    
    //3度带参数
    ProjNo = (INT32)(longitude / ZoneWide+0.5) ;
    longitude0 = ProjNo * ZoneWide ; //--中央子午线
    
    longitude0 = longitude0 * iPI ;//--中央子午线转化未弧度
    latitude0=0;
    longitude1 = longitude * iPI ; //经度转换为弧度
    latitude1 = latitude * iPI ; //纬度转换为弧度
    e2=2*f-f*f;
    ee=e2*(1.0-e2);
    NN=a/sqrt(1.0-e2*sin(latitude1)*sin(latitude1));
    T=tan(latitude1)*tan(latitude1);
    C=ee*cos(latitude1)*cos(latitude1);
    A=(longitude1-longitude0)*cos(latitude1);
    M=a*((1-e2/4-3*e2*e2/64-5*e2*e2*e2/256)*latitude1-(3*e2/8+3*e2*e2/32+45*e2*e2
        *e2/1024)*sin(2*latitude1)
        +(15*e2*e2/256+45*e2*e2*e2/1024)*sin(4*latitude1)-(35*e2*e2*e2/3072)*sin(6*latitude1));
    xval = NN*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*ee)*A*A*A*A*A/120);
    yval = M+NN*tan(latitude1)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
        +(61-58*T+T*T+600*C-330*ee)*A*A*A*A*A*A/720);
    //6度带
    //X0 = 1000000L*(ProjNo+1)+500000L;
    
    //3度带 改写为国家通用坐标 xval表示经度：单位米，yval表示纬度：单位米 与高斯定义x,y轴交换坐标轴
    X0 = 1000000L * ProjNo + 500000L;  //3度带
    Y0 = 0;
    xval = xval + X0; 
    yval = yval+Y0;

    *X= xval;
    *Y= yval;
   // printf("经度转为米 xval=%lf 纬度转为米 yval=%lf\n",xval,yval);

    return 1;
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
    int time;
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
    // printf("公历时间:%d:%d:%d %d:%d:%d",year, month, day, hour, minute, sec);
    time = sec + (minute << 8) + (hour << 16);

    return time;
}

