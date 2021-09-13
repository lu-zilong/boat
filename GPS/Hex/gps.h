#ifndef __GPS__
#define __GPS__ 1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

//标准GI定位定姿消息集 
typedef struct bdfpdb
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
}bdfpdb_recv;

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



#endif
