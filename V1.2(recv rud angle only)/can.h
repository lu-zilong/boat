#ifndef _CAN_H
#define _CAN_H 1  //_CAN_H=1 

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

INT32 Open_CANPort();
INT32 Send_Battery(INT32 frame_mode, INT32 func); // 侧推执行
INT32 Recv_Battery();
INT32 Send_SideDir(INT32 rev);
INT32 Recv_SideDir();
INT32 Close_CANDevice();

#endif 

