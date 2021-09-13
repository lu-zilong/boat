#ifndef _RUD_H
#define _RUD_H 1  //_RUD_H=1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

UINT16 CalcuCRC16(UINT8 *Array, UINT32 length);
INT16 Send_DI(INT32 fd, UINT8 sendbuf[]);
INT8 Send_DO(INT32 fd, UINT8 sendbuf[]);
void Initial_Motor(INT32 fd, INT8 motor);
void Set_Motor_Mode(INT32 fd, INT8 motor, INT8 mode);
void Set_Rudder_Argument(INT32 fd, INT8 motor, INT8 speed, UINT8 acceleration, UINT8 deceleration);
void Send_Rudder(INT32 fd, INT8 motor_sign, double rudder);
INT16 Return(INT32 fd);

#endif

