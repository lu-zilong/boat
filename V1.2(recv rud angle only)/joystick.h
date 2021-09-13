#ifndef _JOYSTICK_H
#define _JOYSTICK_H 1  //_JOYSTICK_H=1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

UINT32 Joystick_Recv(INT32 fd, UINT8 recv_joystick[]);

#endif 

