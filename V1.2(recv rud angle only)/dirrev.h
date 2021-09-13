#ifndef _DIRREV_H
#define _DIRREV_H 1  //_DIRREV_H=1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

UINT8 CalcuCRC_DirRev (UINT8 array[], INT8 length);//读
INT16 Send_Normal (INT32 fd);//写
INT8 Send_DirRev(INT32 fd, INT8 rev); // send the array from industrial PC to thruster

#endif

