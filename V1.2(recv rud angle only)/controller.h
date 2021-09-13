#ifndef _CONTROLLER
#define _CONTROLLER 1  //_CONTROLLER=1

typedef int     INT32;
typedef short   INT16;
typedef char    INT8;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

INT16 Speed_Control(double del_speed[], INT8 goa_jst[], double K, double K1, double T, double T1, double Ts); // send the array from industrial PC to thruster
double Speed_Model(INT8 goa_speed, double curr_v[], double K1, double T1, double T);
double Heading_Control(double del_heading[], double last_goa_rud, double K2, double T1, double T2, double T); // send the array from industrial PC to thruster
double Heading_Model(double goa_rud[], double curr_h[], double K1, double T1, double T);


#endif

