#ifndef __DATA_STRUCT__
#define __DATA_STRUCT__


//大气计算机消息集
typedef struct __atmosinfo__
    {
	short Barometric_altitude; //气压高度
	short Indicated_airspeed;  //指示空速
	short True_airspeed;      //真空速
//    int Atmos_temperature_sum; //大气总温
//    int Atmos_temperature_static;//大气静温
//    int lifting_speed;   //升降速度
//    int angle_attack;  //攻角
//    int angle_sideslip;  //侧滑角
//    int Mach_num;   //马赫数
//    int dynamic_pressure;  //动压
//    int pressure_diff_updown; //上下表面压力差
//    int pressure_diff_leftright; //左右表面压力差
//    int Data_valid_identifier; //数据有效性标识符
//    int Fault_identifier;  //故障标识符
    }Atmosinfo;

//    extern int merge(char *buff);//合并


#endif
