#ifndef _PID_H
#define _PID_H
// #include "stdlib.h"

#include "MOTORandMOVE.h"
#define pid_kp 1.0
#define pid_ki 0.3
#define pid_kd 0.15
struct pid
{
    float err;//上次偏差
    float last_err;//上一次偏差
    float last_last_err;//上上一次偏差
    float kp,ki,kd;//系数
    float pwm;//实际输出pwm增量值
};
//注意，控制时间较小，应该调小各个参数的值
extern struct pid pid_motor_1;
extern struct pid pid_motor_2;
extern struct pid pid_motor_3;
extern struct pid pid_motor_4;
int16_t pid_motor_1_out(float currect,float target);//当前值、设定值
int16_t pid_motor_2_out(float currect,float target);//当前值、设定值
int16_t pid_motor_3_out(float currect,float target);//当前值、设定值
int16_t pid_motor_4_out(float currect,float target);//当前值、设定值
void pid_Init(void);//pid参数初始化。



#endif
