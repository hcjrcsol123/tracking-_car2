#include "MOTORandMOVE.h"

struct pid pid_motor_1;
struct pid pid_motor_2;
struct pid pid_motor_3;
struct pid pid_motor_4;
//创建pid参数结构体变量

void pid_Init(void)//未调试，参数不确定。
{
    pid_motor_1.err = 0.0;//偏差
    pid_motor_1.last_err = 0.0;//上一次偏差
    pid_motor_1.last_last_err = 0.0;//上上次偏差
    pid_motor_1.kp = pid_kp;//P
    pid_motor_1.ki = pid_ki;//I
    pid_motor_1.kd = pid_kd;//D
    pid_motor_1.pwm = 0.0;//输出PWM值

    pid_motor_2.err = 0.0;
    pid_motor_2.last_err = 0.0;
    pid_motor_2.last_last_err = 0.0;
    pid_motor_2.kp = pid_kp;
    pid_motor_2.ki = pid_ki;
    pid_motor_2.kd = pid_kd;
    pid_motor_2.pwm = 0.0;

    pid_motor_3.err = 0.0;
    pid_motor_3.last_err = 0.0;
    pid_motor_3.last_last_err = 0.0;
    pid_motor_3.kp = pid_kp;
    pid_motor_3.ki = pid_ki;
    pid_motor_3.kd = pid_kd;
    pid_motor_3.pwm = 0.0;

    pid_motor_4.err = 0.0;
    pid_motor_4.last_err = 0.0;
    pid_motor_4.last_last_err = 0.0;
    pid_motor_4.kp = pid_kp;
    pid_motor_4.ki = pid_ki;
    pid_motor_4.kd = pid_kd;
    pid_motor_4.pwm = 0.0;
}

int16_t pid_motor_1_out(float currect,float target)//当前值、设定值设定电机1的PID
{

    if(abs(currect-target)>5)//如果偏差过小，则不调整，减少抖动。
    {
        pid_motor_1.err  = target-currect;//设定值减去当前值
        pid_motor_1.pwm += (pid_motor_1.kp*(pid_motor_1.err-pid_motor_1.last_err))+(pid_motor_1.ki*pid_motor_1.err)+(pid_motor_1.kd*(pid_motor_1.err-(2*pid_motor_1.last_err)+pid_motor_1.last_last_err));
        pid_motor_1.last_last_err = pid_motor_1.last_err;
        pid_motor_1.last_err = pid_motor_1.err;//将本次偏差传递到上次偏差，上次偏差传递到上上次偏差。
        // return pid_motor_1.pwm;
    }
    if(pid_motor_1.pwm>=1000)//限制最大值
    {
        pid_motor_1.pwm = 1000;
    }
    if(pid_motor_1.pwm<=0)
    {
        pid_motor_1.pwm = 0;//限制最小值
    }
    return pid_motor_1.pwm;
}

int16_t pid_motor_2_out(float currect,float target)//当前值、设定值，设定电机2的PID
{
    if(abs(currect-target)>5)//如果偏差过小，则不调整，减少抖动。
    {
        pid_motor_2.err  = target-currect;//设定值减去当前值
        pid_motor_2.pwm += (pid_motor_2.kp*(pid_motor_2.err-pid_motor_2.last_err))+(pid_motor_2.ki*pid_motor_2.err)+(pid_motor_2.kd*(pid_motor_2.err-(2*pid_motor_2.last_err)+pid_motor_2.last_last_err));
        pid_motor_2.last_last_err = pid_motor_2.last_err;
        pid_motor_2.last_err = pid_motor_2.err;//计算完成，将本次偏差传递到上次偏差，上次偏差传递到上上次偏差。
        // return pid_motor_2.pwm;
    }
    if(pid_motor_2.pwm>=1000)//限制最大值
    {
        pid_motor_2.pwm = 1000;
    }
    if(pid_motor_2.pwm<=0)
    {
        pid_motor_2.pwm = 0;//限制最小值
    }
    return pid_motor_2.pwm;
}

int16_t pid_motor_3_out(float currect,float target)//当前值、设定值设定电机1的PID
{
    if(abs(currect-target)>5)//如果偏差过小，则不调整，减少占空比抖动过大。
    {
        pid_motor_3.err  = target-currect;//设定值减去当前值
        pid_motor_3.pwm += (pid_motor_3.kp*(pid_motor_3.err-pid_motor_3.last_err))+(pid_motor_3.ki*pid_motor_3.err)+(pid_motor_3.kd*(pid_motor_3.err-(2*pid_motor_3.last_err)+pid_motor_3.last_last_err));
        pid_motor_3.last_last_err = pid_motor_3.last_err;
        pid_motor_3.last_err = pid_motor_3.err;//计算完成，将本次偏差传递到上次偏差，上次偏差传递到上上次偏差。
        // return pid_motor_3.pwm;
    }

    if(pid_motor_3.pwm>=1000)//限制最大值
    {
        pid_motor_3.pwm = 1000;
    }
    if(pid_motor_3.pwm<=0)
    {
        pid_motor_3.pwm = 0;//限制最小值
    }
    return pid_motor_3.pwm;
}

int16_t pid_motor_4_out(float currect,float target)//当前值、设定值设定电机1的PID
{
    if(abs(currect-target)>5)//如果偏差过小，则不调整，减少占空比抖动过大。
    {
        pid_motor_4.err  = target-currect;//设定值减去当前值
        pid_motor_4.pwm += (pid_motor_4.kp*(pid_motor_4.err-pid_motor_4.last_err))+(pid_motor_4.ki*pid_motor_4.err)+(pid_motor_4.kd*(pid_motor_4.err-(2*pid_motor_4.last_err)+pid_motor_4.last_last_err));
        pid_motor_4.last_last_err = pid_motor_4.last_err;
        pid_motor_4.last_err = pid_motor_4.err;//计算完成，将本次偏差传递到上次偏差，上次偏差传递到上上次偏差。
        // return pid_motor_4.pwm;
    }
    if(pid_motor_4.pwm>=1000)//限制最大值
    {
        pid_motor_4.pwm = 1000;
    }
    if(pid_motor_4.pwm<=0)
    {
        pid_motor_4.pwm = 0;//限制最小值
    }
    return pid_motor_4.pwm;
}
