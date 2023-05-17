#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "PID.h"
#include "Delay.h"
#include "USART.h"
#include "MOTORandMOVE.h"
int main(void)
{
    USART3_Init();
    pid_Init();
    motor_Init();
    electric_machinery_direction_control(Forward_direction,Forward_direction,Forward_direction,Forward_direction);
    OLED_Init();
    while (1)
    {

        
        OLED_ShowString(1,1,"input_s:");
        OLED_ShowString(2,1,"pid_s:");
        OLED_ShowString(3,1,"true_s:");
        OLED_ShowString(4,1,"target:");
        Delay_s(1);
        target_speed4+=100;
        if (target_speed4==1000)
        {
            Delay_s(1);
            target_speed4 = -1;
        }
        
        
    }
    
}
