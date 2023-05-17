#include "MOTORandMOVE.h"
/*编码器通道
    TIM3 A6  A7 非重映像
    TIM8 C6  C7 非重映像
    TIM1 E9  E11 完全重映像
    TIM4 D12 D13 完全重映像
PWM输出通道
    TIM2 B10 B11 pwmCH3 CH4 完全重映像 对应电机3、4pwm通道
    TIM5 A1 A2 pwmCH2 CH3 非重映像 对应电机1、2pwm通道
串口中断
    USART3 C10 C11 部分重映像
定时中断
    TIM6 接收编码器数值并调整pwm。
    TIM7(待定)
以下为复用后无法作为GPIO使用的引脚和复用后特殊功能无法使用的引脚：
TIM1复用后：PE7 PE9 PE11 PE13 PE14 PE15 PE8 PE10 PE12无法使用
TIM4复用后：PD12 PD13 PD14 PD15无法使用
TIM2复用后：PA15 PB3 PB10 PB11无法使用
USART3复用后：PD9 PD10 PD11 PD12无法使用
*/

int16_t Temp1,Temp2,Temp3,Temp4;//接收编码器的计数值
int16_t speed1 = 0,speed2=0,speed3 = 0,speed4 = 0;//电机pwm实际输出值
int16_t target_speed1 = 0,target_speed2 = 0,target_speed3 = 0,target_speed4 = 0;//电机pwm目标值

void timer6(void)//初始化基本定时器
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//开启外设时钟
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = (1000-1);
    TIM_TimeBaseInitStructure.TIM_Prescaler = (3600-1);//100ms一次中断
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
    TIM_InternalClockConfig(TIM2);

    //配置定时器中断
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组2

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//响应优先级2
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6,ENABLE);


}

// void timer7(void)//初始化基本定时器
// {
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);//开启外设时钟
//     TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
//     TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//     TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//     TIM_TimeBaseInitStructure.TIM_Period = (2000-1);
//     TIM_TimeBaseInitStructure.TIM_Prescaler = (7200-1);//200ms一次中断
//     TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//     TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);

//     //配置定时器中断
//     TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    
//     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组

//     NVIC_InitTypeDef NVIC_InitStructure;
//     NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级2
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//响应优先级1
//     NVIC_Init(&NVIC_InitStructure);

//     TIM_Cmd(TIM7,ENABLE);
// }


//初始化4个编码器
void Encoder1(void)//TIM3，编码器1
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//1开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//配置GPIO

    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;//应与外部电平的默认值保持一致
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_Initstructure);


    //配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//意思是高低电平极性不反转。
    TIM_TimeBaseInitStructure.TIM_Period = (65536-1);//重装值直接最大，满量程计数，方便换成负数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;//不分频，外部编码器直接驱动定时器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

    //配置输入捕获单元两个通道
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//赋值默认值
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);
    
    //配置编码器
    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//其实就是都不反向。若要调节，可在此处调节极性方向

    TIM_Cmd(TIM3,ENABLE);
}

void Encoder2(void)//用TIM8
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//1开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//配置GPIO

    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;//应与外部电平的默认值保持一致
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_Initstructure);


    //配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//意思是高低电平极性不反转。
    TIM_TimeBaseInitStructure.TIM_Period = (65536-1);//满量程计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;//不分频，外部编码器直接驱动定时器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);

    //配置输入捕获单元两个通道
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//赋值默认值
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM8,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM8,&TIM_ICInitStructure);
    
    //配置编码器
    TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//其实就是都不反向。若要调节，可在此处调节极性方向

    TIM_Cmd(TIM8,ENABLE);

}

void Encoder3(void)//用TIM1
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//1开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//配置GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启AFIO时钟


    //开启复用功能
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;//应与外部电平的默认值保持一致
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_Initstructure);


    //配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//意思是高低电平极性不反转。
    TIM_TimeBaseInitStructure.TIM_Period = (65536-1);//重装值直接最大，满量程计数，方便换成负数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;//不分频，外部编码器直接驱动定时器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);

    //配置输入捕获单元两个通道
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//赋值默认值
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM1,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM1,&TIM_ICInitStructure);
    
    //配置编码器
    TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//其实就是都不反向。若要调节，可在此处调节极性方向

    TIM_Cmd(TIM1,ENABLE);
}

void Encoder4(void)//用TIM4
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//1开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);//配置GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启AFIO时钟


    //开启复用功能
    GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;//应与外部电平的默认值保持一致
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD,&GPIO_Initstructure);


    //配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//意思是高低电平极性不反转。
    TIM_TimeBaseInitStructure.TIM_Period = (65536-1);//重装值直接最大，满量程计数，方便换成负数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;//不分频，外部编码器直接驱动定时器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);

    //配置输入捕获单元两个通道
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//赋值默认值
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM4,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0x6;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM4,&TIM_ICInitStructure);
    
    //配置编码器
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//其实就是都不反向。若要调节，可在此处调节极性方向

    TIM_Cmd(TIM4,ENABLE);
}

int16_t Encoder1_Get(void)
{
    int16_t temp;
    temp = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3,0);//清空cnt
    return temp;
}
int16_t Encoder2_Get(void)
{
    int16_t temp;
    temp = TIM_GetCounter(TIM8);
    TIM_SetCounter(TIM8,0);//清空cnt
    return temp;
}
int16_t Encoder3_Get(void)
{
    int16_t temp;
    temp = TIM_GetCounter(TIM1);
    TIM_SetCounter(TIM1,0);//清空cnt
    return temp;
}
int16_t Encoder4_Get(void)
{
    int16_t temp;
    temp = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4,0);//清空cnt
    return temp;
}

void pwm_TIM5(int16_t PwmParameter)//1KHZ占空比100%
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM5);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (1000-1);   //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = (36-1);   //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//未设置
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//极性不翻转，pwm波形直接输出
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PwmParameter;//CCR
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	TIM_Cmd(TIM5,ENABLE);
}


void pwm_TIM2(int16_t PwmParameter)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启AFIO时钟
    
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (1000-1);   //未设置ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = (36-1);   //未设置PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//未设置
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//极性不翻转，pwm波形直接输出
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PwmParameter;//未设置CCR
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	TIM_Cmd(TIM2,ENABLE);
}

void PWM_Init(int16_t PwmParameter)
{
    pwm_TIM2(PwmParameter);
    pwm_TIM5(PwmParameter);
}

void motor_Direction_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

}

void motor_Init(void)
{
    motor_Direction_GPIO_Init();
    timer6();
    PWM_Init(1000);
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    
}

void electric_machinery_direction_control(uint8_t motor1,uint8_t motor2,uint8_t motor3,uint8_t motor4)//电机转向调节，0正转，1反转
{
	if(motor1==Forward_direction)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	}
	else if(motor1==Reverse)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4);//正负极反转
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	}
    
	if(motor2==Forward_direction)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_4);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	}
	else if(motor2==Reverse)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);
	}
	
	if(motor3==Forward_direction)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_0);
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	else if(motor3==Reverse)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	}
	
	if(motor4==Forward_direction)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	}
	else if(motor4==Reverse)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	}
}

//四个电机的pwm控制
void motor1_pwm_contorl(int16_t speed)
{
    TIM_SetCompare2(TIM5,speed);
}
void motor2_pwm_contorl(int16_t speed)
{
    TIM_SetCompare3(TIM5,speed);
}
void motor3_pwm_contorl(int16_t speed)
{
    TIM_SetCompare3(TIM2,speed);
}
void motor4_pwm_contorl(int16_t speed)
{
    TIM_SetCompare4(TIM2,speed);
}

void target_speed_all_set(int16_t s1,int16_t s2,int16_t s3,int16_t s4)//同时设置4个目标速度
{
    target_speed1 = s1;
    target_speed2 = s2;
    target_speed3 = s3;
    target_speed4 = s4;
}


void motor_control_all(int16_t speed_all_1,int16_t speed_all_2,int16_t speed_all_3,int16_t speed_all_4)
{
    motor1_pwm_contorl(speed_all_1);
    motor2_pwm_contorl(speed_all_2);
    motor3_pwm_contorl(speed_all_3);
    motor4_pwm_contorl(speed_all_4);
}

void TIM6_IRQHandler(void)//读取速度数据、修改占空比中断函数
{
    if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)
    {
        Temp1 = Encoder1_Get();
        Temp2 = Encoder2_Get();
        Temp3 = Encoder3_Get();
        Temp4 = Encoder4_Get();//读取脉冲数
        Temp1 = Temp1*pwm_Speed_proportion;
        Temp2 = Temp2*pwm_Speed_proportion;
        Temp3 = Temp3*pwm_Speed_proportion;
        Temp4 = Temp4*pwm_Speed_proportion;//将脉冲数转换成占空比
        speed1 = pid_motor_1_out(abs(Temp1),target_speed1);
        speed2 = pid_motor_2_out(abs(Temp2),target_speed2);
        speed3 = pid_motor_3_out(abs(Temp3),target_speed3);
        speed4 = pid_motor_4_out(abs(Temp4),target_speed4);//修改占空比
        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
    }

}

