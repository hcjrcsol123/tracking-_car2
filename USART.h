#ifndef _USART_H
#define _USART_H
#include "MOTORandMOVE.h"
extern uint16_t RX_data;//数据接收变量
void USART3_Init(void);
void USART3_SEND_data(uint8_t data);
#endif
