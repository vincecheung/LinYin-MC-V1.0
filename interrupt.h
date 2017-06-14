//*********************************************************
//*       中断处理函数文件，2011-7-13，zhangjt            *
//*  定时器中断、adc中断、USART1发送中断、USART1接收中断  *
//*  CAN通讯中断                                          *
//*********************************************************
#ifndef INTERRUPT_H
#define INTERRUPT_H

extern void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);

extern void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt(void);

extern void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);

extern void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);

extern void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void);

#endif
