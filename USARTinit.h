//==============================================================
// 38400bps
// 串口通讯、CAN通讯初始化; 数值转换计算
// UART_init,(UART1=06CPU通讯，UART2=外部设置使用)
//==============================================================
#ifndef USARTINIT_H
#define USARTINIT_H

extern void Usart_Init(void);
extern unsigned int i2a(unsigned int n, unsigned char *pd, unsigned int m);

extern void a2i(unsigned char *pd, unsigned int m);

extern void str_cpy(unsigned char *ptr1, char *ptr0, unsigned int m);

extern unsigned int temp_10k(unsigned int x);

extern unsigned int temp_5k(unsigned int x);//新100kw控制器的模块自带NTC电阻，25°C时的阻值为5K，官方B值=3375

extern unsigned int tpc_temp(unsigned int _x);

extern unsigned int pt100_temp(unsigned int ad_t3);

extern unsigned int pt100_bridge(unsigned int ad_t3);

extern void can_init_C1(void);

extern void Attack_C1TX0(void);//0号发送器

extern void Attack_C1TX1(void);//1号发送器

extern void Attack_C1TX2(void);//2号发送器

extern void PI_filter(float *previous_adc, float *current_adc, float *proportional);

extern float PI_trk(float *previous_trk, float *current_trk, float P);


#endif

