
#ifndef JOB_H
#define JOB_H
//**********************************************************
#define uchar  unsigned char
//#define uint  unsigned int
typedef unsigned int uint;

#define ADC_N    8
#define MGN_N    20
#define s_MR     0x80                         //主接触器状态1：接通  C0d0
#define sv_RDY   0x40                         //伺服准备状态1：准备好
//#define sv_ALM   0x20                         //伺服报警1：有报警
#define set_P    0x20                         //参数设定 PINA_BIT5=1
#define sv_TRK   0x10                         //过转矩报警
#define sv_PW    0x80                         //过功率报警
#define sv_OC    0x70                         //过流报警
#define sv_OV    0x08                         //过压报警
#define sv_MR    0x04                         //主接触器断开报警
#define sv_OT    0x08                         //过温
#define sv_LV    0x04                         //直流电压过低
#define sv_HV    0x02                         //直流电压过高

//----------------------------------------------------------|
//                        报文位定义 1                      |
//----------------------------------------------------------|
//PORTB &=~(1<<PB1);  清零
//PORTB |=(1<<PB1);   置位
#define contactor_closed_1                  can_TXdata[0][5]|=  (1<<1)  //5.1
#define contactor_closed_0                  can_TXdata[0][5]&= ~(1<<1)
#define powerstage_ready_1                  can_TXdata[0][5]|=  (1<<0)  //5.0
#define powerstage_ready_0                  can_TXdata[0][5]&= ~(1<<0)
// #define powerstage_fault_1                  (can_TXdata[0][4]=can_TXdata[0][4] | 0x04)
// #define powerstage_fault_0                  (can_TXdata[0][4]=can_TXdata[0][4] & 0xfb)
//#define torque_hits_internal_limit_1        (can_TXdata[0][4]=can_TXdata[0][4] | 0x08)
//#define torque_hits_internal_limit_0        (can_TXdata[0][4]=can_TXdata[0][4] & 0xf7)

#define Mity_temperature_limit_1            can_TXdata[0][6]|=  (1<<0)  //6.0 bit
#define Mity_temperature_limit_0            can_TXdata[0][6]&= ~(1<<0)
#define Motor_temperature_limit_1           can_TXdata[0][6]|=  (1<<1)  //6.1 bit
#define Motor_temperature_limit_0           can_TXdata[0][6]&= ~(1<<1)
#define low_voltage_hits_internal_limit_1   can_TXdata[0][5]|=  (1<<6)  //5.6
#define low_voltage_hits_internal_limit_0   can_TXdata[0][5]&= ~(1<<6)
#define high_voltage_hits_internal_limit_1  can_TXdata[0][5]|=  (1<<4)  //5.4 bit
#define high_voltage_hits_internal_limit_0  can_TXdata[0][5]&= ~(1<<4)
//#define envelope_hits_internal_limit_1      (can_TXdata[0][4]|=  (1<<1)  //can_TXdata[0][4] | 0x80)
//#define envelope_hits_internal_limit_0      (can_TXdata[0][4]&= ~(1<<1)  //can_TXdata[0][4] & 0x7f)

#define power_limit_1                       can_TXdata[0][5]|=  (1<<7)  //5.7 bit
#define power_limit_0                       can_TXdata[0][5]&= ~(1<<7)
#define overcurrent_A_1                     can_TXdata[0][5]|=  (1<<3)  //5.3 bit
#define overcurrent_A_0                     can_TXdata[0][5]&= ~(1<<3)

#define overvoltage_1                       can_TXdata[0][5]|=  (1<<5)  //5.5 bit
#define overvoltage_0                       can_TXdata[0][5]&= ~(1<<5)
//#define contractor_1                        (can_TXdata[0][5]=can_TXdata[0][5] | 0x20)
//#define contractor_0                        (can_TXdata[0][5]=can_TXdata[0][5] & 0xdf)
#define resolver_1                          can_TXdata[0][5]|=  (1<<2)  //5.2 bit
#define resolver_0                          can_TXdata[0][5]&= ~(1<<2)

//==============================================================================
//  不同电机情况下，需修改的宏定义汇总，便于整体进行修改
//==============================================================================
#define moto_pole 3                           //电机极数
//------------------------------------------------------------------------------
//  Relay state          7.1    此 2项 状态在时光控制器里与powerstage_ready 重复
//  Powerstage state     7.3    所以没有报文。
//------------------------------------------------------------------------------
//---can报文相关变量-------------------------------------------------------
extern uchar can_TXdata[3][8];       //发送用变量
extern unsigned int can_RXdata[3][8];//接收用变量
extern uint  can_RX_temp;
extern uint M_err_flg,test_ok,test_data[],delay_error,canok_delay;           //0：上位机正常发出指令（CAN）；1：异常，停止伺服系统动作
extern int  adc_data[],adc_samp[];                           //ADC通道号，ADC_N个通道的ADC数据
extern uchar adc_index,adc_pick,adc_ok;                         //存储所开放的5个邮箱数据
extern uchar fe_rd[];
extern uchar fe_wt[];//mity_data.c有调用
extern uchar fe_xx[];
extern uint fe_flg, fe_data[];
extern int  fe_data_tmp;
extern uint ad_data[];

extern uchar Tx_Flag;
extern uchar TXEvtTime;
extern uchar Tx_Evt_Flag;
//----------------------------------------------------------|
//         程序中计算用临时变量，计算使用完成立即释放       |
// 此处变量不可在中断中使用，防止计算中中断更改变量值
//----------------------------------------------------------|
extern unsigned int   Temporary_variables_uint16;
extern int            Temporary_variables_int16;
extern unsigned char  Temporary_variables_uint8;
//----------------------------------------------------------|
//                       数字滤波用变量                     |
//----------------------------------------------------------|
extern uint  DC_C;     //前10个数组做ADC数据记录，11号数组用来累计滤波,数组初始化在data_init()函数里
extern int   DC_offset;
extern int   voltage_bias;//电压偏置值用变量
//-----------------------------------
extern float   DCI_previous,DCI_current,DCI_proportional;
extern int     DCI_data;
//-----------------------------------
extern float   DCV_previous,DCV_current,DCV_proportional;
extern int     DCV_data;
//-----------------------------------
extern float   moto_previous,moto_current,moto_proportional;
extern int     moto_data;
//-----------------------------------

extern uchar  PA;
extern uint   fct_test1,fct_test2;
extern int    int_test;
extern float  rpm_test,code_test;
//----------------------------------------------------------|
//                       usart_&_can用到的数据              |
//----------------------------------------------------------|
extern uchar  can_data_ok;//can 数据读取标志,初始化为 1 开放第一次读取数据
extern uchar  can_data_buff[];
extern uchar  MOB_send_state[];
extern uchar  com_txen,com_rxok,com_index,com_flg,com_buf[];
extern uchar  com_data[];
extern uchar  timer_on,timer_off,block_txen,block_rxen,err_timer;

//----------------------------------------------
extern   int   err_flg, i,loop;
extern   uint  previous_workmode,work_mode, Bwork_mode,Twork_mode, pwr_now, spd,Mity_Status,TMity_Status, fct;
extern   uchar dt_buf[],DataTranWord[],WordTranIndex,dt_com[],dt_err[],USART_receive[],USART_receive_buff[];
extern   uchar p_IN , USART_receive_index,USART_receive_ok,Reset_Flg,run_Flg,stop_flg;
extern   volatile  uchar USART_transmit_index,USART_transmit_ok,USART_transmit_buff[],USART_transmit_state;
extern   float instruction_rpm,actual_rpm,mity_rpm;
extern   float moto_pwm,moto_hzf;//电机转速PWM输出用变量
//-----------------------------------------------
//旋变报警用计数器变量
extern   uint ros_index,ros_delay;
extern   uchar ros_err_flg;
//-----------------------------------
// 实际功率转矩换算用变量
  //uint  dc_A;
extern   unsigned long int DC_power,dc_A;
extern   uchar  handle_position;
extern   float  previous_trk ,current_trk ,  vfb_trk,vfb_n_trk,PID_P;
extern   uint   PID_trk,p_trk,moto_rpm,moto_ratio;
extern   float  proportion_trk,n_trk,Feedback_trk,ADC_proportion;
//-----------------------------------
//  控制器温升变化率报警变量
extern  unsigned int  ClockDelayOne; 
extern  unsigned char EvmcTemperature;
extern  float  low_power,low_freq;
//-----------------------------------
//  母线过流检测用变量
//-----------------------------------
extern unsigned int ocl_flg ,alm_flg ;
extern uchar pol_flg;//三相缺相标志
//----------------------------------------------------
//  can模块ID码用寄存器
//----------------------------------------------------
extern unsigned long int   TX0_id,TX1_id,TX2_id;//发送寄存器 ID码存储
//---------------
extern unsigned long int   RX0_F0id;//接收寄存器“0”，验收过滤寄存器“0”
extern unsigned long int   RX0_F1id;//接收寄存器“0”，验收过滤寄存器“1”
extern unsigned long int   RX0_Mid; //接收寄存器“0”，屏蔽寄存器
//---------------
extern unsigned long int   RX1_F0id;//接收寄存器“1”，验收过滤寄存器“0”
extern unsigned long int   RX1_F1id;//接收寄存器“1”，验收过滤寄存器“1”
extern unsigned long int   RX1_F2id;//接收寄存器“1”，验收过滤寄存器“2”
extern unsigned long int   RX1_F3id;//接收寄存器“1”，验收过滤寄存器“3”
extern unsigned long int   RX1_Mid; //接收寄存器“1”，屏蔽寄存器
//----------------------------------------------------
//  PT100温度电阻的AD换算值，间隔10C°
//----------------------------------------------------
extern const int pt_100res[];

#endif
