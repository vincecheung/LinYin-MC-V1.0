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
uchar can_TXdata[3][8];       //发送用变量
unsigned int can_RXdata[3][8];//接收用变量
uint  can_RX_temp;
uint M_err_flg,test_ok=0,test_data[8],delay_error=0,canok_delay=0;           //0：上位机正常发出指令（CAN）；1：异常，停止伺服系统动作
int  adc_data[ADC_N],adc_samp[ADC_N];                           //ADC通道号，ADC_N个通道的ADC数据
uchar adc_index=0,adc_pick,adc_ok=1;                         //存储所开放的5个邮箱数据
uchar fe_rd[15] = {'1','D','E','F','E','6','0',13};
uchar fe_wt[15] = {'1','D','F','F','E','6','0','0','0','0','0',13};//mity_data.c有调用
uchar fe_xx[15] = {'1','D','F','F','E','6','0','0','0','0','0',13};
uint fe_flg, fe_data[MGN_N];
int  fe_data_tmp;
uint ad_data[4];

uchar Tx_Flag;
uchar TXEvtTime;
uchar Tx_Evt_Flag;
//----------------------------------------------------------|
//         程序中计算用临时变量，计算使用完成立即释放       |
// 此处变量不可在中断中使用，防止计算中中断更改变量值
//----------------------------------------------------------|
unsigned int   Temporary_variables_uint16;
int            Temporary_variables_int16;
unsigned char  Temporary_variables_uint8;
//----------------------------------------------------------|
//                       数字滤波用变量                     |
//----------------------------------------------------------|
uint  DC_C;     //前10个数组做ADC数据记录，11号数组用来累计滤波,数组初始化在data_init()函数里
int   DC_offset=0;
int   voltage_bias;//电压偏置值用变量
//-----------------------------------
float   DCI_previous,DCI_current,DCI_proportional=100.0;
int     DCI_data;
//-----------------------------------
float   DCV_previous,DCV_current,DCV_proportional=100.0;
int     DCV_data;
//-----------------------------------
float   moto_previous,moto_current,moto_proportional=500.0;
int     moto_data;
//-----------------------------------

uchar  PA;
uint   fct_test1,fct_test2;
int    int_test;
float  rpm_test,code_test;
//----------------------------------------------------------|
//                       usart_&_can用到的数据              |
//----------------------------------------------------------|
uchar  can_data_ok=1;//can 数据读取标志,初始化为 1 开放第一次读取数据
uchar  can_data_buff[8];
uchar  MOB_send_state[5];
uchar  com_txen,com_rxok,com_index,com_flg,com_buf[35];
uchar  com_data[35];
uchar  timer_on=0,timer_off=0,block_txen=0,block_rxen=0,err_timer;

//----------------------------------------------
  int   err_flg = 0, i,loop=0;
  uint  previous_workmode,work_mode, Bwork_mode,Twork_mode, pwr_now, spd,Mity_Status,TMity_Status, fct;
  uchar dt_buf[37],DataTranWord[14],WordTranIndex,dt_com[33],dt_err[4],USART_receive[35],USART_receive_buff[35];
  uchar p_IN , USART_receive_index=0,USART_receive_ok=0,Reset_Flg,run_Flg = 1,stop_flg=0;
  volatile  uchar USART_transmit_index=1,USART_transmit_ok=1,USART_transmit_buff[37],USART_transmit_state=0;
  float instruction_rpm,actual_rpm,mity_rpm;
  float moto_pwm,moto_hzf;//电机转速PWM输出用变量
//-----------------------------------------------
//旋变报警用计数器变量
  uint ros_index,ros_delay;
  uchar ros_err_flg=0;
//-----------------------------------
// 实际功率转矩换算用变量
  //uint  dc_A;
  unsigned long int DC_power,dc_A;
  uchar  handle_position;
  float  previous_trk = 0,current_trk = 0,  vfb_trk=0,vfb_n_trk=0,PID_P;
  uint   PID_trk,p_trk,moto_rpm,moto_ratio;
  float  proportion_trk,n_trk,Feedback_trk,ADC_proportion;
//-----------------------------------
//  控制器温升变化率报警变量
  unsigned int  ClockDelayOne; 
  unsigned char EvmcTemperature;
  float  low_power=1.0,low_freq;
//-----------------------------------
//  母线过流检测用变量
//-----------------------------------
unsigned int ocl_flg = 0,alm_flg = 0;
uchar pol_flg=0;//三相缺相标志
//----------------------------------------------------
//  can模块ID码用寄存器
//----------------------------------------------------
unsigned long int   TX0_id,TX1_id,TX2_id;//发送寄存器 ID码存储
//---------------
unsigned long int   RX0_F0id;//接收寄存器“0”，验收过滤寄存器“0”
unsigned long int   RX0_F1id;//接收寄存器“0”，验收过滤寄存器“1”
unsigned long int   RX0_Mid; //接收寄存器“0”，屏蔽寄存器
//---------------
unsigned long int   RX1_F0id;//接收寄存器“1”，验收过滤寄存器“0”
unsigned long int   RX1_F1id;//接收寄存器“1”，验收过滤寄存器“1”
unsigned long int   RX1_F2id;//接收寄存器“1”，验收过滤寄存器“2”
unsigned long int   RX1_F3id;//接收寄存器“1”，验收过滤寄存器“3”
unsigned long int   RX1_Mid; //接收寄存器“1”，屏蔽寄存器
//----------------------------------------------------
//  PT100温度电阻的AD换算值，间隔10C°
//----------------------------------------------------
const int pt_100res[46]={
 0, 27, 54, 82,108,135,161,188,214,239,
265,290,315,339,364,388,412,436,460,483,
506,529,552,575,597,619,641,663,685,706,
728,749,770,790,811,831,851,871,891,911,
930,950,969,988,1007,1023};
