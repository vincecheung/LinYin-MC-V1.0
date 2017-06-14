
#include "p30f6010A.h"  //dsPIC30F6010A标准头文件
#include "math.h"
#include "job.h"
#include "cpuinit.h"
#include "USARTinit.h"
#define FCY 16000000    //此项必须设置在【#include <libpic30.h>】之前，会用的时钟频率的定义
#include <libpic30.h>

//************************************************************************
//       取得06CPU用户参数区的设置数据[#70~#89][$FE8C~$FEB2]
//
//   06CPU 用户参数70开始为设定参数，开始地址FE8C~FEB2，预计使用20个
//   #70   过压报警门限             (设置) $FE8C
//   #71   转速换算PWM脉宽          (设置) $FE8E   (0.1Hz对应的脉冲数/s)*100(扩大100倍)取整;60 * F（0.1Hz）/极对数/60s * 每转脉冲数
//   #72   直流电流量程最大值       (设置) $FE90
//   #73   直流电压量程最大值       (设置) $FE92
//   #74   欠压报警门槛             (设置) $FE94
//   #75   电机过热报警门槛         (设置) $FE96
//   #76   控制器散热器过热报警门槛 (设置) $FE98
//   #77   NTC热敏电阻选择          (设置) $FE9A
//   #78   油门0速开度              (设置) $FE9C
//   #79   油门100%开度对应的频率   (设置) $FE9E
//   #80   电机运行频率限幅值       (设置) $FEA0
//   。。。。。#89（$FEB2)
//************************************************************************
void fe_init(void)
//rtn=0:未取得数据；1：已取得数据
{
   uint i;
   ClrWdt();      //清看门狗定时器                           //喂狗
   //for(i_delay = 0; i_delay < 1000000; i_delay++)Nop();//延时约1秒
         
//--------------------------------------------------------------------------------
//                    过压报警门槛(设置)--fe_data[0]--#70 
//     用于过电压时的功率衰减，在硬件过压保护之前实现衰减，减少控制器过压故障
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE8C",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }   
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[0] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//              电机转速PWM输出----转RPM/MIN(设置)  fe_data[1]--#71 
//   0.1Hz对应的脉冲数*100(扩大100倍)取整  60 * F（0.1Hz）/极对数 * 每转脉冲数
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE8E",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[1] = (dt_buf[1] << 8) + dt_buf[2];
//--------------------------------------------------------------------------------
//                     直流电流最大值(设置)--fe_data[2]-72#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE90",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[2] = (dt_buf[1] << 8) + dt_buf[2];
//--------------------------------------------------------------------------------
//                   直流电压最大值  (设置)--fe_data[3]--#73
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE92",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[3] = (dt_buf[1] << 8) + dt_buf[2];
         //获取电压偏置、电压比值
         Temporary_variables_uint16 = fe_data[3];
         if (fe_data[3]>=10000) //判断为反偏
         {
             voltage_bias = 0;//清偏置电压
             fe_data[3] = fe_data[3] - 10000;//去掉正、反偏置标志
             fe_data[3] = fe_data[3]/1000;   //获得偏置电压值
             voltage_bias = voltage_bias - fe_data[3];//偏置变量加上符号
             fe_data[3] = (Temporary_variables_uint16-10000-fe_data[3]*1000)*10;//获得电压量程范围
         }
         else//判断为正偏
         {
             fe_data[3] = fe_data[3]/1000;
             voltage_bias = fe_data[3];
             fe_data[3] = (Temporary_variables_uint16-voltage_bias*1000)*10;
         }    
//--------------------------------------------------------------------------------
//                    欠压报警门槛    (设置)  fe_data[4]--74#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE94",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[4] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                   电机过热报警门槛    (设置)--fe_data[5]-75#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE96",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[5] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//               控制器散热器过热报警门槛    (设置)--fe_data[6]-76#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE98",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[6] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      控制器NTC电阻选择(设置)--fe_data[7]-77#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE9A",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[7] = (dt_buf[1] << 8) + dt_buf[2];
         if ((fe_data[7]!=5)&&(fe_data[14]!=10)) fe_data[14] = 10;
//**************************************************************************************************************************
//--------------------------------------------------------------------------------
//               油门0速开度,0速时 n/100的油门时=1000VFB(设置)--fe_data[8]-78#
//               VFB = 1000 = 油门开度 n/100，
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)              //标准设置值80=80/100=80%油门开度
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE9C",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[8] = (dt_buf[1] << 8) + dt_buf[2];
         if (fe_data[8]>100) fe_data[8] =100;
         ADC_proportion = fe_data[8]/100.0;
//--------------------------------------------------------------------------------
//   电机到达设置的频率时，油门踩到底100%开度对应的频率(设置)--fe_data[9]-79#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)             //标准设置值5000=50hz
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFE9E",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[9] = ((dt_buf[1] << 8) + dt_buf[2])/10;
         
//--------------------------------------------------------------------------------
//                      电机运行频率上限值(设置)--fe_data[10]-80#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)               //标准设置值20000=200hz
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEA0",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[10] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[11]-81#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEA2",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[11] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[12]-82#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEA4",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[12] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[13]-83#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEA6",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[13] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[14]-84#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEA8",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[14] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[15]-85#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEAA",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[15] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[16]-86#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEAC",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[16] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[17]-87#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEAE",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[17] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[18]-88#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEB0",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[18] = (dt_buf[1] << 8) + dt_buf[2];
         
//--------------------------------------------------------------------------------
//                      【保留】(设置)--fe_data[19]-89#
//--------------------------------------------------------------------------------
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1DEFEB2",7);
         fe_rd[7] = 13;
         for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
       }
         while(!USART_receive_ok)
         {
          }
         USART_receive_ok=0;
         for(i=0; i<5; i++) dt_buf[i] = USART_receive_buff[i];
         a2i(&dt_buf[1],4);
         fe_data[19] = (dt_buf[1] << 8) + dt_buf[2];
         

//====================================================================
//                   CAN程序版本信息，写入系统参数2# PLSI（ED）
//   2#参数只能写入8位十进制，200901=09年1月；后2位为不同CPU板代号
//   01=EV01;04=EVMC0400;14=0400的修改版，如：EVMCON-0400;
//   11=EV01的修改版，如EV01非电桥AD采集的。
//====================================================================
  if (USART_transmit_ok==1)
       {
         __delay_ms(2);
         str_cpy(fe_rd,"1ED1332B3C",11); //1332B3C=20130620,版本的日期,版本
         fe_rd[11] = 13;
         for(i=0; i<12; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              U1TXREG = USART_transmit_buff[0];
          }
         __delay_ms(20);
       }
       USART_transmit_state=0;
}






