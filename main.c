//*****************************************************************************************
// 加入母线过流报警功能,调试完成，看门狗，代码保护
// 晶振改16MHz版,USART和CAN波特率已经更改调好
// 正式程序要求加入电压零点偏置参数设置，及电压低于某值后，取消偏置值的设置功能
// 更改通讯波特率为38400，加入偶校验，改正读错误码地址错误                    
//  v1.1 修改06CPU的reset和fwe对应的6010端口RG2-3为高电平输出，防止意外干扰 ver :20121228
//  无偶校验串口通信设置
//*****************************************************************************************
//CAN总线静态时2.5V；CAN_H 比 CAN_L 高表示的逻辑 0 被称为 显形 位 而用 CAN_L 比 CAN_H 高表
//示的逻辑 1 叫做 隐性 位  
//---------------------------------------------------------------------------------
//   适用硬件：EVMC0615/16                         20140524
//   硬件更改的不同处：
//   EVMC0612/13:ADC4=RB10=直流电流;ADC5=RB11=直流电压;ADC2=RB8=散热器温度;ADC7=RB2=机箱温度
//   EVMC0615/16:ADC4=RB10=直流电流;ADC5=RB11=直流电压;ADC2=RB8=IGBT1温度;ADC6=RB3=IGBT2（新增）;ADC7=RB2=IGBT3
//   其它I/O端口有变动，程序可能会有不兼容问题，详见硬件手册
//   EVAV版带DC/DC控制器无ST信号吸合缓冲，由CAN的RF8控制高压缓冲吸合
//---------------------------------------------------------------------------------
// RB14\RB15定义为电机控制器的接触器吸合、控制器准备好信号，由06CPU的C0D6\C0D7给出
// 加入can通讯超时判断功能，加入油门踏板设置参数读取功能,最高转速限速值读取
// 加入旋变报警及复位处理=错误清转速、转矩值0,有三相缺相检测，无处理
// VCU钥匙复位指令由串口发送，
//                               本版为测试38400bps错误问题临时使用
// 06CPU系统参数10:=【93：9=9600无校验，3=38400无校验;D7：D同前有偶校验，7同前有偶校验】
//                  【块通讯主要关注后面的位，前面位主要影响串口键盘】
//[1;D7;13;28&24]
//注意：必须用户设置参数完整，否则可能影响can数据下发到06CPU；
//主循环周期约300us,有旋变报警周期50ms以上

#include "p30f6010A.h"  //dsPIC30F6010A标准头文件
#include "math.h"
#include "job.h"
#include "cpuinit.h"
#include "USARTinit.h"
#include "interrupt.h"

//********************************************************************************
//  芯片融丝位设置，环境设置
//********************************************************************************
#define FCY 16000000                  //16MHz外部晶振带4倍频，指令周期为16MIPs=62.5ns
//#define FCY 7372800                  //7.3728MHz外部晶振带4倍频，
#include <libpic30.h>
#include "mity_data.h"
//_FOSC(CSW_FSCM_OFF&XT_PLL4);//4、8、16倍频晶振，Failsafe时钟关闭
//_FOSC(CSW_FSCM_OFF&XT_PLL8);//4、8、16倍频晶振，Failsafe时钟关闭
//_FOSC(CSW_FSCM_OFF&XT_PLL16);//16倍频晶振，Failsafe时钟关闭
//-----------------------------
//_FOSC(CSW_FSCM_OFF&HS3_PLL16);//21.333...MHZ主频
//_FOSC(CSW_FSCM_OFF&HS2_PLL8);//16MHZ主频(目前有源晶振仍使用此项设置)
//-----------------------------
_FOSC(CSW_FSCM_OFF&ECIO_PLL4);  //外部有源晶振(可以使用此项)
//_FWDT(WDT_OFF);    // 关闭看门狗定时器
//--_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_1);//开启看门狗，融丝设置1024ms复位
_FBORPOR(PBOR_OFF&MCLR_EN); //掉电复位禁止，MCLR复位时能
//_FGS(GWRP_ON & GSS_HIGH & GSS_ON);  //代码保护
//_FGS(GWRP_ON);//代码保护

//*************************************************
//                     主程序开始                 *
//*************************************************
int main(void)
{
    //--------------------------------
    
    //--------------------------------
    IO_init();
    ClrWdt();//清看门狗定时器
    __delay_ms(500);
    ClrWdt();//清看门狗定时器
    __delay_ms(500);
    ClrWdt();//清看门狗定时器
    __delay_ms(500);//-------延迟启动，等待06CPU启动后才进入工作模式/1800ms
    ClrWdt();//清看门狗定时器
    Usart_Init();
    ClrWdt();//清看门狗定时器
    fe_init();//读取控制器设置参数
    ClrWdt();//清看门狗定时器
    ADC_init();
    T1_Init();
//-----------------------------------
// 上电参数初始化
//-----------------------------------

//======================================================================
//               CAN模块初始化，发送、接收 ID码设置                    |
//  设置ID时参考JOB.H里的定义                                          |
//======================================================================
    //CAN发送ID设置---------------------------------------
	//TX0_id = 0xa0a0a0a;
	TX0_id = 0x1803EEED;//控制器发送数据帧1
	TX1_id = 0x1804EEED;//控制器发送数据帧2
	TX2_id = 0x1FFFFFFF;//此发送器目前没有使用
	//CAN接收ID设置-------0号接收器设置--------------------------------
	RX0_F0id = 0x1801EDEE;//接收寄存器“0”，验收过滤寄存器“0”
	//RX0_F1id = 0x1FFFFFFF;  //接收寄存器“0”，验收过滤寄存器“1”
	RX0_Mid  = 0x1FFFFFFF;  //接收寄存器“0”，屏蔽寄存器
    //--------------------1号接收器设置--------------------------------
	//RX1_F0id = 0x1FFFFFFF;//接收寄存器“1”，验收过滤寄存器“0”
	//RX1_F1id = 0x1FFFFFFF;//接收寄存器“1”，验收过滤寄存器“1”
	//RX1_F2id = 0x1FFFFFFF;//接收寄存器“1”，验收过滤寄存器“2”
	//RX1_F3id = 0x1FFFFFFF;//接收寄存器“1”，验收过滤寄存器“3”
	RX1_Mid  = 0x1FFFFFFF;//接收寄存器“1”，屏蔽寄存器
//---------------
    can_init_C1();
    ClrWdt();//清看门狗定时器
//======================================================================
//                               电流0点修正                           |
//======================================================================
    PORTCbits.RC13 = 0;//D3亮
    PORTDbits.RD13 = 0;//机箱工作指示灯亮
    __delay_ms(100);// 为电流校正做延迟
    ClrWdt();//清看门狗定时器
          DC_offset=adc_data[4];
          DC_offset=512-DC_offset; 
          if ((adc_data[4]>562)||(adc_data[4]<462)) DC_offset=0;


//***************************************************************************
//                                主循环开始                                *
//***************************************************************************
 while(1)
  {
      
//***************************************************************************
//                      直流母线电流过流监测                                *
//***************************************************************************
      ClrWdt();//清看门狗定时器
      if (PORTEbits.RE9 == 0)//过流脉冲信号检测
      {
          ocl_flg = 1;
      }
      if ((PORTDbits.RD14 == 0)&&(ocl_flg == 1)) //ALM 信号检测
      {
          alm_flg = 1;//此标志用于发送过流错误代码，发送完成后要清除
          ocl_flg = 0;//清过流标志位
      }    
      
//***************************************************************************
//                      电机或控制器三相缺相                                *
//***************************************************************************
      if (PORTDbits.RD11 == 0)  pol_flg = 1;
      else  pol_flg = 0;
      
      
//******************************************************************************
//                                                                             *
//  读取上位机发来的 CAN 转矩、转速数据|ADC采集的伺服器温度、电压、电流等数据  *
//     格式转换后、打包到 "du_buf[]"发送给伺服控制器CPU的用户参数区共36个数据  *
//                                                                             *
//******************************************************************************
    if (can_data_ok==0)  //中断内数据交换完成，可以进行can数据处理
    {
     spd = ((can_data_buff[1] << 8)  + can_data_buff[0]);                       //速度指令（单位 1000RPM/1000)
     proportion_trk = ((can_data_buff[3] << 8) + can_data_buff[2]);             //转矩
     work_mode = can_data_buff[4];                                              //工作状态字
     handle_position = can_data_buff[5];                                        //档位状态
     //handle_position &= ~(1<<6);                                                //档位数据里清复位位
     Reset_Flg = can_data_buff[6];                                              //复位标志bit8
     //--------------------------
     //判断复位标志位
     if (Reset_Flg &(1<<1))
     {
       Reset_Flg = 1;             //至位复位标志
     } 
     else Reset_Flg = 0;          //清复位标志
     //--------------------------
     if (can_data_buff[6] &(1<<0)) PORTFbits.RF8 = 0;//EVAC版带DC/DC控制器的高压缓冲吸合控制
     else PORTFbits.RF8 = 1;
     //--------------------------
     can_data_ok=1;                                                             //can数据处理完成，允许中断内再次交换数据
     if (proportion_trk>1000) proportion_trk=999;                                                  //VCU转矩限幅，根据不同电机最大转矩进行限幅，与电机测试数据相关
     if (proportion_trk<2)    proportion_trk=0;
//=============================================================================
//                        速度及转矩报文数值转换部分                           |
//  速度报文为2byte=(65535x0.30518=19999.97≈20000); 10000为0点；范围 +-10000rmp    |
//  频率=转速/60*极对数*100,100频率分辨率// f=n/60*p*100                       |
//  HZP=100f=10n/2,(8极:n=60f/P=60f/4=20f,f=n/60*4)                            |
//=============================================================================
    instruction_rpm = spd;
    instruction_rpm = 0.5 * instruction_rpm - 16384.0;
    if(instruction_rpm >= 0.0)// 正速度指令到指令频率转换计算
    {
      instruction_rpm = instruction_rpm/60*(moto_pole*100);
      if (instruction_rpm>fe_data[10]) instruction_rpm=fe_data[10];
      spd = (uint)(instruction_rpm);
      //handle_position = 1;
    }
    else                      // 负速度指令到指令频率转换计算
    {
      instruction_rpm = -instruction_rpm;
      instruction_rpm=instruction_rpm/60*(moto_pole*100);     //HZP=50f=5n/3,(n=60f/P=60f/2=30f)
      if (instruction_rpm > fe_data[10]) instruction_rpm = fe_data[10];    
      spd = 65535-(uint)(instruction_rpm);
      //handle_position = 2;
    }
    }
    n_trk = proportion_trk;//保留每次循环计算用的初始值，指令位置敏感
//----can数据接收转换完成---
//----------END-------------
//______________________________________________________________
//  can总线复位功能------调试 好有要加入复位功能

    //if (Reset_Flg &(1<<7))  PORTDbits.RD7 = 0;
    //else  PORTDbits.RD7 = 1;
    
//______________________________________________________________
    Bwork_mode &= ~(255<<8);           //清除档位数据
//=============================================================================
//   控制器工作模式字定义：
//   bit7   bit6  bit5  bit4  bit3  bit2  bit1  bit0  低字节位
//   故障   xxxx  xxxx  零锁  制动  速度  力矩  自由
//   128                 16     8     4     2     1
//   bit7   bit6  bit5  bit4  bit3  bit2  bit1  bit0  高字节位
//   保留   xxxx  xxxx  xxxx  xxxx  xxxx 倒车档 前进挡
//   128手刹             16     8     4     2     1   =0 空挡
//=============================================================================
    if (!(work_mode & (1<<7)))   //故障位判断
    {
     if ((work_mode!=1)&&(work_mode!=2)&&(work_mode!=4)&&(work_mode!=10)&&(work_mode!=18)) Bwork_mode = 0;//不是定义的特定位时发“0”=自由模式
     else Bwork_mode = work_mode;
    }
    else //控制器故障处理
    {
      Bwork_mode = 128;//设置故障位
      spd = 0;         //清空转速数据
      n_trk = 0;       //清空转矩数据
    }

//=============================================================================
//   油门相对速度敏感的感觉计算：ADC_proportion=24#用户参数，油门速度敏感百分比
//   25#=fe_data[9])=VFB增幅的截止频率，fe_data[8]=VFB零速的百分比增幅
//=============================================================================
    //--------------------------//如果是零锁&转矩模式
    if ((n_trk >= 2)&&((Bwork_mode==2)||(Bwork_mode==18))) 
    {
     if ((moto_hzf/fe_data[9])<1)//判断是否在低速增幅方式，由速度决定
        {
         n_trk = (n_trk / (1000.0*ADC_proportion))*1000.0;
         n_trk = n_trk-((n_trk - proportion_trk)*(moto_hzf/fe_data[9]));
         if (n_trk>1000) n_trk=999;
         if (n_trk<2)    n_trk=0;
        }//pi计算--------------------------------------
        vfb_trk = vfb_trk+PI_trk(&vfb_trk, &n_trk, 60);//PID
        if (vfb_trk > 999) vfb_trk = 999;
        if (vfb_trk < 0) vfb_trk = 0;
        p_trk = (unsigned int)(vfb_trk);//电动状态转矩
        //电机过温衰减正转速计算-----------------------
        /*if (adc_samp[3] > (fe_data[12]-10)) 
        {
          spd = 7000;
          low_freq = spd;
          low_freq = low_freq - low_freq*(adc_samp[3] - (fe_data[12]-10))/20.0;//15.0作为衰减频率的系数，看夏天是否合适
          spd = (uint)(low_freq);
          if (adc_samp[3] >= (fe_data[12]+10)) spd = 0;//防止温度超限后，计算出现负转速
        }*/
    }
    //--------------------------// 如果是制动模式
    if ((n_trk >= 2)&&(Bwork_mode==10))
    {   //制动转矩电压补偿计算
        //n_trk = 500;
        if((DCV_previous>500)&&(DCV_previous<610))//欠电压-VFB补偿
        {
          n_trk = n_trk + ((610.0-DCV_previous)*1.5);
          if (n_trk>1000) n_trk=999;
          if (n_trk<2)    n_trk=0;
        }
        if(DCV_previous>640)//过压-VFB衰减计算
        {
          if(DCV_previous>700) n_trk = 0;//电压的VFB限幅处理，保证(DCV_previous/710.0)不大于”1”
          else  n_trk = n_trk - n_trk*(DCV_previous/710.0);
        }
        //------------------------
        /*if ((moto_hzf/fe_data[9])<1)//制动转矩衰减计算
        {
         n_trk = n_trk*(moto_hzf/fe_data[9]);
         if (n_trk>1000) n_trk=999;
         if (n_trk<2)    n_trk=0;
        }*/
        vfb_trk = vfb_trk+PI_trk(&vfb_trk, &n_trk, 60);//PID
        if (vfb_trk > 999) vfb_trk = 999;
        if (vfb_trk < 0) vfb_trk = 0;
        p_trk = (unsigned int)(vfb_trk);
    }
    if ((work_mode!=2)&&(work_mode!=4)&&(work_mode!=10)&&(work_mode!=18)) p_trk = 0;
//------------------------------------------------------------------------------
//----can接收错误处理
    if ((M_err_flg)||(ros_err_flg))
    //if (M_err_flg)
    {
      spd=0;
      p_trk=0;
      n_trk=0;
      vfb_trk = 0;
      Bwork_mode = 0;
      proportion_trk=0;
    }
//------------------------------------------------------------------------------
//  ECU发出工作模式与MITY工作状态判断，无错误逻辑给转矩
//------------------------------------------------------------------------------
    TMity_Status = ((Mity_Status&(~(1<<4))));
    Twork_mode = ((Bwork_mode&(~(255<<8)))&(~(1<<4)));           //清除档位、零锁位;
    if ((Twork_mode==TMity_Status)&&(spd<32767)) p_trk = (uint)(p_trk*low_power);
    if ((Twork_mode==TMity_Status)&&(spd>32768))
    {
      if (Bwork_mode==10)//倒车制动模式转矩为正值
      {
        p_trk = (uint)((p_trk)*low_power);
        p_trk = (uint)(p_trk);
      }
      else
      {
        p_trk = (uint)((p_trk)*low_power);//倒车模式转矩为负值
        p_trk = (uint)(65535 - p_trk);
      }
    }
//------------------------------------------------------------------------------
//  从CAN得到的工作模式与前次得到的工作模式不同，说明模式有变动，清转矩等值
//------------------------------------------------------------------------------
    if (Bwork_mode != previous_workmode) //给定模式与CAN模式不同时清
    {
      p_trk=0;
      n_trk=0;
      vfb_trk = 0;
      proportion_trk=0;
    }
    previous_workmode = Bwork_mode;//记录本次当前模式
    if (n_trk < 2) //转矩给定值小于2时清
    {
      vfb_trk = 0;
      p_trk = 0;
    }
    if (Twork_mode!=TMity_Status) //控制器和CAN工作模式不同清
    {
      p_trk=0;
      n_trk=0;
      vfb_trk = 0;
      proportion_trk=0;
    }
    Bwork_mode |= (handle_position<<8);//置位档位数据
//------------------------------------------------------------------------------
    str_cpy(&dt_buf[0],"0D9",3);
    i2a(spd,&dt_buf[3],4);
    i2a((uint)(p_trk),&dt_buf[7],4);
    i2a(Bwork_mode,&dt_buf[11],4);
//*****************************************************************************
//                             模拟量采集处理--开始                           *
//*****************************************************************************
//--------------
// 电流值处理
//--------------
         DCI_current = adc_data[4] + DC_offset;
         adc_samp[0]=(int)(DCI_current);
         //-------------------------
         adc_samp[0] = (int)((adc_samp[0]+1)/(1023.0/(fe_data[2]*2.0)));             //其中（+1）是为了得到1024变800后显示范围是0~800共801个状态
      	 //if (adc_samp[0] < fe_data[2]) adc_samp[0] = 1000+(fe_data[2]-adc_samp[0]);  //大于1000为负数
      	 //else adc_samp[0] = adc_samp[0]-fe_data[2];                                  //小于1000为正数
                                                                                     //带偏移量的电流数据  -200=0; +200=1024;32,400
         if (adc_samp[0] < fe_data[2])
         {
           dc_A = fe_data[9]-adc_samp[0];
           adc_samp[0] = 1000+(fe_data[2]-adc_samp[0]);  //大于1000为负数
         }
      	 else 
         {
           adc_samp[0] = adc_samp[0]-fe_data[2];                           //小于1000为正数
           dc_A = adc_samp[0];
         }
         //带偏移量的电流数据  -200=0; +200=1024;32,400
         if (adc_samp[0]>999) DC_C= (uint)((32000-(adc_samp[0]-1000)/0.05));
         else DC_C=(uint)(32000+(adc_samp[0]/0.05));
         //-------------------------
         can_TXdata[1][4] = (uchar)(DC_C);
         can_TXdata[1][5] = (uchar)(DC_C >> 8);
//-------------
// 电压值处理
//-------------
         //adc_samp[1] = adc_data[5];
         DCV_current = adc_data[5];
         PI_filter(&DCV_previous,&DCV_current,&DCV_proportional);
         adc_samp[1]=(int)(DCV_previous);
         //-------------------------
         adc_samp[1] = (int)((adc_samp[1]+1)/(1023.0/fe_data[3]));             //(+1)原因同电流算法
         
         can_TXdata[1][3] = (uchar)(adc_samp[1] >> 8);
         can_TXdata[1][2] = (uchar)(adc_samp[1]);
         
         //------电压低于量程的1/4时，不加入偏置补偿
         Temporary_variables_int16 = (int)(fe_data[3] / 4);
         if (adc_samp[1] > Temporary_variables_int16) 
         {
             adc_samp[1] = adc_samp[1] + voltage_bias;
             if(run_Flg == 1) //run_Flg,启动程序后只会运行一次的标志，=1会运行，一次后赋值=0就不再运行
             {
                 
                 run_Flg = 0;  
                 stop_flg = 0; //停止运行标志,=0不停，=1停止运行QMCL程序
                 Reset_Flg = 1;
             }    
         }    
//----------------
// 控制器温度处理
//----------------
        adc_samp[2]=adc_data[2];
        fe_data_tmp = (uint)(fe_data[6]);
        if (fe_data[7]==5)   adc_samp[2] = temp_5k(adc_samp[2]);
        if (fe_data[7]==10)  adc_samp[2] = temp_10k(adc_samp[2]);               //温度电阻型号判断5k ？10k？
        if (adc_samp[2]>125) adc_samp[2]=125;                                   //限制温度不高于"125"
        if (adc_samp[2]<(-40)) adc_samp[2]=-40;                                 //限制温度不低于"-30"
        if (fe_data[6] !=0)
        {
          if (adc_samp[2]>=fe_data_tmp)
             {
               //PORTB &=~(1<<PB2); //控制器超温 PORTB=PB2=0
               Mity_temperature_limit_1;
             }
          if (adc_samp[2]<=(fe_data_tmp-5))
             {
               //PORTB |=(1<<PB2); //控制器正常 PORTB=PB2=1
               Mity_temperature_limit_0;
             }
        }
        //else PORTB |=(1<<PB2);
        can_TXdata[1][0] = 40 + adc_samp[2];//根据1939协议，1℃/bit , -40 offset；-40 to 210°C；int转uchar、uint为补码，修正offset、
        
//---------------
// 电机温度处理
//---------------
        moto_current = adc_data[3];
        fe_data_tmp = (uint)(fe_data[5]);
        PI_filter(&moto_previous,&moto_current,&moto_proportional);
        adc_samp[3]=(int)(moto_previous);
        adc_samp[3] = pt100_bridge(adc_samp[3]);
        if (adc_samp[3]>200) adc_samp[3]=200;
        if (fe_data[5] !=0)
        {
          if (adc_samp[3]>=fe_data_tmp)
             {
               //PORTB &=~(1<<PB1); //电机超温 PORTB=PB1=0
               Motor_temperature_limit_1;
             }
          if (adc_samp[3]<=(fe_data_tmp-10))
             {
               //PORTB |=(1<<PB1); //电机正常 PORTCB=PB1=1
               Motor_temperature_limit_0;
             }
        }
        //else  PORTB |=(1<<PB1);
        can_TXdata[1][1] = 40 + adc_samp[3];//根据1939协议，1℃/bit , -40 offset；-40 to 210°C；int转uchar、uint为补码，修正offset、
        //------------------------------
        //电机、控制器，超温限功率处理
        //------------------------------
        if ((can_TXdata[0][6]&(1<<0))||(can_TXdata[0][6]&(1<<1))) low_power = 0.6;
        else low_power = 1;

//-------------------------------------
// 模拟量采集处理--完成
//-------------------------------------
//==============================================================================
//                           adc数据整理完成
//==============================================================================
    //str_cpy(&dt_buf[15],"1111",4);
    i2a(test_ok ,   &dt_buf[15],4);
    dt_buf[19] = 13;
    
    
    //i2a(adc_samp[0],&dt_buf[19],4);
    //i2a(adc_samp[1],&dt_buf[23],4);
    //i2a(adc_samp[2],&dt_buf[27],4);
    //i2a(adc_samp[3],&dt_buf[31],4);
    //dt_buf[35] = 13;
    test_ok++;//发送到user#111用于通讯数据错误判断及恢复通讯
//-------------------------------------
// 采集的数据整理为块发送数据，等待发送
//-------------------------------------
//*********************************************************************************
//USART_transmit_state=0块发送状态, =1块读取时的发送状态, =2块读取时的接收等待状态*
// =3单字节mity错误码读取发送状态,  =4单字节mity错误码读取接收等待状态            *
// =5单字节mity设定参数发送状态,    =6单字节mity设定参数读取接收等待状态          *
// =7 复位指令发送状态
//*********************************************************************************
//块发送开始-----------------------------------------
      if ((USART_transmit_ok==1)&&(USART_transmit_state==0))
      {
          __delay_ms(1);
          for(i=0; i<20; i++) USART_transmit_buff[i]=dt_buf[i];
          if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              USART_transmit_state=7;//过流错误码发送状态
              U1TXREG = USART_transmit_buff[0];
          }
      }
//*********************************************************************************
//                      串口发送QMCL程序运行指令
//*********************************************************************************
       //Reset_Flg = 1;
       if ((Reset_Flg == 1)&&(USART_transmit_state==7))
       {
           if (USART_transmit_ok==1)
           {
               if (stop_flg == 0)
               {// 串口发送运行指令
                   __delay_ms(10);
                    str_cpy(fe_rd,"1FE",3);//1fe<-
                    fe_rd[3] = 13;
                    for(i=0; i<12; i++) USART_transmit_buff[i]=fe_rd[i];
                    if (U1STAbits.TRMT  == 1)         //发送器空
                        {
                            Reset_Flg = 0;                 //过流错误码发送完成，清标志
                            USART_transmit_ok=0;
                            USART_transmit_state = 5;    //设置接收状态标志
                            U1TXREG = USART_transmit_buff[0];
                        }    
                }
                else
                {//串口发送停止运行指令
                    __delay_ms(10);
                    str_cpy(fe_rd,"1FF",3);                //1ff停止运行指令
                    fe_rd[3] = 13;
                    for(i=0; i<12; i++) USART_transmit_buff[i]=fe_rd[i];
                    if (U1STAbits.TRMT  == 1)               //发送器空
                        {
                            Reset_Flg = 0;                 //过流错误码发送完成，清标志
                            stop_flg =  0;                 //清除停止运行标志
                            USART_transmit_ok=0;
                            USART_transmit_state = 5;    //设置接收状态标志
                            U1TXREG = USART_transmit_buff[0];
                        }    
                }    
            }
       }     
       else if (USART_transmit_state==7) USART_transmit_state = 5;//设置接收状态标志

//*********************************************************************************
//                     过流错误码发送  【1DDE16555】
//*********************************************************************************
       if ((alm_flg == 1)&&(USART_transmit_state==5))
       {
           if (USART_transmit_ok==1)
           {
               __delay_ms(50);
               str_cpy(fe_rd,"0DDE16555",9);
               //str_cpy(fe_rd,"1DDFE0055",9);
               fe_rd[9] = 13;
               for(i=0; i<12; i++) USART_transmit_buff[i]=fe_rd[i];
               if (U1STAbits.TRMT  == 1)         //发送器空
               {
                   alm_flg = 0;                 //过流错误码发送完成，清标志
                   USART_transmit_ok=0;
                   USART_transmit_state = 3;    //设置接收状态标志
                   U1TXREG = USART_transmit_buff[0];
               }    
           }    
       }
       else
       {
           if (USART_transmit_state==5) USART_transmit_state = 3;//设置接收状态标志
       }
       
//*********************************************************************************
//                         旋变硬件报警判断及复位段
//  "PORTCbits.RC14"从低电平到高电平的变化输出有效
//*********************************************************************************
    if ((PORTAbits.RA14 == 0)&&(PORTAbits.RA15 == 0))//双路报警认为是正式报警,累计一次报警值
    {
        if (ros_err_flg ==0) ros_index++;
    }    
    //----------------------------------
    //旋变报警后复位超限"n"=45次后，上报CAN 旋变故障
    if (ros_index > 45)
    {
      resolver_1;       //置位旋变报警
      ros_index = 0;
      ros_err_flg = 1;
      stop_flg = 1;     //设置停止运行标志
      Reset_Flg = 1;    //设置复位标志,有旋变报警错误后，停止QMCL程序运行
    }    
    //----------------------------------
    if (PORTAbits.RA14 == 0)//任何单路报警，只复位旋变
    {
        PORTCbits.RC14 = !PORTCbits.RC14;//取反
        __delay_ms(5);
    }    
    if (PORTAbits.RA15 == 0)//任何单路报警，只复位旋变
    {
        PORTCbits.RC14 = !PORTCbits.RC14;//取反
        __delay_ms(5);
    } 

//*********************************************************************************
//                 主接触器吸合,控制器准备好的I/O端口判断
//
//*********************************************************************************
//接触器吸合--------------------------------------------------------
    if (PORTBbits.RB14 == 0) 
    {
        contactor_closed_1;  //1:接通； 0：断开  C0D6
    }    
    else  contactor_closed_0;
    
//控制器准备好？可以运行？-------------------------------------------
    if (PORTBbits.RB15 == 0)                     //C0D7
    {
      
      powerstage_ready_1;                                                       //1：准备好  0：未准备好
      //------清除控制器故障正常后的报警码
      for(i=0; i<3; i++) dt_err[i] = 0XFF;     //清除串口通信读故障信息
      overcurrent_A_0;                                                  //0:无过流
      overvoltage_0;
      //----------------------------------
      if (USART_transmit_state==3) USART_transmit_state=1;//USART_transmit_state=1使能块读取时的发送状态
      //不做判断直接=1会在程序结构上产生错误，每次循环到这里都设置=1，从而影响其他设置
    }
    //----------------------控制器故障没有准备好-----------------------------------------
    else
    {
        powerstage_ready_0;
        //USART_transmit_state=1;
            //---------------------串口通信读故障信息-------------------------------------
            if ((USART_transmit_ok==1)&&(USART_transmit_state==3))
            {
                __delay_ms(5);
                str_cpy(fe_rd,"0DCE165",7);                                               //通讯查询CPU故障历史记录
                //str_cpy(fe_rd,"0DCF0E9",7);
                fe_rd[7] = 13;
                for(i=0; i<9; i++) USART_transmit_buff[i]=fe_rd[i];
                if (U1STAbits.TRMT  == 1)         //发送器空
                {
                USART_transmit_ok=0;
                USART_transmit_state=4;   //USART_transmit_state=4单字节mity错误码读取接收等待状态
                USART_receive_ok=0;
                U1TXREG = USART_transmit_buff[0];
                } 
            }
    }
    if ((USART_receive_ok==1)&&(USART_transmit_state==4))
      {
        USART_receive_ok=0;
        USART_transmit_state=1;//使能块读取时的发送状态
        for(i=0; i<6; i++) dt_err[i] = USART_receive_buff[i];
        a2i(&dt_err[1], 4);//dt_err[2]=故障码
      }
    if ((dt_err[2] == 0) || (dt_err[2] == 1))                               //1:过流
       {
              overcurrent_A_1;
              //powerstage_fault_1;                                               //1：故障
       }
       if (dt_err[2] == 2) 
          {
            overvoltage_1;
            //powerstage_fault_1;                                                 //1：故障
          }

//*********************************************************************************
//                          块接收的发送状态
//USART_transmit_state=0块发送状态, =1块读取时的发送状态, =2块读取时的接收等待状态*
// =3单字节mity错误码读取发送状态,  =4单字节mity错误码读取接收等待状态            *
// =5单字节mity设定参数发送状态,    =6单字节mity设定参数读取接收等待状态          *
//*********************************************************************************
      if ((USART_transmit_ok==1)&&(USART_transmit_state==1))
       {
         __delay_ms(1);
         str_cpy(fe_rd,"0D8",3);
         fe_rd[3] = 13;
         for(i=0; i<5; i++) USART_transmit_buff[i]=fe_rd[i];
         if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              USART_transmit_state=2;//设置接收状态标志
              U1TXREG = USART_transmit_buff[0];
              //__delay_ms(25);
              //PORTCbits.RC13  = !PORTCbits.RC13;
          }
       }   
//*********************************************************************************
//                          块接收状态
//USART_transmit_state=0块发送状态, =1块读取时的发送状态, =2块读取时的接收等待状态*
// =3单字节mity错误码读取发送状态,  =4单字节mity错误码读取接收等待状态            *
// =5单字节mity设定参数发送状态,    =6单字节mity设定参数读取接收等待状态          *
// =10为单字0FDFEXXnnnn 地址发送状态
//*********************************************************************************
       if ((USART_receive_ok==1)&&(USART_transmit_state==2))
       {
           loop=0;                //有串口数据接收清标志
           USART_receive_ok=0;
           USART_transmit_state=10;//设置为单字节地址发送状态
           for(i=0; i<35; i++) dt_com[i] = USART_receive_buff[i];
           
//==============================================================================
//        dt_com[1]电机转速数据、转换后放 can_data[1][0-1]
//==============================================================================
      a2i(&dt_com[1], 4) ;
      fct = (dt_com[1] << 8) + dt_com[2] ;         //HZP=100f=10n/2,(8极:n=60f/P=60f/4=20f,f=n/60*4)
      if(fct>32767)                                // 负速度值
      {
         moto_hzf = (65535-fct)/10.00;
         actual_rpm=65535 - fct;
         actual_rpm = -(60*actual_rpm)/(moto_pole*100);      // n=1.0/5*HZP
         //--------------------------------------------------
         mity_rpm = -actual_rpm;                   //频率换算速度后取绝对值，用于基频以上恒功率的转矩换算
         actual_rpm=(16384.0+actual_rpm)/0.5;
      }
      else                                         // 正速度值
      {
         moto_hzf = fct/10.00;
         actual_rpm=fct;
         actual_rpm=60*actual_rpm/(moto_pole*100); // n=1/5*HZP
         //--------------------------------------------------
         mity_rpm = actual_rpm;                    //频率换算速度后取绝对值，用于基频以上恒功率的转矩换算
         actual_rpm=(16384.0+actual_rpm)/0.5;
      }
      fct=(uint)(actual_rpm);
      can_TXdata[0][1] = fct >> 8;
      can_TXdata[0][0] = fct ;
      
//==============================================================================
//        dt_com[5]电机转矩数据、转换后放 can_data[1][2-3]
//==============================================================================
      a2i(&dt_com[5], 4);
      fct = (dt_com[5] << 8) + dt_com[6] ;         //HZP=100f=10n/2,(8极:n=60f/P=60f/4=20f,f=n/60*4)

          if (fct >32767) fct = 65535 - fct;//正负转矩取绝对值
          if (fct< 1 ) fct = 0;
          can_TXdata[0][3] = (fct >> 8);
          can_TXdata[0][2] = fct ;
//==============================================================================
//        dt_com[9]电机交流电流、转换后放 can_data[1][20-1]
//==============================================================================
      a2i(&dt_com[9], 4);
      fct = (dt_com[9] << 8) + dt_com[10] ;
      fct = fct/10;
      fct = (uint)(32000+fct/0.05);
      can_TXdata[1][7] = fct >> 8;
      can_TXdata[1][6] = fct - (can_TXdata[1][7] << 8);
      
//==============================================================================
//        dt_com[13]控制器当前工作模式、转换后放 can_data[1][20-1]
//==============================================================================
      a2i(&dt_com[13], 4);
      fct = (dt_com[13] << 8) + dt_com[14] ;
      Mity_Status = fct;
      can_TXdata[0][4] = Mity_Status;
//==============================================================================
//-------功率转扭矩  --  dt_com[29]   取得PID 的 P值
//==============================================================================
//------取得PID 的 P值----------------
        a2i(&dt_com[29], 4);
        fct = (dt_com[29] << 8) + dt_com[30] ;
        PID_P = fct;
        if (PID_P == 444)
        {
          while(1);
        }
           //----------------------------------
           //CAN 发送段
           //----------------------------------
            Attack_C1TX0();
            Attack_C1TX1();
            /*
            can_TXdata[2][0] = test_ok;
            can_TXdata[2][1] = test_ok;
            can_TXdata[2][2] = test_ok;
            can_TXdata[2][3] = test_ok;
            can_TXdata[2][4] = test_ok;
            can_TXdata[2][5] = test_ok;
            can_TXdata[2][6] = test_ok;
            can_TXdata[2][7] = test_ok;
            test_ok++;
            Attack_C1TX2(0xa0a0a0a);*/
       }
//*********************************************************************************
//                          单地址字发送状态 = 10
//USART_transmit_state=0块发送状态, =1块读取时的发送状态, =2块读取时的接收等待状态*
// =3单字节mity错误码读取发送状态,  =4单字节mity错误码读取接收等待状态            *
// =5单字节mity设定参数发送状态,    =6单字节mity设定参数读取接收等待状态          *
// =10为单字0FDFEXXnnnn 地址发送状态
//*********************************************************************************
       if ((USART_transmit_ok==1)&&(USART_transmit_state==10))
      {
          if (WordTranIndex == 0)
          {
              str_cpy(&DataTranWord[0],"0DF",3);
              str_cpy(&DataTranWord[3],"FEE0",4);
              i2a(adc_samp[0],&DataTranWord[7],4);
              DataTranWord[11] = 13;
          }
          if (WordTranIndex == 1)
          {
              str_cpy(&DataTranWord[0],"0DF",3);
              str_cpy(&DataTranWord[3],"FEE2",4);
              i2a(adc_samp[1],&DataTranWord[7],4);
              DataTranWord[11] = 13;
          }
          if (WordTranIndex == 2)
          {
              str_cpy(&DataTranWord[0],"0DF",3);
              str_cpy(&DataTranWord[3],"FEE4",4);
              i2a(adc_samp[2],&DataTranWord[7],4);
              DataTranWord[11] = 13;
          }
          if (WordTranIndex == 3)
          {
              str_cpy(&DataTranWord[0],"0DF",3);
              str_cpy(&DataTranWord[3],"FEE6",4);
              i2a(adc_samp[3],&DataTranWord[7],4);
              DataTranWord[11] = 13;
          }
          WordTranIndex++;
          if (WordTranIndex >=4 ) WordTranIndex = 0;
          __delay_ms(1);
          for(i=0; i<12; i++) USART_transmit_buff[i]=DataTranWord[i];
          if (U1STAbits.TRMT  == 1)//发送器空
          {
              USART_transmit_ok = 0;
              USART_transmit_state=0;//块发送状态
              U1TXREG = USART_transmit_buff[0];
          }
      }

//--------------------------------------------------------------------------
//  CAN 模块收、发错误处理
//--------------------------------------------------------------------------
        if ((C1INTFbits.TXBO==1)||(C1INTFbits.TXBP==1)||(C1INTFbits.RXEP))
        {
            //IEC0bits.T1IE  =  0;
            //PORTFbits.RF8 = !PORTFbits.RF8;//取反工作灯
            can_init_C1();//接收错误被动复位
            //IEC0bits.T1IE  =  1;
        }
//--------------------------------------------------------------------------
          loop++;
          if (loop>500) //0.5S无串口数据接收判断
          {
              loop=0;
              Usart_Init();
              USART_transmit_state=0;//接收超时，清标志、使能块发送
              USART_transmit_ok=1;
          }
   }
}

