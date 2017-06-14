//*********************************************************
//*       中断处理函数文件，2011-7-13，zhangjt            *
//*  定时器中断、adc中断、USART1发送中断、USART1接收中断  *
//*  CAN通讯中断                                          *
//*********************************************************
#include "p30f6010A.h"  //dsPIC30F6010A标准头文件
#include "math.h"
#include "job.h"
//#include "USARTinit.h"
//*********************************************************
//           定时器中断处理函数,7.3728M/8X                *
//  3400≈230us,4300≈300us,16384=1.2ms,65534=2.2ms;        *
//  16m晶振后，PR1=4300=270us,160=10um,320=20um,640=40um，*
//  4800=300us,此中断设置为固定定时触发，自动重复方式     *
//*********************************************************
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    DISICNT = 0x3FFF;
    IFS0bits.T1IF = 0;      /* Clear Timer interrupt flag */	
    ADCON1bits.SAMP = 1;    //触发采样开始(300us触发一次采样，然后自动6次采用,单次采样5us)

    //---------------------------------------------
    //  旋变定时计数清除
    if (ros_err_flg == 0)ros_delay++;
    if (ros_delay>8000) //2.4秒清旋变报警计数值，2.4秒内累计超45次，会至位旋变报警位
    {
        ros_index = 0;
        ros_delay = 0;
    }
    //---------------------------------------------
    //can接收超时错误判断
    delay_error++;
    if (delay_error>8000)//8000*300us=2.4秒，无接收数据至位标志“M_err_flg”
    {
        M_err_flg=1;  //接收超时错误标志
        PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
        delay_error=6000;
    }    
    //---------------------------------------------
    //  控制器温升变化率报警
    //if (ClockDelayOne<10) EvmcTemperature = can_TXdata[1][0];
    //if (ClockDelayOne>30000)
    //{
        //ClockDelayOne = 0;
        //if ((can_TXdata[1][0]-EvmcTemperature)>2) ;
    //}    
    //ClockDelayOne++;
 DISICNT = 0x0000;
}	

//*************************************************************
//                  adc中断处理函数                           *
//  由TIMER1中断触发ADC采样，每次中断触发一次，6次引发ADC中断 *
//*************************************************************
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt(void)
{
    DISICNT = 0x3FFF;
    //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
    IFS0bits.ADIF = 0;//清AD中断标志位
    ADCON1bits.DONE = 0;
    //adc_index++;
    //if (adc_index==5)
    //{
        adc_index=0;
        adc_data[6] = ADCBUF0;
        adc_data[0] = ADCBUF1;
        adc_data[1] = ADCBUF2;
        adc_data[2] = ADCBUF3;
        adc_data[3] = ADCBUF4;
        adc_data[4] = ADCBUF5;
        adc_data[5] = ADCBUF6;
        //adc_data[6] = ADCBUF6;
        //adc_data[7] = ADCBUF7;
        //DISICNT = 0x0000;
        //return;
    //}    
    DISICNT = 0x0000;
    //ADCON1bits.SAMP = 1;
 return ;
}

//*********************************************************
//                USART1发送中断处理函数                  *
//*********************************************************
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) 
{
  DISICNT = 0x3FFF;
  IFS0bits.U1TXIF=0;          //清通讯发送中断标志
  //-----------------------
  while(U1STAbits.UTXBF){};
  if (USART_transmit_ok==0)
     {
       U1TXREG = USART_transmit_buff[USART_transmit_index];
       USART_transmit_index++;
     } 
  if (USART_transmit_buff[USART_transmit_index-1]==13)
     {
       USART_transmit_ok=1;
       USART_transmit_index=1;//定位发送数值到第二个数据，第一个数据用于触发发送中断
     }
  DISICNT = 0x0000;
}

//*********************************************************
//                USART1接收中断处理函数                  *
//*********************************************************
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
  uint u_i=0;
  DISICNT = 0x3FFF;
  IFS0bits.U1RXIF=0;          //清通讯发送中断标志
  //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
  //------------------------------------------------
  //收到开始字节符号“#”的判断
  //------------------------------------------------
  USART_receive[USART_receive_index]=U1RXREG;
  if (USART_receive[USART_receive_index]==35) 
  {
      USART_receive_index = 0;
      USART_receive[0] = 35;
  }    
    //-------------------------------------------------
    // 有开始字符“#”后，开始接收数据
    //------------------------------------------------
     if (USART_receive[0]==35) USART_receive_index++;
     if (USART_receive_index>34)//接收数据格式超长丢弃数据
        {
          USART_receive_index=0;
          USART_receive[0]=0;
          USART_receive_ok=0;
        }
     else//没有超长？
     {
     //------------------------------------------------
     //收到数据结束符“回车”，并且转储数据允许，转储数据
     //------------------------------------------------
       if ((USART_receive[USART_receive_index-1]==13)&&(USART_receive_ok==0))
          {
           for(u_i=0; u_i<34; u_i++) USART_receive_buff[u_i]=USART_receive[u_i];
           USART_receive_index=0;
           USART_receive[0]=0;
           USART_receive_ok=1;
          }
     }
     //------------------------------------------------
     //收到数据结束符“回车”，转储数据不允许，丢弃数据
     //------------------------------------------------
     if ((USART_receive[USART_receive_index-1]==13)&&(USART_receive_ok==1))
     {
       USART_receive_index=0;//清数据索引
       USART_receive[0]=0;   //清数据接收开始标志符号
     }

  DISICNT = 0x0000;
}

//*********************************************************
//               CAN C1组合中断处理函数                   *
//*********************************************************
// 应该是接收、发送都相应，由CPU部分的中断向量处理
// 具体由中断程序进行判断是发送还是接收中断
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{
    //uchar i = 0, num_i;
    DISICNT = 0x3FFF;
    //--------can的1秒内有信号处理-------
    if (M_err_flg==1) canok_delay++;
    if (canok_delay>200)//在can链路报错后，连续接收到200个以上的can数据，说明can通讯正常
    {
        M_err_flg=0;
        canok_delay=0;
    }
        delay_error=0;                      //CAN数据接收超时计数器清零

        //C1CTRLbits.ABAT = 1;                         //停止当前发送
        //if (C1INTFbits.IVRIF==1) C1INTFbits.IVRIF=0;    //接收到无效报文的中断标志位
        //if (C1INTFbits.RXWAR==1) C1INTFbits.RXWAR=0;    //接收器处于错误状态，警告位
        //if (C1INTFbits.TXWAR==1) C1INTFbits.TXWAR=0;    //发送器处于错误状态，警告位
        //if (C1INTFbits.EWARN==1) C1INTFbits.EWARN=0;    //发送器或接收器处于错误状态，警告位
        //if (C1INTFbits.RX1OVR==1)C1INTFbits.RX1OVR=0;   //接收缓冲器 RXB1溢出位
        //if (C1INTFbits.RX0OVR==1)C1INTFbits.RX0OVR=0;   //接收缓冲器 RXB0溢出位
        //if (C1INTFbits.ERRIF==1) C1INTFbits.ERRIF=0;    //错误中断标志位 （CiINTF<15:8> 寄存器中的多种中断源）
//-------------------------------发送中断处理部分-----------------------------
        //-----------------------
        //发送缓冲器TXB2中断标志位
        if (C1INTFbits.TX2IF ==1)
        {
            C1INTFbits.TX2IF = 0;//清发送标志
            IFS1bits.C1IF = 0;   //清组合标志
            //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
        }    
        //-----------------------
        //发送缓冲器1中断标志位
        if (C1INTFbits.TX1IF ==1)
        {
            C1INTFbits.TX1IF = 0;//清发送标志
            IFS1bits.C1IF = 0;   //清组合标志
            //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
        }        
        //-----------------------
        //发送缓冲器0中断标志位
        if (C1INTFbits.TX0IF ==1)
        {
            C1INTFbits.TX0IF = 0;//清发送标志
            IFS1bits.C1IF = 0;   //清组合标志
            //C1TX0CONbits.TXREQ = 0;                         //停止当前发送
            //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
        }    
//-------------------------------接收中断处理部分-----------------------------
        //-----------------------
        //接收缓冲器1 中断标志位
        if (C1INTFbits.RX1IF == 1)
        {
            can_RXdata[1][0]= C1RX1B1;
            can_RXdata[1][1]= C1RX1B2;
            can_RXdata[1][2]= C1RX1B3;
            can_RXdata[1][3]= C1RX1B4;
            C1RX1CONbits.RXFUL = 0;//清除接收寄存器满标志
            C1INTFbits.RX1IF = 0;  //清除接收缓冲器1标志
            IFS1bits.C1IF = 0;     //清组合标志
            //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
        }    
        //-----------------------
        //接收缓冲器0 中断标志位
        if (C1INTFbits.RX0IF == 1)
        {
            can_RXdata[0][0]= C1RX0B1;
            can_RXdata[0][1]= C1RX0B2;
            can_RXdata[0][2]= C1RX0B3;
            can_RXdata[0][3]= C1RX0B4;
            C1RX0CONbits.RXFUL = 0;//清除接收寄存器满标志
            C1INTFbits.RX0IF = 0;  //清除接收缓冲器0标志
            IFS1bits.C1IF = 0;     //清组合标志
            //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
            //---接收好的数据交换给can_data_buff--
            if (can_data_ok==1)    //can_data_ok=1  数据已取出，允许放数据到缓存
            {
                can_data_buff[0]=can_RXdata[0][0];    //8个字节的数据放入缓存
                can_data_buff[1]=can_RXdata[0][0]>>8; //8个字节的数据放入缓存  
                can_data_buff[2]=can_RXdata[0][1];    //8个字节的数据放入缓存
                can_data_buff[3]=can_RXdata[0][1]>>8; //8个字节的数据放入缓存  
                can_data_buff[4]=can_RXdata[0][2];    //8个字节的数据放入缓存
                can_data_buff[5]=can_RXdata[0][2]>>8; //8个字节的数据放入缓存  
                can_data_buff[6]=can_RXdata[0][3];    //8个字节的数据放入缓存
                can_data_buff[7]=can_RXdata[0][3]>>8; //8个字节的数据放入缓存   
                can_data_ok=0;  //can数据准备好标志
            }
        }
            
    DISICNT = 0x0000;
}    

