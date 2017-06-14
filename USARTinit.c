//==============================================================
// 38400bps
// 串口通讯、CAN通讯初始化; 数值转换计算
// UART_init,(UART1=06CPU通讯，UART2=外部设置使用)
//==============================================================
#include "p30f6010A.h"  //dsPIC30F6010A标准头文件
#include "math.h"
#include "job.h"

void Usart_Init(void)
{   //USART1
    DISICNT = 0x3FFF;
    //UxBRG波特率设置***********
    U1BRG = 25;//7.3728=38400=23;16mhz=38400=25;19200=51;
    
    //UxMODE工作模式设置********
    
    //U1MODEbits.USIDL=0;     //空闲模式禁止
	//U1MODEbits.ALTIO = 1;   //使用备用I/O口作为通讯端口1-44pin转倒32-35pin
	//U1MODEbits.WALE = 0;    //休眠模式唤醒，=1休眠模式唤醒，=0不唤醒
    //U1MODEbits.LPBACK=0;    //还回模式使能，=1使能，=0禁止
    //U1MODEbits.ABAUD =0;    //自动波特率使能，=0由ICx引脚入，=1由UxRX入
	U1MODEbits.PDSEL=1;       //8位数据，0=8位数无奇偶校验,1=8位偶校验，2=8位奇校验，3=9位无校验
    U1MODEbits.STSEL=0 ;      //0=1个停止位,1=2个停止位

    //UxSTA状态位设置***********
    U1STAbits.UTXISEL=1;      //发送中断的方式选择，=1数据传到发送寄存器，缓冲器空产生，=0数据传发送器，缓冲还要至少一个数据时产生
    U1STAbits.UTXBRK =0;      //终止发送选择，=1不管什么状态，拉低UxTX脚，停止发送，=0正常发送
    IFS0bits.U1TXIF  =0;      //清通讯发送中断标志，可用于轮询判断
    //U1STABITS.UTXBF = 0？;
    //U1STABITS.TRMT  =? ;    //(发送移位器)空的判断位，=1发送器空，缓冲器空;=0发生器非空，或缓冲器有排队
    U1STAbits.URXISEL=0;      //接收中断的方式选择，11 =接收缓冲器满产生，0x=接收到一个字符时产生
    U1STAbits.ADDEN  =0;
    //U1STABITS.RIDLE  =？;
    //U1STABITS.PERR   =？;
    //U1STABITS.FERR   =？;
    //U1STABITS.OERR   =？;
    //U1STABITS.URXDA  =？;
    
    //中断使能设置**************
    IEC0bits.U1TXIE=1;      //发送中断使能 =1 使能; =0 禁止
    IFS0bits.U1TXIF=0;      //清通讯发送中断标志，可用于轮询判断
    IEC0bits.U1RXIE=1;      //接收中断使能 =1 使能; =0 禁止
	IFS0bits.U1RXIF=0;      //清通讯接收中断标志，可用于轮询判断
    //中断优先级设置************
    IPC2bits.U1TXIP = 4;
    IPC2bits.U1RXIP = 4;
    
    //使能UART模块和使能发送****
    U1MODEbits.UARTEN = 1;  //使能usart1通讯端口,=0禁止usart1端口
    U1STAbits.UTXEN   = 1;  //发送使能位，=1使能，=0禁止，IO状态由PORT决定
//-----------------------------------------
    //USART2
	U2BRG = 25;
	//U2MODEbits.ALTIO = 1; //使用备用I/O口作为通讯端口1-44pin转倒32-35pin
	U2MODEbits.UARTEN = 1;  //使能usart2通讯端口
	U2MODEbits.PDSEL=0;     //8位数据，无奇偶校验
	U2MODEbits.STSEL=0 ;    //1个停止位
	U2MODEbits.USIDL=0;     //空闲模式禁止
	U2STAbits.UTXEN = 1;    //使能发送
	U2STAbits.UTXISEL=1;    //发送中断模式选择位
	IFS1bits.U2TXIF=0;
    IFS1bits.U2RXIF=0;
    IEC1bits.U2TXIE=0;
    IEC1bits.U2RXIE=0;
    DISICNT = 0x0000;
    for(i = 0; i < 4160; i++)Nop();//设定完成要等到最少104usec
      
}

//==============================================================
//        i2a 函数
//  返回非0位数； n待变换整数 pd字符数值 m期望字符位数
//  数组中从后面放置转换完的数值，前面清空
//==============================================================
unsigned int i2a(unsigned int n, unsigned char *pd, unsigned int m)
{
  unsigned int j;
  int i;

//转换为十六进制
  for (i = m; i > 0; i--)
  {
    j = n & 0x000f;
    if (j > 9) j += 55;
    else j += 48;
    pd[i-1] = j;
    n >>= 4;
  }
  j = m;
  for (i = 0; i < m; i++)
  {
    if (pd[i] == 0x30)
    {
      j--;
      pd[i] = 0x20;
    }
    else i = 99;
  }

  return (j);                             /* 返回字符串长度 */
}
//==============================================================
//        a2i 函数
// pd字符数值 m字符位数,结果放在数组的前2个字节中
//==============================================================
void a2i(unsigned char *pd, unsigned int m)
{
  unsigned int k;

  for (k = 0; k < m; k++)
  {
    if (*(pd + k) > 57) *(pd + k) -= 55;         //数据去除ASC部分
    else *(pd + k) -= 48;
  }
  *pd = (*pd) * 16 + *(pd + 1);
  *(pd + 1) = *(pd + 2) * 16 + *(pd + 3);
}
//==============================================================
//       串拷贝函数
//==============================================================
void str_cpy(uchar *ptr1, uchar *ptr0, unsigned int m)
{
  uint i;
  for (i = 0; i < m; i++)
  {
    *(ptr1 + i) = *(ptr0 + i);
  }
}


uint temp_10k(uint x)
{
  double ad;

  if (x==0) x=1;
  ad = (double)(1024 - x);
  if (ad < 0) ad = 1.0;
  ad = (double)x / ad;
  ad /= 10;
  ad = log(ad) / 3942;        //负温度系数"B" 官方：b=4050，3950，实际补偿为：3942;
  ad += 1/(double)298;
  ad = 1/ad;

  return( (uint)ad - 273);
}
uint temp_5k(uint x)//新100kw控制器的模块自带NTC电阻，25°C时的阻值为5K，官方B值=3375
{
  double ad;

  if (x==0) x=1;
  ad = (double)(1024 - x);
  if (ad < 0) ad = 1.0;
  ad = (double)x / ad;
  ad /= 5;
  ad = log(ad) / 3375;        //负温度系数"B" 官方：b=3375;
  ad += 1/(double)298;
  ad = 1/ad;                  //当AD=1023=5V时，所得值最小约=221；

  return( (uint)ad - 273);    //221-273=负值；反馈回去uint值=65535-所得值=最低约65483 的数值。
}                             //这里为补码方式，注意主程序低温过限的判断

/*--------------TPC-正温度热敏电阻温度测量计算---------------
因电阻非线性，采取分段测量 TK-20，TK-5，TK+5，TK+15
-----------------------------------------------------------*/
uint tpc_temp(uint _x)
{
  uint ad;
  if (_x<55)  ad=20;                                                           //常温
  if ((_x>55)&(_x<=130)) ad=(uint)(20+(_x-55)*1.26);                                   //TK-20
  if ((_x>130)&(_x<=217)) ad=(uint)(115+(_x-130)*0.17);                                //TK-5
  if ((_x>217)&(_x<=340)) ad=(uint)(130+(_x-217)*0.08);                                //TK+5
  if (_x>340) ad=(uint)(140+(_x-340)*0.07);                                            //TK+15
  return (ad);
}

//-----------------------------------------------------------|
//                  pt100热敏电阻计算                        |
//-----------------------------------------------------------|
uint pt100_temp(uint ad_t3)
{
  uint ad;
  if (ad_t3<170)ad=0;
  if ((ad_t3>169)&&(ad_t3<220)) ad=(uint)(((ad_t3-170)*2.3)*0.9);
  if ((ad_t3>219)&&(ad_t3<227)) ad=(uint)(100+((ad_t3-220)*1.4));
  if (ad_t3>226) ad=(uint)(110+((ad_t3-227)*2.5));

  return (ad);
}
//-----------------------------------------------------------|
//                  pt100桥式热敏电阻计算                    |
//-----------------------------------------------------------|
uint pt100_bridge(uint ad_t3)
{
    float ad_temp;
    uint i,ad;
    ad_temp=ad_t3;// 1.12为修正系数,因为运放的内阻造成的斜率修正。
    if (ad_t3==0) return (0);
    for (i=0;i<=45;i++)
    {
     if ((ad_t3>pt_100res[i])&&(ad_t3<=pt_100res[i+1]))
     {
       ad=(uint)(10*i+10*((ad_temp-pt_100res[i])/(pt_100res[i+1]-pt_100res[i])));
       return (ad);
     }
    }
    return (460);//超过pt_100res[]最后一位数的AD值，AD电压基本=5V，返回460C°
}

//===============================================================================
//   can 模块初始化函数
//  CAN模块的中断使能、标志等，有模块自己的寄存器，并且与CPU之间也有设置寄存器
//  FCY系统时钟频率=(7.3728M x 8)/4=14.7456M,Fcan=can时钟=FCY*CANCKS，不能高于30M
//===============================================================================
void can_init_C1(void)
{
    DISICNT = 0x3FFF;
    //*************************************************************
    //CiCTRL寄存器设置：C1CTRLbits.
    //*************************************************************
    C1CTRLbits.REQOP = 4;          //请求或要设置的工作模式
    while(!(C1CTRLbits.OPMODE==4));//判断设置的工作模式是否已经进入
    C1CTRLbits.CANCAP = 0;         //CAN 报文接收捕捉使能位;1=使能捕捉，0=禁止捕捉,用于帧时间标记的触发
    C1CTRLbits.CSIDL = 0;          //CSIDL：空闲模式停止位;=0,不停止
    C1CTRLbits.ABAT = 0;           //ABAT：中止所有等待的发送位;=0无影响
    C1CTRLbits.CANCKS = 1;         // CAN时钟选择，=0=4xFCY=14.7456x4=58.9824;=1=FCY=14.7456M，不能>30Mhz
    // if (C1CTRLbits.ICODE==xx)    //判断当前发生中断的类型，如：发送？接收？等,只读，只显示当前最高优先级中断码，显示唯一

    //*************************************************************
    // C1CFG1 , C1CFG2 , C2CFG1 ,C2CFG2 寄存器设置: 波特率
    //*************************************************************
    //标准帧108位，扩展帧128位,250kbps->1bit=4us,1bit=4us/TQ时间=4us/271ns≈14TQ(取整)=14.76TQ
    // 1BIT=最多25个TQ
    //-------------------------------------------------------------
    // 同步 , 传播段PRSEG ; 相位段1,SEG1PH ; 相位段2,SEG2PH ; 同步跳转宽度SJW(理解为同步调整宽度)
    //  1TQ     1~8TQ            1~8TQ            2~8TQ          1~4TQ
    //                      采样点=SEG1PH结束点开始
    // 传播段+相位缓冲段1 >=相位缓冲段2 ; 传播段+相位缓冲段1 >=TDELAY ; 相位缓冲段2 > 同步跳转宽度
    //-------------------------------------------------------------
    //  C1CFG1bits.BRP=0时:TQ = 2*(预分频比+1)/FCAN = 2*(C1CFG1bits.BRP + 1)/14745600hz=135.633ns=29.49TQ
    //  C1CFG1bits.BRP=1时:TQ=271ns;4us≈15个TQ≈4.069us,
    //  sjw=1TQ用于补偿15TQ多出的65ns，
    //-------------------------------------------------------------
    C1CFG1bits.SJW      = 1;       //同步跳转宽度
    C1CFG1bits.BRP      = 1;       //波特率预分频比
    C1CFG2bits.WAKFIL   = 0;        //CAN总线滤波器唤醒选择,1=使用总线滤波器唤醒,0=不唤醒
    C1CFG2bits.SEG2PH   = 4;       //相位缓冲段2,4TQ ; 晶振更换后调节为5可用
    C1CFG2bits.SEG2PHTS = 1;       //相位段2 方式选择
    C1CFG2bits.SEG1PH   = 5;       //相位缓冲段 1,5TQ ；晶振更换后调节为6可用，看脉冲周期是否=4us
    C1CFG2bits.SAM      = 1;     ///CAN总线采样位,1=采样三次,0=采样一次
    C1CFG2bits.PRSEG    = 3;//2;       //传播段2TQ
    //*************************************************************
    // 发送寄存器设置、定义部分 
    //CiTXnCON：发送缓冲器状态和控制寄存器；
    //C1TX0CONbits.  C1TX1CONbits.  C1TX2CONbits.三个发送寄存器
    //*************************************************************
    // if(C1TX0CONbits.TXABT ==1)  //报文被中止判断位,1=报文被中止,0=报文未被中止
    // if(C1TX0CONbits.TXLARB ==1) //报文丢失仲裁判断位,1=报文在发送过程中失去仲裁,0=报文在发送过程中不失去仲裁
    // if(C1TX0CONbits.TXERR ==1)  //发送时的错误检测位,1=报文发送时发生总线错误,0=报文发送时未发生总线错误,当 TXREQ置位时此位被清零
    // C1TX0CONbits.TXREQ=0;       //报文发送请求位,1=请求发送,0=如果TXREQ已置位，将中止报文发送，否则不产生影响,报文发送成功后自动清零
    C1TX0CONbits.TXPRI=3;          //报文发送优先级位,11 = 最高报文优先级
    C1TX1CONbits.TXPRI=3;          //报文发送优先级位,11 = 最高报文优先级
    C1TX2CONbits.TXPRI=3;          //报文发送优先级位,11 = 最高报文优先级
    //----------------0-ID------------------
    C1TX0SIDbits.SID5_0   = TX0_id>>18;          //扩展帧中的标准帧 11位ID码
    C1TX0SIDbits.SID10_6  = TX0_id>>24;          //扩展帧中的标准帧 11位ID码
    C1TX0DLCbits.EID5_0   = TX0_id;              //扩展的18位ID码
    C1TX0EIDbits.EID13_6  = TX0_id>>6;
    C1TX0EIDbits.EID17_14 = TX0_id>>14;
    C1TX0SIDbits.SRR      = 0;                  //远程请求控制位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX0SIDbits.TXIDE    = 1;                  //扩展标识符位; 1=发送扩展的标识符; 0=发送标准标识符
    
    C1TX0DLCbits.TXRTR   = 0;                  //远程发送请求位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX0DLCbits.TXRB0   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX0DLCbits.TXRB1   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX0DLCbits.DLC     = 8;                  //发送的数据的长度
    //----------------1-ID------------------
    C1TX1SIDbits.SID5_0   = TX1_id>>18;          //扩展帧中的标准帧 11位ID码
    C1TX1SIDbits.SID10_6  = TX1_id>>24;          //扩展帧中的标准帧 11位ID码
    C1TX1DLCbits.EID5_0   = TX1_id;              //扩展的18位ID码
    C1TX1EIDbits.EID13_6  = TX1_id>>6;
    C1TX1EIDbits.EID17_14 = TX1_id>>14;
    C1TX1SIDbits.SRR      = 0;                  //远程请求控制位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX1SIDbits.TXIDE    = 1;                  //扩展标识符位; 1=发送扩展的标识符; 0=发送标准标识符
    
    C1TX1DLCbits.TXRTR   = 0;                  //远程发送请求位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX1DLCbits.TXRB0   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX1DLCbits.TXRB1   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX1DLCbits.DLC     = 8;                  //发送的数据的长度
    //----------------2-ID------------------
    C1TX2SIDbits.SID5_0   = TX2_id>>18;          //扩展帧中的标准帧 11位ID码
    C1TX2SIDbits.SID10_6  = TX2_id>>24;          //扩展帧中的标准帧 11位ID码
    C1TX2DLCbits.EID5_0   = TX2_id;              //扩展的18位ID码
    C1TX2EIDbits.EID13_6  = TX2_id>>6;
    C1TX2EIDbits.EID17_14 = TX2_id>>14;
    C1TX2SIDbits.SRR      = 0;                  //远程请求控制位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX2SIDbits.TXIDE    = 1;                  //扩展标识符位; 1=发送扩展的标识符; 0=发送标准标识符
    
    C1TX2DLCbits.TXRTR   = 0;                  //远程发送请求位;1 = 报文将请求远程发送;0 = 正常报文发送
    C1TX2DLCbits.TXRB0   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX2DLCbits.TXRB1   = 1;                  //保留位,根据CAN协议，用户必须将这些位置为 1
    C1TX2DLCbits.DLC     = 8;                  //发送的数据的长度

    //*************************************************************
    // 接收寄存器设置、定义部分 
    //*************************************************************
    // CiRX0CON：接收缓冲器状态和控制寄存器
    //--------------------------------------
    //if (C1RX0CONbits.RXFUL==1)  //接收满状态位;1=接收缓冲器接收到报文;0=接收缓冲器准备接收新报文;读取缓冲器后由软件清零
    //if (C1RX0CONbits.RXRTRRO==1)//收到远程传输请求位（只读）；1 = 接收到远程传输请求；0 = 未接收到远程传输请求；此位反映上次装入接收缓冲器 0 的报文的状态
    C1RX0CONbits.DBEN  = 0;       //接收缓冲器0 双缓冲使能位;1=接收缓冲器0 溢出将写入接收缓冲器 1;0=接收缓冲器0 溢出不写入接收缓冲器 1；这里不用双缓冲，避免判断繁琐
    //C1RX0CONbits.JTOFF=0;       //跳转表偏移位 （DBEN 的只读备份）;1 = 允许跳转表在6 和 7之间偏移;0 = 允许跳转表在0 和 1之间偏移
    //if (C1RX0CONbits.FILHIT0==1;0)//指明缓冲器0接收的报文,是由哪个过滤器过滤的；1=接收过滤器1（RXF1）；0=接收过滤器0（RXF0）
    //-------------------------------------------------------------
    // CiRX1CON：接收缓冲器状态和控制寄存器
    //--------------------------------------
    //if (C1RX1CONbits.RXFUL==1)  //接收满状态位;1=接收缓冲器接收到报文;0=接收缓冲器准备接收新报文;读取缓冲器后由软件清零
    //if (C1RX1CONbits.RXRTRRO==1)//收到远程传输请求位（只读）；1 = 接收到远程传输请求；0 = 未接收到远程传输请求；此位反映上次装入接收缓冲器 0 的报文的状态
    //if (C1RX1CONbits.FILHIT0==0~5)//指明缓冲器1接收的报文是由哪个过滤器过滤的，见CiRX0CON；DBEN位置位后这里可能见到（0~1）一般应是2~5
    //---------------------------------------------------------------
    // 数据帧ID码和帧类型接收存储寄存器，反应当前接收数据的ID码和类型
    //---------------------------------------------------------------
    //CiRXnSID：接收缓冲器 n标准标识符  三个寄存器组合完成29位ID码
    //CiRXnEID：接收缓冲器 n扩展标识符  
    //CiRXnDLC：接收缓冲器 n数据长度控制
    //-------------------------------------------------------------
    //CiRXnSID：-------接收缓冲器 n标准标识符
    //if (C1RX0SIDbits.RXIDE == 1) //扩展帧标志;1=接收的为扩展帧;0=接收为标准帧
    //if (C1RX0SIDbits.SRR == 1)   //代替远程请求位（仅当 RXIDE = 1时可用）;1 = 发生远程传输请求;0 = 未发生远程传输请求
    //xx = C1RX0SIDbits.SID        //接收到的标准帧的ID
    //CiRXnEID：-------接收缓冲器 n扩展标识符
    //xx = C1RX0EID                //接收到的扩展帧的ID
    //CiRXnDLC：-------接收缓冲器 n 数据长度控制
    //xx = C1RX0DLCbits.EID5_0     //扩展标识符位
    //if (C1RX0DLCbits.RXRTR ==1)  //接收远程发送请求位;1 = 远程传输请求;0 = 无远程传输请求
    //C1RX0DLCbits.RXRB1           //保留的位
    //C1RX0DLCbits.RXRB0           //保留的位
    //if (C1RX0DLCbits.DLC == x)   //数据长度
    //-------------------------------------------------------------
    //CiRXnBm：--------数据接收存储器，字存储器16位
    //-------------------------------------------------------------
    //xx = C1RX0B1; RXB0缓冲寄存器的4个16位字存储器，合8个字节的数据
    //xx = C1RX0B2; 
    //xx = C1RX0B3; 接收到的数据放在这4个字、8个字节里
    //xx = C1RX0B4;
    //xx = C1RX1B1; RXB1缓冲寄存器的4个--字存储器，=8个字节的数据
    //xx = C1RX1B2; 
    //xx = C1RX1B3; 接收到的数据放在这4个字、8个字节里
    //xx = C1RX1B4;
    //-------------------------------------------------------------
    //                     报文接收过滤器
    //CiRXFnSID：接收缓冲器 n标准标识符  三个寄存器组合完成29位ID码
    //CiRXFnEIDH：接收缓冲器 n扩展标识符
    //CiRXFnEIDL：接收缓冲器 n扩展标识符
    //-------------------------------------------------------------
    //-----------------------------------------
    //F0:
    C1RXF0SIDbits.SID     =  RX0_F0id>>18;
    C1RXF0EIDLbits.EID5_0 =  RX0_F0id;
    C1RXF0EIDH            =  RX0_F0id>>6;  //C1RXF0EIDHbits
    C1RXF0SIDbits.EXIDE   =  1;//扩展帧验证
    //F1:
    C1RXF1SIDbits.SID     =  RX0_F1id>>18;
    C1RXF1EIDLbits.EID5_0 =  RX0_F1id;
    C1RXF1EIDH            =  RX0_F1id>>6;  //C1RXF0EIDHbits
    C1RXF1SIDbits.EXIDE   =  1;//扩展帧验证
    //F2:
    C1RXF2SIDbits.SID     =  RX1_F0id>>18;
    C1RXF2EIDLbits.EID5_0 =  RX1_F0id;
    C1RXF2EIDH            =  RX1_F0id>>6;  //C1RXF0EIDHbits
    C1RXF2SIDbits.EXIDE   =  1;//扩展帧验证
    //F3:
    C1RXF3SIDbits.SID     =  RX1_F1id>>18;
    C1RXF3EIDLbits.EID5_0 =  RX1_F1id;
    C1RXF3EIDH            =  RX1_F1id>>6;  //C1RXF0EIDHbits
    C1RXF3SIDbits.EXIDE   =  1;//扩展帧验证
    //F4:
    C1RXF4SIDbits.SID     =  RX1_F2id>>18;
    C1RXF4EIDLbits.EID5_0 =  RX1_F2id;
    C1RXF4EIDH            =  RX1_F2id>>6;  //C1RXF0EIDHbits
    C1RXF4SIDbits.EXIDE   =  1;//扩展帧验证
    //F5:
    C1RXF5SIDbits.SID     =  RX1_F3id>>18;
    C1RXF5EIDLbits.EID5_0 =  RX1_F3id;
    C1RXF5EIDH            =  RX1_F3id>>6;  //C1RXF0EIDHbits
    C1RXF5SIDbits.EXIDE   =  1;//扩展帧验证
    //-------------------------------------------------------------
    //                报文接收过滤器的屏蔽寄存器
    //CiRXMnSID：接收缓冲器 n标准标识符  三个寄存器组合完成29位ID码
    //CiRXMnEIDH：接收缓冲器 n扩展标识符
    //CiRXMnEIDL：接收缓冲器 n扩展标识符
    //-------------------------------------------------------------
    //M0:
    C1RXM0SIDbits.SID     = RX0_Mid>>18;
    C1RXM0EIDLbits.EID5_0 = RX0_Mid;
    C1RXM0EIDH            = RX0_Mid>>6;
    C1RXM0SIDbits.MIDE    = 1;            //扩展帧匹配
    //M1:
    C1RXM1SIDbits.SID     = RX1_Mid>>18;
    C1RXM1EIDH            = RX1_Mid;
    C1RXM1EIDLbits.EID5_0 = RX1_Mid>>6;
    C1RXM1SIDbits.MIDE   = 1;    //扩展帧匹配
    //M0:
    //C1RXM0SIDbits.SID     = 0;
    //C1RXM0EIDH            = 0;
    //C1RXM0EIDLbits.EID5_0 = 0;
    //C1RXM0SIDbits.MIDE   = 1;    //扩展帧匹配
    //M1:
    //C1RXM1SIDbits.SID     = 0;
    //C1RXM1EIDH            = 0;
    //C1RXM1EIDLbits.EID5_0 = 0;
    //C1RXM1SIDbits.MIDE   = 1;    //扩展帧匹配
    //*************************************************************
    //   CAN模块中断设置    
    //
    //*************************************************************
    //-------------------------------------------------------------
    // 中断优先级 can模块2 & can模块1
    //-------------------------------------------------------------
    IPC6bits.C1IP = 4;        //can1中断优先级
    IPC9bits.C2IP = 4;        //can2中断优先级
    //-------------------------------------------------------------
    // 中断(使能)寄存器
    //-------------------------------------------------------------
    // can1模块
    IEC1bits.C1IE = 1;        //使能CAN1组合中断??---
    C1INTEbits.RXB0IE =1;   //接收缓冲器 0中断使能位
    C1INTEbits.RXB1IE =1;   //接收缓冲器 1中断使能位
    //C1INTEbits.TXB0IE =1;   //发送缓冲器 0 中断使能位??---
    //C1INTEbits.TXB1IE =0;   //发送缓冲器 1 中断使能位
    //C1INTEbits.TXB2IE =0;   //发送缓冲器 2 中断使能位
    //C1INTEbits.ERRIE  =1;     //错误中断使能位??---
    //C1INTEbits.WAKIE  =0;   //总线唤醒活动中断使能位,1 = 使能,0 = 禁止
    //C1INTEbits.IRXIE  =0;   //接收到无效报文的中断使能位,1 = 使能,0 = 禁止
    // can2模块
    //IEC2bits.C2IE = 1;      //使能CAN2组合中断
    //C2INTEbits.RXB0IE =0;   //接收缓冲器 0 中断使能位
    //C2INTEbits.RXB1IE =0;   //接收缓冲器 1 中断使能位
    //C2INTEbits.TXB0IE =0;   //发送缓冲器 0 中断使能位
    //C2INTEbits.TXB1IE =0;   //发送缓冲器 1 中断使能位
    //C2INTEbits.TXB2IE =0;   //发送缓冲器 2 中断使能位
    //C2INTEbits.ERRIE  =1;   //错误中断使能位
    //C2INTEbits.WAKIE  =0;   //总线唤醒活动中断使能位,1 = 使能,0 = 禁止
    //C2INTEbits.IRXIE  =0;   //接收到无效报文的中断使能位,1 = 使能,0 = 禁止
    //-------------------------------------------------------------
    // 中断(标志)寄存器 can模块2 & can模块1
    //-------------------------------------------------------------
    //          IFS1bits.C1IF;     //can1中断组合标志位
    //          IFS2bits.C2IF;     //can2中断组合标志位
    //   C2 & C1INTFbits.RX0IF;    //接收缓冲器0 中断标志位
    //   C2 & C1INTFbits.RX1IF;    //接收缓冲器1 中断标志位
    //   C2 & C1INTFbits.TX0IF;    //发送缓冲器0中断标志位
    //   C2 & C1INTFbits.TX1IF;    //发送缓冲器1中断标志位
    //   C2 & C1INTFbits.TX2IF;    //发送缓冲器2中断标志位
    //   C2 & C1INTFbits.ERRIF;    //错误中断标志位 （CiINTF<15:8> 寄存器中的多种中断源）
    //   C2 & C1INTFbits.WAKIF;    //总线唤醒活动中断标志位
    //   C2 & C1INTFbits.IVRIF;    //接收到无效报文的中断标志位
    //   C2 & C1INTFbits.EWARN;    //发送器或接收器处于错误状态，警告位
    //   C2 & C1INTFbits.RXWAR;    //接收器处于错误状态，警告位
    //   C2 & C1INTFbits.TXWAR;    //发送器处于错误状态，警告位
    //   C2 & C1INTFbits.RXEP;     //接收器处于错误状态，总线被动位
    //   C2 & C1INTFbits.TXEP;     //发送器处于错误状态，总线被动位
    //   C2 & C1INTFbits.TXBO;     //发送器处于错误状态，总线关闭位
    //   C2 & C1INTFbits.RXB1OVR;  //接收缓冲器 RXB1溢出位
    //   C2 & C1INTFbits.RXB0OVR;  //接收缓冲器 RXB0溢出位
    //*************************************************************
    //   设置完成CAN模块进入运行阶段
    //*************************************************************
    C1CTRLbits.REQOP = 0;          //请求或要设置的工作模式
    while(!(C1CTRLbits.OPMODE==0));//判断设置的工作模式是否已经进入
    //C1CTRLbits.REQOP = 7;          //设置为监听所有模式
    //while(!(C1CTRLbits.OPMODE==7));//判断设置的工作模式是否已经进入
    DISICNT = 0x0000;
}

//-----------------------------------------------------------|
//                  CAN发送函数                              |
//  can模块1=C1TX0,C1TX1,C1TX2；can模块2=C2TX0,C2TX1,C2TX2   |
//  每个模块3个发送邮箱，或叫发送缓冲器,发送函数用时约10us   |
//-----------------------------------------------------------|
void Attack_C1TX0(void)//0号发送器
              {
                  
                  if (C1TX0CONbits.TXREQ == 0)
                  {
                    DISICNT = 0x3FFF;
                    unsigned int temp_data;
                    C1INTFbits.TX0IF = 0;//清发送中断标志
                    //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
                    //-------data-to-buff-----------------------
                    //---b1-----------
                    temp_data=can_TXdata[0][1];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[0][0];
                    C1TX0B1=temp_data;
                    //---b2-----------
                    temp_data=can_TXdata[0][3];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[0][2];
                    C1TX0B2=temp_data;
                    //---b3-----------
                    temp_data=can_TXdata[0][5];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[0][4];
                    C1TX0B3=temp_data;
                    //---b4-----------
                    temp_data=can_TXdata[0][7];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[0][6];
                    C1TX0B4=temp_data;
                    C1TX0CONbits.TXREQ=1;// =1请求报文发送
                    DISICNT = 0x0000;
                  }
              }      
void Attack_C1TX1(void)//1号发送器
              {
                  
                  if (C1TX1CONbits.TXREQ == 0)
                  {
                    DISICNT = 0x3FFF;
                    unsigned int temp_data;
                    C1INTFbits.TX1IF = 0;//清发送中断标志
                    //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
                    //-------data-to-buff-----------------------
                    //---b1-----------
                    temp_data=can_TXdata[1][1];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[1][0];
                    C1TX1B1=temp_data;
                    //---b2-----------
                    temp_data=can_TXdata[1][3];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[1][2];
                    C1TX1B2=temp_data;
                    //---b3-----------
                    temp_data=can_TXdata[1][5];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[1][4];
                    C1TX1B3=temp_data;
                    //---b4-----------
                    temp_data=can_TXdata[1][7];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[1][6];
                    C1TX1B4=temp_data;
                    C1TX1CONbits.TXREQ=1;// =1请求报文发送
                    DISICNT = 0x0000;
                  }
              }      
void Attack_C1TX2(void)//2号发送器
              {
                  
                  if (C1TX2CONbits.TXREQ == 0)
                  {
                    DISICNT = 0x3FFF;
                    unsigned int temp_data;
                    C1INTFbits.TX2IF = 0;//清发送中断标志
                    //PORTCbits.RC13 = !PORTCbits.RC13;//取反工作灯
                    //-------data-to-buff-----------------------
                    //---b1-----------
                    temp_data=can_TXdata[2][1];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[2][0];
                    C1TX2B1=temp_data;
                    //---b2-----------
                    temp_data=can_TXdata[2][3];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[2][2];
                    C1TX2B2=temp_data;
                    //---b3-----------
                    temp_data=can_TXdata[2][5];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[2][4];
                    C1TX2B3=temp_data;
                    //---b4-----------
                    temp_data=can_TXdata[2][7];
                    temp_data=temp_data<<8;
                    temp_data|=can_TXdata[2][6];
                    C1TX2B4=temp_data;
                    C1TX2CONbits.TXREQ=1;// =1请求报文发送
                    DISICNT = 0x0000;
                  }
              }      

/*------------------------------------------------------------------------------
                误差比例滤波函数
  previous_adc=上次AD值,current_adc=当前AD值, proportional=滤波比例
-------------------------------------------------------------------------------*/
void PI_filter(float *previous_adc, float *current_adc, float *proportional)
    {
      float adc_filer_err;//2次AD采集的误差值
      adc_filer_err = *current_adc - *previous_adc;//当前值 - 上次值=误差值；
      adc_filer_err = adc_filer_err/ *proportional;
      *previous_adc = *previous_adc + adc_filer_err;
    }
/*------------------------------------------------------------------------------
                误差比例积分通用函数
  previous_adc=上次值,current_adc=当前值, proportional=比例
-------------------------------------------------------------------------------*/
float PI_trk(float *previous_trk, float *current_trk, float P)
    {
      float trk_err;//2次AD采集的误差值
      trk_err = *current_trk - *previous_trk;//当前值 - 上次值=误差值；
      trk_err = trk_err / P;
      //trk_err = trk_err*1.2;
      return ( trk_err );
    }



