
#include "p30f6010A.h"  //dsPIC30F6010A标准头文件

//*********************************************************
//                      IO_init                           *
//      初始化包含文件定义，主要进行CPU的功能初始化       *
//*********************************************************
void IO_init(void)
{
    SRbits.IPL = 3;          //cpu中断优先级，0~7或IPL3=1时 x2
    CORCONbits.IPL3 = 0;     //CPU中断优先级X2;
	INTCON1bits.NSTDIS=1;    //=1 禁止中断嵌套
    //static int XXX,ZZZ;
    //LATE=0;
    //TRISF=0X0000;
    //TRISFbits.TRISF6 = 0;    // 0=输出，1=输入
    //TRISDbits.TRISD11 = 0;   // 0=输出，1=输入
    //PORTFbits.RF6
    //PORTFbits.RF6 = !PORTFbits.RF6;取反操作例子
    //--------------------------------------------------------------------------
    //A 端口方向设置
    TRISAbits.TRISA14 = 1;     //旋变报警信号DOS
    TRISAbits.TRISA15 = 1;     //旋变报警信号LOT
    //--------------------------------------------------------------------------
    //B 端口方向设置(端口为复用模拟量端口可能还有其他类型时，由模拟量的端口设置寄存器进行数字、模拟量设置)
    TRISBbits.TRISB12 = 1; //C0D4输出；
    TRISBbits.TRISB13 = 1; //C0D5输出；
    TRISBbits.TRISB14 = 1; //C0D6输出； 1=输入, 接触器吸合，由06CPU通过C0D6，C0D7给出的准备就绪信号
    TRISBbits.TRISB15 = 1; //C0D7输出； 1=输入，控制器ready
    //--------------------------------------------------------------------------
    //C 端口方向设置;        0=输出，1=输入
    TRISCbits.TRISC1 = 0;    // C4D0的输入
    TRISCbits.TRISC3 = 0;    // C4D1的输入
    TRISCbits.TRISC13 = 0;   // 6010A-CPU运行指示灯
    TRISCbits.TRISC14 = 0;   // 旋变复位（由低到高的变化有效）
    PORTCbits.RC1 = 1;
    PORTCbits.RC3 = 1;
    PORTCbits.RC13 = 1;
    PORTCbits.RC14 = 0;      // 旋变复位（由低到高的变化有效）
    //--------------------------------------------------------------------------
    //D 端口方向设置,// 0=输出，1=输入
    TRISDbits.TRISD0  = 1;  //C6D0
    TRISDbits.TRISD1  = 1;  //C6D1
    TRISDbits.TRISD2  = 1;  //C6D2
    TRISDbits.TRISD3  = 1;  //C6D3
    TRISDbits.TRISD4  = 1;  //C6D4
    TRISDbits.TRISD5  = 1;  //C6D5
    TRISDbits.TRISD6  = 1;  //C6D6
    TRISDbits.TRISD7  = 1;  //C6D7
    //----------------------------
    TRISDbits.TRISD8  = 1;  //C5D0
    TRISDbits.TRISD9  = 1;  //C5D1
    TRISDbits.TRISD10 = 1;  //C5D2
    TRISDbits.TRISD11 = 1;//电机或控制器三相缺相报警，低电平报警，高电平为正常
    TRISDbits.TRISD12 = 1;//C4D7=VBTS=低压电源检测
    TRISDbits.TRISD13 = 0;//控制器外部机箱工作指示灯，RD13=L，指示灯亮，RD13=H，指示灯灭
    TRISDbits.TRISD14 = 1;//06CPU的ALM信号输入口
    TRISDbits.TRISD15 = 1;//风机堵转报警，备用
    //--------------------------------------------------------------------------
    //E 端口方向设置     0=输出，1=输入
    TRISEbits.TRISE0 = 0;//06CPU的C1D0
    TRISEbits.TRISE1 = 0;//06CPU的C1D1
    TRISEbits.TRISE2 = 0;//06CPU的C1D2
    TRISEbits.TRISE3 = 0;//06CPU的C1D3
    TRISEbits.TRISE4 = 0;//06CPU的C1D4
    TRISEbits.TRISE5 = 0;//06CPU的C1D5
    TRISEbits.TRISE6 = 0;//06CPU的C1D6
    TRISEbits.TRISE7 = 0;//06CPU的C1D7
    TRISEbits.TRISE8 = 1;//控制器主接触器的辅助触点
    TRISEbits.TRISE9 = 1;//主回路过流报警信号
    //--------------------------------------------------------------------------
    //E 端口输出设置
    PORTEbits.RE0 = 1;
    PORTEbits.RE1 = 1;
    PORTEbits.RE2 = 1;
    PORTEbits.RE3 = 1;
    PORTEbits.RE4 = 1;
    PORTEbits.RE5 = 1;
    PORTEbits.RE6 = 1;
    PORTEbits.RE7 = 1;
    //PORTEbits.RE8
    //PORTEbits.RE9
    //--------------------------------------------------------------------------
    //F 端口方向设置        0=输出，1=输入（RF口暂时定义为输入，以免影响06CPU控制）
    TRISFbits.TRISF6 = 1;  //C0D0， 控制主接触器吸合信号
    TRISFbits.TRISF7 = 1;  //C0D1， 控制风机运行信号
    TRISFbits.TRISF8 = 0;  //C0D2， 控制预充电接触器信号
    //--------------------------------------------------------------------------
    //G 端口方向设置;        0=输出，1=输入
    //LATGbits.LATG2 = 1;
    //LATGbits.LATG3 = 1;
    TRISGbits.TRISG3 = 0;   //06CPU固化端口FWE/输出低电平固化，高电平无效
    //PORTCbits.RC1 =         C4D0
    //PORTCbits.RC3 =         C4D1
    TRISGbits.TRISG6 = 0;   //C4D2
    TRISGbits.TRISG7 = 0;   //C4D3
    TRISGbits.TRISG8 = 0;   //C4D4
    TRISGbits.TRISG9 = 0;   //C4D5
    TRISGbits.TRISG2 = 0;   //C4D6
    //PORTDbits.RD12 =        C4D7 =VBTS=低压电源检测
    //PORTGbits.RG2 = 1; 
    //PORTGbits.RG3 = 1; 
    PORTG = 255;
}

//*********************************************************
//                  timer1_init                           *
//        初始化包含文件定义，主要进行CPU的功能初始化     *
//*********************************************************
void T1_Init(void)          //修改为16m晶振后，PR1=4300=270us,160=10um,320=20um,640=40um，4800=300us
{
    TMR1           =  0;    //清定时器计数器
    PR1            =4800;   //3400~=230us,4300~=300us,16384=1.2ms,65534=2.2ms; //设定计数器计数上限，及中断时的计数值
    T1CONbits.TCS  =  0;    //定时器时钟源选择,1=来自TxCK引脚的外部时钟,0=内部时钟 （FOSC/4）
    T1CONbits.TSYNC=  0;    //(TCS=1时)定时器外部时钟输入同步选择位,1=同步外部时钟输入,0=不同步,TCS=0时此位不起作用
    T1CONbits.TCKPS=  0;    //定时器输入时钟预分频选择位，(00~01~10~11)=(1,8,64,256)预分频比
    T1CONbits.TGATE=  0;    //定时器门控时间累加使能位,1=门控时间累加使能,0=门控时间累加禁止
    T1CONbits.TSIDL=  0;    //空闲模式选择，1=空闲模式定时器不工作，0=空闲模式继续工作
    IPC0bits.T1IP  =  4;    //Timer1中断优先级
    IFS0bits.T1IF  =  0;    //Timer1中断标志位，1=有中断，0=无中断
    IEC0bits.T1IE  =  1;    //Timer1中断允许位，1=使能，0=取消
    T1CONbits.TON  =  1;    //定时器开控制位,1=启动定时器,0=停止定时器
}


//*****************************************************************
//           ADC_init(06cpu板使用RB6,7,8,9,10,11,六路模拟量端口)  *
//  adc模块要求前级输入阻抗小于5K欧,保证最少1 Tad的采样时间       *
//  ADCON1：A/D控制寄存器1        4011用于电动车，AD采样要求不搞  *
//  ADCON2：A/D控制寄存器2        因此为了简单方便选择扫描采集    *
//  ADCON3：A/D控制寄存器3        4013选择并行采集模式            *
//  ADCHS： A/D输入通道选择寄存器                                 *
//  ADPCFG：A/D端口配置寄存器                                     *
//  ADCSSL：A/D输入扫描选择寄存器-------  共有6个配置、控制寄存器 *
//*****************************************************************
// ADC可能受串口接收中断影响，中断优先级冲突，因而产生速度慢的现象

void ADC_init(void)
{
    
//---------------------------------------
//ADPCFG：A/D端口配置寄存器    0=AD,1=I/O
//---------------------------------------
        //TRISBbits.TRISB6  = 1;  // 0=输出，1=输入
        //TRISBbits.TRISB7  = 1;  //AD相应的I/O控制寄存器也要配置为输入
        //TRISBbits.TRISB8  = 1;
        //TRISBbits.TRISB9  = 1;
        //TRISBbits.TRISB10 = 1;
        //TRISBbits.TRISB11 = 1;
        //ADPCFG = 0x0038;//0000000000111000=AN6-AN7-AN8
        ADPCFGbits.PCFG2  = 0;
        ADPCFGbits.PCFG6  = 0;
        ADPCFGbits.PCFG7  = 0;
        ADPCFGbits.PCFG8  = 0;
        ADPCFGbits.PCFG9  = 0;
        ADPCFGbits.PCFG10 = 0;
        ADPCFGbits.PCFG11 = 0;
        ADPCFGbits.PCFG12 = 1;
        ADPCFGbits.PCFG13 = 1;
        ADPCFGbits.PCFG14 = 1;
        ADPCFGbits.PCFG15 = 1;
//---------------------------------------
//ADCON1：A/D控制寄存器1---8个控制功能位 
//---------------------------------------
        //ADCON1bits.ADON 此位一般在初始化其他所以寄存器后使能，=1模块工作，=0模块关闭。
        ADCON1bits.ADSIDL = 0x00;  //1=模块空闲时关闭，0=模块空闲不关闭，猜测：关闭后还要用ADON来开启吧。麻烦
        ADCON1bits.FORM   = 0x00;  //输出数据格式为整数方式
        ADCON1bits.SSRC   = 0x07;  //111=内部计数器结束采样并开始转换（自动转换方式）
        ADCON1bits.SIMSAM = 0x00;  // 同时采样选择位，=0为顺序逐个采样，=1为同时采样CH0~CH3 通道。
        ADCON1bits.ASAM   = 0x00;  // 采样自动开始位，=1为上次转换结束立即开始采样，=0为SAMP置位后开始采样。
        //ADCON1bits.SAMP 触发采样开始位(启动采样指令)
        //ADCON1bits.DONE 转换完成标志位 =1转换完成，=0转换没有完成(采样转换完成判断标志)
//---------------------------------------
//ADCON2：A/D控制寄存器2---7个控制功能位 
//---------------------------------------
        ADCON2bits.VCFG = 0x03;//参考电压选择：001=外部 VREF+AVSS，011=外部 VREF+,VREF-，0=AVDD+AVSS。(1或3都可以)
        ADCON2bits.CSCNA= 0x01;//MUX A输入多路开关设置的CH0+S/H输入的扫描输入选择位。1=扫描，0=不扫描,CSCNA置位时，会忽略ADCHSbits.CH0SA位
        ADCON2bits.CHPS = 0x00;//采样保持通道选择位，芯片共有4个CH可选，1x=4个ch全用；01=ch0~1；00=ch0，SIMSAM位设置并行或顺序采样
        //ADCON2bits.BUFS   //采样缓冲寄存器状态标志，寄存器可定义成2个8字的或1个16字的，2个8字时填充分先后，由本位判断先8位后8位
        ADCON2bits.SMPI = 0x06;//每次完成几路采样转换后产生中断的设置，本位占用4位，组合正好16，0=完成1通道就产生中断~15=完成16通道后中断
        ADCON2bits.BUFM = 0x00;//采样缓冲寄存器模式选择，1=2个8字的，0=16字模式，和ADCON2bits.BUFS位关联使用。
        ADCON2bits.ALTS = 0x00;//采样输入的多路开关轮换输入模式选择，1=第一个用MUX A，以后的由MUX B和A轮换，0=一直只用MUXA
//---------------------------------------
//ADCON3：A/D控制寄存器3---3个控制功能位         AD最小时钟=154ns；
//---------------------------------------
        ADCON3bits.SAMC = 0x01;//按采样时钟设定采样时间，共5位组合可设置0~31个采样时钟周期
        ADCON3bits.ADRC = 0x00;//ADC时钟源选择，0=系统时钟，1=内部RC时钟
        ADCON3bits.ADCS = 0x02; //ADC时钟分频设置位，0~5共6位，0~64分频比。ADCS=(2*TAD)/TCY -1=2*154ns/(7.3728Mhz*8/4=14.7456=67.8168ns)=3.529=4
        //时钟周期和采样时间要计算清楚，具体再看。
//---------------------------------------
//ADCHS： A/D输入通道与保持通道搭配的选择寄存器
//---------------------------------------

        //ADCHS = 0x0023;
        //ADCHSbits.CH123NB= 0;
        //ADCHSbits.CH123SB= 1;
        //ADCHSbits.CH0NB  = 0;//-VREF
        //ADCHSbits.CH0SB  = 6;//0CH=AN6
        ADCHSbits.CH123NA= 0;
        ADCHSbits.CH123SA= 1;
        ADCHSbits.CH0NA  = 0;//-VREF
        ADCHSbits.CH0SA  = 6;//0CH=AN2
        
//---------------------------------------
//ADCSSL：A/D输入扫描选择寄存器   1=扫描，0=不扫描
//---------------------------------------
        ADCSSLbits.CSSL2 = 1;
        ADCSSLbits.CSSL6 = 1;
        ADCSSLbits.CSSL7 = 1;
        ADCSSLbits.CSSL8 = 1;
        ADCSSLbits.CSSL9 = 1;
        ADCSSLbits.CSSL10= 1;
        ADCSSLbits.CSSL11= 1;

//---------------------------------------
//DSC的各个模块中断是统一管理的，寄存器不为某模块独享
//INTCON1 和 INTCON2 全局中断控制寄存器
//IFSx：中断标志状态寄存器
//IECx：中断允许控制寄存器
//IPCx：中断优先级控制寄存器
//---------------------------------------
        //清中断标志位
        IFS0bits.ADIF = 0;
        IPC2bits.ADIP = 5;
        //设置中断使能位
        IEC0bits.ADIE = 1;
        //开启ADC模块，进入工作状态
        //在所有别的寄存器配置好以后开启
        ADCON1bits.ADON = 1;
        //for(i_delay = 0; i_delay < 160000; i_delay++)Nop();

}

