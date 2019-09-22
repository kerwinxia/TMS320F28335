
#include"DSP28x_Project.h"


__interrupt void cpu_timer0_isr(void);//定义定时器0中断服务函数
__interrupt void epwm1_isr(void);//定义epwm1中断服务函数
void PieVectTable_init(void);//映射中断服务函数（将中断服务函数添加到对应的中断向量表）
void  Device_Peripheral_init(void);//配置相应外设
void interrupt_enable(void);//中断使能
void gpio_init(void);//初始化io口
void gpio_init(void)
{
    InitGpio();//ti初始化程序，可熟悉对应寄存器,可不调用该函数
    EALLOW;
    //GPIO寄存器包含两类
    //1)GpioCtrlRegs 控制寄存器
    //  a)GPxMUX1 选择控制寄存器
    //  b)GPXDIR  方向控制寄存器
    //  c) GPxQSEL1;输入限定控制寄存器
    //2)GpioDataRegs 数据寄存器
    //  a)GPxDAT数据寄存器
    //  b)GPxSET置位寄存器
    //  c)GPxCLEAR清除寄存器
    //  d)GPxTOGGLE取反寄存器
    GpioCtrlRegs.GPAPUD.all = 0xFFFF;      //禁止上拉防止出现意外
    GpioCtrlRegs.GPBPUD.all = 0xFFFF;      //在使用pwm模块时一定先把对应GPIO的上拉功能禁止，防止输出高电平
    GpioCtrlRegs.GPCPUD.all = 0xFFFF;


    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0; // GPIO34 = GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;  // GPIO34 = output

//    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//GPIO39=GPIO
//    GpioCtrlRegs.GPADIR.bit.GPIO0=0;//GPIO39=input
    //配置EPWM模块IO功能
    InitEPwmGpio();



    EDIS;

}
/* *
 * 映射中断服务函数
 * */
void PieVectTable_init(void)
{

   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;//添加定时器0的中断服务映射
   PieVectTable.EPWM1_INT= &epwm1_isr;//添加EPWM1中断服务函数的映射
   EDIS;

}
/* *
*中断使能
* */
void interrupt_enable(void)
{
    IER |= M_INT1;//使能CPU中断INT1     定时器0中断号为INT1.7所以要使能INT1中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//使能TINT0中断
    IER |= M_INT3;//使能CPU中断INT3    EPWM中断号为INT3.1
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;//使能EPWM1_INT中断

    EINT; //开启全局中断
    ERTM;//开启实时中断（调试中断）
    CpuTimer0Regs.TCR.bit.TIE = 1;//CPU定时器中断使能
    CpuTimer0Regs.TCR.bit.TSS = 0;//启动定时器0（或StartCpuTimer0();）
}
/* *
 * 配置相应外设
 * */
void  Device_Peripheral_init(void)
{
    InitCpuTimers();//初始化CPU定时器

    //配置定时器
    //计数时间为us
    ConfigCpuTimer(&CpuTimer0, 150, 300000);//定时器0定时0.3s
    //定时时间计算方式 tim=(1+PRD)*TIMCLK=(1+PRD)*(1+TDR)/Freq 其中PRD为32位，TDR为16位，均分高地位，
    //但对PRD赋值时可采用CpuTimer0.RegsAddr->PRD.all=x;详见 ConfigCpuTimer()函数

    //CPUTimer寄存器(CpuTimer0Regs)包含两大类
    //1)初值装载类寄存器（设定计数时间）
    //  a)PRD/PRDH 定时器周期寄存器
    //  b)TPR/TPRH 定时器预定标计数器寄存器（分频用）
    //2)定时器控制寄存器TCR
    //  a)TSS定时器停止状态位，置1停止计数，置0开锁计数
    //  b)TRB定时器重装位
    //  c)TIE定时器中断使能位，置1使能
    //  d)TIF定时器中断标志位，有中断信号时为1，置1清除标志，写0无效
    //  e)FREE及SOFT定时器仿真方式

    //定时器初值装载步骤
    //计数各寄存器值,tim=(1+PRD)*TIMCLK=(1+PRD)*(1+TDR)/Freq 其中PRD为32位，TDR为16位，均分高地位，
    //CpuTimer0Regs.PRD.all=x;装初值
    //CpuTimer0Regs.TPR.bit.TDDR=x;装载分频值低位和高位！！注意TDDR和PSC构成TPR，TDDR为8位
    //CpuTimer0Regs.TPRH.bit.TDDRH=x;注意TDDRH和PSCH构成TPRH
    //CpuTimer0Regs.TCR.bit.TRB=1;对TIM及PSC进行装填

    PWM_init(&EPWM1,150,10*1000,0,20,50,0);


    //////////////////////////////////////////
    //配置ADC
    InitAdc(); //开启ADC时钟，配置ADC控制寄存器ADCTRL3(采样方式，ADC内核时钟，ADC电源控制)
    //配置ADC控制寄存器
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;//选择级联序列发生器工作方式，0为双序列发生器模式，1位级联模式
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1；//连续运行模式选择，0启停方式，1连续运行方式
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;//选择排序器运行方式（连续工作模式），0禁用排序覆盖，1顺序覆盖
    AdcRegs.ADCTRL1.bit.CPS = 0;     //ADC内核时钟预分频，用来对高速外设时钟HSPCLK进行分频，0:ADCCLK=fclk/1,1:ADCCLK=fclk/2
    AdcRegs.ADCTRL1.bit.ACQ_PS = 1;  //设置采集窗口，SOC脉冲宽度是ACQ_PS个ADCCLK周期数
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;//SEQ1事件管理器A的SOC屏蔽为，0：不通过EPWMxSOC触发启动SEQ1，1：允许触发
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;//SEQ1中断使能，0：静止SEQ1中断请求，1：允许SEQ1中断请求
    AdcRegs.ADCMAXCONV.all = 0X0001;// Setup 2 conv's on SEQ1

    //    DELAY_US(5000L);         // 第一次转换前需延时5ms
//    //配置ADC时钟(ADCTL2)
////    CLKDIV2EN   CLKDIV4EN     ADCCLK
////        0           0          SYSCLK
////        0           1          SYSCLK
////        1           0          SYSCLK / 2
////        1           1          SYSCLK / 4
//    EALLOW;
//    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;
//    AdcRegs.ADCCTL2.bit.CLKDIV4EN = 0;
//    EDIS;
//    DELAY_US(5000L);
//
//    AdcOffsetSelfCal();

}
/**
 * main.c
 */
void main(void)
{
    InitSysCtrl();//系统控制初始化函数，需注意所用外设时钟是否使能
    InitGpio();//ti初始化程序，可熟悉对应寄存器,可不调用该函数

    DINT;  //禁止CPU所有中断
    gpio_init();//用到的io进行初始化

    InitPieCtrl();//初始化PIE控制寄存器

    IER = 0x0000;//禁用CPU中断
    IFR = 0x0000;//清除CPU中断标志

    InitPieVectTable();//初始化PIE中断向量表

    PieVectTable_init();//映射中断服务函数
    Device_Peripheral_init();//配置外设

    interrupt_enable();//使能中断

	for(;;)
	{
	    PWM_steduy(&EPWM1,20,50);

//	  if(GpioDataRegs.GPADAT.bit.GPIO0==0)
//	  {
//	      //DELAY_US(500L);
////	      if(GpioDataRegs.GPADAT.bit.GPIO0==0)
////	      {
//	          GpioDataRegs.GPATOGGLE.bit.GPIO17=1;
//	          DELAY_US(50000L);
//	           //while(GpioDataRegs.GPADAT.bit.GPIO0==0);
////	      }
//	  }
//	      //GpioDataRegs.GPADAT.bit.GPIO17=GpioDataRegs.GPADAT.bit.GPIO0;
//
	}
}

/*
*定时器0中断服务函数
*/
__interrupt void
cpu_timer0_isr(void)
{

    CpuTimer0.InterruptCount++;

    GpioDataRegs.GPATOGGLE.bit.GPIO17=1;//GPIO17状态翻转
//    GpioDataRegs.GPBSET.bit.GPIO34=1;//GPIO34置1
//    GpioDataRegs.GPBCLEAR.bit.GPIO34=1;//GPIO34清零（置0）
//    GpioDataRegs.GPBDAT.bit.GPIO34=0;//GPIO34写0/1（置0/1）



    PieCtrlRegs.PIEACK.bit.ACK1= 1;//退出中断以便接收其他中断
    CpuTimer0Regs.TCR.bit.TIF=1;//清中断标志
    CpuTimer0Regs.TCR.bit.TRB=1;//重新装填

}
/*
*epwm1中断服务函数
*/
__interrupt void
epwm1_isr(void)
{

      EPwm1Regs.ETCLR.bit.INT = 1;

       //
       // Acknowledge this interrupt to receive more interrupts from group 3
       //
       PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

