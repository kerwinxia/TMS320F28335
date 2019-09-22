
#include"DSP28x_Project.h"


__interrupt void cpu_timer0_isr(void);//���嶨ʱ��0�жϷ�����
__interrupt void epwm1_isr(void);//����epwm1�жϷ�����
void PieVectTable_init(void);//ӳ���жϷ����������жϷ�������ӵ���Ӧ���ж�������
void  Device_Peripheral_init(void);//������Ӧ����
void interrupt_enable(void);//�ж�ʹ��
void gpio_init(void);//��ʼ��io��
void gpio_init(void)
{
    InitGpio();//ti��ʼ�����򣬿���Ϥ��Ӧ�Ĵ���,�ɲ����øú���
    EALLOW;
    //GPIO�Ĵ�����������
    //1)GpioCtrlRegs ���ƼĴ���
    //  a)GPxMUX1 ѡ����ƼĴ���
    //  b)GPXDIR  ������ƼĴ���
    //  c) GPxQSEL1;�����޶����ƼĴ���
    //2)GpioDataRegs ���ݼĴ���
    //  a)GPxDAT���ݼĴ���
    //  b)GPxSET��λ�Ĵ���
    //  c)GPxCLEAR����Ĵ���
    //  d)GPxTOGGLEȡ���Ĵ���
    GpioCtrlRegs.GPAPUD.all = 0xFFFF;      //��ֹ������ֹ��������
    GpioCtrlRegs.GPBPUD.all = 0xFFFF;      //��ʹ��pwmģ��ʱһ���ȰѶ�ӦGPIO���������ܽ�ֹ����ֹ����ߵ�ƽ
    GpioCtrlRegs.GPCPUD.all = 0xFFFF;


    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0; // GPIO34 = GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;  // GPIO34 = output

//    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//GPIO39=GPIO
//    GpioCtrlRegs.GPADIR.bit.GPIO0=0;//GPIO39=input
    //����EPWMģ��IO����
    InitEPwmGpio();



    EDIS;

}
/* *
 * ӳ���жϷ�����
 * */
void PieVectTable_init(void)
{

   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;//��Ӷ�ʱ��0���жϷ���ӳ��
   PieVectTable.EPWM1_INT= &epwm1_isr;//���EPWM1�жϷ�������ӳ��
   EDIS;

}
/* *
*�ж�ʹ��
* */
void interrupt_enable(void)
{
    IER |= M_INT1;//ʹ��CPU�ж�INT1     ��ʱ��0�жϺ�ΪINT1.7����Ҫʹ��INT1�ж�
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//ʹ��TINT0�ж�
    IER |= M_INT3;//ʹ��CPU�ж�INT3    EPWM�жϺ�ΪINT3.1
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;//ʹ��EPWM1_INT�ж�

    EINT; //����ȫ���ж�
    ERTM;//����ʵʱ�жϣ������жϣ�
    CpuTimer0Regs.TCR.bit.TIE = 1;//CPU��ʱ���ж�ʹ��
    CpuTimer0Regs.TCR.bit.TSS = 0;//������ʱ��0����StartCpuTimer0();��
}
/* *
 * ������Ӧ����
 * */
void  Device_Peripheral_init(void)
{
    InitCpuTimers();//��ʼ��CPU��ʱ��

    //���ö�ʱ��
    //����ʱ��Ϊus
    ConfigCpuTimer(&CpuTimer0, 150, 300000);//��ʱ��0��ʱ0.3s
    //��ʱʱ����㷽ʽ tim=(1+PRD)*TIMCLK=(1+PRD)*(1+TDR)/Freq ����PRDΪ32λ��TDRΪ16λ�����ָߵ�λ��
    //����PRD��ֵʱ�ɲ���CpuTimer0.RegsAddr->PRD.all=x;��� ConfigCpuTimer()����

    //CPUTimer�Ĵ���(CpuTimer0Regs)����������
    //1)��ֵװ����Ĵ������趨����ʱ�䣩
    //  a)PRD/PRDH ��ʱ�����ڼĴ���
    //  b)TPR/TPRH ��ʱ��Ԥ����������Ĵ�������Ƶ�ã�
    //2)��ʱ�����ƼĴ���TCR
    //  a)TSS��ʱ��ֹͣ״̬λ����1ֹͣ��������0��������
    //  b)TRB��ʱ����װλ
    //  c)TIE��ʱ���ж�ʹ��λ����1ʹ��
    //  d)TIF��ʱ���жϱ�־λ�����ж��ź�ʱΪ1����1�����־��д0��Ч
    //  e)FREE��SOFT��ʱ�����淽ʽ

    //��ʱ����ֵװ�ز���
    //�������Ĵ���ֵ,tim=(1+PRD)*TIMCLK=(1+PRD)*(1+TDR)/Freq ����PRDΪ32λ��TDRΪ16λ�����ָߵ�λ��
    //CpuTimer0Regs.PRD.all=x;װ��ֵ
    //CpuTimer0Regs.TPR.bit.TDDR=x;װ�ط�Ƶֵ��λ�͸�λ����ע��TDDR��PSC����TPR��TDDRΪ8λ
    //CpuTimer0Regs.TPRH.bit.TDDRH=x;ע��TDDRH��PSCH����TPRH
    //CpuTimer0Regs.TCR.bit.TRB=1;��TIM��PSC����װ��

    PWM_init(&EPWM1,150,10*1000,0,20,50,0);


    //////////////////////////////////////////
    //����ADC
    InitAdc(); //����ADCʱ�ӣ�����ADC���ƼĴ���ADCTRL3(������ʽ��ADC�ں�ʱ�ӣ�ADC��Դ����)
    //����ADC���ƼĴ���
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;//ѡ�������з�����������ʽ��0Ϊ˫���з�����ģʽ��1λ����ģʽ
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1��//��������ģʽѡ��0��ͣ��ʽ��1�������з�ʽ
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;//ѡ�����������з�ʽ����������ģʽ����0�������򸲸ǣ�1˳�򸲸�
    AdcRegs.ADCTRL1.bit.CPS = 0;     //ADC�ں�ʱ��Ԥ��Ƶ�������Ը�������ʱ��HSPCLK���з�Ƶ��0:ADCCLK=fclk/1,1:ADCCLK=fclk/2
    AdcRegs.ADCTRL1.bit.ACQ_PS = 1;  //���òɼ����ڣ�SOC��������ACQ_PS��ADCCLK������
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;//SEQ1�¼�������A��SOC����Ϊ��0����ͨ��EPWMxSOC��������SEQ1��1��������
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;//SEQ1�ж�ʹ�ܣ�0����ֹSEQ1�ж�����1������SEQ1�ж�����
    AdcRegs.ADCMAXCONV.all = 0X0001;// Setup 2 conv's on SEQ1

    //    DELAY_US(5000L);         // ��һ��ת��ǰ����ʱ5ms
//    //����ADCʱ��(ADCTL2)
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
    InitSysCtrl();//ϵͳ���Ƴ�ʼ����������ע����������ʱ���Ƿ�ʹ��
    InitGpio();//ti��ʼ�����򣬿���Ϥ��Ӧ�Ĵ���,�ɲ����øú���

    DINT;  //��ֹCPU�����ж�
    gpio_init();//�õ���io���г�ʼ��

    InitPieCtrl();//��ʼ��PIE���ƼĴ���

    IER = 0x0000;//����CPU�ж�
    IFR = 0x0000;//���CPU�жϱ�־

    InitPieVectTable();//��ʼ��PIE�ж�������

    PieVectTable_init();//ӳ���жϷ�����
    Device_Peripheral_init();//��������

    interrupt_enable();//ʹ���ж�

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
*��ʱ��0�жϷ�����
*/
__interrupt void
cpu_timer0_isr(void)
{

    CpuTimer0.InterruptCount++;

    GpioDataRegs.GPATOGGLE.bit.GPIO17=1;//GPIO17״̬��ת
//    GpioDataRegs.GPBSET.bit.GPIO34=1;//GPIO34��1
//    GpioDataRegs.GPBCLEAR.bit.GPIO34=1;//GPIO34���㣨��0��
//    GpioDataRegs.GPBDAT.bit.GPIO34=0;//GPIO34д0/1����0/1��



    PieCtrlRegs.PIEACK.bit.ACK1= 1;//�˳��ж��Ա���������ж�
    CpuTimer0Regs.TCR.bit.TIF=1;//���жϱ�־
    CpuTimer0Regs.TCR.bit.TRB=1;//����װ��

}
/*
*epwm1�жϷ�����
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

