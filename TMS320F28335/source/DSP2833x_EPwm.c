// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:19 $
//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2833x Support Library v2.00.00.00 $
// $Release Date: Mon May 27 06:46:54 CDT 2019 $
// $Copyright:
// Copyright (C) 2009-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//
// Globals
//
struct EPWM_VARS EPWM1;
struct EPWM_VARS EPWM2;
struct EPWM_VARS EPWM3;
struct EPWM_VARS EPWM4;
struct EPWM_VARS EPWM5;
struct EPWM_VARS EPWM6;

//
// InitEPwm - This function initializes the ePWM(s) to a known state.
//
void 
InitEPwm(void)
{
    //
    // Initialize ePWM1/2/3/4/5/6
    //
}
//PWM�������ʼ��
//���������Epwm     epwmģ��ţ�      ��&EPWM1
//         CpuFreq  ϵͳʱ��Ƶ��MHz ��90
//         PwmFreq  PWMƵ��Hz��        ��10*1000=10kHz
//         DeadHand  ����ʱ��us     ��0.1=0.1US
//         dutyfactorA A·ռ�ձ�            20.5=20.5%
//         dutyfactorB B·ռ�ձ�
//         comp  1ʱAB������ʱB·ռ�ձ�ֵ����Ч���ڴ�ʱ��������Ч��0ʱ�ǻ�������·�����������ʱ��������ʧЧ
//
//����ʾ��PWM_init(&EPWM1,90,20*1000,0,20,40,0)
void
PWM_init(struct EPWM_VARS *Epwm, Uint8 CpuFreq, Uint32 PwmFreq, float DeadHand, float dutyfactorA,float dutyfactorB,Uint8 comp)
{

       Uint8 Hspclkdiv,Clkdiv;
       Uint16 Freq_div;//��Ƶϵ��
       EALLOW;
       SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
       EDIS;//��ֹEPWMģ��ͬ��ʱ��
       Epwm->Dead_Hand=comp;
       //���ü���ģʽ��ʱ�ӣ�TBCTL��
       Epwm->RegsAddr->TBCTL.bit.CTRMODE = 2;//���ü�����ʽΪ��������
       Epwm->RegsAddr->TBCTL.bit.PHSEN = 0;//��λ��ʹ�ܣ���ֹTBCTR������λ�Ĵ���TBPHS�е�ֵ��
       Epwm->RegsAddr->TBCTL.bit.PRDLD = 0;//��TBPRD=0ʱ��ӳ��Ĵ���������װ�ص���ǰ������
       Epwm->RegsAddr->TBCTL.bit.SYNCOSEL = 3;//����EPWMxSYNCO�ź�

       Freq_div=Freq_division( &Hspclkdiv,&Clkdiv, CpuFreq,PwmFreq);
       Epwm->RegsAddr->TBCTL.bit.CLKDIV = Clkdiv;
       Epwm->RegsAddr->TBCTL.bit.HSPCLKDIV= Hspclkdiv;
                                         // TBCLK=SYSCLKOUT/(HSPCLKDIV*CLKDIV)

      //�����ز�Ƶ��Tpwm = 2 x TBPRD*(1/TBCLK); Fpwm = 1/(Tpwm).
       //CLKDIV=0,HSPCLKDIV=0ʱF=SYSCLOUT/(2*TBPRD);TBPRD=SYSCLOUT/(2*F)
       Epwm->PRD=(Uint16)(CpuFreq*1000000/(2*PwmFreq*Freq_div));
       Epwm->RegsAddr->TBPRD =Epwm->PRD;//�ز�Ƶ��
       Epwm->RegsAddr->TBCTR = 0;//��ռ�����
       Epwm->RegsAddr->TBPHS.half.TBPHS = 0x0000;           // Phase is 0
       //���ñȽϿ��ƼĴ���
       Epwm->RegsAddr->CMPCTL.bit.SHDWAMODE = 0;//CPMA�Ĵ���������ӳ��ģʽ
       Epwm->RegsAddr->CMPCTL.bit.SHDWBMODE = 0;
       Epwm->RegsAddr->CMPCTL.bit.LOADAMODE = 0;//CPMA��TBCTR=0ʱ��ӳ��Ĵ�����������
       Epwm->RegsAddr->CMPCTL.bit.LOADBMODE = 0;
       //���ñȽ�
       //CMP=TBPRD(1-D);DΪռ�ձ�
       Epwm->RegsAddr->CMPA.half.CMPA = (Uint16)((Epwm->PRD)*(100-dutyfactorA)/100);
       Epwm->RegsAddr->CMPB = (Uint16)((Epwm->PRD)*(100-dutyfactorB)/100);
       //���ñȽ϶���
       Epwm->RegsAddr->AQCTLA.bit.CAU = 2;//CMPA=TBCTR�����ϼ���ʱAΪ��
       Epwm->RegsAddr->AQCTLA.bit.CAD = 1;//CMPA=TBCTR�����¼���ʱAΪ��

       Epwm->RegsAddr->AQCTLB.bit.CBU = 2;//CMPB=TBCTR�����ϼ���ʱBΪ��
       Epwm->RegsAddr->AQCTLB.bit.CBD = 1;//CMPB=TBCTR�����¼���ʱBΪ��
       //��������


       if(comp==1)
       {
           Epwm->RegsAddr->DBCTL.bit.OUT_MODE = 3;//�����½��ؾ���ʱ
           Epwm->RegsAddr->DBCTL.bit.POLSEL = 2;//B·���Է�ת
       }
       else
       {   Epwm->RegsAddr->DBCTL.bit.OUT_MODE = 0;
           Epwm->RegsAddr->DBCTL.bit.POLSEL = 0; //������ת
       }
       Epwm->RegsAddr->DBCTL.bit.IN_MODE = 0;//A·��Ϊ��׼�ź�
       Epwm->RegsAddr->DBRED = (Uint16)(DeadHand*CpuFreq/Freq_div);//��������ʱx������ʱ��
       Epwm->RegsAddr->DBFED = (Uint16)(DeadHand*CpuFreq/Freq_div);//�½�����ʱx������ʱ��
       //����EPWM�ж�ETSEL���˴�ʹ�ò����жϲ��������ã�
       EALLOW;
       SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
       EDIS;//ͬ��epwmģ��ʱ��

}

//PWM�������
//���������Epwm     epwmģ��ţ�      ��&EPWM1
//         dutyfactorA A·ռ�ձ�            20.5=20.5%
//         dutyfactorB B·ռ�ձ�
//        ѡ�񻥲����ʱB·ռ�ձȲ�������Ч
//���þ���PWM_steduy(&EPWM1,20,30)
void
PWM_steduy(struct EPWM_VARS *Epwm, float dutyfactorA,float dutyfactorB)
{
          //CMP=TBPRD(1-D);DΪռ�ձ�
          Epwm->RegsAddr->CMPA.half.CMPA = (Uint16)((Epwm->PRD)*(100-dutyfactorA)/100);
          Epwm->RegsAddr->CMPB = (Uint16)((Epwm->PRD)*(100-dutyfactorB)/100);
}
//
// InitEPwmGpio - This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
void 
InitEPwmGpio(void)
{
    //��ʼ��ָ����Ӧ��ʱ���Ĵ����ĵ�ַָ��
    EPWM1.RegsAddr = &EPwm1Regs;
    EPWM2.RegsAddr = &EPwm2Regs;
    EPWM3.RegsAddr = &EPwm3Regs;
    EPWM4.RegsAddr = &EPwm4Regs;
    EPWM5.RegsAddr = &EPwm5Regs;
    EPWM6.RegsAddr = &EPwm6Regs;

#if DSP28_EPWM1
    InitEPwm1Gpio();
#endif // endif DSP28_EPWM1
#if DSP28_EPWM2
    InitEPwm2Gpio();
#endif // endif DSP28_EPWM2
#if DSP28_EPWM3
    InitEPwm3Gpio();
#endif // endif DSP28_EPWM3
#if DSP28_EPWM4
    InitEPwm4Gpio();
#endif // endif DSP28_EPWM4
#if DSP28_EPWM5    
    InitEPwm5Gpio();
#endif // endif DSP28_EPWM5
#if DSP28_EPWM6
    InitEPwm6Gpio();
#endif // endif DSP28_EPWM6 
}

//
// InitEPwm1Gpio - This function initializes GPIO pins to function as ePWM1
// 
void 
InitEPwm1Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   

    //
    // Configure ePWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM1 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

//
// InitEPwm2Gpio - This function initializes GPIO pins to function as ePWM2
//
void 
InitEPwm2Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

    //
    // Configure ePWM-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM2 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

//
// InitEPwm3Gpio - This function initializes GPIO pins to function as ePWM3
//
void 
InitEPwm3Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)

    //
    // Configure ePWM-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM3 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}

#if DSP28_EPWM4
//
// InitEPwm4Gpio - This function initializes GPIO pins to function as ePWM4
//
void 
InitEPwm4Gpio(void)
{
    EALLOW;
    
    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)

    //
    // Configure ePWM-4 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM4 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
#endif // endif DSP28_EPWM4  

#if DSP28_EPWM5
//
// InitEPwm5Gpio - This function initializes GPIO pins to function as ePWM5
//
void 
InitEPwm5Gpio(void)
{
    EALLOW;
    
    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO9 (EPWM5B)

    //
    // Configure ePWM-5 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM5 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}
#endif // endif DSP28_EPWM5

#if DSP28_EPWM6
//
// InitEPwm6Gpio - This function initializes GPIO pins to function as ePWM6
//
void 
InitEPwm6Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)

    //
    // Configure ePWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM6 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B

    EDIS;
}
#endif // endif DSP28_EPWM6  

//
// InitEPwmSyncGpio - This function initializes GPIO pins to function as ePWM 
// Synch pins
//
void 
InitEPwmSyncGpio(void)
{
    EALLOW;
    
    //
    // Configure EPWMSYNCI
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;  //Enable pull-up on GPIO6 (EPWMSYNCI)
    //GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;//Enable pull-up on GPIO32 (EPWMSYNCI)    

    //
    // Set qualification for selected pins to asynch only
    // This will select synch to SYSCLKOUT for the selected pins.
    // Comment out other unwanted lines.
    //
    
    //
    // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;
    
    //
    //Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)
    //
    //GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;   

    //
    // Configure EPwmSync pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPwmSync 
    // functional pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;  //Enable pull-up on GPIO6(EPWMSYNCI)
    //GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;//Enable pull-up on GPIO32(EPWMSYNCI)    

    //
    // Configure EPWMSYNC0
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    
    //
    // Enable pull-up on GPIO6 (EPWMSYNC0)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;
    
    //
    // Enable pull-up on GPIO33 (EPWMSYNC0)
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;

    //
    // Enable pull-up on GPIO6 (EPWMSYNC0)
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;
    
    //
    // Enable pull-up on GPIO33 (EPWMSYNC0)
    //
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;
}

//
// InitTzGpio -  This function initializes GPIO pins to function as Trip Zone
// (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
void 
InitTzGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ4)

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ5)

    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ6) 
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ6)  

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.  
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ4)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ5)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ5)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ6) 
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ6)  

    //
    // Configure TZ pins using GPIO regs
    // This specifies which of the possible GPIO pins will be TZ functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ4

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ5
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ5

    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ6               
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ6  

    EDIS;
}
//PWMģ���Ƶ����
//���������HSPclkdiv��ַ          ��&HSPclkdiv
//         CLKdiv��ַ                ��&CLKdiv
//         cpuFreq  ϵͳʱ��Ƶ��MHz ��90
//         pwmFreq  PWMƵ��Hz��        ��10*1000=10kHz
//���ز�������Ƶϵ��

Uint16 Freq_division( Uint8 *HSPclkdiv,Uint8 *CLKdiv, Uint8 cpuFreq, Uint32 pwmFreq)
{
    //����Ƶ��ѡ���Ƶϵ��
    Uint16 FREDDIV=0;
    if(pwmFreq >=((cpuFreq*1000000)/(2*65530))) //����ϵͳʱ�ӷ�Ƶ
           {
               *HSPclkdiv = 0;
               *CLKdiv = 0;
               FREDDIV=1;
           }
           else
           {
               if(pwmFreq >=((cpuFreq*1000000)/(2*2*65530))) //2��Ƶ
               {
                   *HSPclkdiv = 1;
                   *CLKdiv = 0;
                   FREDDIV=2;
                 }
               else
               {
                   if(pwmFreq >=((cpuFreq*1000000)/(4*2*65530))) //4��Ƶ
                   {
                       *HSPclkdiv = 2;
                       *CLKdiv = 0;
                       FREDDIV=4;
                   }
                   else
                   {
                         if(pwmFreq >=((cpuFreq*1000000)/(6*2*65530))) //6��Ƶ
                         {
                             *HSPclkdiv = 3;
                             *CLKdiv = 0;
                             FREDDIV=6;
                         }
                         else
                         {
                             if(pwmFreq >=((cpuFreq*1000000)/(8*2*65530))) //8��Ƶ
                              {
                                  *HSPclkdiv = 4;
                                  *CLKdiv = 0;
                                  FREDDIV=8;
                              }
                             else
                             {
                                 if(pwmFreq >=((cpuFreq*1000000)/(10*2*65530))) //10��Ƶ
                                  {
                                      *HSPclkdiv = 5;
                                      *CLKdiv = 0;
                                      FREDDIV=10;
                                  }
                                 else
                                 {
                                     if(pwmFreq >=((cpuFreq*1000000)/(12*2*65530))) //12��Ƶ
                                      {
                                          *HSPclkdiv = 6;
                                          *CLKdiv = 0;
                                          FREDDIV=12;
                                      }
                                     else
                                     {
                                         if(pwmFreq >=((cpuFreq*1000000)/(14*2*65530))) //14��Ƶ
                                           {
                                               *HSPclkdiv = 7;
                                               *CLKdiv = 0;
                                               FREDDIV=14;
                                           }
                                         else
                                         {
                                             if(pwmFreq >=((cpuFreq*1000000)/(16*2*65530))) //16��Ƶ
                                               {
                                                   *HSPclkdiv = 0;
                                                   *CLKdiv = 4;
                                                   FREDDIV=16;
                                               }
                                             else
                                             {
                                                 if(pwmFreq >=((cpuFreq*1000000)/(20*2*65530))) //20��Ƶ
                                                   {
                                                       *HSPclkdiv = 5;
                                                       *CLKdiv = 1;
                                                       FREDDIV=20;
                                                   }
                                                 else
                                                 {
                                                     if(pwmFreq >=((cpuFreq*1000000)/(24*2*65530))) //24��Ƶ
                                                       {
                                                           *HSPclkdiv = 3;
                                                           *CLKdiv = 2;
                                                           FREDDIV=24;
                                                       }
                                                     else
                                                     {
                                                         if(pwmFreq >=((cpuFreq*1000000)/(32*2*65530))) //32��Ƶ
                                                           {
                                                               *HSPclkdiv = 0;
                                                               *CLKdiv = 5;
                                                               FREDDIV=32;
                                                           }
                                                         else
                                                         {
                                                             if(pwmFreq >=((cpuFreq*1000000)/(40*2*65530))) //40��Ƶ
                                                               {
                                                                   *HSPclkdiv = 5;
                                                                   *CLKdiv = 2;
                                                                   FREDDIV=40;
                                                               }
                                                             else
                                                             {
                                                                 if(pwmFreq >=((cpuFreq*1000000)/(48*2*65530))) //48��Ƶ
                                                                   {
                                                                       *HSPclkdiv = 6;
                                                                       *CLKdiv = 2;
                                                                       FREDDIV=48;
                                                                   }
                                                                 else
                                                                 {
                                                                     if(pwmFreq >=((cpuFreq*1000000)/(56*2*65530))) //56��Ƶ
                                                                       {
                                                                           *HSPclkdiv = 7;
                                                                           *CLKdiv = 2;
                                                                           FREDDIV=56;
                                                                       }
                                                                     else
                                                                     {
                                                                         if(pwmFreq >=((cpuFreq*1000000)/(64*2*65530))) //64��Ƶ
                                                                           {
                                                                               *HSPclkdiv = 0;
                                                                               *CLKdiv = 6;
                                                                               FREDDIV=64;
                                                                           }
                                                                         else
                                                                         {
                                                                             if(pwmFreq >=((cpuFreq*1000000)/(80*2*65530))) //80��Ƶ
                                                                               {
                                                                                   *HSPclkdiv = 5;
                                                                                   *CLKdiv = 3;
                                                                                   FREDDIV=80;
                                                                               }
                                                                             else
                                                                             {
                                                                                 if(pwmFreq >=((cpuFreq*1000000)/(96*2*65530))) //96��Ƶ
                                                                                   {
                                                                                       *HSPclkdiv = 6;
                                                                                       *CLKdiv = 3;
                                                                                       FREDDIV=96;
                                                                                   }
                                                                                 else
                                                                                 {
                                                                                     if(pwmFreq >=((cpuFreq*1000000)/(112*2*65530))) //112��Ƶ
                                                                                       {
                                                                                           *HSPclkdiv = 7;
                                                                                           *CLKdiv = 3;
                                                                                           FREDDIV=112;
                                                                                       }
                                                                                     else
                                                                                     {
                                                                                         if(pwmFreq >=((cpuFreq*1000000)/(128*2*65530))) //128��Ƶ
                                                                                           {
                                                                                               *HSPclkdiv = 0;
                                                                                               *CLKdiv = 7;
                                                                                               FREDDIV=128;
                                                                                           }
                                                                                         else
                                                                                         {
                                                                                             if(pwmFreq >=((cpuFreq*1000000)/(144*2*65530))) //144��Ƶ
                                                                                               {
                                                                                                   *HSPclkdiv = 4;
                                                                                                   *CLKdiv = 4;
                                                                                                   FREDDIV=144;
                                                                                               }
                                                                                             else
                                                                                             {
                                                                                                 if(pwmFreq >=((cpuFreq*1000000)/(160*2*65530))) //160��Ƶ
                                                                                                   {
                                                                                                       *HSPclkdiv = 5;
                                                                                                       *CLKdiv = 4;
                                                                                                       FREDDIV=160;
                                                                                                   }
                                                                                                 else
                                                                                                 {
                                                                                                     if(pwmFreq >=((cpuFreq*1000000)/(192*2*65530))) //192��Ƶ
                                                                                                       {
                                                                                                           *HSPclkdiv = 6;
                                                                                                           *CLKdiv = 4;
                                                                                                           FREDDIV=192;
                                                                                                       }
                                                                                                     else
                                                                                                     {
                                                                                                         if(pwmFreq >=((cpuFreq*1000000)/(224*2*65530))) //224��Ƶ
                                                                                                           {
                                                                                                               *HSPclkdiv = 7;
                                                                                                               *CLKdiv = 4;
                                                                                                               FREDDIV=224;
                                                                                                           }
                                                                                                         else
                                                                                                         {
                                                                                                             if(pwmFreq >=((cpuFreq*1000000)/(256*2*65530))) //256��Ƶ
                                                                                                               {
                                                                                                                   *HSPclkdiv = 4;
                                                                                                                   *CLKdiv = 5;
                                                                                                                   FREDDIV=256;
                                                                                                               }
                                                                                                             else
                                                                                                             {
                                                                                                                 if(pwmFreq >=((cpuFreq*1000000)/(320*2*65530))) //320��Ƶ
                                                                                                                   {
                                                                                                                       *HSPclkdiv = 5;
                                                                                                                       *CLKdiv = 5;
                                                                                                                       FREDDIV=320;
                                                                                                                   }
                                                                                                                 else
                                                                                                                 {
                                                                                                                     if(pwmFreq >=((cpuFreq*1000000)/(384*2*65530))) //384��Ƶ
                                                                                                                       {
                                                                                                                           *HSPclkdiv = 6;
                                                                                                                           *CLKdiv = 5;
                                                                                                                           FREDDIV=384;
                                                                                                                       }
                                                                                                                     else
                                                                                                                     {
                                                                                                                         if(pwmFreq >=((cpuFreq*1000000)/(448*2*65530))) //448��Ƶ
                                                                                                                           {
                                                                                                                               *HSPclkdiv = 7;
                                                                                                                               *CLKdiv = 5;
                                                                                                                               FREDDIV=448;
                                                                                                                           }
                                                                                                                         else
                                                                                                                         {
                                                                                                                             if(pwmFreq >=((cpuFreq*1000000)/(512*2*65530))) //512��Ƶ
                                                                                                                               {
                                                                                                                                   *HSPclkdiv = 4;
                                                                                                                                   *CLKdiv = 6;
                                                                                                                                   FREDDIV=512;
                                                                                                                               }
                                                                                                                             else
                                                                                                                             {
                                                                                                                                 if(pwmFreq >=((cpuFreq*1000000)/(640*2*65530))) //640��Ƶ
                                                                                                                                   {
                                                                                                                                       *HSPclkdiv = 5;
                                                                                                                                       *CLKdiv = 6;
                                                                                                                                       FREDDIV=640;
                                                                                                                                   }
                                                                                                                                 else
                                                                                                                                 {

                                                                                                                                          *HSPclkdiv = 6;
                                                                                                                                           *CLKdiv = 6;
                                                                                                                                           FREDDIV=768;


                                                                                                                                 }

                                                                                                                             }

                                                                                                                         }

                                                                                                                     }

                                                                                                                 }

                                                                                                             }

                                                                                                         }

                                                                                                     }

                                                                                                 }

                                                                                             }

                                                                                         }

                                                                                     }

                                                                                 }

                                                                             }

                                                                         }

                                                                     }

                                                                 }

                                                             }

                                                         }

                                                     }

                                                 }

                                             }

                                         }

                                     }
                                 }

                             }
                         }
                    }
               }
           }
    return FREDDIV;
}

//
// End of file
//

