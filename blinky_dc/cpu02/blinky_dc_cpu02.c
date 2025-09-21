
//
// Included Files
//
#include "Ac.h"
#include "F2837xD_Ipc_drivers.h"

#ifdef _FLASH
// These are defined by the linker (see device linker command file)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;
#endif

//
// Defines
//
#define CPU02TOCPU01_PASSMSG  0x0003FBF4    // Used by CPU02 to pass address
#define CPU01TOCPU02_PASSMSG  0x0003FC00   // CPU1 → CPU2 settings                                              // of local variables to perform
                                            // actions on

// Globals
//

double v_avg_temp_32g[3] = { 0.0f };
double v_avg_temp_1g[3] = { 0.0f };

#pragma DATA_SECTION(duty_rec, "SHARERAMGS4");
float duty_rec[1000];

#pragma DATA_SECTION(duty_temp_rec,"SHARERAMGS5");
float duty_temp_rec[1000];
//
// At least 1 volatile global tIpcController instance is required when using
// IPC API Drivers.
//
volatile tIpcController g_sIpcController1;
//volatile tIpcController g_sIpcController2;


// Globals
// Received control parameters
 uint16_t vset = 0;
 uint16_t Iloop = 0;
 uint16_t soft = 0;
 uint16_t fault = 0;
 uint16_t vyz = 415;

// Measured output voltage (to send back)
volatile uint16_t vout = 0,ErrorFlag=0;

volatile uint16_t AdcResult = 0,AdcResult_b = 0;

uint32_t *pulMsgRamCPU1;  // to CPU1
uint32_t *pulMsgRamCPU2;  // from CPU1
//

//
// Function Prototypes
//
__interrupt void CPU01toCPU02IPC0IntHandler(void);

void SetupADC_ppb();
void sogi_pll();
void sogi_pll_duty();
void sogi_pll_IL();
void low_pass_duty();

//#define  PWM_En GpioDataRegs.GPDCLEAR.bit.GPIO112 = 1;
//#define  PWM_Dis GpioDataRegs.GPDSET.bit.GPIO112 = 1;

#include "SOGIPLL.c"
//
// Main
//
void
main(void)
{


#ifdef _FLASH
memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);
#endif

// Initialize system control and clocks
InitSysCtrl();

#ifdef _FLASH
InitFlash();
#endif

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultISR.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();
    EALLOW;
        //Enable PWM1 PWM5 and ADC_A Clock
        CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
        CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
        CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;

        CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
        CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
        CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
        CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

        CpuSysRegs.PCLKCR16.bit.DAC_A = 1;
        CpuSysRegs.PCLKCR16.bit.DAC_B = 1;
        CpuSysRegs.PCLKCR16.bit.DAC_C = 1;
        EDIS;

        // Map ePWM ISR to the PIE vector table
        EALLOW;
        PieVectTable.EPWM1_INT = &epwm1_isr;
        //PieVectTable.EPWM2_INT = &epwm2_isr;
        PieVectTable.EPWM5_INT = &epwm5_isr;

        PieVectTable.TIMER0_INT = &cpu_timer0_isr;

        PieVectTable.ADCA1_INT = &adca0_isr;
        PieVectTable.ADCA2_INT = &adca3_isr;
        PieVectTable.ADCA3_INT = &adca4_isr;
        PieVectTable.ADCC3_INT = &adcc2_isr;
        EDIS;

        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
        EDIS;
        // Initialize ePWM1 and epwm5 module
            initEPwm5();
            // initEPwm2();
            initEPwm1();
            initADCA();

            init_dac();
            //SetupADC_ppb();
            EALLOW;
            CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
            EDIS;

            // Enable interrupt for PIE group 3, interrupt channel 1 (INT3.1)
            PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
            PieCtrlRegs.PIEIER3.bit.INTx2 = 1;

            // Enable interrupt for PIE group 3, interrupt channel 5 (INT3.5)
            PieCtrlRegs.PIEIER3.bit.INTx5 = 1;

            PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable PIE Group 1, Channel 1 (ADCA0)
            //PieCtrlRegs.PIEIER1.bit.INTx2 = 1; // Enable PIE Group 1, Channel 2 (ADCA3)
            PieCtrlRegs.PIEIER1.bit.INTx3 = 1; // Enable PIE Group 1, Channel 3 (ADCA4)
            PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

            // Enable CPU2 to CPU1 IPC INTn in the PIE: Group 1 interrupts
            //
            PieCtrlRegs.PIEIER1.bit.INTx13 = 1;    // CPU2 to CPU1 INT0
            PieCtrlRegs.PIEIER1.bit.INTx14 = 1;    // CPU2 to CPU1 INT1
//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.IPC0_INT = &CPU01toCPU02IPC0IntHandler;
 //   PieVectTable.IPC1_INT = &CPU01toCPU02IPC1IntHandler;
    EDIS;    // This is needed to disable write to EALLOW protected registers

//
// Step 4. Initialize the Device Peripherals:
//
    IPCInitialize(&g_sIpcController1, IPC_INT0, IPC_INT0);
   // IPCInitialize(&g_sIpcController2, IPC_INT1, IPC_INT1);

//
// Step 5. User specific code, enable interrupts:
//

   // Enable CPU INT1 which is connected to Upper PIE IPC INT0-3:
       // Enable EPWM and ADC interrupt in PIE
       IER |= M_INT3;
       IER |= M_INT1;
//
// Enable CPU01 to CPU02 INTn in the PIE: Group 11 interrupts
//
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;   // CPU1 to CPU2 INT0
  //  PieCtrlRegs.PIEIER1.bit.INTx14 = 1;   // CPU1 to CPU2 INT1

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM



//
// Point array to address in CPU02 TO CPU01 MSGRAM for passing
// variable locations
//// Initialize local variables

    pulMsgRamCPU1 = (void *)CPU02TOCPU01_PASSMSG;
    pulMsgRamCPU2 = (void *)CPU01TOCPU02_PASSMSG;



    // Place addresses for IPC usage
        pulMsgRamCPU2[0] = (uint32_t)&vset;
//        pulMsgRamCPU2[1] = (uint32_t)&Iloop;
//        pulMsgRamCPU2[2] = (uint32_t)&soft;
//        pulMsgRamCPU2[3] = (uint32_t)&fault;
//        pulMsgRamCPU2[4] =(uint32_t)&vyz;



        pulMsgRamCPU1[0] = (uint32_t)&AdcResult;
        pulMsgRamCPU1[2] =  (uint32_t)&vout;

        // Flag CPU1 that CPU2 is ready
        IPCLtoRFlagSet(IPC_FLAG17);

    do
       {
           //Start ePWM
           //     EPwm5Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
           EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode


       }
       while (1);
}


// CPU01 to CPU02 INT0 Interrupt Handler - Handles Data Word Reads/Writes
 //
 __interrupt void
 CPU01toCPU02IPC0IntHandler (void)
 {
     //
     // Continue processing messages
     //
     uint32_t command;
     command = IpcRegs.IPCRECVCOM;

     switch (command)
     {
         case IPC_SET_BITS_16:
             IPCLiteRtoLSetBits(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_SET_BITS_32:
             IPCLiteRtoLSetBits(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_CLEAR_BITS_16:
             IPCLiteRtoLClearBits(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_CLEAR_BITS_32:
             IPCLiteRtoLClearBits(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_WRITE_16:
             IPCLiteRtoLDataWrite(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_WRITE_32:
             IPCLiteRtoLDataWrite(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_SET_BITS_16_PROTECTED:
             IPCLiteRtoLSetBits_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_SET_BITS_32_PROTECTED:
             IPCLiteRtoLSetBits_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_CLEAR_BITS_16_PROTECTED:
             IPCLiteRtoLClearBits_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_CLEAR_BITS_32_PROTECTED:
             IPCLiteRtoLClearBits_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_WRITE_16_PROTECTED:
             IPCLiteRtoLDataWrite_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_WRITE_32_PROTECTED:
             IPCLiteRtoLDataWrite_Protected(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_READ_16:
             IPCLiteRtoLDataRead(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_DATA_READ_32:
             IPCLiteRtoLDataRead(IPC_FLAG0, IPC_FLAG31);
             break;
         case IPC_FUNC_CALL:
             IPCLiteRtoLFunctionCall(IPC_FLAG0, IPC_FLAG31);
             break;
         default:
             ErrorFlag = 1;
             break;
     }

     //
     // IPC Lite Driver Functions acknowledge the IPC interrupt.
     // There is no need to ACK the IPC interrupt flag separately.
     //
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
 }



/*Initializing ADC to generate sample's based on epwm5
 * The ADC SOC start of conversion is triggered by EPwm5 */
void initADCA()
{

    switch (mode)
    {
    case 0:
        EALLOW;

        AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;   // 12-bit resolution
        AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;   // Single-ended mode

        AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;   // 12-bit resolution
        AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;   // Single-ended mode

        AdccRegs.ADCCTL2.bit.RESOLUTION = 0;   // 12-bit resolution
        AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;   // Single-ended mode

        AdcdRegs.ADCCTL2.bit.RESOLUTION = 0;   // 12-bit resolution
        AdcdRegs.ADCCTL2.bit.SIGNALMODE = 0;   // Single-ended mode

        // Configure ADC prescaler, pulse position, and power-up
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6;     // ADC clock prescaler
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;  // Set interrupt pulse position
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;     // Power up the ADC

        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;     // ADC clock prescaler
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;  // Set interrupt pulse position
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;     // Power up the ADC

        // Configure ADC prescaler, pulse position, and power-up
        AdccRegs.ADCCTL2.bit.PRESCALE = 6;     // ADC clock prescaler
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;  // Set interrupt pulse position
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;     // Power up the ADC

        AdcdRegs.ADCCTL2.bit.PRESCALE = 6;     // ADC clock prescaler
        AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;  // Set interrupt pulse position
        AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;     // Power up the ADC

        DELAY_US(1000); // Delay for ADC to stabilize after power-up

        // Configure SOC1 for channel ADCA3
        AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;      // Channel 3 (ADCA3)
        AdcaRegs.ADCSOC3CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

        // Configure SOC2 for channel ADCA4
        AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;      // Channel 4 (ADCA4)
        AdcaRegs.ADCSOC4CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

        // Configure SOC1 for channel ADCB3
        AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;      // Channel 3 (ADCB3)
        AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

        // Configure SOC5 for channel ADCB5
        AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;      // Channel 5 (ADCB5)
        AdcbRegs.ADCSOC5CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

        // Configure SOC5 for channel ADCC5
        AdccRegs.ADCSOC5CTL.bit.CHSEL = 5;      // Channel 5 (ADCC5)
        AdccRegs.ADCSOC5CTL.bit.ACQPS = 99;     // Acquisition window size
        AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

        // Configure SOC5 for channel ADCD4
        AdcdRegs.ADCSOC4CTL.bit.CHSEL = 4;      // Channel 4 (ADCD4)
        AdcdRegs.ADCSOC4CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 0x05; // Trigger: ePWM1 SOCA

//        // Configure interrupt for end of SOC0
//        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Trigger INT1 at end of SOC0
//        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT1 flag
//
//        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  // Trigger INT1 at end of SOC0
//        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT1 flag
//
//        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  // Trigger INT1 at end of SOC0
//        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT1 flag
//
//
//        // Configure interrupt for end of SOC1
//        AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 3;  // Trigger INT2 at end of SOC1
//        AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;    // Enable INT2
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  // Clear INT2 flag

        // Configure ADCA interrupt for end of SOC4
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 4;  // Trigger INT3 at end of SOC4
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT3
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT3 flag

        // Configure ADCB interrupt for end of SOC5
        AdcbRegs.ADCINTSEL3N4.bit.INT3SEL = 5;  // Trigger INT3 at end of SOC5
        AdcbRegs.ADCINTSEL3N4.bit.INT3E = 1;    // Enable INT3
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;  // Clear INT3 flag

        // Configure ADCC interrupt for end of SOC5
        AdccRegs.ADCINTSEL3N4.bit.INT3SEL = 5;  // Trigger INT3 at end of SOC5
        AdccRegs.ADCINTSEL3N4.bit.INT3E = 1;    // Enable INT3
        AdccRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;  // Clear INT3 flag

        // Configure ADCD interrupt for end of SOC5
        AdcdRegs.ADCINTSEL3N4.bit.INT3SEL = 4;  // Trigger INT3 at end of SOC4
        AdcdRegs.ADCINTSEL3N4.bit.INT3E = 1;    // Enable INT3
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;  // Clear INT3 flag

        EDIS;

        break;
    case 1:
        EALLOW;

        AdcaRegs.ADCCTL2.bit.RESOLUTION = 1;   // 12-bit resolution
        AdcaRegs.ADCCTL2.bit.SIGNALMODE = 1;   // Single-ended mode

        // Configure ADC prescaler, pulse position, and power-up
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6;     // ADC clock prescaler
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;  // Set interrupt pulse position
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;     // Power up the ADC

        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      // Channel 0 (ADCA0)
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // Trigger: ePWM5 SOCA

        AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;      // Channel 1 (ADCA1)
        AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D; // Trigger: ePWM1 SOCA

        AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;      // Channel 2 (ADCA2)
        AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99;     // Acquisition window size
        AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0D; // Trigger: ePWM1 SOCA

        // Configure interrupt for end of SOC0
        //         AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Trigger INT1 at end of SOC0
        //         AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1
        //         AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT1 flag

        // Configure ADCA interrupt for end of SOC4
        AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 4;  // Trigger INT2 at end of SOC4
        AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;    // Enable INT2
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  // Clear INT2 flag

        EDIS;

        break;

    }
    // Set ADC resolution and signal mode
}

/* Function to initialize ePWM1 module for generating 25K Hz frequency PWM signal */
void initEPwm1()
{
    EALLOW;

    // Time-Base Submodule Configuration
    EPwm1Regs.TBPRD = epwm1_period_value;       // Set period for 25 kHz signal
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;            // Set phase to 0
    //EPwm1Regs.TBCTL.bit.SWFSYNC=1;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Set up-count mode
    EPwm1Regs.TBCTR = 0x0000;                      // Reset counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock divider = 1
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Clock divider = 1

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Counter Compare Submodule Configuration
    EPwm1Regs.CMPA.bit.CMPA = duty_db * epwm1_period_value; // Set compare value for ePWM1A
    EPwm1Regs.CMPB.bit.CMPB = duty_db * epwm1_period_value; // Set compare value for ePWM1B

    // Action Qualifier Submodule Configuration
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;    // Set PWM1A high at counter = 0
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // Clear PWM1A at counter = CMPA
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;  // Set PWM1B at counter = CMPB

    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // Clear PWM1B at counter = 0
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;  // Set PWM1B at counter = CMPB
    EPwm1Regs.AQCTLB.bit.PRD = AQ_CLEAR;  // Set PWM1B at counter = CMPB

//    EPwm1Regs.DBCTL.bit.OUT_MODE =DB_FULL_ENABLE; // Fully-Enable: Enable both A and B
//    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HI  ; // Active High Complementary
//    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//
//    EPwm1Regs.DBCTL2.bit.LOADDBCTLMODE=CC_CTR_ZERO;
//    EPwm1Regs.DBCTL2.bit.SHDWDBCTLMODE=CC_SHADOW;
//    EPwm1Regs.DBRED.bit.DBRED = Single_phase_deadband_1; // Rising Edge High Complementary
//    EPwm1Regs.DBFED.bit.DBFED = Single_phase_deadband_2; // Falling Edge High Complementary

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;          // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4; // Select SOC from from CPMA when counter = 0
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;

    // Enable ePWM interrupt on period match (TBCTR = 0)
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;  // Interrupt when TBCTR = 0
    EPwm1Regs.ETSEL.bit.INTEN = 1;             // Enable EPWM interrupt
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // Generate interrupt on the first event

    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm1Regs.TZSEL.bit.OSHT2 = 1;

    EDIS;

}
void initEPwm2()
{
    EALLOW;

    // Time-Base Submodule Configuration
    EPwm2Regs.TBPRD = epwm2_period_value;       // Set period for 25 kHz signal
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;            // Set phase to 0
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Set up-count mode
    EPwm2Regs.TBCTR = 0x0000;                      // Reset counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;       // Clock divider = 1
    EPwm2Regs.TBCTL.bit.CLKDIV = 4;          // Clock divider = 1

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Counter Compare Submodule Configuration
    EPwm2Regs.CMPA.bit.CMPA = epwm2_duty; // Set compare value for ePWM1A
    EPwm2Regs.CMPB.bit.CMPB = epwm2_duty; // Set compare value for ePWM1B

    // Action Qualifier Submodule Configuration
//    EPwm2Regs.DBCTL.bit.OUT_MODE=3;
//    EPwm2Regs.DBFED.bit.DBFED=low_frq_delay;
//    EPwm2Regs.DBRED.bit.DBRED=low_frq_delay;

    // Enable ePWM interrupt on period match (TBCTR = 0)
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;  // Interrupt when TBCTR = 0
    EPwm2Regs.ETSEL.bit.INTEN = 1;             // Enable EPWM interrupt
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST; // Generate interrupt on the first event

//    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
//    EPwm2Regs.TZSEL.bit.OSHT2 = 1;

    EDIS;

}

/*Function to initialize ePWM5 module for generating the samples at 100k Hz for fining the Maximum magnitude sample  */
void initEPwm5()
{
    EALLOW;

//    EPwm5Regs.ETSEL.bit.SOCAEN = 0;          // Disable SOC on A group
//    EPwm5Regs.ETSEL.bit.SOCASEL = 4; // Select SOC from from CPMA when counter = 0
//    EPwm5Regs.ETPS.bit.SOCAPRD = ET_1ST;

    EPwm5Regs.CMPA.bit.CMPA = 499; // Set compare alue for 50% duty cycle initially

    EPwm5Regs.TBPRD = epwm5_period_value; // Set period to generate 100 kHz signal for ADC sampling for 2000samples
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // freeze counter

    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;           // Phase is 0
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0

    EPwm5Regs.TBCTR = 0x0000;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // Clock divider = 1
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;       // Clock divider = 1

    // Enable ePWM interrupt on period match
    EPwm5Regs.ETSEL.bit.INTSEL = 4;  // Interrupt when TBCTR = 0
    EPwm5Regs.ETSEL.bit.INTEN = 1;             // Enable EPWM interrupt
    EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;     // Generate interrupt on first eent

    EDIS;

}

void SetupADC_ppb()
{

    EALLOW;
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;
    AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;
    AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 2;
    AdcaRegs.ADCPPB4CONFIG.bit.CONFIG = 3;

    AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 2;
    AdccRegs.ADCPPB2CONFIG.bit.CONFIG = 3;
    AdccRegs.ADCPPB3CONFIG.bit.CONFIG = 4;
    AdccRegs.ADCPPB4CONFIG.bit.CONFIG = 5;

//    AdcaRegs.ADCPPB1OFFREF = adca_ppb_a0_ref;
//    AdcaRegs.ADCPPB2OFFREF = adcc_ppb_a1_ref;
    EDIS;
}

/* ePWM ISR to update duty cycle */

/* ePWM5 ISR for sampling frequency 100 k Hz*/
__interrupt void epwm5_isr(void)
{

    EALLOW;
//GpioDataRegs.GPBSET.bit.GPIO50 = 1;
    EDIS;

    epwm5_sampling_sample_count = 0.02 * epwm5_sampling_freq;
    epwm5_period_value = 100000000 / epwm5_sampling_freq;
    EPwm5Regs.TBPRD = epwm5_period_value; // Set period to generate 100 kHz signal for ADC sampling for 2000samples

    sin_a = sinf(2 * M_PI * epwm5_sampling_count / epwm5_sampling_sample_count);

    x0_1g[0] = AdcaResultRegs.ADCRESULT3 * va_mul_factor;
    x1_1g[0] = AdcaResultRegs.ADCRESULT4 * va_mul_factor;
    x2_1g[0] = AdcbResultRegs.ADCRESULT3 * va_mul_factor;

    x0_32g[0] = AdccResultRegs.ADCRESULT5 * va_mul_factor;
    x1_32g[0] = AdcbResultRegs.ADCRESULT5 * va_mul_factor;

    x2_32g[0] = AdcdResultRegs.ADCRESULT4 * va_mul_factor;

    if ( AdcResult++ >= 4096) AdcResult = 0 ;
    if ( vout++ >= 2047) vout = 0 ;


    // Send to CPU1
    IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU1[0], (uint32_t)AdcResult,
                         IPC_LENGTH_16_BITS, IPC_FLAG31);

    // Send to CPU1
            IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU1[2], *((uint32_t*)&vout),
                                 IPC_LENGTH_16_BITS, IPC_FLAG31);



    if (cal_flag == 1)
    {
        I_L_1g_offset += x1_1g[0] * 0.001;
        I_L_32g_offset += x1_32g[0] * 0.001;
        cal_flag_cnt++;
    }
    if (cal_flag_cnt >= 1000)
    {
        cal_flag_cnt = 0;
        cal_flag = 0;
        i_in_alpha_2 = I_L_32g_offset;
        i_in_alpha_1 = I_L_1g_offset;
        I_L_32g_offset = 0;
        I_L_1g_offset = 0;
    }

//if(AdcbResultRegs.ADCRESULT5>1500 && AdcbResultRegs.ADCRESULT5<3500)
//{

    if (AdcbResultRegs.ADCRESULT5 >= 2248)
    {
        I_L_in = (x1_32g[0] - i_in_alpha_2) * i_in_beta_2;
    }
    else
    {
        I_L_in = (x1_32g[0] - i_in_alpha_3) * i_in_beta_3;
    }
////
////}
////else
////{
//    I_L_in=(x1_1g[0]-i_in_alpha_1)*i_in_beta_1;
//
////}

    if (IL_filt == 1)
    {
        I_L = 0.000314061 * I_L_in - 0.000314061 * duty2
                + 1.999330534 * duty_out1 - 0.99937094 * duty_out2;

        duty2 = duty1;
        duty1 = I_L_in;
        duty_out2 = duty_out1;
        duty_out1 = I_L;

    }

    else
    {
        I_L = I_L_in;
    }

    if (AdcdResultRegs.ADCRESULT4 > 100 && AdcdResultRegs.ADCRESULT4 < 3600)
    {
        v_out_in = (x2_32g[0] - v_out_alpha_2) * v_out_beta_2;
    }
    else
        v_out_in = (x2_1g[0] - v_out_alpha_1) * v_out_beta_1;

    if (vo_filt == 1)
    {
        v_out = alpha2 * (v_out_in) + (1 - alpha2) * v_out;
    }
    else

        v_out = v_out_in;

    if (v_out >= v_out_set || fabsf(I_L) >= I_L_set)
    {
        fault_flag = 1;
    }

    if (soft_start == 1)
    {

//================PLL Implementation ===========================
        switch (grid_ref)

        {
        case 0:

            vin_pll = (x0_1g[0] - v_in_alpha_1) * v_in_beta_1;

//   if (AdcaResultRegs.ADCRESULT3>=2300 && AdcaResultRegs.ADCRESULT3<=2700)
//    {
//       vin_pll=(x0_32g[0]-v_in_alpha_2)*v_in_beta_2;
//  }

            break;
        case 1:
            vin_pll = sinf(
                    2.0f * 3.14159265f * 50.0f * (float) epwm5_sampling_count
                            * 0.00002f);
            vin_pll *= v_max;

            vout_pll = -cosf(
                    2.0f * 3.14159265f * 50.0f * (float) epwm5_sampling_count
                            * 0.00002f);
            vout_pll *= v_max;

            break;
        }

        sogi_pll();

        v_in = vd_cos;

        if (Pwm_fault_flg == 0)
        {

//=============PFC_Soft_Coding===============

//Step:1 PLL_sin:
            Pll_sin = cos_theta;
            Pll_sin_db = Pll_sin;
//Step:2 PLL_with_deadband: Dely_step:15-->5.4_degree

            db_pll = 350 * cos_theta;

            if (db_pll > 0 && db_pll <= sin_degree1)
            {
                Pll_sin_db = 0;
            }
            if (db_pll > sin_degree1)
            {
                Pll_sin_db = (db_pll - sin_degree1) * 0.002857143;
            }

            if (db_pll < 0 && db_pll >= -sin_degree1)
            {
                Pll_sin_db = 0;
            }
            if (db_pll < -sin_degree1)
            {
                Pll_sin_db = (db_pll + sin_degree1) * 0.002857143;
            }

            EALLOW;
            EPwm1Regs.DBRED.bit.DBRED = Single_phase_deadband_1; // Rising Edge High Complementary
            EPwm1Regs.DBFED.bit.DBFED = Single_phase_deadband_2; // Falling Edge High Complementary

            EDIS;

        }
    }
    dac_out();

    if (epwm5_sampling_count >= epwm5_sampling_sample_count)
    {
        epwm5_sampling_count = 0;

    }

    epwm5_sampling_count++;
    Leg_a = GpioDataRegs.GPADAT.bit.GPIO27;
    Leg_b = GpioDataRegs.GPBDAT.bit.GPIO55;

    if (Pwm_fault_flg == 0)
    {
        if (Leg_a == 0 || Leg_b == 0)
        {

            Pwm_fault_flg = 1;
            EALLOW;
            EPwm1Regs.TZCTL.bit.TZA = 2;
            EPwm1Regs.TZCTL.bit.TZB = 2;
            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
            PWM_Dis
            ;
            EDIS;
        }
        if (Leg_a == 1 && Leg_b == 1)
        {
            EALLOW;
            PWM_En
            ;
            EDIS;
        }

    }
    //Pwm_fault_flg=0;

    if (I_loop == 0 || fault_flag == 1)
    {
        start = 0;
        EALLOW;
        EPwm1Regs.TZCTL.bit.TZA = 2;
        EPwm1Regs.TZCTL.bit.TZB = 2;
        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        PWM_Dis
        ;
        EDIS;
        duty_state = 1;
        p_flag_pre = 0, n_flag_pre = 0;
        p_flag = 0;
        n_flag = 0;
        duty_pre = 0;
        duty = 0;
        duty_temp_cnt = 0;
        duty_temp_flg = 0;
        I_loop = 0;
        off_flg3 = 0, off_flg2 = 0, off_flg1 = 0, on_flg1 = 0, on_flg2 = 0, on_flg3 =
                0, duty_flg = 0;
        n_on_flag = 1;
        p_on_flag = 0;
        p_db_cnt = 0;
        n_db_cnt = 0;
    }
    EALLOW;

//GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
    EDIS;

    EPwm5Regs.ETCLR.bit.INT = 1;

    // Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm1_isr(void)

{
    //Step:4 Duty with Dead Band with respect to PLL_sin

    if (I_loop == 1 && cos_theta <= 0.0087 && cos_theta > -0.0087)
    {
        start = 1;
    }

    if (I_loop == 1 && start == 1 && fault_flag == 0)
    {
        if (v_control == 1)
        {
            //=====Voltage Control loop=================
            v_err = v_set - v_out;

            v_intgl_max = kv_m_ftr * v_set;

            v_intgl_min = -kv_m_ftr * v_set;

            v_i += v_ki * v_err * dt;

            if (v_i >= v_intgl_max)
            {
                v_i = v_intgl_max;
            }
            if (v_i <= v_intgl_min)
            {
                v_i = v_intgl_min;
            }
            v_p = v_kp * v_err;

            I_Sett = 2 * (v_p + v_i) / vd;

        }

        if (I_Sett > I_Set_max)
            I_Sett = I_Set_max;
        if (I_Sett < -I_Set_max)
            I_Sett = -I_Set_max;

        I_Set = I_Sett * Pll_sin_db;

        //Step:3 Duty Computation with Feed Forward:

        I_Err = I_Set - I_L;

        duty_in = I_Err;

        duty_out = kp_i * I_Err;

        if (pr_control_flg == 1)
        {
            sogi_pll_duty();
        }

        f_f = (v_in / v_out);

        duty = (f_f_gain * f_f) - duty_out;

        duty_temp1 = duty;

        duty_temp = mul_fact * duty_temp1;

        //=======duty record=============
        duty_temp_rec[duty_cnt] = duty_temp;
        duty_rec[duty_cnt] = duty_db;
        duty_cnt++;

        if (duty_cnt >= duty_cnt_flg)
        {
            duty_cnt = 0;
        }

        //===============End==================
        switch (pll_duty)
        {

        case 0:

            if (duty_temp_flg == 1 && duty_temp > sin_degree_2)
            {
                duty_temp_flg = 0;
            }

            if (duty_temp_pre > sin_degree_2 && duty_temp_flg == 0)
            {
                if (db_flag == 1)
                {
                    DELAY_US(delay);
                }
                p_flag = 1;
                n_flag = 0;
                db_flag = 0;
            }

            if (duty_temp_flg == 2 && duty_temp < -sin_degree_2)
            {
                duty_temp_flg = 0;
            }
            if (duty_temp_pre < -sin_degree_2 && duty_temp_flg == 0)
            {
                if (db_flag == 1)
                {
                    DELAY_US(delay);
                }
                p_flag = 0;
                n_flag = 1;
                db_flag = 0;
            }

            if (duty_temp_pre > sin_degree_2 && duty_temp <= sin_degree_2)
            {

                duty_temp_flg = 2;

            }
            if (duty_temp_pre < -sin_degree_2 && duty_temp >= -sin_degree_2)
            {

                duty_temp_flg = 1;

            }
            if (duty_temp_flg == 1)
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            if (duty_temp_flg == 2)
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            break;

        case 1:

            if (duty_temp <= sin_degree_2 && duty_temp >= -sin_degree_3)
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }

            else if (duty_temp > sin_degree_4)
            {

                p_flag = 1;
                n_flag = 0;
                db_flag = 0;
            }
            else if (duty_temp < -sin_degree_5)
            {

                p_flag = 0;
                n_flag = 1;
                db_flag = 0;
            }
            break;
        case 2:

            if ((duty_temp_pre <= -sin_degree_2 && duty_temp > -sin_degree_2
                    && duty_temp < 0)
                    || (duty_temp_pre >= sin_degree_3
                            && duty_temp < sin_degree_3 && duty_temp > 0))
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            else if (duty_temp > sin_degree_4)
            {
                p_flag = 1;
                n_flag = 0;
                db_flag = 0;
            }
            else if (duty_temp < -sin_degree_5)
            {
                p_flag = 0;
                n_flag = 1;
                db_flag = 0;
            }

            break;

        case 3:

            if (duty_temp >= 0 && duty_temp > sin_degree_2)

            {
                p_flag = 1;
                n_flag = 0;
                db_flag = 0;

            }

            else if (duty_temp < 0 && duty_temp < sin_degree_3)
            {
                p_flag = 0;
                n_flag = 1;
                db_flag = 0;

            }
            else
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
            }

            break;
        case 4:

            if (duty_temp >= sin_degree_4)
            {
                p_db_cnt++;
                n_db_cnt = 0;
            }

            else if (duty_temp <= sin_degree_5)
            {
                n_db_cnt++;
                p_db_cnt = 0;
            }
            else
            {
                p_db_cnt = 0;
                n_db_cnt = 0;
            }

            if (duty_temp >= 0 && duty_temp <= sin_degree_4)

            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            else if (duty_temp <= 0 && duty_temp >= sin_degree_5)
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
                off_flg1 = 0;
            }
            else
            {
                db_flag = 0;
            }

            if (duty_temp >= 0 && duty_temp > sin_degree_2
                    && p_db_cnt >= p_db_cnt_set && db_flag == 0)
            {
                p_flag = 1;
                n_flag = 0;
                db_flag = 0;
            }

            if (duty_temp < 0 && duty_temp < sin_degree_3
                    && n_db_cnt >= n_db_cnt_set && db_flag == 0)
            {
                p_flag = 0;
                n_flag = 1;
                db_flag = 0;
            }

            break;
        case 5:

            if (duty_temp > sin_degree_3 && duty_temp <= 0 && duty_flg == 0
                    && on_flg1 == 0)
            {
                duty_flg = 1;
                on_flg1 = 1;

                db_flag = 1;
                p_flag = 0;
                n_flag = 0;

            }
            if (duty_temp >= 0 && duty_temp < sin_degree_2 && duty_flg == 1
                    && on_flg2 == 0)
            {
                duty_flg = 2;
                on_flg2 = 1;
            }
            if (duty_temp > sin_degree_2 && duty_flg == 2 && on_flg3 == 0)
            {
                duty_flg = 3;
                on_flg3 = 1;
                off_flg1 = 0;
                off_flg2 = 0;
                off_flg3 = 0;

                db_flag = 0;
                p_flag = 1;
                n_flag = 0;

            }
            if (duty_temp <= sin_degree_2 && duty_temp >= 0 && duty_flg == 3
                    && off_flg1 == 0)
            {
                duty_flg = 4;
                off_flg1 = 1;

                db_flag = 1;
                p_flag = 0;
                n_flag = 0;

            }
            if (duty_temp <= 0 && duty_temp >= sin_degree_3 && duty_flg == 4
                    && off_flg2 == 0)
            {
                duty_flg = 5;
                off_flg2 = 1;
            }
            if (duty_temp <= sin_degree_3 && duty_flg == 5 && off_flg3 == 0)
            {
                duty_flg = 0;
                off_flg3 = 1;
                on_flg1 = 0;
                on_flg2 = 0;
                on_flg3 = 0;

                db_flag = 0;
                p_flag = 0;
                n_flag = 1;

            }

            break;
        case 6:

            // Main logic
            if (duty_temp > POSITIVE_THRESHOLD_HIGH)
            {
                // Clear negative counter, increment positive counter
                n_db_cnt = 0;
                p_db_cnt++;

                // Only switch if count exceeds threshold
                if (p_db_cnt >= DEADBAND_COUNT_SET)
                {
                    p_flag = 1;
                    n_flag = 0;
                    db_flag = 0;
                }
            }
            else if (duty_temp < NEGATIVE_THRESHOLD_HIGH)
            {
                // Clear positive counter, increment negative counter
                p_db_cnt = 0;
                n_db_cnt++;

                // Only switch if count exceeds threshold
                if (n_db_cnt >= DEADBAND_COUNT_SET)
                {
                    p_flag = 0;
                    n_flag = 1;
                    db_flag = 0;
                }
            }
            else if (duty_temp < POSITIVE_THRESHOLD_LOW
                    && duty_temp > NEGATIVE_THRESHOLD_LOW)
            {
                // Deadband zone
                p_db_cnt = 0;
                n_db_cnt = 0;
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            else
            {
                // Intermediate zone - don't change state but don't count either
                p_db_cnt = 0;
                n_db_cnt = 0;
            }

            break;
        case 7:

            // Main logic
            if (duty_temp >= 0)
            {

                p_db_cnt++;
                n_db_cnt = 0;
                // Only switch if count exceeds threshold
                if (p_db_cnt >= DEADBAND_COUNT_SET
                        && duty_temp > POSITIVE_THRESHOLD_HIGH)
                {
                    p_flag = 1;
                    n_flag = 0;
                    db_flag = 0;
                    n_db_cnt = 0;
                    dead_band_cnt = 0;
                }
                else
                {
                    p_flag = 0;
                    n_flag = 0;
                    db_flag = 1;
                    duty_db = 0;

                }
            }
            else if (duty_temp < 0)
            {

                n_db_cnt++;
                p_db_cnt = 0;
                // Only switch if count exceeds threshold
                if (n_db_cnt >= DEADBAND_COUNT_SET
                        && duty_temp < NEGATIVE_THRESHOLD_HIGH)
                {
                    p_flag = 0;
                    n_flag = 1;
                    db_flag = 0;
                    p_db_cnt = 0;
                    dead_band_cnt = 0;
                }
                else
                {
                    p_flag = 0;
                    n_flag = 0;
                    db_flag = 1;
                    duty_db = 0;
                }
            }

            break;
        case 8:
            // Main logic
            if (duty_temp > 0 && p_on_flag == 0 && n_on_flag == 1
                    && p_db_cnt < p_db_cnt_set)
            {
                p_db_cnt++;
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }

            if (p_db_cnt >= p_db_cnt_set && p_on_flag == 0)
            {
                p_flag = 1;
                n_flag = 0;
                db_flag = 0;
                p_on_flag = 1;
            }

            if (duty_temp < 0 && n_on_flag == 0 && p_on_flag == 1
                    && n_db_cnt < n_db_cnt_set)
            {
                n_db_cnt++;
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                duty_db = 0;
            }
            if (n_db_cnt >= n_db_cnt_set && n_on_flag == 0)
            {
                p_flag = 0;
                n_flag = 1;
                db_flag = 0;
                n_on_flag = 1;
            }
            if (duty_temp > 0.5)
            {
                n_on_flag = 0;
                n_db_cnt = 0;
            }

            if (duty_temp < -0.5)
            {
                p_on_flag = 0;
                p_db_cnt = 0;
            }
            break;
        case 9:
            // Main logic

            if (duty >= 1)
                duty = 1;
            if (duty <= -1)
                duty = -1;

            if (db_pll <= sin_degree1 && db_pll >= -sin_degree1)
            {
                p_flag = 0;
                n_flag = 0;
                db_flag = 1;
                p_db_cnt = 0;
                n_db_cnt = 0;
            }

            if (db_pll >= POSITIVE_THRESHOLD_HIGH)

            {
                n_db_cnt = 0;
                p_db_cnt++;

                // Only switch if count exceeds threshold
                if (p_db_cnt >= DEADBAND_COUNT_SET)
                {
                    p_flag = 1;
                    n_flag = 0;
                    db_flag = 0;
                }
            }

            else if (db_pll <= NEGATIVE_THRESHOLD_HIGH)

            { // Clear positive counter, increment negative counter
                p_db_cnt = 0;
                n_db_cnt++;

                // Only switch if count exceeds threshold
                if (n_db_cnt >= DEADBAND_COUNT_SET)
                {
                    p_flag = 0;
                    n_flag = 1;
                    db_flag = 0;
                }
            }

            break;

        }
        duty_temp_pre = duty_temp;

        //==========Dead Band===================

        if (db_flag == 1)
        {

            EALLOW;

            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;    //  Lw_Fz_Bot_Switch
            GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;   // Lw_fz_Top_Switch
            EPwm1Regs.TZCTL.bit.TZA = 2;       //HI_FZ_Top_Switch
            EPwm1Regs.TZCTL.bit.TZB = 2;      //HI_FZ_Bot_Switch

            EDIS;
            duty_db = 0;

        }

        //==========Positive Cycle===================

        if (p_flag == 1)
        {
            EALLOW;

            GpioDataRegs.GPASET.bit.GPIO2 = 1;    //  Lw_Fz_Bot_Switch
            GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;   // Lw_fz_Top_Switch
            if (p_db_cnt > p_db_cnt_set)
            {
                EPwm1Regs.TZCTL.bit.TZA = 3;       //HI_FZ_Top_Switch
            }

            EPwm1Regs.TZCTL.bit.TZB = 3;      //HI_FZ_Bot_Switch

            duty_db = duty;
            EDIS;
        }
        //==========Nagative Cycle===================

        if (n_flag == 1)
        {

            EALLOW;
            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
            GpioDataRegs.GPASET.bit.GPIO3 = 1;
            EPwm1Regs.TZCTL.bit.TZA = 3;
            if (n_db_cnt > n_db_cnt_set)
            {
                EPwm1Regs.TZCTL.bit.TZB = 3;
            }

            if (duty < 0)
            {
                duty_db = 1.0f + duty;
            }
            else
                duty_db = 1.0f;
            EDIS;
        }

        if (duty_db >= duty_max)
        {
            duty_db = duty_max;
        }
        if (duty_db <= duty_min)
        {
            duty_db = duty_min;
        }
        EALLOW;
        EPwm1Regs.CMPA.bit.CMPA = (duty_db * epwm1_period_value); // Set compare value for ePWM1A
        EPwm1Regs.CMPB.bit.CMPB = (duty_db * epwm1_period_value); // Set compare value for ePWM1B
        EDIS;
    }

    // Clear the interrupt flag
    EALLOW;
    EPwm1Regs.ETCLR.bit.INT = 1;
    EDIS;
    // Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void epwm2_isr(void)
{

    // Clear the interrupt flag
    EPwm2Regs.ETCLR.bit.INT = 1;
    // Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void adca0_isr(void)
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE Group 1
}

__interrupt void adca3_isr(void)
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE Group 1
}

__interrupt void adca4_isr(void)
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE Group 1
}

__interrupt void adcc2_isr(void)
{

    //  AdccRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE Group 1
}

__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void init_dac(void)
{

    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage

    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage

    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
    DaccRegs.DACCTL.bit.LOADMODE = 0;
    DaccRegs.DACCTL.bit.DACREFSEL = 1;

    EDIS;

}

void dac_out(void)
{

    switch (dq_o)
    {
    case 0:
        DAC_C0 = cos_theta;
        DAC_A0 = v_alpha * 0.00375;
        DAC_B0 = v_beta * 0.00375;

        if (DAC_A0 > 1.5)
            DAC_A0 = 1.5;
        if (DAC_A0 < -1.5)
            DAC_A0 = -1.5;
        DacaRegs.DACVALS.bit.DACVALS = 2047 + DAC_A0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_B0 > 1.5)
            DAC_B0 = 1.5;
        if (DAC_B0 < -1.5)
            DAC_B0 = -1.5;
        DacbRegs.DACVALS.bit.DACVALS = 2047 + DAC_B0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_C0 > 1.5)
            DAC_C0 = 1.5;
        if (DAC_C0 < -1.5)
            DAC_C0 = -1.5;
        DaccRegs.DACVALS.bit.DACVALS = 2047 + DAC_C0 * 1365; // perform scaling of 0-3 to 0-4095

        break;
    case 1:
        DAC_C0 = duty;
        DAC_A0 = cos_theta;
        //  DAC_B0= vd*0.0075;

        if (DAC_A0 >= 1.5)
            DAC_A0 = 1.5;
        if (DAC_A0 <= -1.5)
            DAC_A0 = -1.5;
        DacaRegs.DACVALS.bit.DACVALS = 2047 + DAC_A0 * 1365; // perform scaling of 0-3 to 0-4095

//        if (DAC_B0 >3) DAC_B0 = 3;
//        if (DAC_B0 < 0) DAC_B0 = 0;
//        DacbRegs.DACVALS.bit.DACVALS =2047+DAC_B0*1365; // perform scaling of 0-3 to 0-4095

        if (DAC_C0 > 1.5)
            DAC_C0 = 1.5;
        if (DAC_C0 < -1.5)
            DAC_C0 = -1.5;
        DaccRegs.DACVALS.bit.DACVALS = 2047 + DAC_C0 * 1365; // perform scaling of 0-3 to 0-4095

        break;
    case 2:
        DAC_C0 = v_out * 0.00375;
        DAC_A0 = I_L * 0.06;
        DAC_B0 = v_in * 0.003;

//       DAC_C0=0;
//        DAC_A0=0;
//        DAC_B0=0;
        if (DAC_A0 > 1.5)
            DAC_A0 = 1.5;
        if (DAC_A0 < -1.5)
            DAC_A0 = -1.5;
        DacaRegs.DACVALS.bit.DACVALS = 2047 + DAC_A0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_B0 > 1.5)
            DAC_B0 = 1.5;
        if (DAC_B0 < -1.5)
            DAC_B0 = -1.5;
        DacbRegs.DACVALS.bit.DACVALS = 2047 + DAC_B0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_C0 > 1.5)
            DAC_C0 = 1.5;
        if (DAC_C0 < -1.5)
            DAC_C0 = -1.5;
        DaccRegs.DACVALS.bit.DACVALS = 2047 + DAC_C0 * 1365; // perform scaling of 0-3 to 0-4095

        break;

    case 3:
        DAC_C0 = cos_theta;
        DAC_A0 = v_in * 0.00375;
        DAC_B0 = theta * 0.237;

        if (DAC_A0 > 1.5)
            DAC_A0 = 1.5;
        if (DAC_A0 < -1.5)
            DAC_A0 = -1.5;
        DacaRegs.DACVALS.bit.DACVALS = 2047 + DAC_A0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_B0 > 1.5)
            DAC_B0 = 1.5;
        if (DAC_B0 < -1.5)
            DAC_B0 = -1.5;
        DacbRegs.DACVALS.bit.DACVALS = 2047 + DAC_B0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_C0 > 1.5)
            DAC_C0 = 1.5;
        if (DAC_C0 < -1.5)
            DAC_C0 = -1.5;
        DaccRegs.DACVALS.bit.DACVALS = 2047 + DAC_C0 * 1365; // perform scaling of 0-3 to 0-4095

        break;
    case 4:
        DAC_C0 = I_Set * 0.06;
        DAC_A0 = I_L * 0.075;
        DAC_B0 = vin_pll * 0.0037;

        if (DAC_A0 > 1.5)
            DAC_A0 = 1.5;
        if (DAC_A0 < -1.5)
            DAC_A0 = -1.5;
        DacaRegs.DACVALS.bit.DACVALS = 2047 + DAC_A0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_B0 > 1.5)
            DAC_B0 = 1.5;
        if (DAC_B0 < -1.5)
            DAC_B0 = -1.5;
        DacbRegs.DACVALS.bit.DACVALS = 2047 + DAC_B0 * 1365; // perform scaling of 0-3 to 0-4095

        if (DAC_C0 > 1.5)
            DAC_C0 = 1.5;
        if (DAC_C0 < -1.5)
            DAC_C0 = -1.5;
        DaccRegs.DACVALS.bit.DACVALS = 2047 + DAC_C0 * 1365; // perform scaling of 0-3 to 0-4095

        break;
    }

}
//
// End of file
//
//
// End of file
//
