//###########################################################################
//
// FILE:   cpu01_to_cpu02_ipcdrivers_cpu01.c
//
// TITLE:  CPU01 to CPU02 IPC Driver TestExample
//
//! \addtogroup dual_example_list
//! <h1> CPU01 to CPU02 IPC Driver </h1>
//!
//! This example tests all of the basic read/write CPU01 to CPU02 IPC Driver
//! functions available in F2837xD_Ipc_Driver.c.
//! The CPU01 project sends commands to the CPU02 project, which then processes
//! the commands.
//! The CPU02 project responds to the commands sent from the CPU01 project.
//! Note that IPC INT0 and IPC INT1 are used for this example to process IPC
//! commands.
//!
//! \b  Watch \b Variables \b for \b CPU01 : \n
//!  - ErrorCount - Counts # of errors
//!  - pusCPU01BufferPt - Stores 256 16-bit words block to write to CPU02
//!  - pusCPU02BufferPt - Points to beginning of 256 word block received
//!                       back from CPU02
//!  - usWWord16 - 16-bit word to write to CPU02
//!  - ulWWord32 - 32-bit word to write to CPU02
//!  - usRWord16 - 16-bit word to read from CPU02
//!  - ulRWord32 - 32-bit word to read from CPU02
//!
//! \b  Watch \b Variables \b for \b CPU02 : \n
//!  - ErrorFlag  - Indicates an unrecognized command was sent from
//!                 CPU01 to CPU02.
//!
//


//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

//
// Defines
//
#define CPU02TOCPU01_PASSMSG  0x0003FBF4     // CPU02 to CPU01 MSG RAM offsets
#define CPU01TOCPU02_PASSMSG  0x0003FC00   // CPU1 → CPU2 settings

#define GS0SARAM_START        0xC000         // Start of GS0 SARAM



// Globals
//
// Received control parameters
uint16_t vset = 415;
uint16_t Iloop = 0;
uint16_t soft = 0;
uint16_t fault = 0;
uint16_t vyz = 415.0;

// Measured output voltage (to send back)
volatile uint16_t v_out = 0;


// example settings
uint16_t ADC_Result = 0,AdcResult_b=0 ;

uint32_t *pulMsgRamCPU2;   // for sending settings
uint32_t *pulMsgRamCPU1;   // for receiving ADC




void initGPIO2_3();
void initGPIO112();
void initGPIO50_51();

//
// Main
//
void
main(void)
{


//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_SysCtrl.c file and
// illustrates how to set the GPIO to it's default state.
//
 InitGpio();  // Skipped for this example

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



#ifdef _STANDALONE
#ifdef _FLASH
    //
    //  Send boot command to allow the CPU02 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
#else
    //
    //  Send boot command to allow the CPU02 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif
#endif

//
// Step 4. Initialize the Device Peripherals:
//

    // For this case just init GPIO pins for ePWM1
 // Only CPU1 can configure GPIO muxing so this is done here
 // These functions are in the F2837xD_EPwm.c file
 //
     InitEPwm1Gpio();
     InitEPwm2Gpio();
     InitEPwm5Gpio();
     initGPIO2_3();
     initGPIO112();
     initGPIO50_51();

     // Transfer ownership of EPWM1 and ADCA to CPU02
 //

     EALLOW;
     DevCfgRegs.CPUSEL0.bit.EPWM1 = 1;
     DevCfgRegs.CPUSEL0.bit.EPWM2 = 1;
     DevCfgRegs.CPUSEL0.bit.EPWM5 = 1;

     DevCfgRegs.CPUSEL11.bit.ADC_A = 1;
     DevCfgRegs.CPUSEL11.bit.ADC_B = 1;
     DevCfgRegs.CPUSEL11.bit.ADC_C = 1;
     DevCfgRegs.CPUSEL11.bit.ADC_D = 1;

     DevCfgRegs.CPUSEL14.bit.DAC_A = 1;
     DevCfgRegs.CPUSEL14.bit.DAC_B = 1;
     DevCfgRegs.CPUSEL14.bit.DAC_C = 1;
     EDIS;
 //
 // Enable CPU INT1 which is connected to Upper PIE IPC INT0-3:
 //
     IER |= M_INT1;


//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

//
// Initialize local variables

    pulMsgRamCPU1 = (void *)CPU02TOCPU01_PASSMSG;
    pulMsgRamCPU2 = (void *)CPU01TOCPU02_PASSMSG;



    // Wait for CPU2 to be ready
        while(!IPCRtoLFlagBusy(IPC_FLAG17));
        IPCRtoLFlagAcknowledge(IPC_FLAG17);



    for(;;)
    {


        // Send settings to CPU2
                //
                while(IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU2[0], (uint32_t)&vset ,
                                           IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);

//
//                while(IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU2[1], (uint32_t)&Iloop,
//                                           IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);
//
//                while(IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU2[2], (uint32_t)&soft,
//                                                           IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);
//
//                while(IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU2[3], (uint32_t)&fault,
//                                                           IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);
//
//                while(IPCLiteLtoRDataWrite(IPC_FLAG0, pulMsgRamCPU2[4], (uint32_t)&vyz,
//                                                           IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);


                //
                // Read ADC result from CPU2
                //
                while(IPCLiteLtoRDataRead(IPC_FLAG0, pulMsgRamCPU1[0],
                                          IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);

                while(IPCLiteLtoRGetResult(&ADC_Result, IPC_LENGTH_16_BITS,
                                           IPC_FLAG31) != STATUS_PASS);

                while(IPCLiteLtoRDataRead(IPC_FLAG0, pulMsgRamCPU1[2],
                                                          IPC_LENGTH_16_BITS, IPC_FLAG31) != STATUS_PASS);

                while(IPCLiteLtoRGetResult(&v_out, IPC_LENGTH_16_BITS,
                                                           IPC_FLAG31) != STATUS_PASS);



    }
}



/*Initializing GPIO pin 112 to low
 * To change step-up voltage settings*/
void initGPIO112()
{
    EALLOW;
    GpioCtrlRegs.GPDPUD.bit.GPIO112 = 1;
    GpioCtrlRegs.GPDMUX2.bit.GPIO112 = 0;
    GpioDataRegs.GPDCLEAR.bit.GPIO112 = 1;
    GpioCtrlRegs.GPDDIR.bit.GPIO112 = 1;
    EDIS;

}

void initGPIO50_51()
{
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1; //OUTPUT Pin
    EDIS;

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
    GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1; //OUTPUT Pin
    EDIS;
}

void initGPIO2_3()
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1; //OUTPUT Pin

    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1; //OUTPUT Pin
    EDIS;
}
//
// End of file
//
