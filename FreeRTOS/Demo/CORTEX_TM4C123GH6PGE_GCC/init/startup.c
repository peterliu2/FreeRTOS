//*****************************************************************************
//
// startup.c - Boot code for Stellaris.
//
// Copyright (c) 2005-2007 Luminary Micro, Inc.  All rights reserved.
//
// Software License Agreement
//
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
//
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  Any use in violation
// of the foregoing restrictions may subject the user to criminal sanctions
// under applicable laws, as well as to civil liability for the breach of the
// terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 1049 of the Stellaris Driver Library.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"


//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vUART_ISR( void );
extern void vGPIO_ISR( void );
extern void vPortSVCHandler( void );

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
#ifndef STACK_SIZE
#define STACK_SIZE                              256
#endif
static unsigned long pulStack[STACK_SIZE];

//*****************************************************************************
//
// The minimal vector table for a Cortex-M3.  Note that the proper constructs
// must be placed on this to ensure that it ends up at physical address
// 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)),
    // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    vPortSVCHandler,                        // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    xPortPendSVHandler,                     // The PendSV handler
    xPortSysTickHandler,                    // The SysTick handler
    /* */
    IntDefaultHandler,                      // 0:  GPIO Port A
    IntDefaultHandler,                      // 1:  GPIO Port B
    IntDefaultHandler,						// 2:  GPIO Port C
    IntDefaultHandler,                      // 3:  GPIO Port D
    IntDefaultHandler,                      // 4:  GPIO Port E
    vUART_ISR,								// 5:  UART0 Rx and Tx
    IntDefaultHandler,                      // 6:  UART1 Rx and Tx
    IntDefaultHandler,                      // 7:  SSI Rx and Tx
    IntDefaultHandler,                      // 8:  I2C Master and Slave
    IntDefaultHandler,                      // 9:  PWM Fault
    IntDefaultHandler,                      // 10: PWM Generator 0
    IntDefaultHandler,                      // 11: PWM Generator 1
    IntDefaultHandler,                      // 12: PWM Generator 2
    IntDefaultHandler,                      // 13: Quadrature Encoder
    IntDefaultHandler,                      // 14: ADC Sequence 0
    IntDefaultHandler,                      // 15: ADC Sequence 1
    IntDefaultHandler,                      // 16: ADC Sequence 2
    IntDefaultHandler,                      // 17: ADC Sequence 3
    IntDefaultHandler,                      // 18: Watchdog timer
    IntDefaultHandler,                      // 19: Timer 0 subtimer A
    IntDefaultHandler,                      // 20: Timer 0 subtimer B
    IntDefaultHandler,                      // 21: Timer 1 subtimer A
    IntDefaultHandler,                      // 22: Timer 1 subtimer B
    IntDefaultHandler,                      // 23: Timer 2 subtimer A
    IntDefaultHandler,                      // 24: Timer 2 subtimer B
    IntDefaultHandler,                      // 25: Analog Comparator 0
    IntDefaultHandler,                      // 26: Analog Comparator 1
    IntDefaultHandler,                      // 27: Analog Comparator 2
    IntDefaultHandler,                      // 28: System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // 29: FLASH Control
    IntDefaultHandler,                      /* 30:  GPIO Port F				*/
    IntDefaultHandler,                      /* 31:  GPIO Port G				*/
    IntDefaultHandler,                      /* 32:  GPIO Port H				*/
    IntDefaultHandler,                      /* 33:  UART 2					*/
    IntDefaultHandler,						/* 34:  SSI1					*/
    IntDefaultHandler,                      /* 35:  16/32-Bit Timer 3A		*/
    IntDefaultHandler,                      /* 36:  16/32-Bit Timer 3B		*/
    IntDefaultHandler,                      /* 37:  I2C1					*/
    IntDefaultHandler,                      /* 38:  QEI1					*/
    IntDefaultHandler,						/* 39:  CAN0					*/
    IntDefaultHandler,						/* 40:  CAN1					*/
    0,										/* 41:  Reserved				*/
    0,										/* 42:  Reserved				*/
    IntDefaultHandler,                      /* 43:  Hibernation Module		*/
    IntDefaultHandler,						/* 44:  Hibernation Module		*/
    IntDefaultHandler,                      /* 45:  PWM Generator 3			*/
    IntDefaultHandler,						/* 46:  uDMA Software			*/
    IntDefaultHandler,						/* 47:  uDMA Error				*/
    IntDefaultHandler,						/* 48:  ADC1 Sequence 0			*/
    IntDefaultHandler,						/* 49:  ADC1 Sequence 1			*/
    IntDefaultHandler,						/* 50:  ADC1 Sequence 2			*/
    IntDefaultHandler,						/* 51:  ADC1 Sequence 3			*/
    0,										/* 52:  Reserved				*/
    0,										/* 53:  Reserved				*/
    IntDefaultHandler,                      /* 54:  GPIO Port J				*/
    IntDefaultHandler,                      /* 55:  GPIO Port K				*/
    IntDefaultHandler,                      /* 56:  GPIO Port L				*/
    IntDefaultHandler,						/* 57:  SSI2					*/
    IntDefaultHandler,						/* 58:  SSI3					*/
    IntDefaultHandler,                      /* 59:  UART 3					*/
    IntDefaultHandler,                      /* 60:  UART 4					*/
    IntDefaultHandler,                      /* 61:  UART 5					*/
    IntDefaultHandler,                      /* 62:  UART 6					*/
    IntDefaultHandler,                      /* 63:  UART 7					*/
    0,										/* 64:  Reserved				*/
    0,										/* 65:  Reserved				*/
    0,										/* 66:  Reserved				*/
    0,										/* 67:  Reserved				*/
    IntDefaultHandler,                      /* 68:  I2C2					*/
    IntDefaultHandler,                      /* 69:  I2C3					*/
    IntDefaultHandler,                      /* 70:  16/32-Bit Timer 4A		*/
    IntDefaultHandler,                      /* 71:  16/32-Bit Timer 4B		*/
    0,										/* 72:  Reserved				*/
    0,										/* 73:  Reserved				*/
    0,										/* 74:  Reserved				*/
    0,										/* 75:  Reserved				*/
    0,										/* 76:  Reserved				*/
    0,										/* 77:  Reserved				*/
    0,										/* 78:  Reserved				*/
    0,										/* 79:  Reserved				*/
    0,										/* 80:  Reserved				*/
    0,										/* 81:  Reserved				*/
    0,										/* 82:  Reserved				*/
    0,										/* 83:  Reserved				*/
    0,										/* 84:  Reserved				*/
    0,										/* 85:  Reserved				*/
    0,										/* 86:  Reserved				*/
    0,										/* 87:  Reserved				*/
    0,										/* 88:  Reserved				*/
    0,										/* 89:  Reserved				*/
    0,										/* 90:  Reserved				*/
    0,										/* 91:  Reserved				*/
    IntDefaultHandler,                      /* 92:  16/32-Bit Timer 5A		*/
    IntDefaultHandler,                      /* 93:  16/32-Bit Timer 5B		*/
    IntDefaultHandler,						/* 94:  32/64-Bit Timer 0A		*/
    IntDefaultHandler,						/* 95:  32/64-Bit Timer 0B		*/
    IntDefaultHandler,						/* 96:  32/64-Bit Timer 1A		*/
    IntDefaultHandler,						/* 97:  32/64-Bit Timer 1B		*/
    IntDefaultHandler,						/* 98:  32/64-Bit Timer 2A		*/
    IntDefaultHandler,						/* 99:  32/64-Bit Timer 2B		*/
    IntDefaultHandler,						/* 100: 32/64-Bit Timer 3A		*/
    IntDefaultHandler,						/* 101: 32/64-Bit Timer 3B		*/
    IntDefaultHandler,						/* 102: 32/64-Bit Timer 4A		*/
    IntDefaultHandler,						/* 103: 32/64-Bit Timer 4B		*/
    IntDefaultHandler,						/* 104: 32/64-Bit Timer 5A		*/
    IntDefaultHandler,						/* 105: 32/64-Bit Timer 5B		*/
    IntDefaultHandler,						/* 106: System Exception(imprecise)*/
    0,										/* 107: Reserved				*/
    0,										/* 108: Reserved				*/
    IntDefaultHandler,                      /* 109: I2C4					*/
    IntDefaultHandler,                      /* 110: I2C5					*/
    vGPIO_ISR,                      /* 111: GPIO Port M				*/
    IntDefaultHandler,                      /* 112: GPIO Port N				*/
    0,										/* 113: Reserved				*/
    0,										/* 114: Reserved				*/
    0,										/* 115: Reserved				*/
    IntDefaultHandler,                      /* 116: GPIO Port P(Summary or P0)*/
    IntDefaultHandler,                      /* 117: GPIO Port P1			*/
    IntDefaultHandler,                      /* 118: GPIO Port P2			*/
    IntDefaultHandler,                      /* 119: GPIO Port P3			*/
    IntDefaultHandler,                      /* 120: GPIO Port P4			*/
    IntDefaultHandler,                      /* 121: GPIO Port P5			*/
    IntDefaultHandler,                      /* 122: GPIO Port P6			*/
    IntDefaultHandler,                      /* 123: GPIO Port P7			*/
    0,										/* 124: Reserved				*/
    0,										/* 125: Reserved				*/
    0,										/* 126: Reserved				*/
    0,										/* 127: Reserved				*/
    0,										/* 128: Reserved				*/
    0,										/* 129: Reserved				*/
    0,										/* 130: Reserved				*/
    0,										/* 131: Reserved				*/
    0,										/* 132: Reserved				*/
    0,										/* 133: Reserved				*/
    IntDefaultHandler,                      /* 134: PWM1 Generator 0		*/
    IntDefaultHandler,                      /* 135: PWM1 Generator 1		*/
    IntDefaultHandler,                      /* 136: PWM1 Generator 2		*/
    IntDefaultHandler,                      /* 137: PWM1 Generator 2		*/
    IntDefaultHandler,						/* 138: PWM1 Fault				*/
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _ldata;
//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied main() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void ResetISR(void)
{
    uint32_t *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_ldata;
    for(pulDest = &_data; pulDest < &_edata; ) {
        *pulDest++ = *pulSrc++;
    }

    //
    // Zero fill the bss segment.
    //
    //for(pulDest = &_bss; pulDest < &_ebss; ) {
    //    *pulDest++ = 0;
    //}

    //
    // Zero fill the bss segment.
    //
    __asm(	"    ldr     r0, =_bss\n"
            "    ldr     r1, =_ebss\n"
            "    mov     r2, #0\n"
            "    .thumb_func\n"
            "zero_loop:\n"
            "        cmp     r0, r1\n"
            "        it      lt\n"
            "        strlt   r2, [r0], #4\n"
            "        blt     zero_loop");

    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1) {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1) {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1) {
    }
}


