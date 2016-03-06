/* File: startup_ARMCM4.c
 * Purpose: startup file for Cortex-M4 devices.
 *          Should be used with GCC 'GNU Tools ARM Embedded'
 * Version: V1.01
 * Date: 12 June 2014
 *
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include <stdint.h>


/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
#ifndef __START
extern void  _start(void) __attribute__((noreturn));    /* PreeMain (C library entry point) */
#else
extern int  __START(void) __attribute__((noreturn));    /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
extern void SystemInit (void);            /* CMSIS System Initialization      */
#endif


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);                          /* Default empty handler */
void Reset_Handler(void);                            /* Reset Handler */


/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
#define	__STACK_SIZE  0x00000400
#endif
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

#ifndef __HEAP_SIZE
#define	__HEAP_SIZE   0x00000C00
#endif
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif


/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M4 Processor Exceptions */
void NMI_Handler					(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler					(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler					(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler				(void) __attribute__ ((weak, alias("Default_Handler")));

/* ARMCM4 Specific Interrupts */
void GPIOA_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOB_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOC_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOD_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOE_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI0_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM0_FAULT_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM0_GE0_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM0_GE1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM0_GE2_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI0_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC0_SEQ0_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC0_SEQ1_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC0_SEQ2_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC0_SEQ3_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void WT01_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_0A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_0B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_1A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_1B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_2A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_2B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void ANALOG_CMP0_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void ANALOG_CMP1_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void ANALOG_CMP2_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SYSTEM_CTL_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_EEPROM_CTL_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOF_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOG_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOH_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI1_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_3A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_3B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN0_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void HIB_MODULE_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_IRQHandler				    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM_GE3_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMA_SOFTWARE_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMA_ERROR_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_SEQ0_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_SEQ1_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_SEQ2_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_SEQ3_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOJ_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOK_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOL_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI2_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void SSI3_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART6_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_4A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_4B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_5A_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_5B_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_0A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_0B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_1A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_1B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_2A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_2B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_3A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_3B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_4A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_4B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_5A_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER_32BIT_5B_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SYS_EXCEP_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C5_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOM_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPION_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP1_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP2_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP3_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP4_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP5_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP6_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOP7_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_GE0_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_GE1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_GE2_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_GE3_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_FAULT_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
    /* Cortex-M4 Exceptions Handler */
    (pFunc)&__StackTop,                       /*      Initial Stack Pointer     */
    Reset_Handler,                            /*      Reset Handler             */
    NMI_Handler,                              /*      NMI Handler               */
    HardFault_Handler,                        /*      Hard Fault Handler        */
    MemManage_Handler,                        /*      MPU Fault Handler         */
    BusFault_Handler,                         /*      Bus Fault Handler         */
    UsageFault_Handler,                       /*      Usage Fault Handler       */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    0,                                        /*      Reserved                  */
    SVC_Handler,                              /*      SVCall Handler            */
    DebugMon_Handler,                         /*      Debug Monitor Handler     */
    0,                                        /*      Reserved                  */
    PendSV_Handler,                           /*      PendSV Handler            */
    SysTick_Handler,                          /*      SysTick Handler           */

    /* External interrupts */
    GPIOA_IRQHandler,                         /*  0:  GPIO Port A				*/
    GPIOB_IRQHandler,                         /*  1:  GPIO Port B				*/
    GPIOC_IRQHandler,                         /*  2:  GPIO Port C				*/
    GPIOD_IRQHandler,                         /*  3:  GPIO Port D				*/
    GPIOE_IRQHandler,                         /*  4:  GPIO Port	E				*/
    UART0_IRQHandler,                         /*  5:  UART 0					*/
    UART1_IRQHandler,                         /*  6:  UART 1					*/
    SSI0_IRQHandler,						  /*  7:  SSI0						*/
    I2C0_IRQHandler,                          /*  8:  I2C0						*/
    PWM0_FAULT_IRQHandler,                    /*  9:  PWM0 Fault				*/
    PWM0_GE0_IRQHandler,                      /* 10:  PWM0 Generator 0			*/
    PWM0_GE1_IRQHandler,                      /* 11:  PWM0 Generator 1			*/
    PWM0_GE2_IRQHandler,                      /* 12:  PWM0 Generator 2			*/
    QEI0_IRQHandler,                          /* 13:  QEI0						*/
    ADC0_SEQ0_IRQHandler,                     /* 14:  ADC0 Sequence 0			*/
    ADC0_SEQ1_IRQHandler,                     /* 15:  ADC0 Sequence 1			*/
    ADC0_SEQ2_IRQHandler,                     /* 16:  ADC0 Sequence 2			*/
    ADC0_SEQ3_IRQHandler,                     /* 17:  ADC0 Sequence 3			*/
    WT01_IRQHandler,                          /* 18:  Watchdog Timers 0 and 1	*/
    TIMER_0A_IRQHandler,                      /* 19:  16/32-Bit Timer 0A		*/
    TIMER_0B_IRQHandler,                      /* 20:  16/32-Bit Timer 0B		*/
    TIMER_1A_IRQHandler,                      /* 21:  16/32-Bit Timer 1A		*/
    TIMER_1B_IRQHandler,                      /* 22:  16/32-Bit Timer 1B		*/
    TIMER_2A_IRQHandler,                      /* 23:  16/32-Bit Timer 2A		*/
    TIMER_2B_IRQHandler,                      /* 24:  16/32-Bit Timer 2B		*/
    ANALOG_CMP0_IRQHandler,                   /* 25:  Analog Comparator 0		*/
    ANALOG_CMP1_IRQHandler,                   /* 26:  Analog Comparator 1		*/
    ANALOG_CMP2_IRQHandler,                   /* 27:  Analog Comparator 2		*/
    SYSTEM_CTL_IRQHandler,                    /* 28:  System control			*/
    FLASH_EEPROM_CTL_IRQHandler,              /* 29:  Flash Memory Control and EEPROM Control*/
    GPIOF_IRQHandler,                         /* 30:  GPIO Port F				*/
    GPIOG_IRQHandler,                         /* 31:  GPIO Port G				*/
    GPIOH_IRQHandler,                         /* 32:  GPIO Port H				*/
    UART2_IRQHandler,                         /* 33:  UART 2					*/
    SSI1_IRQHandler,						  /* 34:  SSI1						*/
    TIMER_3A_IRQHandler,                      /* 35:  16/32-Bit Timer 3A		*/
    TIMER_3B_IRQHandler,                      /* 36:  16/32-Bit Timer 3B		*/
    I2C1_IRQHandler,                          /* 37:  I2C1						*/
    QEI1_IRQHandler,                          /* 38:  QEI1						*/
    CAN0_IRQHandler,						  /* 39:  CAN0						*/
    CAN1_IRQHandler,						  /* 40:  CAN1						*/
    0,										  /* 41:  Reserved					*/
    0,										  /* 42:  Reserved					*/
    HIB_MODULE_IRQHandler,                    /* 43:  Hibernation Module		*/
    USB_IRQHandler,							  /* 44:  Hibernation Module		*/
    PWM_GE3_IRQHandler,                       /* 45:  PWM Generator 3			*/
    uDMA_SOFTWARE_IRQHandler,                 /* 46:  uDMA Software				*/
    uDMA_ERROR_IRQHandler,                    /* 47:  uDMA Error				*/
    ADC1_SEQ0_IRQHandler,                     /* 48:  ADC1 Sequence 0			*/
    ADC1_SEQ1_IRQHandler,                     /* 49:  ADC1 Sequence 1			*/
    ADC1_SEQ2_IRQHandler,                     /* 50:  ADC1 Sequence 2			*/
    ADC1_SEQ3_IRQHandler,                     /* 51:  ADC1 Sequence 3			*/
    0,										  /* 52:  Reserved					*/
    0,										  /* 53:  Reserved					*/
    GPIOJ_IRQHandler,                         /* 54:  GPIO Port J				*/
    GPIOK_IRQHandler,                         /* 55:  GPIO Port K				*/
    GPIOL_IRQHandler,                         /* 56:  GPIO Port L				*/
    SSI2_IRQHandler,						  /* 57:  SSI2						*/
    SSI3_IRQHandler,						  /* 58:  SSI3						*/
    UART3_IRQHandler,                         /* 59:  UART 3					*/
    UART4_IRQHandler,                         /* 60:  UART 4					*/
    UART5_IRQHandler,                         /* 61:  UART 5					*/
    UART6_IRQHandler,                         /* 62:  UART 6					*/
    UART7_IRQHandler,                         /* 63:  UART 7					*/
    0,										  /* 64:  Reserved					*/
    0,										  /* 65:  Reserved					*/
    0,										  /* 66:  Reserved					*/
    0,										  /* 67:  Reserved					*/
    I2C2_IRQHandler,                          /* 68:  I2C2						*/
    I2C3_IRQHandler,                          /* 69:  I2C3						*/
    TIMER_4A_IRQHandler,                      /* 70:  16/32-Bit Timer 4A		*/
    TIMER_4B_IRQHandler,                      /* 71:  16/32-Bit Timer 4B		*/
    0,										  /* 72:  Reserved					*/
    0,										  /* 73:  Reserved					*/
    0,										  /* 74:  Reserved					*/
    0,										  /* 75:  Reserved					*/
    0,										  /* 76:  Reserved					*/
    0,										  /* 77:  Reserved					*/
    0,										  /* 78:  Reserved					*/
    0,										  /* 79:  Reserved					*/
    0,										  /* 80:  Reserved					*/
    0,										  /* 81:  Reserved					*/
    0,										  /* 82:  Reserved					*/
    0,										  /* 83:  Reserved					*/
    0,										  /* 84:  Reserved					*/
    0,										  /* 85:  Reserved					*/
    0,										  /* 86:  Reserved					*/
    0,										  /* 87:  Reserved					*/
    0,										  /* 88:  Reserved					*/
    0,										  /* 89:  Reserved					*/
    0,										  /* 90:  Reserved					*/
    0,										  /* 91:  Reserved					*/
    TIMER_5A_IRQHandler,                      /* 92:  16/32-Bit Timer 5A		*/
    TIMER_5B_IRQHandler,                      /* 93:  16/32-Bit Timer 5B		*/
    TIMER_32BIT_0A_IRQHandler,                /* 94:  32/64-Bit Timer 0A		*/
    TIMER_32BIT_0B_IRQHandler,                /* 95:  32/64-Bit Timer 0B		*/
    TIMER_32BIT_1A_IRQHandler,                /* 96:  32/64-Bit Timer 1A		*/
    TIMER_32BIT_1B_IRQHandler,                /* 97:  32/64-Bit Timer 1B		*/
    TIMER_32BIT_2A_IRQHandler,                /* 98:  32/64-Bit Timer 2A		*/
    TIMER_32BIT_2B_IRQHandler,                /* 99:  32/64-Bit Timer 2B		*/
    TIMER_32BIT_3A_IRQHandler,                /* 100: 32/64-Bit Timer 3A		*/
    TIMER_32BIT_3B_IRQHandler,                /* 101: 32/64-Bit Timer 3B		*/
    TIMER_32BIT_4A_IRQHandler,                /* 102: 32/64-Bit Timer 4A		*/
    TIMER_32BIT_4B_IRQHandler,                /* 103: 32/64-Bit Timer 4B		*/
    TIMER_32BIT_5A_IRQHandler,                /* 104: 32/64-Bit Timer 5A		*/
    TIMER_32BIT_5B_IRQHandler,                /* 105: 32/64-Bit Timer 5B		*/
    SYS_EXCEP_IRQHandler,					  /* 106: System Exception(imprecise)*/
    0,										  /* 107: Reserved					*/
    0,										  /* 108: Reserved					*/
    I2C4_IRQHandler,                          /* 109: I2C4						*/
    I2C5_IRQHandler,                          /* 110: I2C5						*/
    GPIOM_IRQHandler,                         /* 111: GPIO Port M				*/
    GPION_IRQHandler,                         /* 112: GPIO Port N				*/
    0,										  /* 113: Reserved					*/
    0,										  /* 114: Reserved					*/
    0,										  /* 115: Reserved					*/
    GPIOP_IRQHandler,                         /* 116: GPIO Port P(Summary or P0)*/
    GPIOP1_IRQHandler,                        /* 117: GPIO Port P1				*/
    GPIOP2_IRQHandler,                        /* 118: GPIO Port P2				*/
    GPIOP3_IRQHandler,                        /* 119: GPIO Port P3				*/
    GPIOP4_IRQHandler,                        /* 120: GPIO Port P4				*/
    GPIOP5_IRQHandler,                        /* 121: GPIO Port P5				*/
    GPIOP6_IRQHandler,                        /* 122: GPIO Port P6				*/
    GPIOP7_IRQHandler,                        /* 123: GPIO Port P7				*/
    0,										  /* 124: Reserved					*/
    0,										  /* 125: Reserved					*/
    0,										  /* 126: Reserved					*/
    0,										  /* 127: Reserved					*/
    0,										  /* 128: Reserved					*/
    0,										  /* 129: Reserved					*/
    0,										  /* 130: Reserved					*/
    0,										  /* 131: Reserved					*/
    0,										  /* 132: Reserved					*/
    0,										  /* 133: Reserved					*/
    PWM1_GE0_IRQHandler,                      /* 134: PWM1 Generator 0			*/
    PWM1_GE1_IRQHandler,                      /* 135: PWM1 Generator 1			*/
    PWM1_GE2_IRQHandler,                      /* 136: PWM1 Generator 2			*/
    PWM1_GE3_IRQHandler,                      /* 137: PWM1 Generator 2			*/
    PWM1_FAULT_IRQHandler,                    /* 138: PWM1 Fault				*/
};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void)
{
    uint32_t *pSrc, *pDest;
    uint32_t *pTable __attribute__((unused));

    /*  Firstly it copies data from read only memory to RAM. There are two schemes
     *  to copy. One can copy more than one sections. Another can only copy
     *  one section.  The former scheme needs more instructions and read-only
     *  data to implement than the latter.
     *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
    /*  Multiple sections scheme.
     *
     *  Between symbol address __copy_table_start__ and __copy_table_end__,
     *  there are array of triplets, each of which specify:
     *    offset 0: LMA of start of a section to copy from
     *    offset 4: VMA of start of a section to copy to
     *    offset 8: size of the section to copy. Must be multiply of 4
     *
     *  All addresses must be aligned to 4 bytes boundary.
     */
    pTable = &__copy_table_start__;

    for (; pTable < &__copy_table_end__; pTable = pTable + 3) {
        pSrc  = (uint32_t*)*(pTable + 0);
        pDest = (uint32_t*)*(pTable + 1);
        for (; pDest < (uint32_t*)(*(pTable + 1) + *(pTable + 2)) ; ) {
            *pDest++ = *pSrc++;
        }
    }
#else
    /*  Single section scheme.
     *
     *  The ranges of copy from/to are specified by following symbols
     *    __etext: LMA of start of the section to copy from. Usually end of text
     *    __data_start__: VMA of start of the section to copy to
     *    __data_end__: VMA of end of the section to copy to
     *
     *  All addresses must be aligned to 4 bytes boundary.
     */
    pSrc  = &__etext;
    pDest = &__data_start__;

    for ( ; pDest < &__data_end__ ; ) {
        *pDest++ = *pSrc++;
    }
#endif /*__STARTUP_COPY_MULTIPLE */

    /*  This part of work usually is done in C library startup code. Otherwise,
     *  define this macro to enable it in this startup.
     *
     *  There are two schemes too. One can clear multiple BSS sections. Another
     *  can only clear one section. The former is more size expensive than the
     *  latter.
     *
     *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
     *  Otherwise define macro __STARTUP_CLEAR_BSS to choose the later.
     */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
    /*  Multiple sections scheme.
     *
     *  Between symbol address __zero_table_start__ and __zero_table_end__,
     *  there are array of tuples specifying:
     *    offset 0: Start of a BSS section
     *    offset 4: Size of this BSS section. Must be multiply of 4
     */
    pTable = &__zero_table_start__;

    for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
        pDest = (uint32_t*)*(pTable + 0);
        for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
            *pDest++ = 0;
        }
    }
#elif defined (__STARTUP_CLEAR_BSS)
    /*  Single BSS section scheme.
     *
     *  The BSS section is specified by following symbols
     *    __bss_start__: start of the BSS section.
     *    __bss_end__: end of the BSS section.
     *
     *  Both addresses must be aligned to 4 bytes boundary.
     */
    pDest = &__bss_start__;

    for ( ; pDest < &__bss_end__ ; ) {
        *pDest++ = 0ul;
    }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    SystemInit();
#endif

#ifndef __START
#define __START _start
#endif
    __START();

}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{

    while(1);
}
