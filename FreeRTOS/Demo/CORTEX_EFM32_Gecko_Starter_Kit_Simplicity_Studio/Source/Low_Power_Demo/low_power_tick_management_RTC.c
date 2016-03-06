/*
    FreeRTOS V9.0.0rc1 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* SiLabs library includes. */
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_burtc.h"
#include "em_rmu.h"
#include "em_int.h"
#include "sleep.h"

#define lpINCLUDE_TEST_TIMER	1

/* SEE THE COMMENTS ABOVE THE DEFINITION OF configCREATE_LOW_POWER_DEMO IN
FreeRTOSConfig.h
This file contains functions that will override the default implementations
in the RTOS port layer.  Therefore only build this file if the low power demo
is being built. */
#if( configCREATE_LOW_POWER_DEMO == 2 )

#define mainTIMER_FREQUENCY_HZ	( 4096UL ) /* 32768 clock divided by 8. */

/*
 * The low power demo does not use the SysTick, so override the
 * vPortSetupTickInterrupt() function with an implementation that configures
 * a low power clock source.  NOTE:  This function name must not be changed as
 * it is called from the RTOS portable layer.
 */
void vPortSetupTimerInterrupt( void );

/*
 * Override the default definition of vPortSuppressTicksAndSleep() that is
 * weakly defined in the FreeRTOS Cortex-M port layer with a version that
 * manages the RTC clock, as the tick is generated from the low power RTC
 * and not the SysTick as would normally be the case on a Cortex-M.
 */
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );

/* If lpINCLUDE_TEST_TIMER is defined then the BURTC is used to generate
interrupts that will wake the processor prior to the expected idle time
completing.  The timer interval can be altered to test different
scenarios. */
#if( lpINCLUDE_TEST_TIMER == 1 )
static void prvSetupTestTimer( void );
#endif

/*-----------------------------------------------------------*/

/* Calculate how many clock increments make up a single tick period. */
static const uint32_t ulReloadValueForOneTick = ( mainTIMER_FREQUENCY_HZ / configTICK_RATE_HZ );

/* Will hold the maximum number of ticks that can be suppressed. */
static uint32_t xMaximumPossibleSuppressedTicks = 0;

/* Flag set from the tick interrupt to allow the sleep processing to know if
sleep mode was exited because of a timer interrupt or a different interrupt. */
static volatile uint32_t ulTickFlag = pdFALSE;

/* As the clock is only 32KHz, it is likely a value of 1 will be enough. */
static const uint32_t ulStoppedTimerCompensation = 0UL;

/*-----------------------------------------------------------*/

void vPortSetupTimerInterrupt( void )
{
    RTC_Init_TypeDef xRTCInitStruct;
    const uint32_t ulMAX24BitValue = 0xffffffUL;

    xMaximumPossibleSuppressedTicks = ulMAX24BitValue / ulReloadValueForOneTick;

    /* Configure the RTC to generate the RTOS tick interrupt. */

    /* LXFO setup.  For rev D use 70% boost */
    CMU->CTRL = ( CMU->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK ) | CMU_CTRL_LFXOBOOST_70PCENT;
#if defined( EMU_AUXCTRL_REDLFXOBOOST )
    EMU->AUXCTRL = (EMU->AUXCTRL & ~_EMU_AUXCTRL_REDLFXOBOOST_MASK) | EMU_AUXCTRL_REDLFXOBOOST;
#endif

    /* Ensure LE modules are accessible. */
    CMU_ClockEnable( cmuClock_CORELE, true );

    /* Use LFXO. */
    CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFXO );

    /* Use 8x divider to reduce energy. */
    CMU_ClockDivSet( cmuClock_RTC, cmuClkDiv_8 );

    /* Enable clock to the RTC module. */
    CMU_ClockEnable( cmuClock_RTC, true );
    xRTCInitStruct.enable = false;
    xRTCInitStruct.debugRun = false;
    xRTCInitStruct.comp0Top = true;
    RTC_Init( &xRTCInitStruct );

    /* Disable RTC0 interrupt. */
    RTC_IntDisable( RTC_IFC_COMP0 );

    /* The tick interrupt must be set to the lowest priority possible. */
    NVIC_SetPriority( RTC_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY );
    NVIC_ClearPendingIRQ( RTC_IRQn );
    NVIC_EnableIRQ( RTC_IRQn );
    RTC_CompareSet( 0, ulReloadValueForOneTick );
    RTC_IntClear( RTC_IFC_COMP0 );
    RTC_IntEnable( RTC_IF_COMP0 );
    RTC_Enable( true );

    /* If lpINCLUDE_TEST_TIMER is defined then the BURTC is used to generate
    interrupts that will wake the processor prior to the expected idle time
    completing.  The timer interval can be altered to test different
    scenarios. */
#if( lpINCLUDE_TEST_TIMER == 1 )
    prvSetupTestTimer();
#endif
}
/*-----------------------------------------------------------*/

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCountBeforeSleep, ulCountAfterSleep;
    eSleepModeStatus eSleepAction;
    TickType_t xModifiableIdleTime;

    /* THIS FUNCTION IS CALLED WITH THE SCHEDULER SUSPENDED. */

    /* Make sure the RTC reload value does not overflow the counter. */
    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks ) {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    /* Calculate the reload value required to wait xExpectedIdleTime tick
    periods. */
    ulReloadValue = ulReloadValueForOneTick * xExpectedIdleTime;
    if( ulReloadValue > ulStoppedTimerCompensation ) {
        /* Compensate for the fact that the RTC is going to be stopped
        momentarily. */
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Stop the RTC momentarily.  The time the RTC is stopped for is accounted
    for as best it can be, but using the tickless mode will inevitably result
    in some tiny drift of the time maintained by the kernel with respect to
    calendar time.  The count is latched before stopping the timer as stopping
    the timer appears to clear the count. */
    ulCountBeforeSleep = RTC_CounterGet();
    RTC_Enable( false );

    /* If this function is re-entered before one complete tick period then the
    reload value might be set to take into account a partial time slice, but
    just reading the count assumes it is counting up to a full ticks worth - so
    add in the difference if any. */
    ulCountBeforeSleep += ( ulReloadValueForOneTick - RTC_CompareGet( 0 ) );

    /* Enter a critical section but don't use the taskENTER_CRITICAL() method as
    that will mask interrupts that should exit sleep mode. */
    INT_Disable();
    __asm volatile( "dsb" );
    __asm volatile( "isb" );

    /* The tick flag is set to false before sleeping.  If it is true when sleep
    mode is exited then sleep mode was probably exited because the tick was
    suppressed for the entire xExpectedIdleTime period. */
    ulTickFlag = pdFALSE;

    /* If a context switch is pending then abandon the low power entry as the
    context switch might have been pended by an external interrupt that	requires
    processing. */
    eSleepAction = eTaskConfirmSleepModeStatus();
    if( eSleepAction == eAbortSleep ) {
        /* Restart tick and count up to whatever was left of the current time
        slice. */
        RTC_CompareSet( 0, ( ulReloadValueForOneTick - ulCountBeforeSleep ) + ulStoppedTimerCompensation );
        RTC_Enable( true );

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        INT_Enable();
    } else {
        /* Adjust the reload value to take into account that the current time
        slice is already partially complete. */
        ulReloadValue -= ulCountBeforeSleep;
        RTC_CompareSet( 0, ulReloadValue );

        /* Restart the RTC. */
        RTC_Enable( true );

        /* Allow the application to define some pre-sleep processing. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );

        /* xExpectedIdleTime being set to 0 by configPRE_SLEEP_PROCESSING()
        means the application defined code has already executed the WAIT
        instruction. */
        if( xModifiableIdleTime > 0 ) {
            __asm volatile( "dsb" );
            SLEEP_Sleep();
            __asm volatile( "isb" );
        }

        /* Allow the application to define some post sleep processing. */
        configPOST_SLEEP_PROCESSING( xModifiableIdleTime );

        /* Stop RTC.  Again, the time the SysTick is stopped for is accounted
        for as best it can be, but using the tickless mode will	inevitably
        result in some tiny drift of the time maintained by the	kernel with
        respect to calendar time.  The count value is latched before stopping
        the timer as stopping the timer appears to clear the count. */
        ulCountAfterSleep = RTC_CounterGet();
        RTC_Enable( false );

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        INT_Enable();
        __asm volatile( "dsb" );
        __asm volatile( "isb" );

        if( ulTickFlag != pdFALSE ) {
            /* The tick interrupt has already executed, although because this
            function is called with the scheduler suspended the actual tick
            processing will not occur until after this function has exited.
            Reset the reload value with whatever remains of this tick period. */
            ulReloadValue = ulReloadValueForOneTick - ulCountAfterSleep;
            RTC_CompareSet( 0, ulReloadValue );

            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be processed as
            soon as this function exits, the tick value	maintained by the tick
            is stepped forward by one less than the	time spent sleeping.  The
            actual stepping of the tick appears later in this function. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        } else {
            /* Something other than the tick interrupt ended the sleep.  How
            many complete tick periods passed while the processor was
            sleeping?  Add back in the adjustment that was made to the reload
            value to account for the fact that a time slice was part way through
            when this function was called. */
            ulCountAfterSleep += ulCountBeforeSleep;
            ulCompleteTickPeriods = ulCountAfterSleep / ulReloadValueForOneTick;

            /* The reload value is set to whatever fraction of a single tick
            period remains. */
            ulCountAfterSleep -= ( ulCompleteTickPeriods * ulReloadValueForOneTick );
            ulReloadValue = ulReloadValueForOneTick - ulCountAfterSleep;

            if( ulReloadValue == 0 ) {
                /* There is no fraction remaining. */
                ulReloadValue = ulReloadValueForOneTick;
                ulCompleteTickPeriods++;
            }

            RTC_CompareSet( 0, ulReloadValue );
        }

        /* Restart the RTC so it runs up to the alarm value.  The alarm value
        will get set to the value required to generate exactly one tick period
        the next time the RTC interrupt executes. */
        RTC_Enable( true );

        /* Wind the tick forward by the number of tick periods that the CPU
        remained in a low power state. */
        vTaskStepTick( ulCompleteTickPeriods );
    }
}
/*-----------------------------------------------------------*/

void RTC_IRQHandler( void )
{
    ulTickFlag = pdTRUE;

    if( RTC_CompareGet( 0 ) != ulReloadValueForOneTick ) {
        /* Set RTC interrupt to one RTOS tick period. */
        RTC_Enable( false );
        RTC_CompareSet( 0, ulReloadValueForOneTick );
        RTC_Enable( true );
    }

    RTC_IntClear( _RTC_IFC_MASK );

    /* Critical section which protect incrementing the tick*/
    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if( xTaskIncrementTick() != pdFALSE ) {
            /* Pend a context switch. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}
/*-----------------------------------------------------------*/

#if( lpINCLUDE_TEST_TIMER == 1 )

/* If lpINCLUDE_TEST_TIMER is defined then the BURTC is used to generate
interrupts that will wake the processor prior to the expected idle time
completing.  The timer interval can be altered to test different
scenarios. */
static void prvSetupTestTimer( void )
{
    BURTC_Init_TypeDef xBURTCInitStruct = BURTC_INIT_DEFAULT;
    const uint32_t ulBURTClockHz = 2000UL, ulInterruptFrequency = 1000UL;
    const uint32_t ulReload = ( ulBURTClockHz / ulInterruptFrequency );

    /* Ensure LE modules are accessible. */
    CMU_ClockEnable( cmuClock_CORELE, true );

    /* Enable access to BURTC registers. */
    RMU_ResetControl( rmuResetBU, false );

    /* Generate periodic interrupts from BURTC. */
    xBURTCInitStruct.mode   = burtcModeEM3;		/* Operational in EM3. */
    xBURTCInitStruct.clkSel = burtcClkSelULFRCO;/* ULFRCO clock. */
    xBURTCInitStruct.clkDiv = burtcClkDiv_1;	/* 2kHz ULFRCO clock. */
    xBURTCInitStruct.compare0Top = true;		/* Wrap on COMP0. */
    BURTC_IntDisable( BURTC_IF_COMP0 );
    BURTC_Init( &xBURTCInitStruct );

    NVIC_SetPriority( BURTC_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY );
    NVIC_ClearPendingIRQ( BURTC_IRQn );
    NVIC_EnableIRQ( BURTC_IRQn );
    BURTC_CompareSet( 0, ulReload );
    BURTC_IntClear( BURTC_IF_COMP0 );
    BURTC_IntEnable( BURTC_IF_COMP0 );
    BURTC_CounterReset();
}

#endif
/*-----------------------------------------------------------*/

#if( lpINCLUDE_TEST_TIMER == 1 )

/* If lpINCLUDE_TEST_TIMER is defined then the BURTC is used to generate
interrupts that will wake the processor prior to the expected idle time
completing.  The timer interval can be altered to test different
scenarios. */
volatile uint32_t ulTestTimerCounts = 0;

void BURTC_IRQHandler( void )
{
    /* Nothing to do here - just testing the code in the scenario where a
    tickless idle period is ended prior to the expected maximum idle time
    expiring. */
    BURTC_IntClear( _RTC_IFC_MASK );
    ulTestTimerCounts++;
}

#endif
/*-----------------------------------------------------------*/

#endif /* ( configCREATE_LOW_POWER_DEMO == 2 ) */
