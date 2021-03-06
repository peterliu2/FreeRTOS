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


/*
 * This project contains an application demonstrating the use of the
 * FreeRTOS.org mini real time scheduler on the TI DK-TM4C123G Tiva C Series
 * Development kit.  See http://www.FreeRTOS.org for more information.
 *
 * main() simply sets up the hardware, creates all the demo application tasks,
 * then starts the scheduler.  http://www.freertos.org/a00102.html provides
 * more information on the standard demo tasks.
 *
 * In addition to a subset of the standard demo application tasks, main.c also
 * defines the following tasks:
 *
 * + A 'Print' task.  The print task is the only task permitted to access the
 * LCD - thus ensuring mutual exclusion and consistent access to the resource.
 * Other tasks do not access the LCD directly, but instead send the text they
 * wish to display to the print task.  The print task spends most of its time
 * blocked - only waking when a message is queued for display.
 *
 * + A 'Button handler' task.  The eval board contains a user push button that
 * is configured to generate interrupts.  The interrupt handler uses a
 * semaphore to wake the button handler task - demonstrating how the priority
 * mechanism can be used to defer interrupt processing to the task level.  The
 * button handler task sends a message both to the LCD (via the print task) and
 * the UART where it can be viewed using a dumb terminal (via the UART to USB
 * converter on the eval board).  NOTES:  The dumb terminal must be closed in
 * order to reflash the microcontroller.  A very basic interrupt driven UART
 * driver is used that does not use the FIFO.  115200 baud is used.
 *
 * + A 'check' task.  The check task only executes every five seconds but has a
 * high priority so is guaranteed to get processor time.  Its function is to
 * check that all the other tasks are still operational and that no errors have
 * been detected at any time.  If no errors have every been detected 'PASS' is
 * written to the display (via the print task) - if an error has ever been
 * detected the message is changed to 'FAIL'.  The position of the message is
 * changed for each write.
 */

/* Environment includes. */
//#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Environment includes. */
#include "DriverLib.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY				( ( TickType_t ) 2000 / portTICK_PERIOD_MS )

/* UART configuration - we use queue and Uart FIFO function. */
#define mainBAUD_RATE				( 115200 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Demo board specifics. */
#define mainPUSH_BUTTON             GPIO_PIN_0

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )

/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware( void );

/*
 * The 'check' task, as described at the top of this file.
 */
static void vCheckTask( void *pvParameters );

/*
 * The task that is woken by the ISR that processes GPIO interrupts originating
 * from the push button.
 */
static void vButtonHandlerTask( void *pvParameters );

/*
 * The task that controls access to the LCD.
 */
static void vPrintTask( void *pvParameter );

/* String that is transmitted on the UART. */
static const char Message[] = "Task woken by button interrupt! -----";

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xButtonSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;

/* dispaly context */
static tContext sContext;
/*-----------------------------------------------------------*/


/**
 * @brief The error routine that is called if the driver librart encounters an error.
 *
 * @Param pcFileName error file name
 * @Param line error line
 */
/* ----------------------------------------------------------------------------*/
#ifdef DEBUG
void __error__( const char *pcFileName, const uint32_t line )
{
    while( 1 );
}

void vAssertCalled( const char *pcFileName, const uint32_t line ) {
	while( 1 );
}

#endif

/**
 * @brief main function
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int main( void )
{
    /* Configure the clocks, UART consule and GPIO. */
    prvSetupHardware();

    /* Create the semaphore used to wake the button handler task from the GPIO
    ISR. */
    vSemaphoreCreateBinary( xButtonSemaphore );
    xSemaphoreTake( xButtonSemaphore, 0 );

    /* Create the queue used to pass message to vPrintTask. */
    xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ) );

    /* Start the standard demo tasks. */
    vStartIntegerMathTasks( tskIDLE_PRIORITY );
    vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    vStartCommandLineProcessTask( tskIDLE_PRIORITY );

    /* Start the tasks defined within the file. */
    xTaskCreate( vCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
    xTaskCreate( vButtonHandlerTask, "Status", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );
    xTaskCreate( vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient heap to start the
    scheduler. */

    return 0;
}
/*-----------------------------------------------------------*/

/**
 * @brief
 *
 * @Param pvParameters
 */
/* ----------------------------------------------------------------------------*/
static void vCheckTask( void *pvParameters )
{
    portBASE_TYPE xErrorOccurred = pdFALSE;
    TickType_t xLastExecutionTime;
    UBaseType_t xNumofTasks;

    char pass[] = "PASS,task:    ";
    char fail[] = "FAIL,task:    ";

    char * const pcPassMessage = pass;
    char * const pcFailMessage = fail;

    /* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
    works correctly. */
    xLastExecutionTime = xTaskGetTickCount();

    for( ;; ) {
        /* Perform this check every mainCHECK_DELAY milliseconds. */
        vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );

        /* Has an error been found in any task? */
        if( xAreIntegerMathsTaskStillRunning() != pdTRUE ) {
            xErrorOccurred = pdTRUE;
        }

        if( xArePollingQueuesStillRunning() != pdTRUE ) {
            xErrorOccurred = pdTRUE;
        }

        if( xAreSemaphoreTasksStillRunning() != pdTRUE ) {
            xErrorOccurred = pdTRUE;
        }

        if( xAreBlockingQueuesStillRunning() != pdTRUE ) {
            xErrorOccurred = pdTRUE;
        }

        /* Get the current numbers of tasks  */
        xNumofTasks = uxTaskGetNumberOfTasks();

        /* Send either a pass or fail message.  If an error is found it is
        never cleared again.  We do not write directly to the LCD, but instead
        queue a message for display by the print task. */
        if( xErrorOccurred == pdTRUE ) {
            snprintf( &pcFailMessage[10], 3, "%lu", xNumofTasks );
            xQueueSend( xPrintQueue, &pcFailMessage, portMAX_DELAY );
        } else {
            snprintf( &pcPassMessage[10], 3, "%lu", xNumofTasks );
            xQueueSend( xPrintQueue, &pcPassMessage, portMAX_DELAY );
        }
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Configure UART0
 */
/* ----------------------------------------------------------------------------*/
static void prvConfigureUART(void)
{
    /* Enable UART0 Tx/Rx pins :GPIOA */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );

    /* Configure PA0, PA1 for UART mode */
    GPIOPinConfigure( GPIO_PA0_U0RX );
    GPIOPinConfigure( GPIO_PA1_U0TX );
    GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    /* configure Uart consult */
    UARTStdioConfig( 0, mainBAUD_RATE, SysCtlClockGet() );
}

/**
 * @brief Configure Hardware
 */
/* ----------------------------------------------------------------------------*/
static void prvSetupHardware( void )
{
    tRectangle sRect;

    /* Setup the PLL. 10MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_20 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ );

	/* Set the number of bits of preemptable priority */
	IntPriorityGroupingSet( 3 );		// on the Tiva C and E Series family, three bits are available for hard-ware interrupt prioritization and therefore priority grouping values of three through seven have the same effect

    /* Setup the push button. PM0 */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOM );
    GPIOPinTypeGPIOInput( GPIO_PORTM_BASE, mainPUSH_BUTTON);
    GPIOIntTypeSet( GPIO_PORTM_BASE, mainPUSH_BUTTON, GPIO_FALLING_EDGE );
    GPIOPadConfigSet( GPIO_PORTM_BASE, mainPUSH_BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU );

    MAP_IntPrioritySet( INT_GPIOM, 0xC0 /* configKERNEL_INTERRUPT_PRIORITY */ );
    GPIOIntEnable( GPIO_PORTM_BASE, mainPUSH_BUTTON );
    IntEnable( INT_GPIOM );

    /* Configure the UART.  */
    prvConfigureUART();

    /* Initialize the LCD driver */
    CFAL96x64x16Init();

    /* Initialize the graphics contex  */
    GrContextInit( &sContext, &g_sCFAL96x64x16 );

    /* Fill the top part of the screen with blue to creat the banner  */
    sRect.i16XMin = 0U;
    sRect.i16YMin = 0U;
    sRect.i16XMax = GrContextDpyWidthGet( &sContext ) - 1U;
    sRect.i16YMax = 9U;
    GrContextForegroundSet( &sContext, ClrDarkBlue );
    GrRectFill( &sContext, &sRect) ;

    /* Put a white box around the banner. */
    GrContextForegroundSet( &sContext, ClrWhite );
    GrRectDraw( &sContext, &sRect );

    /* Put the application name in the middle of the banner. */
    GrContextFontSet( &sContext, g_psFontFixed6x8 );
    GrStringDrawCentered( &sContext, "RTOS Demo", -1, GrContextDpyWidthGet( &sContext ) / 2U, 4U, 0U );
}
/*-----------------------------------------------------------*/

static void vButtonHandlerTask( void *pvParameters )
{
    const char *pcInterruptMessage = "Int";

    for( ;; ) {
        /* Wait for a GPIO interrupt to wake this task. */
        while( xSemaphoreTake( xButtonSemaphore, portMAX_DELAY ) != pdPASS );

        /* print something throught Uart */
        printf( "%s\r\n", Message );

        /* Queue a message for the print task to display on the LCD. */
        xQueueSend( xPrintQueue, &pcInterruptMessage, portMAX_DELAY );

        /* Make sure we don't process bounces. */
        vTaskDelay( mainDEBOUNCE_DELAY );
    }
}

/*-----------------------------------------------------------*/

void GPIOM_IRQHandler( void )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Clear the interrupt. */
    GPIOIntClear( GPIO_PORTM_BASE, mainPUSH_BUTTON );

    /* Wake the button handler task. */
    xSemaphoreGiveFromISR( xButtonSemaphore, &xHigherPriorityTaskWoken );

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void vPrintTask( void *pvParameters )
{
    char *pcMessage;
    unsigned portBASE_TYPE uxLine = 0;
    tRectangle sRect;

    for( ;; ) {
        /* Wait for a message to arrive. */
        xQueueReceive( xPrintQueue, &pcMessage, portMAX_DELAY );

        /* Write the message to the LCD. */
        if (uxLine > 4) {
            uxLine = 1;

            sRect.i16XMin = 0;
            sRect.i16YMin = 10;
            sRect.i16XMax = GrContextDpyWidthGet( &sContext ) - 1;
            sRect.i16YMax = GrContextDpyHeightGet( &sContext ) - 1;
            GrContextForegroundSet( &sContext, ClrBlack );
            GrRectFill( &sContext, &sRect);
            GrContextForegroundSet( &sContext, ClrWhite );
        } else {
            uxLine++;
        }
        GrStringDrawCentered( &sContext, pcMessage, -1, GrContextDpyWidthGet(&sContext) / 2, 10 + (uxLine * 10), 0 );
    }
}

