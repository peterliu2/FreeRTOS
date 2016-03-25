/**
 * @file commands.c
 * @brief The commands and command functions for utils/cmdline.[ch]
 * @author peter liu <peter.liu.work@gmail.com.tw>
 * @version 0.1
 * @date 2016-03-17
 */

#include <string.h>

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"

/* Environment includes */
#include "DriverLib.h"

/* stack size for vCommandLineProcessTask */
#define commandsSTACK_SIZE configMINIMAL_STACK_SIZE

// command line buffer to store commands.
static char cmd[ UART_RX_BUFFER_SIZE ];

// command line prompt
#define PRINT_PROMPT printf( "Cmd> ")

/**
 * @brief Task for process the input command line.
 *
 * @Param pvParameter
 */
/* ----------------------------------------------------------------------------*/
static void vCommandLineProcessTask( void * pvParameters )
{
    for( ;; ) {
        PRINT_PROMPT;

        /* wait for uart consult to receive command */
        fgets( cmd, sizeof( cmd ), stdin );

        // null terminal
        cmd[ strlen( cmd ) - 1 ] = 0;

        int iCommandResult =  CmdLineProcess( cmd );

        switch( iCommandResult ) {
            case CMDLINE_BAD_CMD:
                printf( "bad command\r\n" );
                break;

            case CMDLINE_TOO_MANY_ARGS:
                printf( "too many arguments\r\n" );
                break;

            case CMDLINE_TOO_FEW_ARGS:
                printf( "too few arguments\r\n" );
                break;

            case CMDLINE_INVALID_ARG:
                printf( "invalid arguments\r\n" );
                break;

            case 0:
                // command OK!
                break;

            default:
                printf( "unknow error\r\n" );
        }
    }
}

/**
 * @brief start task.
 *
 * @Param uxPriority
 * @Param semaphore
 */
/* ----------------------------------------------------------------------------*/
void vStartCommandLineProcessTask( UBaseType_t uxPriority )
{
    if ( xTaskCreate( vCommandLineProcessTask, "CmdLinePross", commandsSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t *) NULL ) != pdPASS ) {
        while( 1 );
    }
}


/**
 * @brief Print Memory Layout.
 *
 * @Param argc
 * @Param argv[]]
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
static int iPrintMemoryLayout( int argc, char *argv[] )
{
    if( argc > 1 ) {
        return CMDLINE_INVALID_ARG;
    }

    extern unsigned long _text;
    extern unsigned long _etext;
    extern unsigned long _data;
    extern unsigned long _edata;
    extern unsigned long _bss;
    extern unsigned long _ebss;
    extern unsigned long _heap_start;
    extern char *heap_point;
    extern unsigned long _heap_limit;

    printf( "_text          = 0x%X\r\n",      ( unsigned int ) &_text );
    printf( "_etext         = 0x%X\r\n",      ( unsigned int ) &_etext );
    printf( "Sizeof(.text)  = %d bytes\r\n",  ( &_etext - &_text ) * 4);
    printf( "---------------------------\r\n" );
    printf( "_data          = 0x%X\r\n",      ( unsigned int ) &_data );
    printf( "_edata         = 0x%X\r\n",      ( unsigned int ) &_edata );
    printf( "Sizeof(.data)  = %d bytes\r\n",  ( &_edata - &_data ) * 4 );
    printf( "---------------------------\r\n" );
    printf( "_bss           = 0x%X\r\n",      ( unsigned int ) &_bss );
    printf( "_ebss          = 0x%X\r\n",      ( unsigned int ) &_ebss );
    printf( "Sizeof(.bss)   = %d bytes\r\n",  ( &_ebss - &_bss ) * 4 );
    printf( "---------------------------\r\n" );
    printf( "_heap_start    = 0x%X\r\n",      ( unsigned int ) &_heap_start );
    printf( "_heap_point    = 0x%X",          ( unsigned int ) heap_point );
    printf( " : usage: %d bytes\r\n",         ( heap_point - ( char * ) &_heap_start ) );
    printf( "_heap_limit    = 0x%X\r\n",      ( unsigned int ) &_heap_limit );
    printf( "Sizeof(heap)   = %d bytes\r\n",  ( &_heap_limit - &_heap_start ) * 4 );
    printf( "Free heap      = %d bytes\r\n",  ( ( ( &_heap_limit - &_heap_start ) * 4 ) - ( heap_point - ( char * ) &_heap_start ) ) );
    printf( "---------------------------\r\n" );

    return 0;
}

static int iPrintCpuUsage( int argc, char *argv[] ) 
{
	static uint32_t timerInit = 0;
	
	if( timerInit == 0 ) {
		CPUUsageInit( SysCtlClockGet(), 1000, 5 );
		timerInit = 1;
	}
	
	uint32_t CpuUsage = CPUUsageTick();
	
	//CpuUsage = 0x000F000A;
	printf( "CPU usage: %lu.%lu%%\r\n", CpuUsage >> 16, CpuUsage & 0xFFFF );

	return 0;
}

tCmdLineEntry g_psCmdTable[] = {
    { "printMemoryLayout",	iPrintMemoryLayout, "print memory layout" },
    { "printCpuUsage",		iPrintCpuUsage,		"print CPU usage" },
 
	{ NULL, NULL, NULL },	// end mark
};
