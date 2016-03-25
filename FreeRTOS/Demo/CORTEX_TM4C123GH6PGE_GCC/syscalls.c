/**
 * @file syscalls.c
 * @brief The stub functions for Newlib.
 *		  see https://sourceware.org/newlib/libc.html#Syscalls
 * @author peter liu <peter.liu.work@gmail.com.tw>
 * @version 0.1
 * @date 2016-03-15
 */

#include <errno.h>
#undef errno
extern int errno;

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/times.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "DriverLib.h"

/**
 * @brief Close a file.
 *
 * @Param file
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _close( int file )
{
    return -1;
}

/**
 * @brief A pointer to a list of environmnet variables and their values.
 *        For a minimal environment, this empty list is adequate.
 */
/* ----------------------------------------------------------------------------*/
char *__env[ 1 ] = { 0 };
char **environ = __env;

/**
 * @brief Transfer control to a new process.
 *	      Minimal implementation (for a system without processes)
 *
 * @Param name
 * @Param argv
 * @Param env
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _execve( const char *name, char * const argv[], char * const env[] )
{
    errno = ENOMEM;
    return -1;
}

/**
 * @brief Create a new process.
 *		  Minimal implementation (for a system without processes)
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _fork( void )
{
    errno = EAGAIN;
    return -1;
}


/**
 * @brief Status of an open file.
 *	      For consistency with other minimal implementations in these examples, all files are regarded as character special devices.
 *
 * @Param file
 * @Param st
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _fstat( int file, struct stat *st )
{
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * @brief Process-ID.
 *        this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _getpid( void )
{
    return 1;
}


/**
 * @brief Query whether output stream is a terminal.
 *        For consistency with the other minimal implementations, which only support output to stdout, this minimal implementation is suggested
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _isatty( int file )
{
    return 1;
}

/**
 * @brief Send a signal.
 *
 * @Param pid
 * @Param sig
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _kill( int pid, int sig )
{
    errno = EINVAL;
    return -1;
}

/**
 * @brief Establish a new name for a existing file.
 *
 * @Param old
 * @Param new
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _link( const char *old, const char *new )
{
    errno = EMLINK;
    return -1;
}

/**
 * @brief Set position in a file.
 *
 * @Param file
 * @Param ptr
 * @Param dir
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
off_t _lseek( int file, off_t ptr, int dir )
{
    return 0;
}


/**
 * @brief Open a file.
 *
 * @Param name
 * @Param flags
 * @Param mode
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _open( const char *name, int flags, int mode )
{
    return -1;
}



/**
 * @brief Increase program dada space.
 *        As malloc and related functions depend on this, it is useful to have a working implementation. The following suffices for a standalone system; it exploits the symbol _end automatically defined by the GNU linker
 *
 * @Param incr
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
char *heap_point = 0;
caddr_t _sbrk(unsigned int incr)
{
    extern unsigned long _heap_start;
    extern unsigned long _heap_limit;

    static char *prev_heap_point;

    if (heap_point == 0) {
        heap_point = (caddr_t)&_heap_start;
    }

    prev_heap_point = heap_point;

    if (heap_point + incr > (caddr_t)&_heap_limit) {
        write( 1, "Heap and Stack collision\n", 25 );
        errno = ENOMEM;
        return (caddr_t)-1;
        //abort();
    }

    heap_point += incr;

    return (caddr_t) prev_heap_point;
}


/**
 * @brief Status of a file (by name).
 *
 * @Param file
 * @Param st
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _stat( const char *file, struct stat *st )
{
    st->st_mode = S_IFCHR;
    return 0;
}


/**
 * @brief Timing information for current process.
 *
 * @Param buf
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
clock_t _times( struct tms *buf )
{
    return -1;
}


/**
 * @brief Remove a file's directory entry.
 *
 * @Param name
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _unlink( const char *name )
{
    errno = ENOENT;
    return -1;
}


/**
 * @brief Wait for a child process.
 *
 * @Param status
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _wait( int *status )
{
    errno = ECHILD;
    return -1;
}

/**
 * @brief Write to a file.
 *        libc subroutines will use this system routine for output to all files, including stdout—so if you need to generate any output, for example to a serial port for debugging, you should make your minimal write capable of doing this. The following minimal implementation is an incomplete example; it relies on a outbyte subroutine (not shown; typically, you must write this in assembler from examples provided by your hardware manufacturer) to actually perform the output.
 *
 * @Param file
 * @Param ptr
 * @Param len
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _write( int file, const void *ptr, size_t len )
{
    if( file == STDOUT_FILENO ) {
        //if( xPortIsInsideInterrupt() == pdTRUE) {
        //    return ( int ) UARTwrite( ( const char * )ptr, ( BaseType_t )len );
        //} else {
        return ( int ) UARTwrite( ( const char * )ptr, ( BaseType_t )len );
        //}
    }
    return 0;
}

/**
 * @brief Read from a file.
 *
 * @Param file
 * @Param ptr
 * @Param len
 *
 * @return
 */
/* ----------------------------------------------------------------------------*/
int _read( int file, void *ptr, size_t len )
{
    if( file == STDIN_FILENO ) {
        return UARTgets( ( char *) ptr, ( uint32_t ) len );
    }

    return 0;
}
