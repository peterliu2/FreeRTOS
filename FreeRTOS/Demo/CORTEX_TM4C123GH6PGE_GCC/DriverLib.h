#ifndef DRIVER_LIB_H
#define DRIVER_LIB_H

#define UART_BUFFERED

#include "tm4c123gh6pge.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"

#include "grlib/grlib.h"

#include "drivers/cfal96x64x16.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"
#include "utils/cpu_usage.h"

#include "commands.h"

#endif
