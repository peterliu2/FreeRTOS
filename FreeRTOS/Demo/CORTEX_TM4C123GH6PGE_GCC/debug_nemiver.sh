#!/bin/bash

# check argument counts
[ "$#" != "2" ] && echo -e "$0: error argument counts (Must 2)\n\t1-> debug file loaction(full path)\n\t2-> openocd scription file\n" && exit 1

# print openOCD informations
echo "debug file loaction(full path): " $1
echo "openocd scription file: " $2

# The base directory for TivaWare.
ROOT=.

# location of OpenOCD Board .cfg files
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# start xterm with openocd in the background
xterm -e openocd -f $OPENOCD_BOARD_DIR/$2 &

# save the PID of the background process
XTERM_PID=$!

# wait a bit to be sure the hardware is ready
sleep 2

# execute some initialisation commands via gdb
arm-none-eabi-gdb --batch --command=${ROOT}/init.gdb $1 

# start the gdb gui
nemiver --remote=localhost:3333 --gdb-binary="$(which arm-none-eabi-gdb)" $1

# close xterm when the user has exited nemiver
kill $XTERM_PID
