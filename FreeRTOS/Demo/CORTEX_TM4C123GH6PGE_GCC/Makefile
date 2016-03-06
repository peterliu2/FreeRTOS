#******************************************************************************
#
# Makefile - Rules for building the driver library and examples.
#
# Copyright (c) 2005,2006 Luminary Micro, Inc.  All rights reserved.
#
# Software License Agreement
#
# Luminary Micro, Inc. (LMI) is supplying this software for use solely and
# exclusively on LMI's Stellaris Family of microcontroller products.
#
# The software is owned by LMI and/or its suppliers, and is protected under
# applicable copyright laws.  All rights are reserved.  Any use in violation
# of the foregoing restrictions may subject the user to criminal sanctions
# under applicable laws, as well as to civil liability for the breach of the
# terms and conditions of this license.
#
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
# OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
# LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
#
#******************************************************************************

#
# Defines the part type the this project uses.
#
PART=TM4C123GH6PGE

#
# The base directory for TivaWare.
#
ROOT=.

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# source files locations.
#
RTOS_SOURCE_DIR=../../Source
DEMO_SOURCE_DIR=../Common/Minimal

#
# Where to find source files that do not live in this directory.
#
VPATH=${RTOS_SOURCE_DIR}:						\
	  ${RTOS_SOURCE_DIR}/portable/MemMang:		\
	  ${RTOS_SOURCE_DIR}/portable/GCC/ARM_CM4F:	\
	  ${DEMO_SOURCE_DIR}:						\
	  ${ROOT}/drivers:							\
	  ${ROOT}/init								\

#
# Where to find header files that do not live in the source directory.
#
IPATH=	${ROOT}										\
		${RTOS_SOURCE_DIR}/include  				\
		${RTOS_SOURCE_DIR}/portable/GCC/ARM_CM4F 	\
		${DEMO_SOURCE_DIR}/../include

#
# Object files to generate.
#
OBJS= ${COMPILER}/main.o	\
	  ${COMPILER}/list.o    \
      ${COMPILER}/queue.o   \
      ${COMPILER}/tasks.o   \
      ${COMPILER}/port.o    \
      ${COMPILER}/heap_1.o  \
	  ${COMPILER}/BlockQ.o	\
	  ${COMPILER}/PollQ.o	\
	  ${COMPILER}/integer.o	\
	  ${COMPILER}/semtest.o \
	  ${COMPILER}/cfal96x64x16.o

INIT_OBJS=${COMPILER}/startup.o

LIBS=${ROOT}/driverlib/${COMPILER}/libdriver.a   ${ROOT}/grlib/${COMPILER}/libgr.a

#
# The default rule, which causes init to be built.
#
all: ${COMPILER}
all: ${COMPILER}/RTOSDemo.axf

#
# The rule to clean out all the build products
#
clean:
	@rm -rf ${COMPILER} ${wildcard *.bin} RTOSDemo.axf 
	
#
# The rule to create the target directory
#
${COMPILER}:
	@mkdir -p ${COMPILER}


${COMPILER}/RTOSDemo.axf: ${INIT_OBJS} ${OBJS} ${LIBS} 

SCATTER_RTOSDemo=standalone.ld
ENTRY_RTOSDemo=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1 

#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__

DIRS = driverlib grlib ${ROOT}


build_all:
	@for i in ${DIRS};					\
	do									\
		if [ -f $${i}/Makefile ];		\
		then							\
			make -C $${i} || exit $$?;	\
		fi;								\
	done

clean_all:
	@for i in ${DIRS};             \
	 do                            \
	     if [ -f $${i}/Makefile ]; \
	     then                      \
	         make -C $${i} clean;  \
	     fi;                       \
	 done

clean_bulid: clean_all
clean_bulid: build_all

clean_bulid_flash: clean_bulid
	@echo  "\033[1m flash '${COMPILER}/RTOSDemo.bin' into uC \033[0m"
	@lm4flash  ${COMPILER}/RTOSDemo.bin

#
# The rule to rebuild the project and debug it with Nemiver.
#
 
 debug_nemiver debug_cgdb: clean_bulid


debug_nemiver:
	${ROOT}/debug_nemiver.sh ${shell pwd}/${COMPILER}/RTOSDemo.axf ek-tm4c123gxl.cfg

#
# The rule to rebuild the project and debug it with cgdb.
#
debug_cgdb:
	${ROOT}/debug_cgdb.sh ${shell pwd}/${COMPILER}/RTOSDemo.axf ek-tm4c123gxl.cfg



