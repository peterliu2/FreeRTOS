/***********************************************************************/
/*                                                                     */
/*  FILE        :resetprg.c                                            */
/*  DATE        :Wed, Aug 11, 2010                                     */
/*  DESCRIPTION :Reset Program                                         */
/*  CPU TYPE    :Other                                                 */
/*                                                                     */
/*  This file is generated by Renesas Project Generator (Ver.4.50).    */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/



/*********************************************************************
*
* Device     : RX/RX200
*
* File Name  : resetprg.c
*
* Abstract   : Reset Program.
*
* History    : 1.00  (2009-08-07)
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright(c) 2009 Renesas Technology Corp.
*               And Renesas Solutions Corp.,All Rights Reserved.
*
*********************************************************************/

#include	<machine.h>
#include	<_h_c_lib.h>
//#include	<stddef.h>					// Remove the comment when you use errno
//#include 	<stdlib.h>					// Remove the comment when you use rand()
#include	"typedefine.h"
#include	"stacksct.h"

#pragma inline_asm Change_PSW_PM_to_UserMode
static void Change_PSW_PM_to_UserMode(void);

#ifdef __cplusplus
extern "C" {
#endif
void PowerON_Reset_PC(void);
void main(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus				// Use SIM I/O
extern "C" {
#endif
extern void _INIT_IOLIB(void);
extern void _CLOSEALL(void);
#ifdef __cplusplus
}
#endif

#define PSW_init  0x00010000
#define FPSW_init 0x00000100

//extern void srand(_UINT);		// Remove the comment when you use rand()
//extern _SBYTE *_s1ptr;				// Remove the comment when you use strtok()

//#ifdef __cplusplus				// Use Hardware Setup
//extern "C" {
//#endif
//extern void HardwareSetup(void);
//#ifdef __cplusplus
//}
//#endif

//#ifdef __cplusplus			// Remove the comment when you use global class object
//extern "C" {					// Sections C$INIT and C$END will be generated
//#endif
//extern void _CALL_INIT(void);
//extern void _CALL_END(void);
//#ifdef __cplusplus
//}
//#endif

#pragma section ResetPRG

#pragma entry PowerON_Reset_PC

void PowerON_Reset_PC(void)
{
    set_intb((void*)__sectop("C$VECT"));
//	set_fpsw(FPSW_init);

    _INITSCT();

//	_INIT_IOLIB();					// Remove the comment when you use SIM I/O

//	errno=0;						// Remove the comment when you use errno
//	srand((_UINT)1);				// Remove the comment when you use rand()
//	_s1ptr=NULL;					// Remove the comment when you use strtok()

//	HardwareSetup();				// Use Hardware Setup
    nop();

//	_CALL_INIT();					// Remove the comment when you use global class object

    set_psw(PSW_init);				// Set Ubit & Ibit for PSW
//	Change_PSW_PM_to_UserMode();	// DO NOT CHANGE TO USER MODE IF USING FREERTOS!
    ( void ) Change_PSW_PM_to_UserMode; // Just to avoid compiler warnings.

    main();

//	_CLOSEALL();					// Use SIM I/O

//	_CALL_END();					// Remove the comment when you use global class object

    brk();
}

static void Change_PSW_PM_to_UserMode(void)
{
    MVFC   PSW,R1
    OR     #00100000h,R1
    PUSH.L R1
    MVFC   PC,R1
    ADD    #10,R1
    PUSH.L R1
    RTE
    NOP
    NOP
}
