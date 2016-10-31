/*
 * tty.h
 *
 *  Created on: Oct 17, 2016
 *      Author: epenate
 */

#ifndef TTY_H_
#define TTY_H_

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/drivers/UART.h>
#include "Board.h"

#include <ctype.h>
#include <stdio.h>
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef	__TTY_GLOBAL
	#define	__TTY_EXT
#else
	#define __TTY_EXT	extern
#endif
__TTY_EXT void initTTY(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* TTY_H_ */
