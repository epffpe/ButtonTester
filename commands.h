/*
 * commands.h
 *
 *  Created on: Oct 18, 2016
 *      Author: epenate
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/drivers/UART.h>

//*****************************************************************************
//
// Defines for the command line argument parser provided as a standard part of
// TivaWare.  hibernate example application uses the command line parser to
// extend functionality to the serial port.
//
//*****************************************************************************

#define CMDLINE_MAX_ARGS 2


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

#ifdef	__COMMANDS_GLOBAL
	#define	__COMMANDS_EXT
#else
	#define __COMMANDS_EXT	extern
#endif

//*****************************************************************************
//
// Declaration for the callback functions that will implement the command line
// functionality.  These functions get called by the command line interpreter
// when the corresponding command is typed into the command line.
//
//*****************************************************************************
//__COMMANDS_EXT int CMD_help (int argc, char **argv);
////extern int CMD_hib (int argc, char **argv);
////extern int CMD_date (int argc, char **argv);
////extern int CMD_time12(int argc, char **argv);
////extern int CMD_time24(int argc, char **argv);
//__COMMANDS_EXT int CMD_cls(int argc, char **argv);

//__COMMANDS_EXT UART_Handle uartHandle;
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif




#endif /* COMMANDS_H_ */
