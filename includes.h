/*
 * includes.h
 *
 *  Created on: Oct 18, 2016
 *      Author: epenate
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdint.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_types.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"
#include "cmdline.h"
#include "tty.h"
#include "commands.h"

/* Board Header file */
#include "Board.h"

#include "tty.h"
#include "ttybin.h"





#ifdef __cplusplus
extern "C"  {
#endif

#ifdef	__INCLUDES_GLOBAL
	#define	__INCLUDES_EXT
#else
	#define __INCLUDES_EXT	extern
#endif


#ifdef __cplusplus
}
#endif





#ifdef __cplusplus


#endif



#endif /* INCLUDES_H_ */
