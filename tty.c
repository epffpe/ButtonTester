/*
 * tty.c
 *
 *  Created on: Oct 17, 2016
 *      Author: epenate
 */


#include "includes.h"
#include "utils/cmdline.h"

#define CNTLQ      0x11
#define CNTLS      0x13
#define DEL        0x7F
#define BACKSPACE  0x08
#define CR         0x0D
#define LF         0x0A

// http://www.google.com/url?q=http%3A%2F%2Fen.wikipedia.org%2Fwiki%2FANSI_escape_code%23Colors&sa=D&sntz=1&usg=AFQjCNEf_SdnjOkoNdH0NmZcdzLk1yy4ew

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32;1m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

Task_Struct task1Struct;
Char task1Stack[2048];


const char prompt[] = "\r" ANSI_COLOR_GREEN "epenate@ALTOAviation: > " ANSI_COLOR_RESET;

/*----------------------------------------------------------------------------
  Line Editor
 *----------------------------------------------------------------------------*/
void getline (UART_Handle uart, char *line, int n)
{
	int  cnt = 0;
	char c;
	char space;
	unsigned int nbytes;

	nbytes = nbytes;

	do {
		nbytes = UART_read(uart, &c, 1);
		if (c == CR) c = LF;  					/* read character                 */
		if (c == BACKSPACE  ||  c == DEL) {     /* process backspace              */
			if (cnt != 0)  {
				cnt--;                              /* decrement count                */
				line--;                             /* and line pointer               */
				space = BACKSPACE;
				UART_write(uart, &space, 1);                /* echo backspace                 */
				space = ' ';
				UART_write(uart, &space, 1);
//				putchar (' ');
				space = BACKSPACE;
				UART_write(uart, &space, 1);
			}
		}
		else if (c != CNTLQ && c != CNTLS) {    /* ignore Control S/Q             */
			*line = c;
			UART_write(uart, &c, 1);
//			putchar (*line = c);                  /* echo and store character       */
			line++;                               /* increment line pointer         */
			cnt++;                                /* and count                      */
		}
	} while (cnt < n - 1  &&  c != LF);       /* check limit and line feed      */
	*(line - 1) = 0;                          /* mark end of string             */
}

/*
 *  ======== ttyFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void ttyFxn(UArg arg0, UArg arg1)
{
	UART_Handle uart;
	UART_Params uartParams;
	int n;
	char cmdbuf[64], buff[64];
	int32_t i32CmdStatus;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;
	uart = UART_open(Board_UART0, &uartParams);
	//	UART_read(uart, cmdbuf, 1);
//	uartHandle = uart;
	while(1){
		UART_write(uart, prompt, sizeof(prompt));
		getline (uart, cmdbuf, 64);
		//
		// Pass the line from the user to the command processor.
		// It will be parsed and valid commands executed.
		//
		i32CmdStatus = CmdLineProcess(uart, cmdbuf);

		//
		// Handle the case of bad command.
		//
		if(i32CmdStatus == CMDLINE_BAD_CMD)
		{
			n = sprintf(buff, ANSI_COLOR_RED "Command not recognized!\n" ANSI_COLOR_RESET);
			UART_write(uart, buff, n);
		}

		//
		// Handle the case of too many arguments.
		//
		else if(i32CmdStatus == CMDLINE_TOO_MANY_ARGS)
		{
			n = sprintf(buff, ANSI_COLOR_RED "Too many arguments for command processor!\n" ANSI_COLOR_RESET);
			UART_write(uart, buff, n);
		}

		//
		// Handle the case of too few arguments.
		//
		else if(i32CmdStatus == CMDLINE_TOO_FEW_ARGS)
		{
			n = sprintf(buff, ANSI_COLOR_RED "Too few arguments for command processor!\n" ANSI_COLOR_RESET);
			UART_write(uart, buff, n);
		}
	}
}

void initTTY(void){
	Task_Params task1Params;

	Task_Params_init(&task1Params);
	task1Params.arg0 = 1000;
	task1Params.stackSize = 2048;
	task1Params.stack = &task1Stack;
	Task_construct(&task1Struct, (Task_FuncPtr)ttyFxn, &task1Params, NULL);
}


