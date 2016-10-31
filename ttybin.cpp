/*
 * ttybin.c
 *
 *  Created on: Oct 21, 2016
 *      Author: epenate
 */


#include "includes.h"
#include <ctype.h>
#include <stdio.h>
#include <string>         // std::string
#include <string.h>         // std::string
#define CNTLQ      0x11
#define CNTLS      0x13
#define DEL        0x7F
#define BACKSPACE  0x08
#define CR         0x0D
#define LF         0x0A
#define ESC        0x1B


Task_Struct taskTTYBinStruct;
Char taskTTYBinStack[2048];

typedef struct ttycmd {
	char val[24];
	void (*func)(char *par, char *cmdbuf);
} TTYCMD;

typedef struct {
	int tipo;
	int addr;
	int payload;
}VEST_CMD_t;

volatile uint8_t QTMode = 0;


typedef void (*QTCMD_TypeDef)(UART_Handle uart, char *par, char *cmdbuf);
void QT_QTLoop(UART_Handle uart, char *par, char *cmdbuf);
int QT_checksum(int *datos, unsigned int N);



/*
 *  ======== ttyFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void ttyBinFxn(UArg arg0, UArg arg1)
{
	char *next;
	//	char input, cmdbuf[64];
	char msg[128];
	UART_Params uartParams;
	UART_Handle uart;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;//115900
	uart = UART_open(Board_UART0, &uartParams);

	if (uart == NULL) {
		System_abort("Error opening the UART");
	}

	while (1) {
		QTMode = 1;
		QT_QTLoop(uart, next, msg);
	}
}

void ttybin_init(void){
	Task_Params task1Params;

	Task_Params_init(&task1Params);
	task1Params.arg0 = 1000;
	task1Params.stackSize = 2048;
	task1Params.stack = &taskTTYBinStack;
	Task_construct(&taskTTYBinStruct, (Task_FuncPtr)ttyBinFxn, &task1Params, NULL);
}

/****************************************************************/




void qDoNothing(UART_Handle uart, char *par, char *cmdbuf){
	if (QT_checksum( (int *)par, 2) == 0) {

	}
}

/****************************************************************/

/****************************************************************/
static const QTCMD_TypeDef qcomandos[] = {
		qDoNothing,					/*  0*/
		qDoNothing,					/*  1*/
		qDoNothing,					/*  2*/
		qDoNothing,					/*  3*/
		qDoNothing,					/*  4*/
		qDoNothing,					/*  5*/
		qDoNothing,					/*  6*/
		qDoNothing,					/*  7*/
		qDoNothing,					/*  8*/
		qDoNothing,					/*  9*/

		qDoNothing,					/* 10*/
		qDoNothing,					/* 11*/
		qDoNothing,					/* 12*/
		qDoNothing,					/* 13*/
		qDoNothing,					/* 14*/
		qDoNothing,					/* 15*/
		qDoNothing,					/* 16*/
		qDoNothing,					/* 17*/
		qDoNothing,					/* 18*/
		qDoNothing,					/* 19*/

		qDoNothing,					/* 20*/
		qDoNothing,					/* 21*/
		qDoNothing,					/* 22*/
		qDoNothing,					/* 23*/
		qDoNothing,					/* 24*/
		qDoNothing,					/* 25*/
		qDoNothing,					/* 26*/
		qDoNothing,					/* 27*/
		qDoNothing,					/* 28*/
		qDoNothing,					/* 29*/

		qDoNothing,					/* 30*/
		qDoNothing,					/* 31*/
		qDoNothing,					/* 32*/
		qDoNothing,					/* 33*/
		qDoNothing,					/* 34*/
		qDoNothing,					/* 35*/
		qDoNothing,					/* 36*/
		qDoNothing,					/* 37*/
		qDoNothing,					/* 38*/
		qDoNothing,					/* 39*/

		qDoNothing,					/* 40*/
		qDoNothing,					/* 41*/
		qDoNothing,					/* 42*/
		qDoNothing,					/* 43*/
		qDoNothing,					/* 44*/
		qDoNothing,					/* 45*/
		qDoNothing,					/* 46*/
		qDoNothing,					/* 47*/
		qDoNothing,					/* 48*/
		qDoNothing,					/* 49*/

		qDoNothing,					/* 50*/
		qDoNothing,					/* 51*/
		qDoNothing,					/* 52*/
		qDoNothing,					/* 53*/
		qDoNothing,					/* 54*/
		qDoNothing,					/* 55*/
		qDoNothing,					/* 56*/
		qDoNothing,					/* 57*/
		qDoNothing,					/* 58*/
		qDoNothing,					/* 59*/

		//	QT::qReset
};
#define QTCMD_COUNT   (sizeof (qcomandos) / sizeof (qcomandos[0]))



void QT_qtRecibe(UART_Handle uart, char *par, char *cmdbuf){

	unsigned int n, tipo;
	unsigned int *tipoptr = (unsigned int *) par;
	tipo = *tipoptr;


	if (tipo & 0x80000000UL){
		tipo &= 0x00FFFFFFFUL;
		if ( tipo < QTCMD_COUNT) {
			//qcomandos[*tipoptr].func(par, comm, cmdbuf);
			if (qcomandos[tipo]){
				qcomandos[tipo](uart, par, cmdbuf);
			}

		}else{
			n = sprintf(cmdbuf, "\rError\n");
			UART_write(uart, cmdbuf, n);
			//		BSP_UART0_bprintf(cmdbuf, "\rError\n");
		}
	}

}
/****************************************************************/
int QT_checksum(int *datos, unsigned int N){
	unsigned int i;
	int  sum = 0;

	for (i = N/4; i!=0; i--){
		sum += *(datos++);
		sum += *(datos++);
		sum += *(datos++);
		sum += *(datos++);
	}
	for (i = N&3; i!=0; i--) {
		sum += *(datos++);
	}
	return sum;
}
/****************************************************************/
int QT_creatrama(char * buffer, int tipo, char *data, int len){
	unsigned int i;
	int longitud = 8;
	int *bufferptr = (int *)&buffer[4];
	int *datoptr = (int *)data;
	int  sum = 0, temp;
	/* Header */
	buffer[0] = 'A';
	buffer[1] = 'L';
	buffer[2] = 'T';
	buffer[3] = 'O';
	/* tipo */
	sum = tipo | 0x80000000;
	*bufferptr++ = sum;
	for (i = len/4; i != 0; i--){
		temp = *datoptr++;
		*bufferptr++ = temp;
		sum += temp;
		longitud += 4;
	}
	*bufferptr++ = - sum;
	longitud += 4;
	buffer[longitud++] = '\r';
	buffer[longitud++] = '\n';
	buffer[longitud++] = '\r';
	buffer[longitud++] = '\n';
	return longitud;
}

/****************************************************************/
/****************************************************************/
void QT_QTLoop(UART_Handle uart, char *par, char *cmdbuf){
	int nbytes;
	std::string paqueterecibido;
	std::string finTrama ("\r\n\r\n");
	std::string inicioTrama ("ALTO");
	cmdbuf[0] = 0;
	while(QTMode){
		GPIO_write(Board_LED1, Board_LED_OFF);
		nbytes = UART_read(uart, cmdbuf, 1);
		GPIO_write(Board_LED1, Board_LED_ON);
		std::string trama(cmdbuf,nbytes);

		paqueterecibido += trama;

		std::size_t indice = paqueterecibido.find(finTrama);

		while( indice != std::string::npos){
			indice = paqueterecibido.find("ALTO");
			if ( indice != std::string::npos){
				paqueterecibido = paqueterecibido.substr(indice + 4);
				indice = paqueterecibido.find(finTrama);
				if ( indice != std::string::npos){
					std::string str = paqueterecibido.substr(0, indice);
					QT_qtRecibe(uart, (char *)str.data(), cmdbuf);
					paqueterecibido = paqueterecibido.substr(indice + 4);
					indice = paqueterecibido.find(finTrama);
				}
			}
		}
		if (paqueterecibido.size() > 256) paqueterecibido.clear();
	}

}


