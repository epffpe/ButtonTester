/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
#include "includes.h"


#define FSMOK				(0)
#define	FSMERRORLOW			(-1)
#define	FSMERRORHIGH		(-2)




#define TASKSTACKSIZE   512


Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

volatile bool g_bStart = false;
volatile unsigned int g_ui32counter = 0;
volatile unsigned int g_ui32button = 0;
volatile unsigned int g_ui32errorLow = 0;
volatile unsigned int g_ui32errorHigh = 0;


//extern Void ttyFxn(UArg arg0, UArg arg1);

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */

typedef enum {
	LOW,
	HIGHT
} state_t;

int FSM(state_t *state, uint32_t input)
{
	int result;
	switch(*state){
	case LOW:
		if (!input){
			result = FSMERRORLOW;
		}else{
			*state = HIGHT;
			result = FSMOK;
		}
		break;
	case HIGHT:
		if (input){
			result = FSMERRORHIGH;
		}else{
			*state = LOW;
			result = FSMOK;
		}
		break;
	default:
		break;
	}
	return (result);
}

Void heartBeatFxn(UArg arg0, UArg arg1)
{
	state_t state = HIGHT;
	unsigned int l_ui32Button;
	int l_i32Result;

	uint32_t pui32Data[4];
	uint32_t ui32EEPROMInit, ui32EEPROMSize;

	g_ui32counter = 0, g_ui32button = 0;
	ui32EEPROMSize = ui32EEPROMSize;



	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
		Task_sleep(1);
	}

	ui32EEPROMInit = EEPROMInit();

	if (ui32EEPROMInit != EEPROM_INIT_OK){};

	ui32EEPROMSize = EEPROMSizeGet();

	EEPROMRead(pui32Data, 0x000, sizeof(pui32Data));

	g_ui32counter = pui32Data[0];
	g_ui32button = pui32Data[1];
	g_ui32errorLow = pui32Data[2];
	g_ui32errorHigh = pui32Data[3];

	while (1) {
		if ( g_bStart && (g_ui32counter < 1e6) ){
			g_ui32counter++;
			GPIO_write(Board_LED0, 1);
			Task_sleep(150);

			l_ui32Button = GPIO_read(Board_BUTTON1);
			l_i32Result = FSM(&state, l_ui32Button);


			switch(l_i32Result){
			case FSMERRORLOW:
				g_ui32errorLow++;
				break;
			case FSMERRORHIGH:
				g_ui32errorHigh++;
				break;
			default:
				g_ui32button++;
				break;
			}
//			if (l_i32Result == FSMOK) {
//				g_ui32button++;
//			}else{
//				g_ui32errorLow++;
//			}

			Task_sleep(250);

			GPIO_write(Board_LED0, 0);
			Task_sleep(200);

			l_ui32Button = GPIO_read(Board_BUTTON1);
			l_i32Result = FSM(&state, l_ui32Button);


			switch(l_i32Result){
			case FSMERRORLOW:
				g_ui32errorLow++;
				break;
			case FSMERRORHIGH:
				g_ui32errorHigh++;
				break;
			default:
				break;
			}

			pui32Data[0] = g_ui32counter;
			pui32Data[1] = g_ui32button;
			pui32Data[2] = g_ui32errorLow;
			pui32Data[3] = g_ui32errorHigh;

			EEPROMProgram(pui32Data, 0x000, sizeof(pui32Data));

			Task_sleep(400);
		}else{
			Task_sleep(100);
		}

	}
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    // Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    initTTY();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_OFF);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}



