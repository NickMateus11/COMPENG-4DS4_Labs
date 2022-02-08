/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_uart.h"

#define TARGET_UART UART4

volatile char ch;
volatile int new_char = 0;

void setupUART()
{
	uart_config_t config;
	UART_GetDefaultConfig(&config);

	config.baudRate_Bps = 57600;
	config.enableTx = true;
	config.enableRx = true;
	config.enableRxRTS = true;
	config.enableTxCTS = true;

	UART_Init(TARGET_UART, &config, CLOCK_GetFreq(kCLOCK_BusClk));

	/******** Enable Interrupts *********/
	UART_EnableInterrupts(TARGET_UART, kUART_RxDataRegFullInterruptEnable);
	EnableIRQ(UART4_RX_TX_IRQn);
}

void UART4_RX_TX_IRQHandler()
{
	UART_GetStatusFlags(TARGET_UART);
	ch = UART_ReadByte(TARGET_UART);
	new_char = 1;
}

/*!
 * @brief Main function
 */
int main(void)
{
    char txbuff[] = "Hello World\r\n";

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    setupUART();

    PRINTF("%s", txbuff);
    for (int i=0; i<10; i++)
    	UART_WriteBlocking(TARGET_UART, txbuff, sizeof(txbuff) - 1);

    while (1)
    {
		if(new_char)
		{
			new_char = 0;
			PUTCHAR(ch);
		}
	}
}
