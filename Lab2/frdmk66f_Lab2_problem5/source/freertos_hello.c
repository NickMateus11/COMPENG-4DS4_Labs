/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
#define TARGET_UART UART4

void setupUART() {
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
	NVIC_SetPriority(UART4_RX_TX_IRQn, 2);
	EnableIRQ(UART4_RX_TX_IRQn);
}

void UART4_RX_TX_IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	UART_GetStatusFlags(TARGET_UART);

	char ch = UART_ReadByte(TARGET_UART);
	switch (ch) {
		case 'a':
			xEventGroupSetBitsFromISR(event_group_global_handler, LEFT_BIT, &
				xHigherPriorityTaskWoken);
		break;
		case 's':
			xEventGroupSetBitsFromISR(event_group_global_handler, DOWN_BIT, &
				xHigherPriorityTaskWoken);
		break;
		case 'd':
			xEventGroupSetBitsFromISR(event_group_global_handler, RIGHT_BIT, &
				xHigherPriorityTaskWoken);
		break;
		case 'w':
			xEventGroupSetBitsFromISR(event_group_global_handler, UP_BIT, &
				xHigherPriorityTaskWoken);
		break;
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*!
 * @brief Application entry point.
 */
int main(void) {

	BaseType_t status;

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();

	setupUART();

	event_group_global_handler = xEventGroupCreate();
	status = xTaskCreate(consumer_event, "consumer", 200,
			(void*) event_group_global_handler, 3, NULL);

	if (status != pdPASS) {
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

	vTaskStartScheduler();
	while (1)
		;
}
