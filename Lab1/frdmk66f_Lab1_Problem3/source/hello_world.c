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
#include "fsl_ftm.h"

#define FTM_MOTOR	FTM0
#define FTM_CHANNEL_DC_MOTOR   kFTM_Chnl_0

#define FTM_CHANNEL_SERVO_MOTOR kFTM_Chnl_3

#define TARGET_UART UART4

volatile char ch;
volatile int new_char = 0;

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;
	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTOR));
	mod = FTM_MOTOR->MOD;
	if (dutyCycle == 0U)
	{
		/* Signal stays low */
		cnv = 0;
	}
	else
	{
		cnv = mod * dutyCycle;
		/* For 100% duty cycle */
		if (cnv >= mod)
		{
			cnv = mod + 1U;
		}
	}
	FTM_MOTOR->CONTROLS[channel].CnV = cnv;
}

void setupPWM()
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = FTM_CHANNEL_DC_MOTOR;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}

void setupPWM_SERVO()
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = FTM_CHANNEL_SERVO_MOTOR;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}

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

void delay(void){
	for (int i=0; i< 800000; i++){
		__asm("NOP"); /* delay */
	}
}

/*!
 * @brief Main function
 */
int main(void)
{
    char txbuff[] = "Hello World\r\n";

    int input_DC, input_SERVO;
    float dutyCycle_DC, dutyCycle_SERVO;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    setupPWM();
    setupPWM_SERVO();

	/******* Delay *******/
	for(volatile int i = 0U; i < 1000000; i++)
	__asm("NOP");

	// reset
	updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, 0.0615);
	updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, 0.075);
	FTM_SetSoftwareTrigger(FTM_MOTOR, true);

    setupUART();

    //PRINTF("%s", txbuff);
    // Send multiple times - hopefully RF is stable enough
    for (int i=0; i<5; i++){
    	UART_WriteBlocking(TARGET_UART, txbuff, sizeof(txbuff) - 1);
    	delay(); printf("Sent: %s", txbuff);
    }

    while (1){

		char input[8];
		int i = 0;
		printf("Enter servo angle into terminal (-100, 100)\n");
		while (1){
			if(new_char)
			{
				new_char = 0;
				PUTCHAR(ch);

				input[i++] = ch;

				if (ch==13){ // newline
					break;
				}
			}
		}
		input_SERVO = atoi(input);
		printf("SERVO Value: %d\n", input_SERVO);

		dutyCycle_SERVO = input_SERVO * 0.025f/100.0f + 0.075;
		updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dutyCycle_DC);
		FTM_SetSoftwareTrigger(FTM_MOTOR, true);

		printf("Enter motor speed into terminal (-100, 100)\n");

		i = 0;
		while (1){
			if(new_char)
			{
				new_char = 0;
				PUTCHAR(ch);

				input[i++] = ch;

				if (ch==13){
					break;
				}
			}
		}
		input_DC = atoi(input);
		printf("MOTOR Value: %d\n", input_DC);

		dutyCycle_DC = input_DC * 0.025f/100.0f + 0.0615;
		updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, dutyCycle_SERVO);
		FTM_SetSoftwareTrigger(FTM_MOTOR, true);
    }
}
