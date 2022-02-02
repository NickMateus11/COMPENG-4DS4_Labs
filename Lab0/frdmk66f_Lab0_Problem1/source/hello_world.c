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

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

#define MEM_LOC_INT(x) *((int*)x)
#define MEM_LOC_CHAR(x) *((char*)x)
#define MEM_LOC_SHORT(x) *((short*)x)
#define ARBITRARY_LOC1 MEM_LOC_CHAR(0x20001000)
#define ARBITRARY_LOC2 MEM_LOC_INT(0x20001001)
#define ARBITRARY_LOC3 MEM_LOC_SHORT(0X20001005)
#define ARBITRARY_LOC4 MEM_LOC_INT(0X20001007)

void testFunction()
{
	ARBITRARY_LOC1 = 0xAC;
	ARBITRARY_LOC2 = 0xAABBCCDD;
	ARBITRARY_LOC3 = 0xABCD;
	ARBITRARY_LOC4 = 0xAABBCCDD;

}



int main(void)
{
    char ch;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("hello world.\r\n");
    testFunction();
    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }


}
