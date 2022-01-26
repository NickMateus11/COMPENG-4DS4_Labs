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

void testFunctionA()
{
	int x = 0;
	int *ptr = &x;
	int *ptr_location = 0x20001000;

	*ptr = 10;
	*ptr_location = 11;
}

void testFunctionB()
{
	int *ptr_location = (int*)0x20001000;
	*ptr_location = 11;

	*((int*)0x20001004) = 12;
	int x = *((int*)0x20001004);
}

void problem1()
{
	uint8_t *base = (uint8_t *)0x20001000;

	*(uint8_t  *)(base + 0x0) = 0xAC;		//Loc1
	*(uint32_t *)(base + 0x1) = 0xAABBCCDD; //Loc2
	*(uint16_t *)(base + 0x5) = 0xABCD;		//Loc3
	*(uint32_t *)(base + 0x7) = 0xAABBCCDD;	//Loc4
}

typedef struct
//typedef struct __attribute__((__packed__))
{
	int location_1;
	char location_2;
	int location_3;
}ARBITRARY_MODULE;

#define MODULE ((ARBITRARY_MODULE*)0x20001000)

void testFunction2()
{
	MODULE->location_1 = 0xAAAAAAAA;
	MODULE->location_2 = 0xBB;
	MODULE->location_3 = 0xCCCCCCCC;
}

struct struct1
{
	char x2;
	int x1;
};
struct struct2
{
	short x2;
	int x1;
};
struct struct3
{
	int x1;
	short x2;
};
struct struct4
{
	struct inner_struct
	{
		char x1;
		short x2;
		int x3;
	} inner_struct_1;
	int x1;
};


/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("hello world.\r\n");

    testFunctionA(); //Experiment 2 - A
    testFunctionB(); //Experiment 2 - B
    testFunction2(); //Experiment 2 - C

    struct struct1 s1 = {0xFF,0xAAAAAAAA};
    struct struct2 s2 = {0xFFFF,0xAAAAAAAA};
    struct struct3 s3 = {0xAAAAAAAA,0xFFFF};
//    struct inner_struct si = {0xFF, 0xBBBB, 0xAAAAAAAA};
    struct struct4 s4 = {0xFF, 0xBBBB, 0xCCCCCCCC, 0xAAAAAAAA};

    PRINTF("%d\n", sizeof(s1));
    PRINTF("%d\n", sizeof(s2));
    PRINTF("%d\n", sizeof(s3));
    PRINTF("%d\n", sizeof(s4));

    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }
}
