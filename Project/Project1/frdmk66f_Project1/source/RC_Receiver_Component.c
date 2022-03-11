#include "RC_Receiver_Component.h"

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;

typedef struct {
	uint16_t header;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
} RC_Values;

void setupRCReceiverComponent()
{
	BaseType_t status;
	setupRCPins();

	setupUART_RC();

    /*************** RC Task ***************/
	//Create RC Semaphore
	rc_hold_semaphore = (SemaphoreHandle_t*) malloc(1 * sizeof(SemaphoreHandle_t));
	rc_hold_semaphore = xSemaphoreCreateBinary();

	//Create RC Task
	status = xTaskCreate(rcTask, "rc", 200, (void*)rc_hold_semaphore, 2, NULL);
	if (status != pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}
}

void setupRCPins()
{
	//Configure RC pins
	CLOCK_EnableClock(kCLOCK_PortC);
	PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTC, 13U, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);
	CLOCK_EnableClock(kCLOCK_PortE);
	PORT_SetPinMux(PORTE, 27U, kPORT_MuxAlt3);
}

void setupUART_RC()
{
	//setup UART for RC receiver
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 57600;
	config.enableTx = true;
	config.enableRx = true;
	config.enableRxRTS = true;
	config.enableTxCTS = true;
	UART_Init(TARGET_UART, &config, CLOCK_GetFreq(kCLOCK_BusClk));
}

void rcTask(void* pvParameters)
{
	//RC task implementation
	SemaphoreHandle_t* semaphores = (SemaphoreHandle_t*)pvParameters;
	SemaphoreHandle_t hold_semaphore = semaphores;

	BaseType_t status;
	uart_config_t config;
	RC_Values rc_values;
	uint8_t* ptr = (uint8_t*) &rc_values;
	int motor, angle;

	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;

	UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	while(1)
	{
//		status = xSemaphoreTake(hold_semaphore, portMAX_DELAY);
//		if (status != pdPASS)
//		{
//			PRINTF("Failed to acquire hold_semaphore\r\n");
//			while (1);
//		}

		UART_ReadBlocking(UART1, ptr, 1);
		if(*ptr != 0x20)
			continue;

		UART_ReadBlocking(UART1, &ptr[1], sizeof(rc_values) - 1);
		if(rc_values.header == 0x4020)
		{

			//right joy stick for forward/backward
//			printf("Channel 1 = %d\t", rc_values.ch1);
			printf("Channel 2 = %d\t", rc_values.ch2);
			motor = (int)(rc_values.ch2 * 1.0f/5.0f - 300);

			status = xQueueSendToBack(motor_queue, (void*) &motor, portMAX_DELAY);
			if (status != pdPASS)
			{
				PRINTF("Queue Send failed!.\r\n");
				while (1);
			}

			//left joy stick for left/right
//			printf("Channel 3 = %d\t", rc_values.ch3);
			printf("Channel 4 = %d\t", rc_values.ch4);
			angle = (int)(rc_values.ch4 * 1.0f/5.0f - 300);

			status = xQueueSendToBack(angle_queue, (void*) &angle, portMAX_DELAY);
			if (status != pdPASS)
			{
				PRINTF("Queue Send failed!.\r\n");
				while (1);
			}

//			printf("Channel 5 = %d\t", rc_values.ch5);
//			printf("Channel 6 = %d\t", rc_values.ch6);
//			printf("Channel 7 = %d\t", rc_values.ch7);
//			printf("Channel 8 = %d\r\n", rc_values.ch8);
		}
//		xSemaphoreGive(hold_semaphore);
	}
}


