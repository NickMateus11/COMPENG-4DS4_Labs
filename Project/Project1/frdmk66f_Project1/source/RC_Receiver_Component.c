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
	xSemaphoreGive(rc_hold_semaphore);

}

void setupRCPins()
{
	//Configure RC pins
	CLOCK_EnableClock(kCLOCK_PortC);
	PORT_SetPinMux(PORTC, 3U, kPORT_MuxAlt3);
}

void setupUART_RC()
{
	//setup UART for RC receiver
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;

	UART_Init(RC_UART, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}

void rcTask(void* pvParameters)
{
	//RC task implementation



	BaseType_t status;
	RC_Values rc_values;
	uint8_t* ptr = (uint8_t*) &rc_values;
	msg_struct_t motor = {.type=0, .val=0};
	int angle, motor_prev, angle_prev;
	motor_prev = 0;
	angle_prev = 0;


	while(1)
	{

		status = xSemaphoreTake(rc_hold_semaphore, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Failed to acquire hold_semaphore\r\n");
			while (1);
		}

		UART_ReadBlocking(RC_UART, ptr, 1);

		if(*ptr != 0x20) {
			xSemaphoreGive(rc_hold_semaphore);
			continue;
		}

		UART_ReadBlocking(RC_UART, &ptr[1], sizeof(rc_values) - 1);
		if(rc_values.header == 0x4020)
		{

//			printf("Channel 2 = %d\t", rc_values.ch2);
			if(rc_values.ch2 != motor_prev){
				//right joy stick for forward/backward
	//			printf("Channel 1 = %d\t", rc_values.ch1);
				printf("Channel 2 = %d\t\n", rc_values.ch2);
				motor.val = (int)(rc_values.ch2 * 1.0f/5.0f - 300);
				printf("Channel 2 motor value = %d\t\n", motor.val);
				motor_prev = rc_values.ch2;

				status = xQueueSendToBack(motor_queue, (void*) &motor, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}
			}else if(rc_values.ch2 == 1500){
				motor.val = 0;
				status = xQueueSendToBack(motor_queue, (void*) &motor, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}
			}


//			printf("Channel 4 = %d and prev = %dt\n", rc_values.ch4,angle_prev);
			if(rc_values.ch4 != angle_prev){
				//left joy stick for left/right
	//			printf("Channel 3 = %d\t", rc_values.ch3);
				printf("Channel 4 = %d\t\n", rc_values.ch4);
				angle = (int)(-1 * (rc_values.ch4 * 1.0f/5.0f - 300));
				angle_prev = rc_values.ch4;

				status = xQueueSendToBack(angle_queue, (void*) &angle, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}
			}

//			printf("Channel 5 = %d\t", rc_values.ch5);
//			printf("Channel 6 = %d\t", rc_values.ch6);
//			printf("Channel 7 = %d\t", rc_values.ch7);
//			printf("Channel 8 = %d\r\n", rc_values.ch8);
		}
		xSemaphoreGive(rc_hold_semaphore);
	}
}


