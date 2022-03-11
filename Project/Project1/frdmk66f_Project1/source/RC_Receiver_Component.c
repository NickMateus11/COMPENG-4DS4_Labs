#include "RC_Receiver_Component.h"

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;

void setupRCReceiverComponent()
{
	BaseType_t status;
	setupRCPins();

	setupUART_RC();

    /*************** RC Task ***************/
	//Create RC Semaphore
	SemaphoreHandle_t* rc_hold_semaphore = (SemaphoreHandle_t*) malloc(3 * sizeof(SemaphoreHandle_t));
	rc_hold_semaphore[0] = xSemaphoreCreateBinary();

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

}


