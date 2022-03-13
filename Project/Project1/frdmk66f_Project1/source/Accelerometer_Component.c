#include "Accelerometer_Component.h"

static fxos_handle_t fxosHandle;
static uint8_t data_scale;

void setupAccelerometerComponent()
{
	setupAccelerometerPins();
	voltageRegulator_enable();
	accelerometer_enable();

	spi_init();

    /*************** Accelerometer Task ***************/
	//Create Accelerometer Task
	int status = xTaskCreate(accelerometerTask, "accelerometerTask", 200, (void*)NULL, 2, NULL);
	if (status != pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}
}

void setupAccelerometerPins()
{
	//Initialize Accelerometer Pins
	gpio_pin_config_t pin_config = {
			.pinDirection = kGPIO_DigitalOutput,
			.outputLogic = 0U};
		GPIO_PinInit(GPIOB, 8, &pin_config);
		GPIO_PinWrite(GPIOB, 8, 1U);
}

void voltageRegulator_enable()
{
	//Enable voltage Regulator
	gpio_pin_config_t pin_config = {
			.pinDirection = kGPIO_DigitalOutput,
			.outputLogic = 0U};
		GPIO_PinInit(GPIOB, 8, &pin_config);
		GPIO_PinWrite(GPIOB, 8, 1U);
}

void accelerometer_enable()
{
	//Enable accelerometer
}

void spi_init()
{
    //Initialize SPI
	dspi_master_config_t masterConfig;
	/* Master config */
	masterConfig.whichCtar = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate = 500000;
	masterConfig.ctarConfig.bitsPerFrame = 8U;
	masterConfig.ctarConfig.cpol =
	kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / 500000;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / 500000;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / 500000;
	masterConfig.whichPcs = kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
	masterConfig.enableContinuousSCK = false;
	masterConfig.enableRxFifoOverWrite = false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint = kDSPI_SckToSin0Clock;
	DSPI_MasterInit(SPI1, &masterConfig, BUS_CLK);
}

status_t SPI_Send(uint8_t regAddress, uint8_t value)
{
	dspi_transfer_t masterXfer;
	uint8_t *masterTxData = (uint8_t*)malloc(3);

	masterTxData[0] = regAddress | 0x80; //set the most significant bit
	masterTxData[1] = regAddress & 0x80; //Clear the least significant
	masterTxData[2] = value; //set value

	masterXfer.txData = masterTxData;
	masterXfer.rxData = NULL;
	masterXfer.dataSize = 3;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 |
			kDSPI_MasterPcsContinuous;

	status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer);

	free(masterTxData);

	return ret;

}

status_t SPI_receive(uint8_t regAddress, uint8_t *rxBuff, uint8_t rxBuffSize)
{
	dspi_transfer_t masterXfer;
		uint8_t *masterTxData = (uint8_t*)malloc(rxBuffSize + 2);
		uint8_t *masterRxData = (uint8_t*)malloc(rxBuffSize + 2);

		masterTxData[0] = regAddress & 0x7F; //Clear the most significant bit
		masterTxData[1] = regAddress & 0x80; //Clear the least significant 7 bits

		masterXfer.txData = masterTxData;
		masterXfer.rxData = masterRxData;
		masterXfer.dataSize = rxBuffSize + 2;
		masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 |
		kDSPI_MasterPcsContinuous;

		status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer);
		memcpy(rxBuff, &masterRxData[2], rxBuffSize);

		free(masterTxData);
		free(masterRxData);

		return ret;
}

void accelerometerTask(void* pvParameters)
{
	//Accelerometer task implementation
}
