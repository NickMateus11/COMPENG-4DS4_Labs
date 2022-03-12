#include "FreeRTOS.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include "Motor_Control_Component.h"
#include "RC_Receiver_Component.h"
#include "Terminal_Component.h"
#include "LED_Component.h"
#include "Accelerometer_Component.h"

int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootClocks();
//
//    setupMotorComponent();
//    setupRCReceiverComponent();
//    setupTerminalComponent();
    setupLEDComponent();
//    setupAccelerometerComponent();

    // TODO: this code goes into the motor thread
    int speed = -20; // some speed value
    xQueueSendToBack(led_queue, (void*) (&speed), portMAX_DELAY);

    vTaskStartScheduler();

    while(1)
    {}
}
