/*
 * OS_Init.c
 *
 *  Created on: Apr 26, 2024
 *      Author: Hussein El-Shamy
 */

#include "APP/App.h"
#include "MCAL/GPIO/gpio.h"
#include "MCAL/ADC/adc.h"
#include "MCAL/UART/uart0.h"
#include "MCAL/GPTM/GPTM.h"

static void GPIO_PORTF_Set_Call_Back( void (*CallBackDriverButton)(void),void (*CallBackPassengerButton) (void) ){
    PORTF_Call_Back_Func[0] = CallBackDriverButton;
    PORTF_Call_Back_Func[1] = CallBackPassengerButton;
}

static void prvSetupHardware(void)
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    /*GPIO*/
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    GPIO_SW3EdgeTriggeredInterruptInit();
    GPIO_PORTF_Set_Call_Back(DriverButtonPressed,PassengerButtonPressed);
    /*ADC*/
    ADC_PE0_PE1_init();
    /*UART*/
    UART0_Init();
    /*GPTM*/
    GPTM_WTimer0Init();
}

static void CreateTasks (void){

    /* Create two binary semaphores */
    xEventGroup = xEventGroupCreate();
    xBinarySemaphore_Heater_Intensity_Setted_Driver_Seat = xSemaphoreCreateBinary();
    xBinarySemaphore_Heater_Intensity_Setted_Passernger_Seat = xSemaphoreCreateBinary();

    xTaskCreate(vRunTimeMeasurementsTask,
                "Run time measurement Task",
                256,
                NULL,
                1,
                &xTask0Handle);

     /* Create Tasks here */
     xTaskCreate(vPeriodic_Task_ReadTemp_Seat,
                 "Read Temperature for Driver's Seat",
                 64,
                 (void*) Driver_Seat,
                 4,
                 &xTask1Handle);

     xTaskCreate(vPeriodic_Task_ReadTemp_Seat,
                 "Read Temperature for Passenger's Seat",
                 64,
                 (void*) Passenger_Seat,
                 4,
                 &xTask2Handle);

     xTaskCreate(vPeriodic_Task_SetIntensity_Seat,
                 "Set Heating Intensity for Driver's Seat",
                 64,
                 (void*) Driver_Seat,
                 3,
                 &xTask3Handle);

     xTaskCreate(vPeriodic_Task_SetIntensity_Seat,
                 "Set Heating Intensity for Passenger's Seat",
                 64,
                 (void*) Passenger_Seat,
                 3,
                 &xTask4Handle);

     xTaskCreate(vPeriodic_Task_ControlHeating_Seat,
                 "Control Heating for Driver's Seat",
                 64,
                 (void*) Driver_Seat,
                 2,
                 &xTask5Handle);

     xTaskCreate(vPeriodic_Task_ControlHeating_Seat,
                 "Control Heating for Passenger's Seat",
                 64,
                 (void*) Passenger_Seat,
                 2,
                 &xTask6Handle);

     xTaskCreate(vPeriodic_Task_DisplayTempData_LCD,
                 "Display Temperature Data on LCD Screen",
                 128,
                 NULL,
                 2,
                 &xTask7Handle);

     vTaskSetApplicationTaskTag( xTask0Handle, ( void * ) 1 );
     vTaskSetApplicationTaskTag( xTask1Handle, ( void * ) 2 );
     vTaskSetApplicationTaskTag( xTask2Handle, ( void * ) 3 );
     vTaskSetApplicationTaskTag( xTask3Handle, ( void * ) 4 );
     vTaskSetApplicationTaskTag( xTask4Handle, ( void * ) 5 );
     vTaskSetApplicationTaskTag( xTask5Handle, ( void * ) 6 );
     vTaskSetApplicationTaskTag( xTask6Handle, ( void * ) 7 );
     vTaskSetApplicationTaskTag( xTask7Handle, ( void * ) 8 );
}

void OS_Start(void){

    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    CreateTasks();

    /* Now all the tasks have been started - start the scheduler.

     NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
     The processor MUST be in supervisor mode when vTaskStartScheduler is
     called.  The demo applications included in the FreeRTOS.org download switch
     to supervisor mode prior to main being called.  If you are not using one of
     these demo application projects then ensure Supervisor mode is used here. */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
     available for the idle task to be created. */
    for (;;) ;

}
