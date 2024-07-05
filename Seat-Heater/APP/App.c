#include "FreeRTOS.h"
#include "GPIO/gpio.h"
#include "ADC/adc.h"
#include "uart0.h"
#include "App.h"
#include "task.h"
#include "event_groups.h"
#include "MCAL/GPTM/GPTM.h"

volatile uint8 g_Button_States      [NO_OF_SEATES] = { HEATER_OFF , HEATER_OFF };
sint16         g_Seats_Temp         [NO_OF_SEATES] = { 0 , 0 };
uint8          g_Heater_intensity   [NO_OF_SEATES] = { HEATER_OFF , HEATER_OFF };

uint32 ullTasksOutTime[9];
uint32 ullTasksInTime[9];
uint32 ullTasksTotalTime[9];
uint32 ullTasksExecutionTime[9];

EventGroupHandle_t xEventGroup;

TaskHandle_t xTask0Handle;
TaskHandle_t xTask1Handle;
TaskHandle_t xTask2Handle;
TaskHandle_t xTask3Handle;
TaskHandle_t xTask4Handle;
TaskHandle_t xTask5Handle;
TaskHandle_t xTask6Handle;
TaskHandle_t xTask7Handle;

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}

void vRunTimeMeasurementsTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        uint8 ucCounter, ucCPU_Load;
        uint32 ullTotalTasksTime = 0;
        vTaskDelayUntil(&xLastWakeTime, RUNTIME_MEASUREMENTS_TASK_PERIODICITY);
        for(ucCounter = 1; ucCounter < 9; ucCounter++)
        {
            ullTotalTasksTime += ullTasksTotalTime[ucCounter];
        }
        ucCPU_Load = (ullTotalTasksTime * 100) /  GPTM_WTimer0Read();

        taskENTER_CRITICAL();

        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("============================= [Run Time Measurements] =======================================\r\n");
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("# Run time measurement                       | ");
        UART0_SendInteger(ullTasksExecutionTime[1] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Read Temperature for Driver's Seat         | ");
        UART0_SendInteger(ullTasksExecutionTime[2] / 10 );
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Read Temperature for Passenger's Seat      | ");
        UART0_SendInteger(ullTasksExecutionTime[3] / 10 );
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Set Heating Intensity for Driver's Seat    | ");
        UART0_SendInteger(ullTasksExecutionTime[4] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Set Heating Intensity for Passenger's Seat | ");
        UART0_SendInteger(ullTasksExecutionTime[5] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Control Heating for Driver's Seat          | ");
        UART0_SendInteger(ullTasksExecutionTime[6] / 10 );
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Control Heating for Passenger's Seat       | ");
        UART0_SendInteger(ullTasksExecutionTime[7] / 10 );
        UART0_SendString(" msec \r\n");

        UART0_SendString("# Display Temperature Data on LCD Screen     | ");
        UART0_SendInteger(ullTasksExecutionTime[8] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("# CPU Load                                   | ");
        UART0_SendInteger(ucCPU_Load);
        UART0_SendString("% \r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");

        taskEXIT_CRITICAL();
    }
}

void vPeriodic_Task_ReadTemp_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32 adc_value = 0;
    uint8 previous_temperature[NO_OF_SEATES] = {0,0};
    uint8 APP_TEMP_CHANGED_BIT = AppTemp_Driver_Changed_BIT;
    uint8 seat_type = (uint8)pvParameters;

    for (;;)
    {

        switch (seat_type)
        {
        case Driver_Seat:
            adc_value = ADC_read_PE0();
            APP_TEMP_CHANGED_BIT = AppTemp_Driver_Changed_BIT;
            break;
        case Passenger_Seat:
            adc_value = ADC_read_PE1();
            APP_TEMP_CHANGED_BIT = AppTemp_Passenger_Changed_BIT;
            break;
        }

        g_Seats_Temp[seat_type] = (uint8) (((uint32)adc_value * 45)/ 4095);

        if(previous_temperature[seat_type] < g_Seats_Temp[seat_type] -3 || previous_temperature[seat_type] > g_Seats_Temp[seat_type] +3)
            xEventGroupSetBits(xEventGroup, APP_TEMP_CHANGED_BIT);

        previous_temperature[seat_type] = g_Seats_Temp[seat_type];

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5000 ) );
    }
}


void vPeriodic_Task_SetIntensity_Seat(void *pvParameters)
{
    EventBits_t xEventGroupValue;
    SemaphoreHandle_t Semaphore = xBinarySemaphore_Heater_Intensity_Setted_Driver_Seat;
    const EventBits_t xBitsToWaitFor = (AppButton_Driver_Pressed_BIT | AppTemp_Driver_Changed_BIT);
    const EventBits_t xBitsToWaitFor2 = (AppButton_Passenger_Pressed_BIT | AppTemp_Passenger_Changed_BIT);
    uint8 seat_type = (uint8) pvParameters;
    uint8 current_temp = g_Seats_Temp[seat_type];
    uint8 desiresd_temp = g_Button_States[seat_type];
    uint8 current_intensity = g_Heater_intensity[seat_type];
    uint8 flag = pdFALSE;

    for (;;)
    {

        switch(seat_type){

        case Driver_Seat:
            xEventGroupValue = xEventGroupWaitBits( xEventGroup, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);
            if (((xEventGroupValue & AppTemp_Driver_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Driver_Pressed_BIT) != 0))){
                flag = pdTRUE;
                Semaphore = xBinarySemaphore_Heater_Intensity_Setted_Driver_Seat;
            }
            break;

        case Passenger_Seat:
            xEventGroupValue = xEventGroupWaitBits( xEventGroup, xBitsToWaitFor2, pdTRUE, pdFALSE, portMAX_DELAY);
            if (((xEventGroupValue & AppTemp_Passenger_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Passenger_Pressed_BIT) != 0))){
                flag = pdTRUE;
                Semaphore = xBinarySemaphore_Heater_Intensity_Setted_Passernger_Seat;
            }
            break;
        }
        current_temp = g_Seats_Temp[seat_type];
        desiresd_temp = g_Button_States[seat_type];
        current_intensity = g_Heater_intensity[seat_type];

        if(flag == pdTRUE){
            if(current_temp < 5 || current_temp >40){
                g_Heater_intensity[seat_type] = TEMP_OUT_OF_RANGE_ERROR;
            }
            else{
                if (current_intensity != HEATER_OFF)
                {
                    if (current_temp < desiresd_temp - 10)
                        g_Heater_intensity[seat_type] = HIGH_INTENSITY;
                    else if (current_temp >= desiresd_temp - 10 && current_temp < desiresd_temp - 5)
                        g_Heater_intensity[seat_type] = MED_INTENSITY;
                    else if (current_temp >= desiresd_temp - 5  && current_temp < desiresd_temp - 2)
                        g_Heater_intensity[seat_type] = LOW_INTENSITY;
                    else if (current_temp > desiresd_temp)
                        g_Heater_intensity[seat_type] = HEATER_OFF;
                }
                else{
                    if (current_temp < desiresd_temp - 3)
                        g_Heater_intensity[seat_type] = LOW_INTENSITY;
                }
            }
            xSemaphoreGive(Semaphore);
            flag = pdFALSE;
        }
    }
}


void vPeriodic_Task_ControlHeating_Seat(void *pvParameters)
{
    uint8 current_intensity = 0;
    uint8 seat_type = (uint8) pvParameters;
    for (;;)
    {

        switch(seat_type){
        case Driver_Seat:
            xSemaphoreTake(xBinarySemaphore_Heater_Intensity_Setted_Driver_Seat,portMAX_DELAY); break;
        case Passenger_Seat:
            xSemaphoreTake(xBinarySemaphore_Heater_Intensity_Setted_Passernger_Seat,portMAX_DELAY); break;
        }

        current_intensity =  g_Heater_intensity[seat_type];

        switch (current_intensity)
        {
        case TEMP_OUT_OF_RANGE_ERROR:
            GPIO_AllLedOff(seat_type);
            GPIO_RedLedOn(seat_type);
            break;

        case HEATER_OFF:
            GPIO_AllLedOff(seat_type);
            break;

        case LOW_INTENSITY:
            GPIO_AllLedOff(seat_type);
            GPIO_GreenLedOn(seat_type);
            break;

        case MED_INTENSITY:
            GPIO_AllLedOff(seat_type);
            GPIO_BlueLedOn(seat_type);
            break;

        case HIGH_INTENSITY:
            GPIO_AllLedOff(seat_type);
            if(seat_type==Driver_Seat){
            GPIO_BlueLedOn(0);
            GPIO_GreenLedOn(0);
            }else{
            GPIO_YellowLedOn();
            }
            break;
        }
    }
}


void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {

        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("===================================== [DASHBOARD] ===========================================\r\n");
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");
        UART0_SendString("               Driver Seat               |             Passenger Seat                        \r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");

        UART0_SendString("Current Temperature:                     \r\n");
        UART0_SendString("                      ");

        if(g_Seats_Temp[Driver_Seat] < 5 || g_Seats_Temp[Driver_Seat] >40){
        UART0_SendString("ERROR");
        }else{
        UART0_SendInteger(g_Seats_Temp[Driver_Seat]);
        }
        UART0_SendString("                                     ");
        if(g_Seats_Temp[Passenger_Seat] < 5 || g_Seats_Temp[Passenger_Seat] >40){
        UART0_SendString("ERROR");
        }else{
        UART0_SendInteger(g_Seats_Temp[Passenger_Seat]);
        }
        UART0_SendString("\r\n");

        UART0_SendString("Desired Temperature:                      \r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(g_Button_States[Driver_Seat]);
        UART0_SendString("                                    ");
        UART0_SendInteger(g_Button_States[Passenger_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Heater Intensity Temperature:             \r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(g_Heater_intensity[Driver_Seat]);
        UART0_SendString("                                     ");
        UART0_SendInteger(g_Heater_intensity[Passenger_Seat]);
        UART0_SendString("\r\n");

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5000 ) );

    }
}


void DriverButtonPressed(void){

    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    switch(g_Button_States[Driver_Seat]){
    case HEATER_OFF:
        g_Button_States[Driver_Seat] = HEATER_LOW;
        break;
    case HEATER_LOW:
        g_Button_States[Driver_Seat] = HEATER_MEDIUM;
        break;
    case HEATER_MEDIUM:
        g_Button_States[Driver_Seat] = HEATER_HIGH;
        break;
    case HEATER_HIGH:
        g_Button_States[Driver_Seat] = HEATER_OFF;
        break;
    }

    xEventGroupSetBitsFromISR(xEventGroup, AppButton_Driver_Pressed_BIT,&pxHigherPriorityTaskWoken);
}

void PassengerButtonPressed(void){

    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    switch(g_Button_States[Passenger_Seat]){
    case HEATER_OFF:
        g_Button_States[Passenger_Seat] = HEATER_LOW;
        break;
    case HEATER_LOW:
        g_Button_States[Passenger_Seat] = HEATER_MEDIUM;
        break;
    case HEATER_MEDIUM:
        g_Button_States[Passenger_Seat] = HEATER_HIGH;
        break;
    case HEATER_HIGH:
        g_Button_States[Passenger_Seat] = HEATER_OFF;
        break;
    }

    xEventGroupSetBitsFromISR(xEventGroup, AppButton_Passenger_Pressed_BIT,&pxHigherPriorityTaskWoken);
}
