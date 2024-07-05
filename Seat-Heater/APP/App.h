/*
 * App.h
 *
 *  Created on: Apr 26, 2024
 *      Author: Hussein El-Shamy
 */

#ifndef APP_H_
#define APP_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "GPIO/gpio.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

#define TEMP_OUT_OF_RANGE_ERROR ((uint8)44)
#define LOW_INTENSITY           ((uint8)1)
#define MED_INTENSITY           ((uint8)2)
#define HIGH_INTENSITY          ((uint8)3)

#define NO_OF_SEATES            ((uint8)2)
#define Driver_Seat             ((uint8)0)
#define Passenger_Seat          ((uint8)1)

#define HEATER_OFF              ((uint8)0)
#define HEATER_LOW              ((uint8)25)
#define HEATER_MEDIUM           ((uint8)30)
#define HEATER_HIGH             ((uint8)35)


/* Definitions for the event bits in the event group. */
#define AppButton_Driver_Pressed_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a task. */
#define AppButton_Passenger_Pressed_BIT ( 1UL << 1UL ) /* Event bit 1, which is set by a task. */

#define AppIntensity_Driver_Selected_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a task. */
#define AppIntensity_Passenger_Selected_BIT ( 1UL << 1UL )  /* Event bit 1, which is set by a task. */
#define AppTemp_Driver_Changed_BIT ( 1UL << 2UL )
#define AppTemp_Passenger_Changed_BIT ( 1UL << 3UL )
#define RUNTIME_MEASUREMENTS_TASK_PERIODICITY (1000U)


xSemaphoreHandle xBinarySemaphore_Heater_Intensity_Setted_Driver_Seat;
xSemaphoreHandle xBinarySemaphore_Heater_Intensity_Setted_Passernger_Seat;

/* Used to hold the handle of tasks */
extern TaskHandle_t xTask0Handle;
extern TaskHandle_t xTask1Handle;
extern TaskHandle_t xTask2Handle;
extern TaskHandle_t xTask3Handle;
extern TaskHandle_t xTask4Handle;
extern TaskHandle_t xTask5Handle;
extern TaskHandle_t xTask6Handle;
extern TaskHandle_t xTask7Handle;

extern volatile uint8 g_Button_States[NO_OF_SEATES];
extern sint16 g_Seats_Temp[NO_OF_SEATES];
extern uint8 g_Heater_intensity[NO_OF_SEATES];
extern EventGroupHandle_t xEventGroup;

extern uint32 ullTasksOutTime[9];
extern uint32 ullTasksInTime[9];
extern uint32 ullTasksTotalTime[9];
extern uint32 ullTasksExecutionTime[9];

/* FreeRTOS tasks */
void vPeriodic_Task_ReadTemp_Seat(void *pvParameters);
void vPeriodic_Task_SetIntensity_Seat(void *pvParameters);
void vPeriodic_Task_ControlHeating_Seat(void *pvParameters);
void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters);
void vRunTimeMeasurementsTask(void *pvParameters);
void DriverButtonPressed(void);
void PassengerButtonPressed(void);


#endif /* APP_H_ */
