/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.h
 *
 * Description: Header file for the TM4C123GH6PM DIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: Edges for Training Team
 *
 ***********************************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include "std_types.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define GPIO_PORTF_PRIORITY_MASK      0xFF1FFFFF
#define GPIO_PORTF_PRIORITY_BITS_POS  21
#define GPIO_PORTF_INTERRUPT_PRIORITY 5

#define PRESSED                 ((uint8)0x00)
#define RELEASED                ((uint8)0x01)

#define SW1_PF4                 ((uint8)0)
#define SW2_PF0                 ((uint8)1)

/* Enable Exceptions ... This Macro enable IRQ interrupts, Programmable Systems Exceptions and Faults by clearing the I-bit in the PRIMASK. */
#define Enable_Exceptions()    __asm(" CPSIE I ")

/* Disable Exceptions ... This Macro disable IRQ interrupts, Programmable Systems Exceptions and Faults by setting the I-bit in the PRIMASK. */
#define Disable_Exceptions()   __asm(" CPSID I ")

/* Enable Faults ... This Macro enable Faults by clearing the F-bit in the FAULTMASK */
#define Enable_Faults()        __asm(" CPSIE F ")

/* Disable Faults ... This Macro disable Faults by setting the F-bit in the FAULTMASK */
#define Disable_Faults()       __asm(" CPSID F ")

/* Go to low power mode while waiting for the next interrupt */
#define Wait_For_Interrupt()   __asm(" WFI ")

void GPIOPortA_Handler(void);
void GPIOPortF_Handler(void);

void GPIO_BuiltinButtonsLedsInit(void);
void GPIO_RedLedOn(uint8 seat);
void GPIO_BlueLedOn(uint8 seat);
void GPIO_GreenLedOn(uint8 seat);
void GPIO_YellowLedOn(void);

void GPIO_RedLedOff(void);
void GPIO_BlueLedOff(void);
void GPIO_GreenLedOff(void);
void GPIO_AllLedOff(uint8 seat);

void GPIO_RedLedToggle(void);
void GPIO_BlueLedToggle(void);
void GPIO_GreenLedToggle(void);

uint8 GPIO_SW1GetState(void);
uint8 GPIO_SW2GetState(void);

void GPIO_SW1EdgeTriggeredInterruptInit(void);
void GPIO_SW2EdgeTriggeredInterruptInit(void);
void GPIO_SW3EdgeTriggeredInterruptInit(void);

extern void (*PORTF_Call_Back_Func[2])(void);
#endif /* GPIO_H_ */
