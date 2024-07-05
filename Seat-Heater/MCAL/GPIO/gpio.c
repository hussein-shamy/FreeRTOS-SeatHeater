/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.c
 *
 * Description: Source file for the TM4C123GH6PM DIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: Edges for Training Team
 *
 ***********************************************************************************************/
#include "gpio.h"
#include "tm4c123gh6pm_registers.h"
#include "APP/App.h"

void (*PORTF_Call_Back_Func[2])(void) =
{   NULL_PTR,NULL_PTR };

void GPIO_BuiltinButtonsLedsInit(void)
{
    /*
     * PF0 --> SW2
     * PF1 --> Red LED
     * PF2 --> Blue LED
     * PF3 --> Green LED
     * PF4 --> SW1
     */

    /* Enable clock for PORTF and wait for clock to start */
    SYSCTL_RCGCGPIO_REG |= 0x20;
    while (!(SYSCTL_PRGPIO_REG & 0x20));

    /* Enable clock for PORTD and wait for clock to start */
    SYSCTL_RCGCGPIO_REG |= 0x08;
    while (!(SYSCTL_PRGPIO_REG & 0x08));

    GPIO_PORTF_LOCK_REG = 0x4C4F434B; /* Unlock the GPIO_PORTF_CR_REG */
    GPIO_PORTF_CR_REG |= (1 << 0); /* Enable changes on PF0 */
    GPIO_PORTF_AMSEL_REG &= 0xE0; /* Disable Analog on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_PCTL_REG &= 0xFFF00000; /* Clear PMCx bits for PF0, PF1, PF2, PF3 and PF4 to use it as GPIO pins */
    GPIO_PORTF_DIR_REG &= ~(1 << 0) & ~(1 << 4); /* Configure PF0 & PF4 as input pins */
    GPIO_PORTF_DIR_REG |= ((1 << 1) | (1 << 2) | (1 << 3)); /* Configure PF1, PF2 & PF3 as output pins */
    GPIO_PORTF_AFSEL_REG &= 0xE0; /* Disable alternative function on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_PUR_REG |= ((1 << 0) | (1 << 4)); /* Enable pull-up on PF0 & PF4 */
    GPIO_PORTF_DEN_REG |= 0x1F; /* Enable Digital I/O on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_DATA_REG &= ~(1 << 1) & ~(1 << 2) & ~(1 << 3); /* Clear bits 1, 2 & 3 in Data register to turn off the LEDs */

    GPIO_PORTD_PCTL_REG &= 0xFFFF0000; /* Clear PMCx bits for PF0, PF1, PF2, PF3 and PF4 to use it as GPIO pins */
    GPIO_PORTD_AMSEL_REG &= 0x0F; /* Disable Analog on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTD_DIR_REG |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) ; /* Configure PD0,1,2,3 as output pins */
    GPIO_PORTD_AFSEL_REG &= 0x0F; /* Disable alternative function on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTD_DEN_REG |= 0x0F; /* Enable Digital I/O on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTD_DATA_REG &= ~(1 << 0) & ~(1 << 1) & ~(1 << 2) & ~(1 << 3); /* Clear bits 0, 1, 2 & 3 in Data register to turn off the LEDs */

}

void GPIO_RedLedOn(uint8 seat)
{
    switch(seat){
    case 0:
        GPIO_PORTF_DATA_REG |= (1 << 1); /* Red LED ON */
        break;
    case 1:
        GPIO_PORTD_DATA_REG |= (1<<3);
        break;
    }
}

void GPIO_BlueLedOn(uint8 seat)
{
    switch(seat){
    case 0:
        GPIO_PORTF_DATA_REG |= (1 << 2); /* Blue LED ON */
        break;
    case 1:
        GPIO_PORTD_DATA_REG |= (1<<2);
        break;
    }
}

void GPIO_GreenLedOn(uint8 seat)
{
    switch(seat){
    case 0:
    GPIO_PORTF_DATA_REG |= (1 << 3); /* Green LED ON */
        break;
    case 1:
        GPIO_PORTD_DATA_REG |= (1<<1);
        break;
    }
}

void GPIO_YellowLedOn(void)
{
        GPIO_PORTD_DATA_REG |= (1<<0);
}


void GPIO_RedLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1 << 1); /* Red LED OFF */
}

void GPIO_BlueLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1 << 2); /* Blue LED OFF */
}

void GPIO_GreenLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1 << 3); /* Green LED OFF */
}

void GPIO_AllLedOff(uint8 seat)
{
    switch(seat){
    case 0:
    GPIO_PORTF_DATA_REG &= 0xFFF1; /* All LEDs OFF */
        break;
    case 1:
    GPIO_PORTD_DATA_REG &= 0xFFF0; /* All LEDs OFF */
        break;
    }
}

void GPIO_RedLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1 << 1); /* Red LED is toggled */
}

void GPIO_BlueLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1 << 2); /* Blue LED is toggled */
}

void GPIO_GreenLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1 << 3); /* Green LED is toggled */
}

uint8 GPIO_SW1GetState(void)
{
    return ((GPIO_PORTF_DATA_REG >> 4) & 0x01);
}

uint8 GPIO_SW2GetState(void)
{
    return ((GPIO_PORTF_DATA_REG >> 0) & 0x01);
}

/* GPIO PORTA External Interrupt - ISR */
void GPIOPortA_Handler(void)
{
    /*Do nothing*/
    GPIO_PORTA_ICR_REG |= (1 << 0); /* Clear Trigger flag for PF0 (Interrupt Flag) */
}

void GPIOPortF_Handler(void)
{
    if ((GPIO_PORTF_RIS_REG & 0x01) >> 0)
    {
        GPIO_PORTF_ICR_REG |= (1 << 0); /* Clear Trigger flag for PF0 (Interrupt Flag) */
        PORTF_Call_Back_Func[0]();
    }
    else if ((GPIO_PORTF_RIS_REG & 0x10) >> 4)
    {
        GPIO_PORTF_ICR_REG |= (1 << 4); /* Clear Trigger flag for PF4 (Interrupt Flag) */
        PORTF_Call_Back_Func[1]();
    }
}

void GPIO_SW1EdgeTriggeredInterruptInit(void)
{
    GPIO_PORTF_IS_REG &= ~(1 << 4); /* PF4 detect edges */
    GPIO_PORTF_IBE_REG &= ~(1 << 4); /* PF4 will detect a certain edge */
    GPIO_PORTF_IEV_REG &= ~(1 << 4); /* PF4 will detect a falling edge */
    GPIO_PORTF_ICR_REG |= (1 << 4); /* Clear Trigger flag for PF4 (Interrupt Flag) */
    GPIO_PORTF_IM_REG |= (1 << 4); /* Enable Interrupt on PF4 pin */
    /* Set GPIO PORTF priority as 5 by set Bit number 21, 22 and 23 with value 2 */
    NVIC_PRI7_REG = (NVIC_PRI7_REG & GPIO_PORTF_PRIORITY_MASK)
                    | (GPIO_PORTF_INTERRUPT_PRIORITY << GPIO_PORTF_PRIORITY_BITS_POS);
    NVIC_EN0_REG |= 0x40000000; /* Enable NVIC Interrupt for GPIO PORTF by set bit number 30 in EN0 Register */
}

void GPIO_SW2EdgeTriggeredInterruptInit(void)
{
    GPIO_PORTF_IS_REG &= ~(1 << 0); /* PF0 detect edges */
    GPIO_PORTF_IBE_REG &= ~(1 << 0); /* PF0 will detect a certain edge */
    GPIO_PORTF_IEV_REG &= ~(1 << 0); /* PF0 will detect a falling edge */
    GPIO_PORTF_ICR_REG |= (1 << 0); /* Clear Trigger flag for PF0 (Interrupt Flag) */
    GPIO_PORTF_IM_REG |= (1 << 0); /* Enable Interrupt on PF0 pin */
    /* Set GPIO PORTF priority as 5 by set Bit number 21, 22 and 23 with value 2 */
    NVIC_PRI7_REG = (NVIC_PRI7_REG & GPIO_PORTF_PRIORITY_MASK)
                    | (GPIO_PORTF_INTERRUPT_PRIORITY << GPIO_PORTF_PRIORITY_BITS_POS);
    NVIC_EN0_REG |= 0x40000000; /* Enable NVIC Interrupt for GPIO PORTF by set bit number 30 in EN0 Register */
}

void GPIO_SW3EdgeTriggeredInterruptInit(void)
{

    /* Enable clock for PORTA and wait for clock to start */
    SYSCTL_RCGCGPIO_REG |= 0x01;
    while (!(SYSCTL_PRGPIO_REG & 0x01))
        ;

    /******************* Configure PA7 as input PIN ***********************************************/
    GPIO_PORTA_LOCK_REG = 0x4C4F434B; /* Unlock the GPIO_PORTA_CR_REG */
    GPIO_PORTA_CR_REG |= (1 << 7); /* Enable changes on PA7 */
    GPIO_PORTA_AMSEL_REG &= 0x7F; /* Disable Analog on PA7 */
    GPIO_PORTA_PCTL_REG &= 0x0FFFFFFF; /* Clear PMCx bits for PF0, PF1, PF2, PF3 and PF4 to use it as GPIO pins */
    GPIO_PORTA_DIR_REG &= ~(1 << 7); /* Configure PA7 input pins */
    GPIO_PORTA_AFSEL_REG &= 0x7F; /* Disable alternative function on PA7 */
    GPIO_PORTA_PUR_REG |= (1 << 7); /* Enable pull-up on PA7 */
    GPIO_PORTA_DEN_REG |= (1 << 7); /* Enable Digital I/O on PA7 */

    /******************* Interrupt Enable on PA7 ***********************************************/
    GPIO_PORTA_IS_REG &= ~(1 << 7); /* PA7 detect edges */
    GPIO_PORTA_IBE_REG &= ~(1 << 7); /* PA7 will detect a certain edge */
    GPIO_PORTA_IEV_REG &= ~(1 << 7); /* PA7 will detect a falling edge */
    GPIO_PORTA_ICR_REG |= (1 << 7); /* Clear Trigger flag for PA7 (Interrupt Flag) */
    GPIO_PORTA_IM_REG |= (1 << 7); /* Enable Interrupt on PA7 pin */
    /* Set GPIO PORTA priority as 5 by set Bit number 21, 22 and 23 with value 2 */
    NVIC_PRI0_REG = (NVIC_PRI0_REG & 0xFFFFFF1F)
                    | (GPIO_PORTF_INTERRUPT_PRIORITY << 5);
    NVIC_EN0_REG |= 0x00000001; /* Enable NVIC Interrupt for GPIO PORTA by set bit number 0 in EN0 Register */
}
