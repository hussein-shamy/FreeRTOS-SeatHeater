/******************************************************************************
 *
 * Module: ADC
 *
 * File Name: adc.c
 *
 * Description: Source file for the TivaC ADC driver
 *
 * Author: Hussein El-Shamy
 *
 *******************************************************************************/

#include "adc.h"
#include "common_macros.h"
#include "tm4c123gh6pm_registers.h"

/*
 * Description :
 * Function initializes GPIO pins PE0 and PE1 for ADC functionality
 */
void ADC_PE0_PE1_init(void)
{
    /* Enable the ADC clock */
    SET_BIT(SYSCTL_RCGCADC_REG, 0);

    /* Enable clock to GPIO Port E */
    SET_BIT(SYSCTL_RCGCGPIO_REG, 4);
    while (!(SYSCTL_PRGPIO_REG & 0x10));

    /* Configure GPIO PE0 and PE1 as analog inputs */
    SET_BIT(GPIO_PORTE_AFSEL_REG, 0);
    SET_BIT(GPIO_PORTE_AFSEL_REG, 1);
    CLEAR_BIT(GPIO_PORTE_DEN_REG, 0);
    CLEAR_BIT(GPIO_PORTE_DEN_REG, 1);
    SET_BIT(GPIO_PORTE_AMSEL_REG, 0);
    SET_BIT(GPIO_PORTE_AMSEL_REG, 1);

    /* Initialize Sample Sequencer 3 */
    CLEAR_BIT(ADC0_ACTSS_REG, ASEN3);       /* Disable SS3 during configuration */
    ADC0_SSMUX3_REG = PE0_AIN_3;            /* Set PE0 (AIN3) as input */
    SET_BIT(ADC0_SSCTL3_REG,END0);          /* Enable End of Sequence */
    SET_BIT(ADC0_SSCTL3_REG,IE0);           /* Enable Sample Interrupt */
    SET_BIT(ADC0_ACTSS_REG, ASEN3);         /* Enable SS3 after configuration */
}

/*
 * Description :
 * Function reads analog data from ADC channel connected to PE0
 * and converts it to digital using the ADC driver.
 */
uint16 ADC_read_PE0(void)
{
    uint16 adc_value = 0;

    /* Setting PE0 as an input channel */
    CLEAR_BIT(ADC0_ACTSS_REG, ASEN3);           /* Disable SS3 during configuration */
    ADC0_SSCTL3_REG &= ~(1<<3);                   /* Disable Temp Sensor */
    ADC0_SSMUX3_REG = PE0_AIN_3;                /* Set PE0 (AIN3) as input */
    SET_BIT(ADC0_ACTSS_REG, ASEN3);             /* Enable SS3 after configuration */
    SET_BIT(ADC0_PSSI_REG, SS3);                /* Initiate sampling */
    while ((ADC0_RIS_REG & (1 << INR3)) == 0);  /* Wait until sample conversion is completed */

    adc_value = ADC0_SSFIFO3_REG;               /* Read converted value */
    SET_BIT(ADC0_ISC_REG,IN3);                  /* Clear conversion flag */

    return adc_value;
}

/*
 * Description :
 * Function reads analog data from ADC channel connected to PE1
 * and converts it to digital using the ADC driver.
 */
uint16 ADC_read_PE1(void)
{
    uint16 adc_value = 0;

    /* Setting PE1 as an input channel */
    CLEAR_BIT(ADC0_ACTSS_REG, ASEN3); 			/* Disable SS3 during configuration */
   // ADC0_SSCTL3_REG |= (1<<3);                   /* Set Temp Sensor */
    ADC0_SSMUX3_REG = PE1_AIN_2; 				/* Set PE1 (AIN2) as input */
    SET_BIT(ADC0_ACTSS_REG, ASEN3); 			/* Enable SS3 after configuration */

    SET_BIT(ADC0_PSSI_REG, SS3); 				/* Initiate sampling */
    while ((ADC0_RIS_REG & (1 << INR3)) == 0);  /* Wait until sample conversion is completed */

    adc_value = ADC0_SSFIFO3_REG;				/* Read converted value */
    ADC0_ISC_REG |= (1 << IN3); 				/* Clear conversion flag */

    return adc_value;
}
