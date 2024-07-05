 /******************************************************************************
 *
 * Module: ADC
 *
 * File Name: adc.h
 *
 * Description: header file for the TivaC ADC driver
 *
 * Author: Hussein El-Shamy
 *
 *******************************************************************************/

#ifndef MCAL_ADC_ADC_H_
#define MCAL_ADC_ADC_H_

#include "std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

#define ASEN3               3
#define ALWAYS_SAMPLE       0xF000
#define PE0_AIN_3 			3
#define PE1_AIN_2 			2
#define END0      			1
#define SS3      			3
#define INR3                3
#define IN3                 3
#define IE0                 2

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Description :
 * Function initializes GPIO pins PE0 and PE1 for ADC functionality
 */
void ADC_PE0_PE1_init(void);

/*
 * Description :
 * Function reads analog data from ADC channel connected to PE0
 * and converts it to digital using the ADC driver.
 */
uint16 ADC_read_PE0(void);

/*
 * Description :
 * Function reads analog data from ADC channel connected to PE1
 * and converts it to digital using the ADC driver.
 */
uint16 ADC_read_PE1(void);

#endif /* MCAL_ADC_ADC_H_ */
