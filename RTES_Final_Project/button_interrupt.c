/*******************************************************************************
 * Copyright (C) 2023 by Jithendra and Suhas
 *
 * Redistribution, modification, or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. Users are
 * permitted to modify this and use it to learn about the field of embedded
 * software. Jithendra, Suhas and the University of Colorado are not liable for
 * any misuse of this material.
 * ****************************************************************************/
/**
 * @file button_interrupt.c
 * @brief Button initialization
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
#define TARGET_IS_TM4C123_RA1
#include "button_interrupt.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

/**
 * @func    button_init
 * @brief   Initializes the GPIO pins and interrupts for button functionality
 * @param   None
 * @return  None
 * @reference   TM4C123GH6PM examples
 */
void button_init()
{
    // Enable the GPIO Port F peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Configure pins 1, 2, and 3 of Port F as GPIO outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Configure pin 4 of Port F as a GPIO input
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Configure pin 4 of Port F with a weak pull-up resistor
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Enable interrupts for pin 4 of Port F
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Configure pin 4 of Port F to trigger interrupts on falling edge
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);

    // Enable the GPIO Port F interrupt
    ROM_IntEnable(INT_GPIOF);

    // Enable the global interrupt flag
    ROM_IntMasterEnable();
}

