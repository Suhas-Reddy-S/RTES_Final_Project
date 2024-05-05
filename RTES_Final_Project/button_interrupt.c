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
