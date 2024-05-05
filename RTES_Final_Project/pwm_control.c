#define TARGET_IS_TM4C123_RA1
#include "pwm_control.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

void heating_pwm_init()
{
    // Set the PWM clock to run at the system clock divided by 1
    SysCtlPWMClockSet(SYSCTL_SYSDIV_1);

    // Enable PWM0 peripheral and GPIO port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure pin B6 as PWM output
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    // Configure PWM generator 0 in up-down count mode with no synchronization
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the period of PWM generator 0 to 200000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 200000);

    // Set the initial pulse width to 10% of the period
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);

    // Enable PWM output for PWM0 output 0
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    // Enable PWM generator 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}


void seat_pwm_init()
{
    // Set the PWM clock to run at the system clock divided by 1
    SysCtlPWMClockSet(SYSCTL_SYSDIV_1);

    // Enable PWM0 peripheral and GPIO port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure pin B7 as PWM output
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

    // Configure PWM generator 0 in up-down count mode with no synchronization
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the period of PWM generator 0 to 200000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 200000);

    // Enable PWM output for PWM0 output 1
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    // Enable PWM generator 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    // Enable GPIO port C and configure pins C6 and C7 as GPIO outputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Set pin C6 high and pin C7 low
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}


void servo_init(){
    // Set the PWM clock to the system clock/8.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN |
    PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 40000);
}

void servo_write(float deg){
    float duty = ((deg / 90) + 0.4);
    float ticks = duty * 2;
    float divf = (40 / ticks);
    int divfact = (int) divf;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,
                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_1) / divfact);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

    SysCtlDelay((SysCtlClockGet() * 0.3) / 3);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
    PWMGenDisable(PWM1_BASE, PWM_GEN_1);
}
