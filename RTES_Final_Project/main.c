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
 * @file main.c
 * @brief Occupant safety
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */

// Include necessary header files
#define TARGET_IS_TM4C123_RA1
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "accelerometer.h"
#include "temperature_sensor.h"
#include "button_interrupt.h"
#include "pwm_control.h"

// Define constants and variables
#define ACC_SCALING (2048)
#define SCALE (3)
#define SAM_DEADLINE (10)
#define ABD_DEADLINE (20)

#define SEAT_TIGHT_ITER (3)
#define SPEED_OFFSET (4)
#define ACC_OFFSET (5);

// Function prototypes
static void service1(void *params);
static void service2(void *params);
static void service3(void *params);
static void service4(void *params);
static void service5(void *params);
static void Sequencer_thread(void *params);

// Semaphore handles
xSemaphoreHandle semSched, semS1, semS2, semS3, semS4, semS5;

// Abort flags
volatile bool abortS1 = false, abortS2 = false, abortS3 = false, abortS4 = false;

// Timing parameters
volatile uint32_t T1 = 1 * SCALE;
volatile uint32_t T2 = 2 * SCALE;
volatile uint32_t T3 = 20 * SCALE;
volatile uint32_t T4 = 20 * SCALE;

volatile uint32_t event_start = 0;

volatile uint32_t speed = 0;

#ifdef DEBUG
void
error(char *pcFilename, uint32_t ui32Line)
{
}

#endif

// Stack overflow hook
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    // Handle stack overflow
    while (1)
    {
    }
}


/**
 * @func    ConfigureUART
 * @brief   Configures UART0 for communication
 * @param   None
 * @return  None
 * @reference   TM4C123GH6PM Example
 */
void ConfigureUART(void)
{
    // Enable the GPIO peripheral for Port A
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable the UART peripheral
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure PA0 as the UART RX pin
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);

    // Configure PA1 as the UART TX pin
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);

    // Set PA0 and PA1 as UART pins
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set the clock source for UART0 to the precision internal oscillator (PIOSC)
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Configure UART0 with standard I/O settings
    // Parameters: 0 for the UART instance, baud rate of 230400, and system clock frequency of 16,000,000
    UARTStdioConfig(0, 230400, 16000000);
}


/**
 * @func    timer0_init
 * @brief   Initializes Timer0 for periodic operation
 * @param   None
 * @return  None
 * @reference   TM4C123GH6PM Example
 */
void timer0_init(){
    // Enable Timer0 peripheral
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure Timer0A to run in periodic mode at 1000Hz
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / 1000);

    // Enable Timer0A interrupt
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable Timer0A
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}


/**
 * @func    Timer0IntHandler
 * @brief   Interrupt handler for Timer0
 * @param   None
 * @return  None
 * @reference   TM4C123GH6PM Datasheet - Timer0 Section
 */
void Timer0IntHandler(void)
{
    // Clear the interrupt flag for Timer A timeout
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Release the semaphore to signal the sequencer
    xSemaphoreGive(semSched);
}


/**
 * @func    ButtonHandler
 * @brief   Interrupt handler for button press event
 * @param   None
 * @return  None
 * @reference   TM4C123GH6PM Example
 */
void ButtonHandler(void)
{
    // Disable interrupt for GPIO pin 4 on Port F
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Write GPIO pin 1 on Port F to HIGH
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

    // Set abort flags for services 1 and 2
    abortS1 = true;
    abortS2 = true;

    // Release semaphores for services 3, 4, and 5
    xSemaphoreGive(semS3);
    xSemaphoreGive(semS4);
    xSemaphoreGive(semS5);

    // Record the event start time
    event_start = xTaskGetTickCount();
}


/**
 * @func    Sequencer_thread
 * @brief   Task for managing the sequencing of events
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void Sequencer_thread(void *params)
{
    // Static variable to keep track of scheduler count
    static volatile uint32_t schedCnt = 0;

    // Print a message indicating that the sequencer has started
    UARTprintf("Sequencer Started at %u msec\n", xTaskGetTickCount());

    // Infinite loop for sequencer operation
    while (1)
    {
        // Wait indefinitely for the scheduler semaphore
        xSemaphoreTake(semSched, portMAX_DELAY);
        // Increment the scheduler count
        schedCnt++;

        // Check if it's time to release semaphore S1
        if (schedCnt % T1 == 0 && !abortS1)
        {
            xSemaphoreGive(semS1);
        }

        // Check if it's time to release semaphore S2
        if (schedCnt % T2 == 0 && !abortS2)
        {
            xSemaphoreGive(semS2);
        }

        // Check if it's time to release semaphore S3 (abort condition)
        if (schedCnt % T1 == 0 && abortS1)
        {
            xSemaphoreGive(semS3);
        }

        // Check if it's time to release semaphore S4 (abort condition)
        if (schedCnt % T2 == 0 && abortS2)
        {
            xSemaphoreGive(semS4);
        }
    }
}


/**
 * @func    service1
 * @brief   Task for processing data from accelerometer
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void service1(void *params)
{
    // Static variables to store accelerometer and gyroscope readings
    static volatile uint16_t acc_xh = 0, acc_xl = 0;
    static volatile uint16_t gyro_xh = 0, gyro_xl = 0;

    // Loop until the abortS1 flag is set
    while (!abortS1)
    {
        // Wait indefinitely for semaphore S1
        xSemaphoreTake(semS1, portMAX_DELAY);

        // Turn on an LED (assuming GPIO_PIN_3 is connected to an LED)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        // Read accelerometer data
        acc_xh = (read_from_accelerometer(I2C0_BASE, 0x68, 0x3B) << 8);
        acc_xl = read_from_accelerometer(I2C0_BASE, 0x68, 0x3C);
        acc_xh |= acc_xl;

        // Read gyroscope data
        gyro_xh = (read_from_accelerometer(I2C0_BASE, 0x68, 0x43) << 8);
        gyro_xl = read_from_accelerometer(I2C0_BASE, 0x68, 0x44);

        // Processing accelerometer data
        acc_xh /= ACC_SCALING - ACC_OFFSET;
        speed += acc_xh;

        // Turn off the LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

        // Print accelerometer data
        //UARTprintf("A: %d\n", acc_xh);
    }

    // Print a message indicating that Service 1 is exiting
    UARTprintf("S1 exit\n");

    // Delete the task
    vTaskDelete(NULL);
}


/**
 * @func    service2
 * @brief   Task for processing data from temperature sensor
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void service2(void *params)
{
    // Variable to store temperature reading
    uint16_t temp;

    // Initial duty cycle for PWM
    uint16_t duty_cycle = (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);

    // Loop until the abortS2 flag is set
    while (!abortS2)
    {
        // Wait indefinitely for semaphore S2
        xSemaphoreTake(semS2, portMAX_DELAY);

        // Turn on an LED (assuming GPIO_PIN_2 is connected to an LED)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        // Read temperature data
        temp = tmp_readdata() * 2;

        // Adjust PWM duty cycle based on temperature
        if (temp + TEMP_OFFSET >= 260)
        {
            duty_cycle = (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);
            PWMPulseWidthSet(
                PWM0_BASE, PWM_OUT_0,
                duty_cycle);
        }
        else
        {
            duty_cycle = (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 3);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                             duty_cycle);
        }

        // Turn off the LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        // Print temperature data
        //UARTprintf("\rT: %d\n", temp);
    }
    // Print a message indicating that Service 2 is exiting
    UARTprintf("S2 exit\n");
    // Delete the task
    vTaskDelete(NULL);
}


/**
 * @func    service3
 * @brief   Task for managing seat adjustment mechanism
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void service3(void *params)
{
    // Variable to count iterations
    int itr = 0;

    // Variable to store end time for service 3
    volatile uint32_t s3_end_time = 0;

    // Loop until the abortS3 flag is set
    while (!abortS3)
    {
        // Check if semaphore S3 is taken
        xSemaphoreTake(semS3, portMAX_DELAY);

        // Adjust PWM pulse width for PWM output 1
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10) * 9);

        // Increment iteration count
        itr++;

        // Check if iterations exceed a certain threshold
        if (itr > SEAT_TIGHT_ITER)
        {
            // Activate seat tightening mechanism
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                             (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10) * 10);
        }

        // Check if iterations exceed the speed offset plus 3
        if (itr > SPEED_OFFSET + 4)
        {
            // Exit the loop if conditions are met
            break;
        }
    }

    // Disable PWM output for PWM0 output 1
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    // Get the end time for service 3
    s3_end_time = xTaskGetTickCount();

    // Check if SAM deadline is missed
    if ((s3_end_time - event_start) > SAM_DEADLINE)
    {
        UARTprintf("SAM deadline missed T: %d S: %d E: %d itr %d\n", s3_end_time - event_start, event_start, s3_end_time, itr);
    }
    else
    {
        UARTprintf("SAM Successful T: %d S: %d E: %d itr %d\n", s3_end_time - event_start, event_start, s3_end_time, itr);
    }

    // Delete the task
    vTaskDelete(NULL);
}

/**
 * @func    service4
 * @brief   Task for managing temperature control mechanism
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void service4(void *params)
{
    // Variable to store temperature reading
    uint16_t temp;

    // Duty cycle for PWM output
    uint16_t duty_cycle = (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);

    // Variable to store end time for service 4
    uint32_t s4_end_time = 0;

    // Loop until the abortS4 flag is set
    while (!abortS4)
    {
        // Wait indefinitely for semaphore S4
        xSemaphoreTake(semS4, portMAX_DELAY);

        // Turn on an LED (assuming GPIO_PIN_2 is connected to an LED)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        // Read temperature data
        temp = tmp_readdata() * 2;

        // If temperature is above a certain threshold, adjust PWM output
        if (temp + TEMP_OFFSET >= 285)
        {
            PWMPulseWidthSet(
                PWM0_BASE,
                PWM_OUT_0,
                (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 5) * 2 - (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10));
        }
        else
        {
            PWMPulseWidthSet(
                PWM0_BASE, PWM_OUT_0,
                ((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)) / 10) * 10);
        }

        // Get current time
        s4_end_time = xTaskGetTickCount();

        // If deadline is reached, exit loop
        if ((s4_end_time - event_start) >= ABD_DEADLINE)
        {
            break;
        }

        // Turn off the LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        // Print temperature data and execution time
        UARTprintf("\rT: %d\n", temp);
    }

    // Check if ABD deadline is missed
    if ((s4_end_time - event_start) > ABD_DEADLINE)
    {
        UARTprintf("ABD deadline missed T: %d S: %d E: %d\n", s4_end_time - event_start, event_start, s4_end_time);
    }
    else
    {
        UARTprintf("ABD Successful T: %d S: %d E: %d\n", s4_end_time - event_start, event_start, s4_end_time);
    }

    // Delete the task
    vTaskDelete(NULL);
}


/**
 * @func    service5
 * @brief   Task for deploying airbag
 * @param   params: Pointer to task parameters (unused)
 * @return  None
 * @reference   FreeRTOS API Documentation - Semaphore Management
 */
static void service5(void *params)
{
    // Wait indefinitely for semaphore S5
    xSemaphoreTake(semS5, portMAX_DELAY);

    // Deploy airbag by writing to servo
    servo_write(10);

    // Get current time
    uint32_t end_time = xTaskGetTickCount();

    // Print message indicating successful airbag deployment along with execution time
    UARTprintf("Air bag deployed T: %d S: %d E: %d\n", end_time - event_start, event_start, end_time);

    // Delete the task
    vTaskDelete(NULL);
}


/**
 * @func    main
 * @brief   Entry point of the program
 * @param   None
 * @return  None
 * @reference   FreeRTOS API Documentation - Task Management
 */
void main(void)
{
    // Enable lazy stacking for floating-point instructions
    ROM_FPULazyStackingEnable();

    // Configure the system clock to use the main oscillator with a 16MHz crystal
    ROM_SysCtlClockSet(
            SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    // Configure UART communication
    ConfigureUART();

    // Initialize I2C0
    init_i2c0();

    // Initialize button
    button_init();

    // Initialize temperature sensor
    tmp_sensor_init();

    // Initialize PWM for heating
    heating_pwm_init();

    // Initialize PWM for seat control
    seat_pwm_init();

    // Initialize servo motor
    servo_init();
    servo_write(90);

    // Initialize Timer0 for scheduler
    timer0_init();

    // Print a welcome message
    UARTprintf("\nRTES Final Project\n\n");

    // Initialize semaphores
    semSched = xSemaphoreCreateMutex();
    semS1 = xSemaphoreCreateMutex();
    semS2 = xSemaphoreCreateMutex();
    semS3 = xSemaphoreCreateMutex();
    semS4 = xSemaphoreCreateMutex();
    semS5 = xSemaphoreCreateMutex();

    // Restrict services to start before the sequencer
    xSemaphoreTake(semS1, portMAX_DELAY);
    xSemaphoreTake(semS2, portMAX_DELAY);
    xSemaphoreTake(semS3, portMAX_DELAY);
    xSemaphoreTake(semS4, portMAX_DELAY);
    xSemaphoreTake(semS5, portMAX_DELAY);

    // Create tasks for sequencer and services
    xTaskCreate(Sequencer_thread, "Sequencer Thread", 128, NULL,
                tskIDLE_PRIORITY + 6, NULL);
    xTaskCreate(service1, "Service 1", 128, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(service2, "Service 2", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(service3, "Service 3", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(service4, "Service 4", 128, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(service5, "Service 5", 128, NULL, tskIDLE_PRIORITY + 3, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Code should never reach here, but keep an infinite loop just in case
    while (1)
        ;
}
