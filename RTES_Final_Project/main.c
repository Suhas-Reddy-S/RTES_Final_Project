/*
 * @file main.c
 * @description
 *
 * Occcupant Safety
 *
 * @Authors
 * Jithendra H S
 * Suhas Reddy
 *
 *
 * References:
 *
 * Date: 29th April 2024
 *
 * */
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

#define ACC_SCALING (2048)
#define SCALE (1)
#define SAM_DEADLINE (10)
#define ABD_DEADLINE (20)

#define SEAT_TIGHT_ITER (3)
#define SPEED_OFFSET (20)
#define ACC_OFFSET (5);

static void service1(void *params);
static void service2(void *params);
static void service3(void *params);
static void service4(void *params);
static void service5(void *params);
static void Sequencer_thread(void *params);

xSemaphoreHandle semSched, semS1, semS2, semS3, semS4, semS5;
volatile bool abortS1 = false, abortS2 = false, abortS3 = false, abortS4 = false;
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

void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    while (1)
    {
    }
}

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

void Timer0IntHandler(void)
{
    // Clear the interrupt flag for Timer A timeout
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Release the semaphore to signal the sequencer
    xSemaphoreGive(semSched);
}

void ButtonHandler(void)
{
    // Disable interrupt for GPIO pin 4 on Port F
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Write GPIO pin 1 on Port F to HIGH
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

    abortS1 = true;
    abortS2 = true;
    xSemaphoreGive(semS3);
    xSemaphoreGive(semS4);
    xSemaphoreGive(semS5);
    event_start = xTaskGetTickCount();
}


// Sequencer_thread function definition
// This function runs at a frequency of 1000Hz
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
        if (schedCnt % T1 == 0 && abortS1){
            xSemaphoreGive(semS3);
        }
        if (schedCnt % T2 == 0 && abortS2){
            xSemaphoreGive(semS4);
        }
    }

}

// Service 1 thread function definition
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

        acc_xh /= ACC_SCALING - ACC_OFFSET;
        speed += acc_xh;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

        // Print accelerometer
        //UARTprintf("A: %d\n", acc_xh);

    }

    // Print a message indicating that Service 1 is exiting
    //UARTprintf("S1 exit\n");

    // Delete the task
    vTaskDelete(NULL);
}

// Service 2 thread function definition
static void service2(void *params)
{
    // Variable to store temperature reading
    uint16_t temp;

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
    // Delete the task
    vTaskDelete(NULL);
}

// Service 3 thread function definition
static void service3(void *params)
{
    int itr = 0;
    volatile uint32_t s3_end_time = 0;
    // Loop until the abortS3 flag is set
    while (!abortS3)
    {
        // Check if semaphore S3 is taken
        xSemaphoreTake(semS3, portMAX_DELAY);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)/ 10) * 9);

        itr++;
        if(itr > SEAT_TIGHT_ITER) {
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                                     (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)/ 10) * 10);
        }

        if(itr > SPEED_OFFSET + 3){
            break;
        }
    }
    // Disable PWM output for PWM0 output 1
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    s3_end_time = xTaskGetTickCount();
    if((s3_end_time - event_start) > SAM_DEADLINE){
        UARTprintf("SAM deadline missed T: %d S: %d E: %d\n", s3_end_time - event_start, event_start, s3_end_time);
    }else{
        UARTprintf("SAM Successful T: %d S: %d E: %d\n", s3_end_time - event_start, event_start, s3_end_time);
    }

    // Delete the task
    vTaskDelete(NULL);
}

// Service 4 thread function definition
static void service4(void *params)
{
    // Variable to store temperature reading
    uint16_t temp;

    uint16_t duty_cycle = (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);

    uint32_t s4_end_time = 0;

    // Loop until the abortS2 flag is set
    while (!abortS4)
    {
        // Wait indefinitely for semaphore S2
        xSemaphoreTake(semS4, portMAX_DELAY);

        // Turn on an LED (assuming GPIO_PIN_2 is connected to an LED)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        // Read temperature data
        temp = tmp_readdata() * 2;

        // If abortS1 is set, invert the PWM output

        if (temp + 10 >= 285)
        {
            PWMPulseWidthSet(
                    PWM0_BASE,
                    PWM_OUT_0,
                    (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 5) * 2
                            - (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10));
        }
        else
        {
            PWMPulseWidthSet(
                    PWM0_BASE, PWM_OUT_0,
                    ((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)) / 10) * 10);
        }
        s4_end_time = xTaskGetTickCount();
        if((s4_end_time - event_start) >= ABD_DEADLINE){
            break;
        }
        // Turn off the LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        // Print temperature data and execution time
        UARTprintf("\rT: %d\n", temp);
    }

    if ((s4_end_time - event_start) > ABD_DEADLINE)
    {
        UARTprintf("ABD deadline missed T: %d S: %d E: %d\n", s4_end_time - event_start, event_start, s4_end_time);
    }
    else
    {
        UARTprintf("ABD Successful T: %d S: %d E: %d\n", s4_end_time - event_start, event_start, s4_end_time);
    }
    vTaskDelete(NULL);
}

// Service 5 thread function definition
static void service5(void *params)
{
    xSemaphoreTake(semS5, portMAX_DELAY);
    servo_write(10);
    uint32_t end_time = xTaskGetTickCount();
    UARTprintf("Air bag deployed T: %d S: %d E: %d\n", end_time - event_start, event_start, end_time);
    abortS1 = false;
    abortS2 = false;
    event_start = 0;
    vTaskDelete(NULL);
}

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

    servo_init();
    servo_write(90);

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
                tskIDLE_PRIORITY + 5, NULL);
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
