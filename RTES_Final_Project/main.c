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
 * Date: 9th April 2024
 *
 * */
//#define TARGET_IS_TM4C123_RA1
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdbool.h"

#define SCALE (2)
void init_i2c0(void);
void button_init();
void write_to_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                            uint8_t nargs, ...);
uint32_t read_from_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                                 uint8_t reg);
uint16_t tmp_readdata(void);
void tmp_sensor_init();
void pwm_init();

xSemaphoreHandle semSched, semS1, semS2, semS3;
volatile bool abortS1 = false, abortS2 = false, abortS3 = false;
volatile uint32_t T1 = 1 * SCALE;
volatile uint32_t T2 = 2 * SCALE;
volatile uint32_t T3 = 1 * SCALE;

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

void button_init()
{
    //
    // Enable the GPIO port F.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4,
    GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOF);
    ROM_IntMasterEnable();
}

void init_i2c0(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    // ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // Set true for high-speed mode

    // Wake MPU6050 from sleep
    write_to_accelerometer(I2C0_BASE, 0x68, 2, 0x6B, 0x00); // Write 0 to PWR_MGMT_1 register to wake up MPU6050
}

void write_to_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                            uint8_t nargs, ...)
{
    va_list vargs;
    va_start(vargs, nargs);
    uint8_t regAddress = va_arg(vargs, uint8_t); // First argument is always the register address

    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, false);
    I2CMasterDataPut(ui32Base, regAddress);
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(ui32Base))
        ;
    uint8_t i = 0;
    for (i = 1; i < nargs; i++)
    {
        I2CMasterDataPut(ui32Base, va_arg(vargs, uint8_t));
        if (i == nargs - 1)
        {
            I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        }
        else
        {
            I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_CONT);
        }
        while (I2CMasterBusy(ui32Base))
            ;
    }
    va_end(vargs);
}

uint32_t read_from_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                                 uint8_t reg)
{
    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, false);
    I2CMasterDataPut(ui32Base, reg);
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(ui32Base))
        ;

    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, true);
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(ui32Base))
        ;
    return I2CMasterDataGet(ui32Base);
}

void Timer0IntHandler(void)
{
    static volatile uint32_t g_tick = 0;
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    g_tick++;

    if (g_tick % 1 == 0)
    {
        xSemaphoreGive(semSched); // release sequencer semaphore every 1 millisecond
    }

}

void ButtonHandler(void)
{
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    abortS1 = true;
}

void ConfigureUART(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 230400, 16000000);
}

void tmp_sensor_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
    GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER, 1000000, 16);
    SSIEnable(SSI0_BASE);
}

uint16_t tmp_readdata(void)
{
    uint32_t ui32Data;
    uint16_t tempData;

    while (SSIDataGetNonBlocking(SSI0_BASE, &ui32Data))
        ;

    SSIDataPut(SSI0_BASE, 0x00);
    SSIDataGet(SSI0_BASE, &ui32Data);
    tempData = (uint16_t) (ui32Data >> 2) * 0.25;

    return tempData;
}


void heating_pwm_init()
{
    SysCtlPWMClockSet(SYSCTL_SYSDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
    PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 200000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void seat_pwm_init()
{
    SysCtlPWMClockSet(SYSCTL_SYSDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
    PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 200000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0));
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_7, 0);
}

// Runs @ 1000Hz
static void Sequencer_thread(void *params)
{
static volatile uint32_t schedCnt = 0;
UARTprintf("Sequencer Started at %u msec\n", xTaskGetTickCount());
while (1)
{
    xSemaphoreTake(semSched, portMAX_DELAY);
    schedCnt++;
    if (schedCnt % T1 == 0)
    {
        xSemaphoreGive(semS1);
    }
    if (schedCnt % T2 == 0)
    {
        xSemaphoreGive(semS2);
    }
    if (schedCnt % T3 == 0 && abortS1)
    {
        xSemaphoreGive(semS3);
    }
}
}

// Runs @ 5Hz
static void service1(void *params)
{
static volatile uint16_t acc_xh = 0, acc_xl = 0;
static volatile uint16_t gyro_xh = 0, gyro_xl = 0;
uint32_t startTime, endTime;
while (!abortS1)
{
    xSemaphoreTake(semS1, portMAX_DELAY);
    startTime = xTaskGetTickCount();
    acc_xh = (read_from_accelerometer(I2C0_BASE, 0x68, 0x3B) << 8);
    acc_xl = read_from_accelerometer(I2C0_BASE, 0x68, 0x3C);
    acc_xh |= acc_xl;
    gyro_xh = (read_from_accelerometer(I2C0_BASE, 0x68, 0x43) << 8);
    gyro_xl = read_from_accelerometer(I2C0_BASE, 0x68, 0x44);
    endTime = xTaskGetTickCount();
    //UARTprintf("A: %d, G: %d, E: %d\n", acc_xh, gyro_xh, endTime - startTime);
}
UARTprintf("S1 exit\n");
vTaskDelete(NULL);
}

static void service2(void *params)
{
    uint32_t startTime, endTime;
    uint16_t temp;
while (!abortS2)
{
    xSemaphoreTake(semS2, portMAX_DELAY);
    startTime = xTaskGetTickCount();
    temp = tmp_readdata();
    if (abortS1)
    {
        PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, true);
    }
    endTime = xTaskGetTickCount();
    UARTprintf("\rT: %d, E: %d\n", temp, endTime - startTime);
}
}

static void service3(void *params)
{
    uint32_t startTime, endTime, releaseTime;
while (!abortS3)
{
    if(xSemaphoreTake(semS3, portMAX_DELAY) == 0) {
        releaseTime = xTaskGetTickCount();
    }
    startTime = xTaskGetTickCount();
    if (abortS1)
    {
        PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
    }
    endTime = xTaskGetTickCount();
    UARTprintf("\rS3 E: %d\n", endTime - startTime);
}
}

void main(void)
{
ROM_FPULazyStackingEnable();

ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
SYSCTL_XTAL_16MHZ);

ConfigureUART();
init_i2c0();
button_init();
tmp_sensor_init();
heating_pwm_init();
seat_pwm_init();

UARTprintf("\nRTES Final Project\n\n");

// Initialize semaphores
semSched = xSemaphoreCreateMutex();
semS1 = xSemaphoreCreateMutex();
semS2 = xSemaphoreCreateMutex();
semS3 = xSemaphoreCreateMutex();

// Restrict services to start before sequencer
xSemaphoreTake(semS1, portMAX_DELAY);
xSemaphoreTake(semS2, portMAX_DELAY);
xSemaphoreTake(semS3, portMAX_DELAY);

//  Enabling Timer Peripheral
ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / 1000); // Configure timer0A to run at 1000HZ

ROM_IntEnable(INT_TIMER0A);
ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

ROM_TimerEnable(TIMER0_BASE, TIMER_A);

xTaskCreate(Sequencer_thread, "Sequencer Thread", 128, NULL,
            tskIDLE_PRIORITY + 5, NULL);
xTaskCreate(service1, "Service 1", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
xTaskCreate(service2, "Service 2", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
xTaskCreate(service3, "Service 3", 128, NULL, tskIDLE_PRIORITY + 4, NULL);

vTaskStartScheduler();
while (1)
    ;
}
