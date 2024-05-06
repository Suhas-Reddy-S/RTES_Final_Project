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
 * @file temperature_sensor.c
 * @brief Temperature sensor initialization and Access API's
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
#define TARGET_IS_TM4C123_RA1
#include "temperature_sensor.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
/**
 * @func    tmp_sensor_init
 * @brief   Initializes the temperature sensor (TMP) via SPI (SSI0)
 * @param   None
 * @return  None
 * @reference   TMP Sensor Datasheet, TM4C123GH6PM example
 */
void tmp_sensor_init()
{
    // Enable the SSI0 peripheral and GPIO port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure pins PA2, PA3, PA4, PA5 as SSI0 pins
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Configure SSI0 to operate as a master, using Motorola SPI mode 0, with a clock speed of 1 MHz and 16-bit data frames
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 16);

    // Enable the SSI0 module
    SSIEnable(SSI0_BASE);
}

/**
 * @func    tmp_readdata
 * @brief   Reads temperature data from the TMP sensor
 * @param   None
 * @return  uint16_t: Temperature data (in degrees Celsius)
 * @reference   TMP Sensor Datasheet, TM4C123GH6PM example
 */
uint16_t tmp_readdata(void)
{
    uint32_t ui32Data;
    uint16_t tempData;

    // Wait until there is no data in the receive FIFO
    while (SSIDataGetNonBlocking(SSI0_BASE, &ui32Data))
        ;

    // Send a dummy byte (0x00) to initiate the SPI transaction and receive data from the temperature sensor
    SSIDataPut(SSI0_BASE, 0x00);
    SSIDataGet(SSI0_BASE, &ui32Data);

    // Convert the received data to temperature using the sensor's data format
    tempData = (uint16_t)(ui32Data >> 2) * 0.25;

    // Return the temperature data
    return tempData;
}


