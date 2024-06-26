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
 * @file accelerometer.c
 * @brief Accelerometer initialization and access APIs
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
#define TARGET_IS_TM4C123_RA1
#include "accelerometer.h"
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
#include "driverlib/i2c.h"
#include "stdarg.h"

/**
 * @func    init_i2c0
 * @brief   Initializes I2C0 peripheral and GPIO port B
 * @param   None
 * @return  None
 * @reference   TivaWare Peripheral Driver Library User's Guide
 */
void init_i2c0(void)
{
    // Enable the I2C0 peripheral and GPIO port B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 (SCL) and B3 (SDA)
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Set pin types for I2C0
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Initialize the I2C master with the system clock frequency and enable high-speed mode
    ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    // Wake up MPU6050 from sleep by writing to its PWR_MGMT_1 register
    write_to_accelerometer(I2C0_BASE, 0x68, 2, 0x6B, 0x00);
}
/**
 * @func    write_to_accelerometer
 * @brief   Writes data to the specified register of the accelerometer via I2C communication
 * @param   ui32Base: Base address of the I2C module
 * @param   ui8SlaveAddr: 7-bit slave address of the accelerometer device
 * @param   nargs: Number of arguments to be written, including the register address
 * @param   ...: Variable number of arguments to be written, starting with the register address followed by data bytes
 * @return  None
 */
void write_to_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                            uint8_t nargs, ...)
{
    // Initialize variable argument list
    va_list vargs;
    va_start(vargs, nargs);

    // Extract the register address from the variable arguments
    uint8_t regAddress = va_arg(vargs, uint8_t); // First argument is always the register address

    // Set the slave address and direction to write
    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, false);

    // Put the register address into the master data register
    I2CMasterDataPut(ui32Base, regAddress);

    // Initiate a burst send start
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until the I2C master is not busy
    while (I2CMasterBusy(ui32Base))
        ;

    // Loop through the remaining arguments
    uint8_t i = 0;
    for (i = 1; i < nargs; i++)
    {
        // Put the next data byte into the master data register
        I2CMasterDataPut(ui32Base, va_arg(vargs, uint8_t));

        // Determine whether to send a finish or continue command based on the current byte's position
        if (i == nargs - 1)
        {
            I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        }
        else
        {
            I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_CONT);
        }

        // Wait until the I2C master is not busy
        while (I2CMasterBusy(ui32Base))
            ;
    }

    // End variable argument list
    va_end(vargs);
}

/**
 * @func    read_from_accelerometer
 * @brief   Reads data from the specified register of the accelerometer via I2C communication
 * @param   ui32Base: Base address of the I2C module
 * @param   ui8SlaveAddr: 7-bit slave address of the accelerometer device
 * @param   reg: Register address to read from
 * @return  Data read from the specified register
 */
uint32_t read_from_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                                 uint8_t reg)
{
    // Set the slave address and direction to write the register address
    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, false);

    // Put the register address into the master data register
    I2CMasterDataPut(ui32Base, reg);

    // Initiate a single send command
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait until the I2C master is not busy
    while (I2CMasterBusy(ui32Base))
        ;

    // Set the slave address and direction to read data
    I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, true);

    // Initiate a single receive command
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait until the I2C master is not busy
    while (I2CMasterBusy(ui32Base))
        ;

    // Return the data read from the master data register
    return I2CMasterDataGet(ui32Base);
}


