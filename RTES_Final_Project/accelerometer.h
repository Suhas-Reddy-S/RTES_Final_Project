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
 * @file accelerometer.h
 * @brief Accelerometer initialization and access APIs
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
#include <stdint.h>
void init_i2c0(void);
void write_to_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                            uint8_t nargs, ...);
uint32_t read_from_accelerometer(uint32_t ui32Base, uint8_t ui8SlaveAddr,
                                 uint8_t reg);
