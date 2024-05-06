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
 * @file temperature_sensor.h
 * @brief Temperature sensor initialization and Access API's
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
#include <stdint.h>
#define TEMP_OFFSET (5)
void tmp_sensor_init();
uint16_t tmp_readdata(void);
