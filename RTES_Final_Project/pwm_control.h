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
 * @file pwm_control.h
 * @brief PWM initialization and control API's
 * @author Jithendra and Suhas
 * @date 2024-4-29
 */
void heating_pwm_init();
void seat_pwm_init();
void servo_init();
void servo_write(float deg);
