/**
 * @file gpio.h
 * @author Harish Kumar Shivaramappa (harishkumarsedu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef GPIO_H
#define GPIO_H

#include "stm32h743.h"

#define GREEN_LED 0 // PB0
#define RED_LED 14 // PB14

// LEDS configure 
void LED_Configure(uint8_t LED);
void LED_Toggle(uint8_t LED);

#endif