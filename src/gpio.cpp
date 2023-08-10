/**
 * @file gpio.c
 * @author Harish Kumar Shivaramappa (harishkumarsedu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "gpio.h"
#include "stm32h743.h"

void LED_Configure(uint8_t LED)
{
  RCC->AHB4ENR |= 1<<1; // enable portB clock 
  GPIOB->MODER &= ~(3 << LED*2);
  GPIOB->MODER |= (OUTPUT << LED*2);
}

void LED_Toggle(uint8_t LED)
{
    BITFLIP(GPIOB->ODR,LED);
}