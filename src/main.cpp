/**
 * @file main.cpp
 * @author Harish Kumar Shivaramappa (harishkumarsedu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "stm32h743.h"
// #include "gpio.h"
#define GREEN_LED 0 // PB0
#define RED_LED 14 // PB14

// LEDS configure 
void LED_Configure(uint8_t LED);
void LED_Toggle(uint8_t LED);

int main()
{
    
  LED_Configure(RED_LED);

	while(1)
	{

    LED_Toggle(RED_LED);

    for (uint32_t i = 0; i < 1000000000; i++);
    
  }
  
  
  return 0 ;
}
