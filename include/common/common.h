/**
 * @file common.h
 * @author Harish Kumar Shivaramappa (harishkumarsedu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef COMMON__H
#define COMMON__H

#include <stdint.h>

#define __IO volatile

// Define basic functions 

#define BITSET(REG,BIT) (REG |=  1 << BIT)
#define BITCLEAR(REG,BIT) (REG |= ~ (1<< BIT))
#define BITFLIP(REG,BIT) (REG ^= 1<<BIT)

typedef enum 
{
    LOW,
    HIGH,
} Digital_GPIO_Values ; 



typedef enum
{
    False,
    True,
} State ;

// These bits are written by software to configure the I/O mode.
// 00: Input mode 
// 01: General purpose output mode
// 10: Alternate function mode
// 11: Analog mode (reset state)
typedef enum
{
    INPUT,
    OUTPUT,
    ALTERNATE_FUNCTION,
    ANALOG,
} GPIO_Mode;
#endif