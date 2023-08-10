/**
 * @file stm32h743zi.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef STM32H743ZI__H
#define STM32H743ZI__H

#include "common/common.h"

#define PERIPH_BASE               (0x40000000UL) /*!< Base address of : AHB/APB Peripherals                                                   */
#define QSPI_BASE                 (0x90000000UL) /*!< Base address of : QSPI memories  accessible over AXI                                    */

#define FLASH_BANK1_BASE          (0x08000000UL) /*!< Base address of : (up to 1 MB) Flash Bank1 accessible over AXI                          */
#define FLASH_BANK2_BASE          (0x08100000UL) /*!< Base address of : (up to 1 MB) Flash Bank2 accessible over AXI                          */
#define FLASH_END                 (0x081FFFFFUL) /*!< FLASH end address                                                                       */

/* Legacy define */
#define FLASH_BASE                FLASH_BANK1_BASE

/*!< Device electronic signature memory map */
#define UID_BASE                  (0x1FF1E800UL)            /*!< Unique device ID register base address */
#define FLASHSIZE_BASE            (0x1FF1E880UL)            /*!< FLASH Size register base address */

/*!< Peripheral memory map */
#define D2_APB1PERIPH_BASE        PERIPH_BASE
#define D2_APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define D2_AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define D2_AHB2PERIPH_BASE       (PERIPH_BASE + 0x08020000UL)

#define D1_APB1PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)
#define D1_AHB1PERIPH_BASE       (PERIPH_BASE + 0x12000000UL)

#define D3_APB1PERIPH_BASE       (PERIPH_BASE + 0x18000000UL)
#define D3_AHB1PERIPH_BASE       (PERIPH_BASE + 0x18020000UL)


/*!< D3_AHB1PERIPH peripherals */
#define GPIOA_BASE            (D3_AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (D3_AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (D3_AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (D3_AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (D3_AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (D3_AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (D3_AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000UL)
#define GPIOJ_BASE            (D3_AHB1PERIPH_BASE + 0x2400UL)
#define GPIOK_BASE            (D3_AHB1PERIPH_BASE + 0x2800UL)
#define RCC_BASE              (D3_AHB1PERIPH_BASE + 0x4400UL)

typedef struct
{
 __IO uint32_t CR;             /*!< RCC clock control register,                                              Address offset: 0x00  */
 __IO uint32_t HSICFGR;        /*!< HSI Clock Calibration Register,                                          Address offset: 0x04  */
 __IO uint32_t CRRCR;          /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
 __IO uint32_t CSICFGR;        /*!< CSI Clock Calibration Register,                                          Address offset: 0x0C  */
 __IO uint32_t CFGR;           /*!< RCC clock configuration register,                                        Address offset: 0x10  */
 uint32_t     RESERVED1;       /*!< Reserved,                                                                Address offset: 0x14  */
 __IO uint32_t D1CFGR;         /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
 __IO uint32_t D2CFGR;         /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
 __IO uint32_t D3CFGR;         /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
 uint32_t     RESERVED2;       /*!< Reserved,                                                                Address offset: 0x24  */
 __IO uint32_t PLLCKSELR;      /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
 __IO uint32_t PLLCFGR;        /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
 __IO uint32_t PLL1DIVR;       /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
 __IO uint32_t PLL1FRACR;      /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
 __IO uint32_t PLL2DIVR;       /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
 __IO uint32_t PLL2FRACR;      /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
 __IO uint32_t PLL3DIVR;       /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
 __IO uint32_t PLL3FRACR;      /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
 uint32_t      RESERVED3;      /*!< Reserved,                                                                Address offset: 0x48  */
 __IO uint32_t  D1CCIPR;       /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
 __IO uint32_t  D2CCIP1R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
 __IO uint32_t  D2CCIP2R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
 __IO uint32_t  D3CCIPR;       /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
 uint32_t      RESERVED4;      /*!< Reserved,                                                                Address offset: 0x5C  */
 __IO uint32_t  CIER;          /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
 __IO uint32_t  CIFR;          /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
 __IO uint32_t  CICR;          /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
 uint32_t     RESERVED5;       /*!< Reserved,                                                                Address offset: 0x6C  */
 __IO uint32_t  BDCR;          /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
 __IO uint32_t  CSR;           /*!< RCC clock control & status register,                                     Address offset: 0x74  */
 uint32_t     RESERVED6;       /*!< Reserved,                                                                Address offset: 0x78  */
 __IO uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
 __IO uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
 __IO uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
 __IO uint32_t AHB4RSTR;       /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
 __IO uint32_t APB3RSTR;       /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
 __IO uint32_t APB1LRSTR;      /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
 __IO uint32_t APB1HRSTR;      /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
 __IO uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
 __IO uint32_t APB4RSTR;       /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
 __IO uint32_t GCR;            /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
 uint32_t     RESERVED8;       /*!< Reserved,                                                                Address offset: 0xA4  */
 __IO uint32_t D3AMR;          /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
 uint32_t     RESERVED11[9];    /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
 __IO uint32_t RSR;            /*!< RCC Reset status register,                                               Address offset: 0xD0  */
 __IO uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
 __IO uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
 __IO uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
 __IO uint32_t AHB4ENR;        /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
 __IO uint32_t APB3ENR;        /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
 __IO uint32_t APB1LENR;       /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
 __IO uint32_t APB1HENR;       /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
 __IO uint32_t APB2ENR;        /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
 __IO uint32_t APB4ENR;        /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
 uint32_t      RESERVED12;      /*!< Reserved,                                                                Address offset: 0xF8  */
 __IO uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
 __IO uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
 __IO uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
 __IO uint32_t AHB4LPENR;      /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
 __IO uint32_t APB3LPENR;      /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
 __IO uint32_t APB1LLPENR;     /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
 __IO uint32_t APB1HLPENR;     /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
 __IO uint32_t APB2LPENR;      /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
 __IO uint32_t APB4LPENR;      /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
 uint32_t     RESERVED13[4];   /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */

} RCC_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset,               Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)
#endif