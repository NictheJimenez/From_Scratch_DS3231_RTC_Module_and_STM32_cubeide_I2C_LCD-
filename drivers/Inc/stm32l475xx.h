/*
 * stm32l475xx.h
 *
 *  Created on: 2022
 *      Author: Nicthe Jimenez
 */


#ifndef INC_STM32L475XX_H_
#define INC_STM32L475XX_H_

#include <stdint.h>
#include <stddef.h>
//#include <iostream>
#include <string.h>
#define __vo                                  volatile
#define __weak                                __attribute__((weak))

/*
 ******************************************************************************************************
                                       Processor Specific Details
*******************************************************************************************************
                                       ARM Cortex Mx Processor NVIC ISERx Register Addresses => (Interrupt Set-enable Registers)
 ******************************************************************************************************
 */
#define NVIC_ISER0                          ((__vo uint32_t *) 0xE000E100)
#define NVIC_ISER1                          ((__vo uint32_t *) 0xE000E104)
#define NVIC_ISER2                          ((__vo uint32_t *) 0xE000E108)
#define NVIC_ISER3                          ((__vo uint32_t *) 0xE000E10c)
/*
 * ****************************************************************************************************
 *                                   ARM Cortex Mx Processor NVIC ICERx Register Addresses => (Interrupt Clear-enable Registers)
 * ****************************************************************************************************
 */
#define NVIC_ICER0                         ((__vo uint32_t *) 0XE000E180)
#define NVIC_ICER1                         ((__vo uint32_t *) 0XE000E184)
#define NVIC_ICER2                         ((__vo uint32_t *) 0XE000E188)
#define NVIC_ICER3                         ((__vo uint32_t *) 0XE000E18C)
/*
 * ****************************************************************************************************
 *                                   ARM Cortex Mx Processor NVIC ICERx Register Addresses => (Interrupt Priority Registers)
 * ****************************************************************************************************
 */
#define NVIC_PR_BASEADDR                   ((__vo uint32_t *) 0xE000E400)
/*
 * ****************************************************************************************************
 *                                   ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 * ****************************************************************************************************
 */
#define NO_PR_BITS_IMPLEMENTED             4
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR                        0x08000000UL
#define FLASH                                  FLASH_BASEADDR
#define ROM                                    0x1FFF0000UL
#define SRAM1_BASEADDR                        0x20000000UL
#define SRAM1                                  SRAM1_BASEADDR
/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR                       0x40000000UL
#define APB1_PERIPH_BASEADDR                  PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR                  0x40010000UL
#define AHB1_PERIPH_BASEADDR                  0x40020000UL
#define AHB2_PERIPH_BASEADDR                  0x48000000UL
/*
 * Base addresses of pripherals which are hanging on AHB1 bus
 */
#define RCC_BASEADDR                          (AHB1_PERIPH_BASEADDR + 0x1000)
/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x0000)   //0x48000000UL
#define GPIOB_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x0400)   //0x48000400UL
#define GPIOC_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x0800)   //0x48000800UL
#define GPIOD_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x0C00)   //0x48000C00UL
#define GPIOE_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x1000)   //0x48001000UL
#define GPIOF_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x1400)   //0x48001400UL
#define GPIOG_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x1800)   //0x48001800UL
#define GPIOH_BASEADDR                        (AHB2_PERIPH_BASEADDR + 0x1C00)   //0x48001C00UL
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define SPI2_BASEADDR                         (APB1_PERIPH_BASEADDR + 0x3800)    //0x40003800UL
#define SPI3_BASEADDR                         (APB1_PERIPH_BASEADDR + 0x3C00)    //0x40003C00UL
#define USART2_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x4400)    //0x40004400UL
#define USART3_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x4800)    //0x40004800UL
#define UART4_BASEADDR                        (APB1_PERIPH_BASEADDR + 0x4C00)    //0x40004C00UL
#define UART5_BASEADDR                        (APB1_PERIPH_BASEADDR + 0x5000)    //0x40005000UL
#define I2C1_BASEADDR                         (APB1_PERIPH_BASEADDR + 0x5400)    //0x40005400UL
#define I2C2_BASEADDR                         (APB1_PERIPH_BASEADDR + 0x5800)    //0x40005800UL
#define I2C3_BASEADDR                         (APB1_PERIPH_BASEADDR + 0x5C00)    //0x40005C00UL
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR                        (APB2_PERIPH_BASEADDR + 0x0400)     //0x40010400UL
#define USART1_BASEADDR                      (APB2_PERIPH_BASEADDR + 0x3800)     //0x40013800UL
#define SPI1_BASEADDR                        (APB2_PERIPH_BASEADDR + 0X3000)     //0x40013000UL
#define SYSCFG_BASEADDR                      (APB2_PERIPH_BASEADDR + 0X0000)     //0x40010000UL
/*
 *Peripheral register definition structure for GPIO
 */

typedef struct{
	__vo uint32_t MODER;                         /*<GPIO port mode register                                                     Address offset: 0x00>*/
	__vo uint32_t OTYPER;                        /*<GPIO port output type register                                              Address offset: 0x04>*/
	__vo uint32_t OSPEEDR;                       /*<GPIO port output speed register                                             Address offset: 0x08>*/
	__vo uint32_t PUPDR;                         /*<GPIO port pull-up/pull-down register                                        Address offset: 0x0C>*/
	__vo uint32_t IDR;                           /*<GPIO port input data register                                               Address offset: 0x10>*/
	__vo uint32_t ODR;                           /*<GPIO port output data register                                              Address offset: 0x14>*/
	__vo uint32_t BSRR;                          /*<GPIO port bit set/reset register                                            Address offset: 0x18>*/
	__vo uint32_t LCKR;                          /*<GPIO port configuration lock register                                       Address offset: 0x1C>*/
	__vo uint32_t AFRL[2];                       /*<GPIO alternate function low-high register                                   Address offset: 0x20 and Address offset: 0x24>*/
	__vo uint32_t BRR;                           /*<GPIO port bit reset register                                                Address offset: 0x28>*/
	__vo uint32_t ASCR;                          /*<GPIO port analog switch control register                                    Address offset: 0x2C>*/
}GPIO_RegDef_t;
/*
 * Peripheral register definition structure for RCC
 */
typedef struct{
	__vo uint32_t CR;                           /*<RCC Clock control register                                                     Address offset: 0x00>*/
	__vo uint32_t ICSCR;                        /*<RCC Internal clock sources calibration register                                Address offset: 0x04>*/
	__vo uint32_t CFGR;                         /*<RCC Clock configuration register                                               Address offset: 0x08>*/
	__vo uint32_t PLLCFGR;                      /*<RCC PLL configuration register                                                 Address offset: 0x0C>*/
	__vo uint32_t PLLSAI1CFGR;                  /*<RCC PLLSAI1 configuration register                                             Address offset: 0x10>*/
	__vo uint32_t PLLSAI2CFGR;                  /*<RCC PLLSAI2 configuration register                                             Address offset: 0x14>*/
	__vo uint32_t CIER;                         /*<RCC Clock interrupt enable register                                            Address offset: 0x18>*/
	__vo uint32_t CIFR;                         /*<RCC Clock interrupt flag register                                              Address offset: 0x1C>*/
	__vo uint32_t CICR;                         /*<RCC Clock interrupt clear register                                             Address offset: 0x20>*/
         uint32_t Reserved0;                                                                                                      /*<Address offset: 0x24>*/
    __vo uint32_t AHB1RSTR;                     /*<RCC AHB1 peripheral reset register                                             Address offset: 0x28>*/
    __vo uint32_t AHB2RSTR;                     /*<RCC AHB2 peripheral reset register                                             Address offset: 0x2C>*/
    __vo uint32_t AHB3RSTR;                     /*<RCC AHB3 peripheral reset register                                             Address offset: 0x30>*/
         uint32_t Reserved1;                                                                                                      /*<Address offset: 0x34>*/
    __vo uint32_t APB1RSTR[2];                  /*<RCC APB1 peripheral reset register 1-2                                         Address offset: 0x38 and Address offset: 0x3C>*/
    __vo uint32_t APB2RSTR;                     /*<RCC APB2 peripheral reset register                                             Address offset: 0x40>*/
         uint32_t Reserved2;                                                                                                       /*<Address offset: 0x44>*/
    __vo uint32_t AHB1ENR;                      /*<RCC AHB1 peripheral clock enable register                                       Address offset: 0x48>*/
    __vo uint32_t AHB2ENR;                      /*<RCC AHB2 peripheral clock enable register                                       Address offset: 0x4C>*/
    __vo uint32_t AHB3ENR;                      /*<RCC AHB3 peripheral clock enable register                                       Address offset: 0x50>*/
         uint32_t Reserved3;                                                                                                       /*<Address offset: 0x54>*/
    __vo uint32_t APB1ENR[2];                  /*<RCC APB1 peripheral clock enable register 1-2                                    Address offset: 0x58 and Address offset: 0x5C>*/
    __vo uint32_t APB2ENR;                      /*<RCC APB2 peripheral clock enable register                                       Address offset: 0x60>*/
         uint32_t Reserved4;                                                                                                       /*<Address offset: 0x64>*/
    __vo uint32_t AHB1SMENR;                    /*<RCC AHB1 peripheral clocks enable in Sleep and Stop modes register              Address offset: 0x68>*/
    __vo uint32_t AHB2SMENR;                    /*<RCC AHB2 peripheral clocks enable in Sleep and Stop modes register              Address offset: 0x6C>*/
    __vo uint32_t AHB3SMENR;                    /*<RCC AHB3 peripheral clocks enable in Sleep and Stop modes register              Address offset: 0x70>*/
         uint32_t Reserved5;                                                                                                       /*<Address offset: 0x74>*/
    __vo uint32_t APB1SMENR[2];                 /*<RCC APB1 peripheral clocks enable in Sleep and Stop modes register 1-2          Address offset: 0x78 and Address offset: 0x7C>*/
    __vo uint32_t APB2SMENR;                    /*<RCC APB2 peripheral clocks enable in Sleep and Stop modes register              Address offset: 0x80>*/
         uint32_t Reserved6;                                                                                                       /*<Address offset: 0x84>*/
    __vo uint32_t CCIPR;                        /*<RCC Peripherals independent clock configuration register                        Address offset: 0x88>*/
    __vo uint32_t BDCR;                         /*<RCC Backup domain control register                                              Address offset: 0x90>*/
    __vo uint32_t CSR;                          /*<RCC Control/status register                                                     Address offset: 0x94>*/
    __vo uint32_t CRRCR;                        /*<RCC Clock recovery RC register                                                  Address offset: 0x98>*/
    __vo uint32_t CCIPR2;                       /*<RCC Peripherals independent clock configuration register                        Address offset: 0x9C>*/
}RCC_RegDef_t;
/*
 *Peripheral register definition structure for EXTI
 */
typedef struct{
	__vo uint32_t IMR1;                           /*<EXTI Interrupt mask register 1                                                Address offset:0x00                Reset value: 0xFF82 0000 >*/
	__vo uint32_t EMR1;                           /*<EXTI Event mask register 1                                                    Address offset: 0x04               Reset value: 0x0000 0000>*/
	__vo uint32_t RTSR1;                          /*<EXTI Rising trigger selection register 1                                      Address offset: 0x08               Reset value: 0x0000 0000>*/
	__vo uint32_t FTSR1;                          /*<EXTI Falling trigger selection register 1                                     Address offset: 0x0C               Reset value: 0x0000 0000>*/
	__vo uint32_t SWIER1;                         /*<EXTI Software interrupt event register 1                                      Address offset: 0x10               Reset value: 0x0000 0000>*/
	__vo uint32_t PR1;                            /*<EXTI Pending register 1                                                       Address offset: 0x14               Reset value: 0x0000 0000>*/
}EXTI_RegDef_t;
/*
 *Peripheral register definition structure for SYSCFG
 */
typedef struct{
	__vo uint32_t MEMRMP;                         /*<SYSCFG memory remap register                                                   Address offset:0x00                                                                                       Reset value: 0x0000 000X (X is the memory mode selected by the BOOT0 pin and BOOT1 option bit)>*/
	__vo uint32_t CFGR1;                          /*<SYSCFG configuration register 1                                                Address offset: 0x04                                                                                      Reset value: 0x7C00 0001>*/
	__vo uint32_t EXTICR[4];                      /*<SYSCFG external interrupt configuration register 1-2-3-4                       Address offset: 0x08  and  Address offset: 0x0C and Address offset: 0x10  and Address offset: 0x14        Reset value: 0x0000 0000>*/
	__vo uint32_t SCSR;                          /*<SYSCFG SRAM2 control and status register                                        Address offset: 0x18                                                                                      Reset value: 0x0000 0000>*/
	__vo uint32_t CFGR2;                         /*<SYSCFG configuration register 2                                                 Address offset: 0x1C                                                                                      Reset value: 0x0000 0000>*/
	__vo uint32_t SWPR;                          /*<SYSCFG SRAM2 write protection register                                          Address offset: 0x20                                                                                      Reset value: 0x0000 0000>*/
	__vo uint32_t SKR;                           /*<SYSCFG SRAM2 key register                                                       Address offset: 0x24                                                                                      Reset value: 0x0000 0000>*/
	__vo uint32_t SWPR2;                         /*<SYSCFG SRAM2 write protection register 2                                        Address offset: 0x28                                                                                      Reset value: 0x0000 0000>*/
}SYSCFG_RegDef_t;
/*
 * Peripheral register definition structure for SPI
 */
typedef struct{
	__vo uint32_t CR1;                          /*<SPI control register 1                                                           Address offset: 0x00                Reset value: 0x0000>*/
	__vo uint32_t CR2;                          /*<SPI control register 2                                                           Address offset: 0x04                Reset value: 0x0700>*/
	__vo uint32_t SR;                           /*<SPI status register                                                              Address offset: 0x08                Reset value: 0x0002>*/
	__vo uint32_t DR;                           /*<SPI data register                                                                Address offset: 0x0C>*/
	__vo uint32_t CRCPR;                        /*<SPI CRC polynomial register                                                      Address offset: 0x10                Reset value: 0x0007>*/
	__vo uint32_t RXCRCR;                       /*<SPI Rx CRC register                                                              Address offset: 0x14                Reset value: 0x0000>*/
	__vo uint32_t TXCRCR;                       /*<SPI Tx CRC register                                                              Address offset: 0x18                Reset value: 0x0000>*/
}SPI_RegDef_t;
/*
 * Peripheral register definition structure for I2C
 */
typedef struct{
	__vo uint32_t CR1;                          /*<I2C control register 1                                                           Address offset: 0x00                Reset value: 0x0000 0000>*/
	__vo uint32_t CR2;                          /*<I2C control register 2                                                           Address offset: 0x04                Reset value: 0x0000 0000>*/
	__vo uint32_t OAR1;                         /*<I2C own address 1 register                                                       Address offset: 0x08                Reset value: 0x0000 0000>*/
	__vo uint32_t OAR2;                         /*<I2C own address 2 register                                                       Address offset: 0x0C                Reset value: 0x0000 0000>*/
	__vo uint32_t TIMINGR;                      /*<I2C timing register                                                              Address offset: 0x10                Reset value: 0x0000 0000>*/
	__vo uint32_t TIMEOUTR;                     /*<I2C timeout register                                                             Address offset: 0x14                Reset value: 0x0000 0000>*/
	__vo uint32_t ISR;                          /*<I2C interrupt and status register                                                Address offset: 0x18                Reset value: 0x0000 0001>*/
	__vo uint32_t ICR;                          /*<I2C interrupt clear register                                                     Address offset: 0x1C                Reset value: 0x0000 0000>*/
	__vo uint32_t PECR;                         /*<I2C PEC register                                                                 Address offset: 0x20                Reset value: 0x0000 0000>*/
	__vo uint32_t RXDR;                         /*<I2C receive data register                                                        Address offset: 0x24                Reset value: 0x0000 0000>*/
	__vo uint32_t TXDR;                         /*<I2C transmit data register                                                       Address offset: 0x28                Reset value: 0x0000 0000>*/
}I2C_RegDef_t;
/*
 * Peripheral register definition structure for USART
 */
typedef struct{
	__vo uint32_t CR1;                          /*<USART control register 1                                                           Address offset: 0x00                Reset value: 0x0000 0000>*/
	__vo uint32_t CR2;                          /*<USART control register 2                                                           Address offset: 0x04                Reset value: 0x0000 0000>*/
	__vo uint32_t CR3;                          /*<USART control register 3                                                           Address offset: 0x08                Reset value: 0x0000 0000>*/
	__vo uint32_t BRR;                          /*<USART baud rate register                                                           Address offset: 0x0C                Reset value: 0x0000 0000>*/
	__vo uint32_t GTPR;                         /*<USART guard time and prescaler register                                            Address offset: 0x10                Reset value: 0x0000 0000>*/
	__vo uint32_t RTOR;                         /*<USART receiver timeout register                                                    Address offset: 0x14                Reset value: 0x0000 0000>*/
	__vo uint32_t RQR;                          /*<USART request register                                                             Address offset: 0x18                Reset value: 0x0000 0000>*/
    __vo uint32_t ISR;                          /*<USART interrupt and status register                                                Address offset: 0x1C                Reset value: 0x0200 00C0>*/
    __vo uint32_t ICR;                          /*<USART interrupt flag clear register                                                Address offset: 0x20                Reset value: 0x0000 0000>*/
    __vo uint32_t RDR;                          /*<USART receive data register                                                        Address offset: 0x24                Reset value: 0x0000 0000>*/
    __vo uint32_t TDR;                          /*<USART transmit data register                                                       Address offset: 0x28                Reset value: 0x0000 0000>*/
}USART_RegDef_t;
/*
 * Peripheral definition (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA                                ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                                ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                                ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                                ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                                ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                                ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                                ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                                ((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC                                  ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI                                 ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG                               ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1                                 ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2                                 ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3                                 ((SPI_RegDef_t*) SPI3_BASEADDR)

#define I2C1                                 ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2                                 ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3                                 ((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1                               ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2                               ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3                               ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4                                ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5                                ((USART_RegDef_t*) UART5_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 0)
#define GPIOB_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 1)
#define GPIOC_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 2)
#define GPIOD_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 3)
#define GPIOE_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 4)
#define GPIOF_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 5)
#define GPIOG_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 6)
#define GPIOH_PCLK_EN()            (RCC -> AHB2ENR |= 1 << 7)
/*
 * Clock Enable Macros for I2cx peripherals
 */
#define I2C1_PCLK_EN()             (RCC -> APB1ENR[0] |= 1 << 21)
#define I2C2_PCLK_EN()             (RCC -> APB1ENR[0] |= 1 << 22)
#define I2C3_PCLK_EN()             (RCC -> APB1ENR[0] |= 1 << 23)
/*
 * Clock Enable Macros for UARTx peripherals
 */
#define USART1_PCLK_EN()           (RCC -> APB2ENR |= 1 << 14)
#define USART2_PCLK_EN()           (RCC -> APB1ENR[1] |= 1 << 17)
#define USART3_PCLK_EN()           (RCC -> APB1ENR[1] |= 1 << 18)
#define UART4_PCLK_EN()            (RCC -> APB1ENR[1] |= 1 << 19)
#define UART5_PCLK_EN()            (RCC -> APB1ENR[1] |= 1 << 20)
/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()            (RCC -> APB2ENR |= 1 << 12)
#define SPI2_PCLK_EN()            (RCC -> APB1ENR[1] |= 1 << 14)
#define SPI3_PCLK_EN()            (RCC -> APB1ENR[1] |= 1 << 15)
/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()         (RCC -> APB2ENR |= (1 << 0))
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()            (RCC -> AHB2ENR &= ~(1 << 7))
/*
 * Clock Disable Macros for I2cx peripherals
 */
#define I2C1_PCLK_DI()             (RCC -> APB1ENR[1] &= ~(1 << 21))
#define I2C2_PCLK_DI()             (RCC -> APB1ENR[1] &= ~(1 << 22))
#define I2C3_PCLK_DI()             (RCC -> APB1ENR[1] &= ~(1 << 23))
/*
 * Clock Disable Macros for UARTx peripherals
 */
#define USART1_PCLK_DI()           (RCC -> APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()           (RCC -> APB1ENR[1] &= ~(1 << 17))
#define USART3_PCLK_DI()           (RCC -> APB1ENR[1] &= ~(1 << 18))
#define UART4_PCLK_DI()            (RCC -> APB1ENR[1] &= ~(1 << 19))
#define UART5_PCLK_DI()            (RCC -> APB1ENR[1] &= ~(1 << 20))
/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()            (RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()            (RCC -> APB1ENR[1] &= ~( 1 << 14))
#define SPI3_PCLK_DI()            (RCC -> APB1ENR[1] &= ~(1 << 15))
/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 0));  (RCC -> AHB2RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 1));  (RCC -> AHB2RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 2));  (RCC -> AHB2RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 3));  (RCC -> AHB2RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 4));  (RCC -> AHB2RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 5));  (RCC -> AHB2RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 6));  (RCC -> AHB2RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()        do{ (RCC -> AHB2RSTR |= (1 << 7));  (RCC -> AHB2RSTR &= ~(1 << 7));}while(0)
/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()        do{ (RCC -> APB2RSTR |= (1 << 12));  (RCC -> APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 14));  (RCC -> APB1RSTR[1] &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 15));  (RCC -> APB1RSTR[1] &= ~(1 << 15));}while(0)
/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 21));  (RCC -> APB1RSTR[1] &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 22));  (RCC -> APB1RSTR[1] &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 23));  (RCC -> APB1RSTR[1] &= ~(1 << 23));}while(0)
/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()        do{ (RCC -> APB2RSTR |= (1 << 14));  (RCC -> APB2RSTR &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 17));  (RCC -> APB1RSTR[1] &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()        do{ (RCC -> APB1RSTR[1] |= (1 << 18));  (RCC -> APB1RSTR[1] &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()         do{ (RCC -> APB1RSTR[1] |= (1 << 19));  (RCC -> APB1RSTR[1] &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()         do{ (RCC -> APB1RSTR[1] |= (1 << 20));  (RCC -> APB1RSTR[1] &= ~(1 << 20));}while(0)
/*
 * Returns Port Code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(pGPIOx)                          ((pGPIOx == GPIOA) ? 0 : \
														        (pGPIOx == GPIOB) ? 1 : \
														        (pGPIOx == GPIOC) ? 2 : \
                                                                (pGPIOx == GPIOD) ? 3 : \
                                                                (pGPIOx == GPIOE) ? 4 : \
                                                                (pGPIOx == GPIOF) ? 5 : \
                                                                (pGPIOx == GPIOG) ? 6 : \
                                                                (pGPIOx == GPIOH) ? 7 : 0)
/*
 * IRQ (Interrupt Request) Number of STM32L475xx MCU
 */
#define IRQ_NO_EXTI0                               6
#define IRQ_NO_EXTI1                               7
#define IRQ_NO_EXTI2                               8
#define IRQ_NO_EXTI3                               9
#define IRQ_NO_EXTI4                               10
#define IRQ_NO_EXTI9_5                             23
#define IRQ_NO_EXTI15_10                           40
#define IRQ_NO_SPI1                                35
#define IRQ_NO_SPI2                                36
#define IRQ_NO_SPI3                                51
#define IRQ_NO_I2C1_EV                             31
#define IRQ_NO_I2C1_ER                             32
#define IRQ_NO_I2C2_EV                             33
#define IRQ_NO_I2C2_ER                             34
#define IRQ_NO_I2C3_EV                             72
#define IRQ_NO_I2C3_ER                             73

/*
 * Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0                              0
#define NVIC_IRQ_PRI1                              17
#define NVIC_IRQ_PRI2                              2
#define NVIC_IRQ_PRI3                              3
#define NVIC_IRQ_PRI4                              4
#define NVIC_IRQ_PRI5                              5
#define NVIC_IRQ_PRI6                              6
#define NVIC_IRQ_PRI7                              7
#define NVIC_IRQ_PRI8                              8
#define NVIC_IRQ_PRI9                              9
#define NVIC_IRQ_PRI10                             10
#define NVIC_IRQ_PRI11                             11
#define NVIC_IRQ_PRI12                             12
#define NVIC_IRQ_PRI13                             13
#define NVIC_IRQ_PRI14                             14
#define NVIC_IRQ_PRI15                             15
#define NVIC_IRQ_PRI16                             16
#define NVIC_IRQ_PRI17                             1
/*
 * **********************************************************************************************
 *  Bit position definitions of SPI peripheral
 * **********************************************************************************************
 */
/*
 *SPI control register 1 (SPIx_CR1)
*/
#define SPI_CR1_CPHA                           0
#define SPI_CR1_CPOL                           1
#define SPI_CR1_MSTR                           2
#define SPI_CR1_BR                             3
#define SPI_CR1_SPE                            6
#define SPI_CR1_LSBFIRST                       7
#define SPI_CR1_SSI                            8
#define SPI_CR1_SSM                            9
#define SPI_CR1_RXONLY                         10
#define SPI_CR1_DFF                            11
#define SPI_CR1_CRCNEXT                        12
#define SPI_CR1_CRCEN                          13
#define SPI_CR1_BIDIOE                         14
#define SPI_CR1_BIDIMODE                       15
/*
*SPI control register 2 (SPIx_CR2)
*/
#define SPI_CR2_RXDMAEN                         0
#define SPI_CR2_TXDMAEN                         1
#define SPI_CR2_SSOE                            2
#define SPI_CR2_NSSP                            3
#define SPI_CR2_FRF                             4
#define SPI_CR2_ERRIE                           5
#define SPI_CR2_RXNEIE                          6
#define SPI_CR2_TXEIE                           7
#define SPI_CR2_DS                              8
#define SPI_CR2_FRXTH                           12
#define SPI_CR2_LDMARX                          13
#define SPI_CR2_ LDMATX                         14
//#define SPI_CR1_RES                           15
/*
 *SPI status register (SPIx_SR)
 */
#define SPI_SR_RXNE                             0
#define SPI_SR_TXE                              1
//#define SPI_SR_ Res                           2
//#define SPI_SR_ Res                           3
#define SPI_SR_CRCERR                           4
#define SPI_SR_MODF                             5
#define SPI_SR_OVR                              6
#define SPI_SR_BSY                              7
#define SPI_SR_FRE                              8
#define SPI_SR_FRLVL                            9
#define SPI_SR_FTLVL                            11
//#define SPI_SR_ Res                           13
//#define SPI_SR_ Res                           14
//#define SPI_SR_ Res                           15
/*
 * **********************************************************************************************
 *  Bit position definitions of I2C peripheral
 * **********************************************************************************************
 */
/*
 * I2C control register 1 (I2C_CR1)
 */
#define I2C_CR1_PE                              0
#define I2C_CR1_TXIE                            1
#define I2C_CR1_RXIE                            2
#define I2C_CR1_ADDRIE                          3
#define I2C_CR1_NACKIE                          4
#define I2C_CR1_STOPIE                          5
#define I2C_CR1_TCIE                            6
#define I2C_CR1_ERRIE                           7
#define I2C_CR1_DNF                             8
#define I2C_CR1_ANFOFF                          12
//#define I2C_CR1_Res                           13
#define I2C_CR1_TXDMAEN                         14
#define I2C_CR1_RXDMAEN                         15
#define I2C_CR1_SBC                             16
#define I2C_CR1_NOSTRETCH                       17
#define I2C_CR1_WUPEN                           18
#define I2C_CR1_GCEN                            19
#define I2C_CR1_SMBHEN                          20
#define I2C_CR1_SMBDEN                          21
#define I2C_CR1_ALERTEN                         22
#define I2C_CR1_PECEN                           23
//#define I2C_CR1_Res                           24
//#define I2C_CR1_Res                           25
//#define I2C_CR1_Res                           26
//#define I2C_CR1_Res                           27
//#define I2C_CR1_Res                           28
//#define I2C_CR1_Res                           29
//#define I2C_CR1_Res                           30
//#define I2C_CR1_Res                           31
//#define I2C_CR1_Res                           32
/*
 * I2C control register 2 (I2C_CR2)
 */
#define I2C_CR2_SADD                            0
#define I2C_CR2_RDWRN                           10
#define I2C_CR2_ADD10                           11
#define I2C_CR2_HEAD10R                         12
#define I2C_CR2_START                           13
#define I2C_CR2_STOP                            14
#define I2C_CR2_NACK                            15
#define I2C_CR2_NBYTES                          16
#define I2C_CR2_RELOAD                          24
#define I2C_CR2_AUTOEND                         25
#define I2C_CR2_PECBYTE                         26
/*
 * I2C own address 1 register (I2C_OAR1)
 */
#define I2C_OAR1_OA1                             0
#define I2C_OAR1_OA1MODE                         10
#define I2C_OAR1_OA1EN                           15
/*
 * I2C own address 2 register (I2C_OAR2)
 */
#define I2C_OAR2_OA2                             0
#define I2C_OAR2_OA2MSK                          8
#define I2C_OAR2_OA2EN                           15
/*
 * I2C interrupt and status register (I2C_ISR)
 */
#define I2C_ISR_TXE                              0
#define I2C_ISR_TXIS                             1
#define I2C_ISR_RXNE                             2
#define I2C_ISR_ADDR                             3
#define I2C_ISR_NACKF                            4
#define I2C_ISR_STOPF                            5
#define I2C_ISR_TC                               6
#define I2C_ISR_TCR                              7
#define I2C_ISR_BERR                             8
#define I2C_ISR_ARLO                             9
#define I2C_ISR_OVR                              10
#define I2C_ISR_PECERR                           11
#define I2C_ISR_TIMEOUT                          12
#define I2C_ISR_ALERT                            13
#define I2C_ISR_BUSY                             15
#define I2C_ISR_DIR                              16
#define I2C_ISR_ADDCODE                          17
/*
 * I2C receive data register (I2C_RXDR)
 */
#define I2C_RXDR_RXDATA                          0
/*
 * I2C transmit data register (I2C_RXDR)
 */
#define I2C_TXDR_TXDATA                          0
/*
 * I2C timing register (I2C_TIMINGR)
 */
#define I2C_TIMINGR_SCLL                         0
#define I2C_TIMINGR_SCLH                         8
#define I2C_TIMINGR_SDADEL                       16
#define I2C_TIMINGR_SCLDEL                       20
#define I2C_TIMINGR_PRESC                        28
/*
 * I2C interrupt clear register (I2C_ICR)
 */
#define I2C_ICR_ADDRCF                           3
#define I2C_ICR_NACKCF                           4
#define I2C_ICR_STOPCF                           5
#define I2C_ICR_BERRCF                           8
#define I2C_ICR_ARLOCF                           9
#define I2C_ICR_OVRCF                            10
#define I2C_ICR_PECCF                            11
#define I2C_ICR_TIMOUTCF                         12
#define I2C_ICR_ALERTCF                          13
/*
 *System clock switch status
 */
#define RCC_CFGR_SWS_MSI                          0
#define RCC_CFGR_SWS_HSI                          1
#define RCC_CFGR_SWS_HSE                          2
#define RCC_CFGR_SWS_PLL                          3
/*
 * **********************************************************************************************
 *  Bit position definitions of USARTx peripheral
 * **********************************************************************************************
 */
/*
 * USART control register 1 (USART_CR1)
 */
#define USART_CR1_UE                              0
#define USART_CR1_UESM                            1
#define USART_CR1_RE                              2
#define USART_CR1_TE                              3
#define USART_CR1_IDLEIE                          4
#define USART_CR1_RXNEIE                          5
#define USART_CR1_TCIE                            6
#define USART_CR1_TXEIE                           7
#define USART_CR1_PEIE                            8
#define USART_CR1_PS                              9
#define USART_CR1_PCE                             10
#define USART_CR1_WAKE                            11
#define USART_CR1_M0                              12
#define USART_CR1_MME                             13
#define USART_CR1_CMIE                            14
#define USART_CR1_OVER8                           15
#define USART_CR1_DEDT                            16
#define USART_CR1_DEAT                            21
#define USART_CR1_RTOIE                           26
#define USART_CR1_EOBIE                           27
#define USART_CR1_M1                              28
/*
 * USART control register 2 (USART_CR2)
 */
#define USART_CR2_ADDM7                           4
#define USART_CR2_LBDL                            5
#define USART_CR2_LBDIE                           6
#define USART_CR2_LBCL                            8
#define USART_CR2_CPHA                            9
#define USART_CR2_CPOL                            10
#define USART_CR2_CLKEN                           11
#define USART_CR2_STOP                            12
#define USART_CR2_LINEN                           14
#define USART_CR2_SWAP                            15
#define USART_CR2_RXINV                           16
#define USART_CR2_TXINV                           17
#define USART_CR2_DATAINV                         18
#define USART_CR2_MSBFIRST                        19
#define USART_CR2_ABREN                           20
#define USART_CR2_ABRMOD                          21
#define USART_CR2_RTOEN                           23
#define USART_CR2_ADD30                           24
#define USART_CR2_ADD74                           28
/*
 * USART control register 3 (USART_CR3)
 */
#define USART_CR3_EIE                             0
#define USART_CR3_IREN                            1
#define USART_CR3_IRLP                            2
#define USART_CR3_HDSEL                           3
#define USART_CR3_NACK                            4
#define USART_CR3_SCEN                            5
#define USART_CR3_DMAR                            6
#define USART_CR3_DMAT                            7
#define USART_CR3_RTSE                            8
#define USART_CR3_CTSE                            9
#define USART_CR3_CTSIE                           10
#define USART_CR3_ONEBIT                          11
#define USART_CR3_OVRDIS                          12
#define USART_CR3_DDRE                            13
#define USART_CR3_DEM                             14
#define USART_CR3_DEP                             15
#define USART_CR3_SCARCNT0                        17
#define USART_CR3_SCARCNT1                        18
#define USART_CR3_SCARCNT2                        19
#define USART_CR3_WUS0                            20
#define USART_CR3_WUS1                            21
#define USART_CR3_WUFIE                           22
#define USART_CR3_UCESM                           23
#define USART_CR3_TCBGTIE                         24
/*
 * USART baud rate register (USART_BRR)
 */
#define USART_BRR_BRR                             0
/*
 *USART guard time and prescaler register (USART_GTPR)
 */
#define USART_GTPR_PSC                             0
#define USART_GTPR_GT                              8
/*
 * USART receiver timeout register (USART_RTOR)
 */
#define USART_RTOR_RTO150                          0
#define USART_RTOR_RTO2316                         16
#define USART_RTOR_BLEN                            24
/*
 * USART request register (USART_RQR)
 */
#define USART_RQR_ABRRQ                           0
#define USART_RQR_SBKRQ                           1
#define USART_RQR_MMRQ                            2
#define USART_RQR_RXFRQ                           3
#define USART_RQR_TXFRQ                           4
/*
 * USART interrupt and status register (USART_ISR)
 */
#define USART_ISR_PE                              0
#define USART_ISR_FE                              1
#define USART_ISR_NF                              2
#define USART_ISR_ORE                             3
#define USART_ISR_IDLE                            4
#define USART_ISR_RXNE                            5
#define USART_ISR_TC                              6
#define USART_ISR_TXE                             7
#define USART_ISR_LBDF                            8
#define USART_ISR_CTSIF                           9
#define USART_ISR_CTS                             10
#define USART_ISR_RTOF                            11
#define USART_ISR_EOBF                            12
#define USART_ISR_ABRE                            14
#define USART_ISR_ABRF                            15
#define USART_ISR_BUSY                            16
#define USART_ISR_CMF                             17
#define USART_ISR_SBKF                            18
#define USART_ISR_RWU                             19
#define USART_ISR_WUF                             20
#define USART_ISR_TEACK                           21
#define USART_ISR_REACK                           22
#define USART_ISR_TCBGT                           25
/*
 * USART interrupt flag clear register (USART_ICR)
 */
#define USART_ICR_PECF                             0
#define USART_ICR_FECF                             1
#define USART_ICR_NCF                              2
#define USART_ICR_ORECF                            3
#define USART_ICR_IDLECF                           4
#define USART_ICR_TCCF                             6
#define USART_ICR_TCBGTCF                          7
#define USART_ICR_LBDCF                            8
#define USART_ICR_CTSCF                            9
#define USART_ICR_RTOCF                            11
#define USART_ICR_EOBCF                            12
#define USART_ICR_CMCF                             17
#define USART_ICR_WUCF                             20
/*
 * USART receive data register (USART_RDR)
 */
#define USART_RDR_RDR                               0
/*
 * USART transmit data register (USART_TDR)
 */
#define USART_TDR_TDR                               0
/*
 * Some generic macros
 */
#define ENABLE                    1
#define DISABLE                   0
#define SET                       ENABLE
#define RESET                     DISABLE
#define GPIO_PIN_SET              SET
#define GPIO_PIN_RESET            RESET
#define FLAG_RESET                RESET
#define FLAG_SET                  SET

#include "stm32l475xx_gpio_driver.h"
#include "stm32l475xx_spi_driver.h"
#include "stm32l475xx_i2c_driver.h"
#include "stm32l475xx_usart_driver.h"
#include "stm32l475xx_rcc_driver.h"
#endif /* INC_STM32L475XX_H_ */
