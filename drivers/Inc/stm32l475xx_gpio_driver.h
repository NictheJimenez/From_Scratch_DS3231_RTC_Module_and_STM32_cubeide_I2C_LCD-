/*
 * stm32l475xx_gpio_driver.h
 *
 *  Created on: May 20, 2022
 *      Author: nicth
 */

#ifndef INC_STM32L475XX_GPIO_DRIVER_H_
#define INC_STM32L475XX_GPIO_DRIVER_H_

#include "stm32l475xx.h"

/*
 * GPIO Pin Configuration structure
 */
typedef struct{
uint8_t GPIO_PinNumber;
uint8_t GPIO_PinMode;                                          /*< possible values from @GPIO_PIN_MODES >*/
uint8_t GPIO_PinOPType;                                        /*< possible values from @GPIO_PIN_OP_TYPE >*/
uint8_t GPIO_PinSpeed;                                         /*< possible values from @GPIO_SPEED_MODES >*/
uint8_t GPIO_PinPuPdControl;                                   /*< possible values from @GPIO_PIN_PUPD >*/
uint8_t GPIO_PinAltFunMode;                                    /*< possible values from @GPIO_PIN_ALT_FN >*/
}GPIO_PinConfig_t;
/*
 * Handle structure for GPIOx pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;                                                        /*<GPIOx Base Address of the GPIO port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConf;                                                /*<GPIO Pin configuration settings>*/
}GPIO_Handle_t;
/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0                                         0
#define GPIO_PIN_NO_1                                         1
#define GPIO_PIN_NO_2                                         2
#define GPIO_PIN_NO_3                                         3
#define GPIO_PIN_NO_4                                         4
#define GPIO_PIN_NO_5                                         5
#define GPIO_PIN_NO_6                                         6
#define GPIO_PIN_NO_7                                         7
#define GPIO_PIN_NO_8                                         8
#define GPIO_PIN_NO_9                                         9
#define GPIO_PIN_NO_10                                        10
#define GPIO_PIN_NO_11                                        11
#define GPIO_PIN_NO_12                                        12
#define GPIO_PIN_NO_13                                        13
#define GPIO_PIN_NO_14                                        14
#define GPIO_PIN_NO_15                                        15
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes Input-mode/General-purpose-output-mode/Alternate-function-mode/Analog-mode types
 */
#define GPIO_MODE_IN                                           0
#define GPIO_MODE_OUT                                          1
#define GPIO_MODE_ALTFN                                        2
#define GPIO_MODE_ANALOG                                       3
#define GPIO_MODE_IT_FT                                        4
#define GPIO_MODE_IT_RT                                        5
#define GPIO_MODE_IT_FRT                                       6
/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output  push-pull/open-drain types
 */
#define GPIO_OP_TYPE_PP                                        0
#define GPIO_OP_TYPE_OD                                        1
/*
 * @GPIO_SPEED_MODES
 * GPIO pin possible output speed  Low-speed/ Medium-speed/High-speed/Very-high-speed types
 */
#define GPIO_SPEED_LOW                                         0
#define GPIO_SPEED_MEDIUM                                      1
#define GPIO_SPEED_FAST                                        2
#define GPIO_SPEED_HIGH                                        3
/*
 * @GPIO_PIN_PUPD
 * GPIO pin possible pull-up/pull-down types
 */
#define GPIO_NO_PUPD                                           0
#define GPIO_PU                                                1
#define GPIO_PD                                                2

/*
 * ***************************************************************************************************************************************
 *                                APIs supported by GPIO driver
 *
 * ***************************************************************************************************************************************
 */
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);
/*
 * Init - DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);
/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber,uint8_t DataTx);
void GPIO_WriteToOuputPort(GPIO_RegDef_t * pGPIOx, uint16_t DataTx);
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





























#endif /* INC_STM32L475XX_GPIO_DRIVER_H_ */
