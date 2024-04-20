/*
 * stm32l475xx_usarts_driver.h
 *
 *  Created on: Aug 20, 2022
 *      Author: nicth
 */

#ifndef INC_STM32L475XX_USART_DRIVER_H_
#define INC_STM32L475XX_USART_DRIVER_H_
#include "stm32l475xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct{
	uint8_t USART_Mode;
	uint8_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;
/*
 *Handle structure for USARTx peripheral
 */
typedef struct{
	USART_RegDef_t *pUSARTx;       /*<This hold the base address of USARTx(x = num ) peripheral>*/
	USART_Config_t USARTConfig;
}USART_Handle_t;
/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX                 0
#define USART_MODE_ONLY_RX                 1
#define USART_MODE_TXRX                    2
/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000
/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD                 2
#define USART_PARITY_EN_EVEN                1
#define USART_PARITY_DISABLE                0
/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS                 0
#define USART_WORDLEN_9BITS                 1
/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1                    0
#define USART_STOPBITS_0_5                  1
#define USART_STOPBITS_2                    2
#define USART_STOPBITS_1_5                  3
/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE         	0
#define USART_HW_FLOW_CTRL_CTS          	1
#define USART_HW_FLOW_CTRL_RTS          	2
#define USART_HW_FLOW_CTRL_CTS_RTS      	3
/*
 * USART related status flag definitions
 */
#define USART_FLAG_PE                                (1 << USART_ISR_PE)
#define USART_FLAG_FE                                (1 << USART_ISR_FE)
#define USART_FLAG_NF                                (1 << USART_ISR_NF)
#define USART_FLAG_ORE                               (1 << USART_ISR_ORE)
#define USART_FLAG_IDLE                              (1 << USART_ISR_IDLE)
#define USART_FLAG_RXNE                              (1 << USART_ISR_RXNE)
#define USART_FLAG_TC                                (1 << USART_ISR_TC)
#define USART_FLAG_TXE                               (1 << USART_ISR_TXE)
#define USART_FLAG_LBDF                              (1 << USART_ISR_LBDF)
#define USART_FLAG_CTSIF                             (1 << USART_ISR_CTSIF)
#define USART_FLAG_CTS                               (1 << USART_ISR_CTS)
#define USART_FLAG_RTOF                              (1 << USART_ISR_RTOF)
#define USART_FLAG_EOBF                              (1 << USART_ISR_EOBF)
#define USART_FLAG_ABRE                              (1 << USART_ISR_ABRE)
#define USART_FLAG_ABRF                              (1 << USART_ISR_ABRF)
#define USART_FLAG_BUSY                              (1 << USART_ISR_BUSY)
#define USART_FLAG_CMF                               (1 << USART_ISR_CMF)
#define USART_FLAG_SBKF                              (1 << USART_ISR_SBKF)
#define USART_FLAG_RWU                               (1 << USART_ISR_RWU)
#define USART_FLAG_WUF                               (1 << USART_ISR_WUF)
#define USART_FLAG_TEACK                             (1 << USART_ISR_TEACK)
#define USART_FLAG_REACK                             (1 << USART_ISR_REACK)
#define USART_FLAG_TCBGT                             (1 << USART_ISR_TCBGT)
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);
/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);
/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_Handle_t *pHandle, uint32_t BaudRate);
/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32L475XX_USART_DRIVER_H_ */
