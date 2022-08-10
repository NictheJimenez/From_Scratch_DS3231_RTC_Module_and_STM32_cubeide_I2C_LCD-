/*
 *  stm32l475xx_i2c_drivers.h

 */

#ifndef INC_STM32L475XX_I2C_DRIVERS_H_
#define INC_STM32L475XX_I2C_DRIVERS_H_
#include "stm32l475xx.h"
/*
 *  Configuration structure for I2Cx peripheral
 */
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;
/*
 * Handle structure for I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer; /* To store the app. Tx buffer address*/
	uint8_t *pRxBuffer; /*To store the app. Rx buffer address*/
	uint32_t TxLen;     /*To store Tx len*/
	uint32_t RxLen;     /*To store Rx Len */
	uint8_t TxRxState;  /*To store communication state*/
	uint8_t DevAddr;    /*To store slave/device address*/
	uint32_t Rxsize;    /*To store Rx size*/
	uint8_t Sr;         /* To store repeated start value*/
}I2C_Handle_t;

/*
 *  I2C application state
 */
#define I2C_READY                0
#define I2C_BUSY_IN_RX           1
#define I2C_BUSY_IN_TX           2
/*
 *  @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM         100000
#define I2C_SCL_SPEED_FM4K       400000
#define I2C_SCL_SPEED_FM2K       200000
/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE           0
#define I2C_ACK_DISABLE          1
/*
 *
 */
/*
 * I2C related status flag definitions
 */
#define I2C_FLAG_TXE                             (1 << I2C_ISR_TXE)
#define I2C_FLAG_TXIS                            (1 << I2C_ISR_TXIS)
#define I2C_FLAG_RXNE                            (1 << I2C_ISR_RXNE)
#define I2C_FLAG_ADDR                            (1 << I2C_ISR_ADDR)
#define I2C_FLAG_NACKF                           (1 << I2C_ISR_NACKF)
#define I2C_FLAG_STOPF                           (1 << I2C_ISR_STOPF)
#define I2C_FLAG_TC                              (1 << I2C_ISR_TC)
#define I2C_FLAG_TCR                             (1 << I2C_ISR_TCR)
#define I2C_FLAG_BERR                            (1 << I2C_ISR_BERR)
#define I2C_FLAG_ARLO                            (1 << I2C_ISR_ARLO)
#define I2C_FLAG_OVR                             (1 << I2C_ISR_OVR)
#define I2C_FLAG_PECERR                          (1 << I2C_ISR_PECERR)
#define I2C_FLAG_TIMEOUT                         (1 << I2C_ISR_TIMEOUT)
#define I2C_FLAG_ALERT                           (1 << I2C_ISR_ALERT)
#define I2C_FLAG_BUSY                            (1 << I2C_ISR_BUSY)
#define I2C_FLAG_DIR                             (1 << I2C_ISR_DIR)
#define I2C_FLAG_ADDCODE                         (1 << I2C_ISR_ADDCODE)
/*
 *
 */
/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);
/*
 * Master Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t RorWTransfer, uint8_t AutoendEnorDi, uint8_t RestartdEnorDi);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t RorWTransfer);
/*
 * Data Send and Receive
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t RorWTransfer, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t RorWTransfer, uint8_t Sr);
/*
 * Slave receive/ request data
 */
void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle,uint8_t data);
void I2C_SlaveReceiveData(I2C_Handle_t* pI2Cx, uint8_t *pRxBuffer);
/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/*
 * other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx,uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx,uint8_t EnorDi);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t * pI2CHandle,uint8_t I2C_EventApp);

//OTHERS
#define I2C_EV_TX_CMPLT                      1
#define I2C_WRITE_TRANSFER_MM                0
#define I2C_READ_TRANSFER_MM                 1
#define I2C_READ_TRANSFER_SM                 0
#define I2C_WRITE_TRANSFER_SM                1
#define I2C_AUTOEND_DISABLE_MM               0
#define I2C_AUTOEND_ENABLE_MM                1
#endif /* INC_STM32L475XX_I2C_DRIVERS_H_ */
