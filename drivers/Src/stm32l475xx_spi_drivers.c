/*
 *                                                             stm32l475xx_spi_drivers.c
 */
#include "stm32l475xx_spi_driver.h"
/*
 *some private helper function prototype
 */
static void SPI_TXE_Interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_Interrupt_handle(SPI_Handle_t *pSPIHandle);
/*
 * **************************************************************************************************************************************
 *                                          Peripheral Clock setup
 ****************************************************************************************************************************************
 *
 *                  This function enables or disables peripheral clock for the given GPIO port
 *                  Param <base address of the GPIO peripheral><ENABLE or DISABLE macros>
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if(EnorDi ==ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
}

/*
 * *******************************************************************************************************************************
 *                                                            SPI Init
 *********************************************************************************************************************************
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
//first lets configure the SPI_CR1 register
	uint32_t tempRegCR1 = 0, tempRegCR2 = 0;
	// enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle -> pSPIx, ENABLE);
	//1. configure the device mode
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//2.  configure the bus config
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		tempRegCR1 &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempRegCR1 |= 1 << SPI_CR1_BIDIMODE;
	}else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDI mode should be cleared
		tempRegCR1 &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempRegCR1 |= 1 <<  SPI_CR1_RXONLY;
	}
    //3. configure the spi serial clock speed(baud rate)
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_SClkSpeed << SPI_CR1_BR;
	//4. configure the DFF
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. configure the CPOL
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_CPol << SPI_CR1_CPOL;
	//6. configure the CPHA
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_CPha << SPI_CR1_CPHA;
	tempRegCR1 |= pSPIHandle -> SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle -> pSPIx -> CR1 = tempRegCR1;
	//DFF configure DS
	tempRegCR2 |= pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR2_DS;
	pSPIHandle -> pSPIx -> CR2 = tempRegCR2;
}
/*
 * **********************************************************************************************************************************
 *                                                             SPI_DeInit
 * ***********************************************************************************************************************************
  */
void SPI_DeInit(SPI_RegDef_t* pSPIx){
	//  SPI1 reset by APB2 peripheral reset register (RCC_APB2RSTR)
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	//	SPI2-SPI3 reset by APB1 peripheral reset register 1 (RCC_APB1RSTR1[1])
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}
 uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName){
	if(pSPIx -> SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t * pTxBuffer, uint16_t Len){
while(Len >0){
	//1. wait until TxE is set
	while((SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET));
		//while(!(pSPIx -> SR & (1 << 1)));
		//2. Check the DFF bit in CR2 DFF (Data size -> DS)
		if((pSPIx -> CR1 & (1 << SPI_CR1_DFF))){
			// 16 bit DFF
			//1. load the data in to the DR
			pSPIx -> DR = *((uint16_t*) pTxBuffer);
			Len --;
			Len --;
			(uint16_t*) pTxBuffer ++;
		}else{
			// 8 bit DFF
			*((uint8_t*)&pSPIx -> DR) = * pTxBuffer;
			Len --;
			pTxBuffer ++;
		}
	}
}
void SPI_ReciveData(SPI_RegDef_t* pSPIx, uint8_t * pRxBuffer, uint32_t Len){
	while(Len >0){
		//1. wait until RxNE is set
		while(!(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET));
		//while(!(pSPIx -> SR & (1 << 1)));
			//2. Check the DFF bit in CR1 DFF (Data size -> DS)
			if((pSPIx -> CR1 & (1 << SPI_CR1_DFF))){
				// 16 bit DFF
				//1. load the data from DR to the RxBuffer address
				*((uint16_t*) pRxBuffer) = pSPIx -> DR;
				Len --;
				Len --;
				(uint16_t*) pRxBuffer ++;
			}else{
				// 8 bit DFF
				*pRxBuffer = pSPIx -> DR;
				Len --;
				pRxBuffer ++;
			}
	}
}
/*
 * SPI_PeripheralControl
 */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/*
 * SPI_SSIConfig
 */
void SPI_SSIConfig(SPI_RegDef_t* pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx -> CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/*
 * SPI_SSOEConfig
 */
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx -> CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx -> CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
/*
 * SPI_NSSPConfig
 */
void SPI_NSSPConfig(SPI_RegDef_t* pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx -> CR2 |= (1 << SPI_CR2_NSSP);
	}else{
		pSPIx -> CR2 &= ~(1 << SPI_CR2_NSSP);
	}
}
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(IRQNumber <= 31){
				        // program ISER0 register
				*NVIC_ISER0 |= (1 << IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64){
				        // program ISER1 register
				*NVIC_ISER1 |= (1 << IRQNumber % 32);
			}else if(IRQNumber >= 64 && IRQNumber < 96){
				       // program ISER2 register
				*NVIC_ISER2 |= (1 << IRQNumber % 64);
			}
		}else{
			if(IRQNumber <= 31){
				        // program ICER0 register
				*NVIC_ICER0 &= ~(1 << IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64){
				        // program ICER1 register
				*NVIC_ICER1 &= ~(1 << IRQNumber % 32);
			}else if(IRQNumber >= 64 && IRQNumber < 96){
				        // program ICER2 register
				*NVIC_ICER2 &= ~(1 << IRQNumber % 64);
			}
		}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. First lets find out the IPR register
		uint8_t IPRx = IRQNumber / 4;
		uint8_t IPRx_Section = IRQNumber % 4;
		uint8_t Shift_Amount = (8 * IPRx_Section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << Shift_Amount);
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
//First lets check TXE
	uint8_t temp1, temp2;
	temp1 = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//handle interrupt handle
		SPI_TXE_Interrupt_handle(pSPIHandle);

	}
	//check for RXNE
	temp1 = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		//handle interrupt handle
		SPI_RXNE_Interrupt_handle(pSPIHandle);
	}
	//check for OVR flag
	temp1 = pSPIHandle -> pSPIx -> SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle -> pSPIx -> CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2){
		//handle interrupt handle
		SPI_OVR_Interrupt_handle(pSPIHandle);

	}


}
/*
 * Data Send and Receive IT
 */
uint8_t SPI_SendDataIT(SPI_Handle_t * pSPIHandle, uint8_t * pTxBuffer, uint16_t Len){
	uint8_t state = pSPIHandle ->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. Save the TX buffer address and len information in some global variables
		pSPIHandle -> pTxBuffer = pTxBuffer;
		pSPIHandle -> TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that no other can take over
		//same SPI peripheral until transmission is over
		pSPIHandle -> TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_TXEIE);
		//4. Data transmission will be handled by the ISR code
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t * pSPIHandle, uint8_t * pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle ->RxState;
	if(state != SPI_BUSY_IN_RX){
		//1. Save the TX buffer address and len information in some global variables
		pSPIHandle -> pRxBuffer = pRxBuffer;
		pSPIHandle -> RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that no other can take over
		//same SPI peripheral until transmission is over
		pSPIHandle -> RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_RXNEIE);
		//4. Data transmission will be handled by the ISR code
	}
	return state;
}
/*
 *some private helper function implementation
 */
static void SPI_TXE_Interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the DFF bit in CR2 DFF (Data size -> DS)
	if((pSPIHandle -> pSPIx -> CR1 & (1 << SPI_CR1_DFF))){
		// 16 bit DFF
		//1. load the data in to the DR
		pSPIHandle -> pSPIx -> DR = *((uint16_t*) pSPIHandle -> pTxBuffer);
		pSPIHandle -> TxLen --;
		pSPIHandle -> TxLen --;
		(uint16_t*) pSPIHandle -> pTxBuffer ++;
	}else{
		// 8 bit DFF
	    pSPIHandle -> pSPIx -> DR = *pSPIHandle -> pTxBuffer;
		pSPIHandle -> TxLen --;
		pSPIHandle -> pTxBuffer ++;
	}
	if(! pSPIHandle -> TxLen){
		//TxLen is zero, so close the spi transmission and inform the application that
		//Tx is over.
		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void SPI_RXNE_Interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the DFF bit in CR2 DFF (Data size -> DS)
		if((pSPIHandle -> pSPIx -> CR1 & (1 << SPI_CR1_DFF))){
			// 16 bit DFF
			//1. load the data in to the DR
			*((uint16_t*) pSPIHandle -> pRxBuffer) = pSPIHandle -> pSPIx -> DR;
			pSPIHandle -> RxLen -= 2;
			pSPIHandle -> pRxBuffer --;
			pSPIHandle -> pRxBuffer --;
		}else{
			// 8 bit DFF
		    *pSPIHandle -> pRxBuffer = pSPIHandle -> pSPIx -> DR;
			pSPIHandle -> RxLen --;
			pSPIHandle -> pRxBuffer --;
		}
		if(! pSPIHandle -> TxLen){
			//TxLen is zero, so close the spi transmission and inform the application that
			//Tx is over.
			//this prevents interrupts from setting up of TXE flag
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}
static void SPI_OVR_Interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle -> TxState != SPI_BUSY_IN_TX){
	temp = pSPIHandle -> pSPIx ->DR;
	temp = pSPIHandle -> pSPIx -> SR;
	}
	(void)temp;
	//2. inform the application
	//SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
void SPI_CloseTransmission(SPI_Handle_t * pSPIHandle){
	pSPIHandle -> pSPIx -> CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle -> pTxBuffer = NULL;
	pSPIHandle -> TxLen= 0;
	pSPIHandle -> TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t * pSPIHandle){
	pSPIHandle -> pSPIx -> CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle -> pRxBuffer = NULL;
	pSPIHandle -> RxLen= 0;
	pSPIHandle -> RxState = SPI_READY;
}
void SPI_ClearOVRFlag(SPI_RegDef_t * pSPIx){
	uint8_t temp;
	temp = pSPIx -> DR;
	temp = pSPIx ->SR;
	(void)temp;
}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t * pSPIHandle,uint8_t SPI_EventApp){
	//This is a weak implementation, the application may override this function
}
