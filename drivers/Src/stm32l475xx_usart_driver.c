/*
 *                                                             stm32l475xx_usart_drivers.c
 */
#include "stm32l475xx_usart_driver.h"
static char* decToHex(int32_t decNum);
static char* HexToBin(char hexnum);
/*
 * ***********************************************************************************************************
 * Peripheral Clock setup
 * ***********************************************************************************************************
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi ==ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5){
			UART5_PCLK_EN();
	}else
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		}
	}
}
/*
 * ***********************************************************************************************************
 * Init and De-init
 * ***********************************************************************************************************
 */
 void USART_Init(USART_Handle_t *pUSARTHandle){
	 //Temporary variable
	 	uint32_t tempreg=0;

	 /******************************** Configuration of CR1******************************************/

	 	//Implement the code to enable the Clock for given USART peripheral
	 	USART_PeriClockControl(pUSARTHandle -> pUSARTx, ENABLE);

	 	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	 	if( pUSARTHandle -> USARTConfig.USART_Mode == USART_MODE_ONLY_RX){
	 		pUSARTHandle -> pUSARTx -> CR1 &= ~(1 << USART_CR1_TE);

	 		//Implement the code to enable the Receiver bit field
	 		tempreg |= (1 << USART_CR1_RE);
	 	}else if(pUSARTHandle -> USARTConfig.USART_Mode == USART_MODE_ONLY_TX){
	 		pUSARTHandle -> pUSARTx -> CR1 &= ~(1 << USART_CR1_RE);

	 		//Implement the code to enable the Transmitter bit field
	 		tempreg |= (1 << USART_CR1_TE);
	 	}else if(pUSARTHandle -> USARTConfig.USART_Mode == USART_MODE_TXRX){

	 		//Implement the code to enable the both Transmitter and Receiver bit fields
	 		tempreg |= ( ( 1 <<  USART_CR1_RE) | ( 1 <<  USART_CR1_TE) );
	 	}

	     //Implement the code to configure the Word length configuration item
	 	tempreg |= pUSARTHandle ->USARTConfig.USART_WordLength << USART_CR1_M1;

	     //Configuration of parity control bit fields
	 	if( pUSARTHandle -> USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN){

	 		//Implement the code to enale the parity control
	 		tempreg |= ( 1 << USART_CR1_PCE);

	 		//Implement the code to enable EVEN parity
	 		tempreg &= ~(1 << USART_CR1_PS);

	 		//Not required because by default EVEN parity will be selected once you enable the parity control
	 	}else if(pUSARTHandle -> USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD){

	 		//Implement the code to enable the parity control
	 	    tempreg |= ( 1 << USART_CR1_PCE);

	 	    //Implement the code to enable ODD parity
	 	    tempreg |= ( 1 << USART_CR1_PS);
	 	}

	    //Program the CR1 register
	 	pUSARTHandle -> pUSARTx -> CR1 = tempreg;

	 /******************************** Configuration of CR2******************************************/

	 	tempreg=0;

	 	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	 	tempreg |= pUSARTHandle -> USARTConfig.USART_NoOfStopBits << USART_CR2_STOP;

	 	//Program the CR2 register
	 	pUSARTHandle -> pUSARTx -> CR2 = tempreg;

	 /******************************** Configuration of CR3******************************************/

	 	tempreg=0;

	 	//Configuration of USART hardware flow control
	 	if(pUSARTHandle -> USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){

	 		//Implement the code to enable CTS flow control
	 		tempreg &= ~( 1 << USART_CR3_RTSE);
	 		tempreg |= ( 1 << USART_CR3_CTSE);

	 	}else if (pUSARTHandle -> USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	 	{
	 		//Implement the code to enable RTS flow control
	 		tempreg &= ~( 1 << USART_CR3_CTSE);
	 		tempreg |= ( 1 << USART_CR3_RTSE);

	 	}else if (pUSARTHandle -> USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	 	{
	 		//Implement the code to enable both CTS and RTS Flow control
	 		tempreg |= ( ( 1 <<  USART_CR3_RTSE) | ( 1 <<  USART_CR3_CTSE) );
	 	}

	 	pUSARTHandle -> pUSARTx -> CR3 = tempreg;

	 /******************************** Configuration of BRR(Baudrate register)******************************************/
	 	//Implement the code to configure the baud rate
	    USART_SetBaudRate(pUSARTHandle -> pUSARTx, pUSARTHandle -> USARTConfig.USART_Baud);
}
void USART_DeInit(USART_RegDef_t *pUSARTx){
	    //APB2 peripheral reset register (RCC_APB2RSTR) : USART1
		if(pUSARTx == USART1){
			USART1_REG_RESET();
		//APB1 peripheral reset register 1 (RCC_APB1RSTR1) :USART2, USART3, UART4, UART5
		}else if(pUSARTx == USART2){
			USART2_REG_RESET();
		}else if(pUSARTx == USART3){
			USART3_REG_RESET();
		}else if(pUSARTx == UART4){
			UART4_REG_RESET();
		}else if(pUSARTx == UART5){
			UART5_REG_RESET();
		}
}
/*
 * ***********************************************************************************************************
 * Data Send and Receive
 * ***********************************************************************************************************
 */
 void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
		uint16_t *pdata;
	   //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len-1; i++){
			//Implement the code to wait until TXE flag is set in the SR
			while(! USART_GetFlagStatus(pUSARTHandle -> pUSARTx, USART_FLAG_TXE));

	         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
			if(pUSARTHandle -> USARTConfig.USART_WordLength== USART_WORDLEN_9BITS){
				//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*) pTxBuffer;
				pUSARTHandle -> pUSARTx ->TDR = (*pdata & (uint16_t)0x01FF);

				//check for USART_ParityControl
				if(pUSARTHandle -> USARTConfig.USART_ParityControl== USART_PARITY_DISABLE){
					//No parity is used in this transfer. so, 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					pTxBuffer++;
					pTxBuffer++;
				}else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pTxBuffer++;
				}
			}else
			{
				//This is 8bit data transfer
				pUSARTHandle -> pUSARTx -> TDR = (*pTxBuffer  & (uint8_t)0xFF);

				//Implement the code to increment the buffer address
				pTxBuffer++;
			}
		}

		//Implement the code to wait till TC flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len){

}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){

}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

}
/*
 * ***********************************************************************************************************
 * IRQ Configuration and ISR handling
 * ***********************************************************************************************************
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. First lets find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t Shift_Amount = (8 * IPRx_Section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << Shift_Amount);
}
 void USART_IRQHandling(USART_Handle_t *pHandle){

}
/*
 * **********************************************************************************************************
 * Other Peripheral Control APIs
 * **********************************************************************************************************
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pUSARTx -> CR1 |= (1 << USART_CR1_UE);
	}else{
		pUSARTx -> CR1 &= ~(1 << USART_CR1_UE);
	}
}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){
	if(pUSARTx -> ISR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){

}
/*
 * ***********************************************************************************************************
 * USART_SetBaudRate
 * ***********************************************************************************************************
 */
void USART_SetBaudRate(USART_Handle_t *pHandle, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
  uint16_t tempreg=0, temp = 0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pHandle -> pUSARTx== USART1){
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else if(pHandle -> pUSARTx == USART2 || pHandle -> pUSARTx == USART3 || pHandle -> pUSARTx == UART4 || pHandle -> pUSARTx == UART5)
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pHandle -> pUSARTx -> CR1 & (1 << USART_CR1_OVER8)){
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((2 * PCLKx) / (BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ( PCLKx / BaudRate);
  }
 // char* ptrHexNum = decToHex(usartdiv);

  if(pHandle -> pUSARTx -> CR1 & ( 1 << USART_CR1_OVER8)){
	  //OVER8 = 1 , over sampling by 8
	  temp = (usartdiv & 0xF);
	  temp >>= 1;
	  tempreg |= temp;

	  temp = 0;
	  temp = (usartdiv & 0xFFF0);
	  tempreg |= temp;

   }else
   {
	   //over sampling by 16
	   tempreg |= usartdiv;

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR


  //copy the value of tempreg in to BRR register
  pHandle -> pUSARTx -> BRR = tempreg;
}
// function to convert decimal to hexadecimal
static char* decToHex(int32_t decNum){
    // char array to store hexadecimal number
    char hexaDeciNum[50];
    // counter for hexadecimal number array
    int i = 0;
    while (decNum != 0)
    {
        /* temporary variable to
        store right most digit*/
        int temp = 0;
        // Get the right most digit
        temp = decNum % 16;
        // check if temp < 10
        if (temp < 10)
        {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else
        {
            hexaDeciNum[i] = temp + 55;
            i++;
        }
        decNum = decNum / 16; // get the quotient
    }
    char* ptrhexaDeciNum = hexaDeciNum;
    return  ptrhexaDeciNum;
}

/*
 * **********************************************************************************************************
 * Application callback
 * **********************************************************************************************************
 */
//void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
