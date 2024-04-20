/*
 * stm32l475xx_rcc_driver.c
 */
#include "stm32l475xx_rcc_driver.h"

// software to control the division factor of the AHB clock
uint16_t AHB_preScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
// software to control the division factor of the APB1 clock
uint16_t APB1_preScaler[4] = {2, 4, 8, 16};
// software to control the division factor of the APB2 clock
uint16_t APB2_preScaler[4] = {2, 4, 8, 16};
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apbp1;

	clksrc = ((RCC ->CFGR >> 2) & 0x3);

	if(clksrc == RCC_CFGR_SWS_MSI){
		//SystemClk = RCC_MSIOutClk();
	}else if(clksrc == RCC_CFGR_SWS_HSI){
		SystemClk = 16000000;
	}else if(clksrc == RCC_CFGR_SWS_HSE){
		SystemClk = 8000000;
	}else if(clksrc == RCC_CFGR_SWS_PLL){
		//SystemClk = RCC_PLLOutClk();
	}

	//: AHB prescaler
	temp = ((RCC ->CFGR >> 4) & 0xf);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_preScaler[temp - 8];
	}

	//: APB1 prescaler
	temp = ((RCC ->CFGR >> 8) & 0x7);

	if(temp < 4){
		apbp1 = 1;
	}else{
		apbp1 = APB1_preScaler[temp - 4];
	}

	pclk1 = ((SystemClk / ahbp) / apbp1);

return pclk1;
}

uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, temp, ahbp, apbp2;

	clksrc = ((RCC ->CFGR >> 2) & 0x3);

	if(clksrc == RCC_CFGR_SWS_MSI){
		//SystemClk = RCC_MSIOutClk();
	}else if(clksrc == RCC_CFGR_SWS_HSI){
		SystemClk = 16000000;
	}else if(clksrc == RCC_CFGR_SWS_HSE){
		SystemClk = 8000000;
	}else if(clksrc == RCC_CFGR_SWS_PLL){
		//SystemClk = RCC_PLLOutClk();
	}

	//: AHB prescaler
	temp = ((RCC ->CFGR >> 4) & 0xf);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_preScaler[temp - 8];
	}

	//: APB2 prescaler
	temp = ((RCC ->CFGR >> 11) & 0x7);

	if(temp < 4){
		apbp2 = 1;
	}else{
		apbp2 = APB2_preScaler[temp - 4];
	}

	pclk2 = ((SystemClk / ahbp) / apbp2);

	return pclk2;
}
