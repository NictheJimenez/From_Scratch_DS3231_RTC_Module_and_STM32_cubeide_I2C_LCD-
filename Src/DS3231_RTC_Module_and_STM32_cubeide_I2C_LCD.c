
#include <stdio.h>
#include <ds3231.h>
#include "lcdx.h"

I2C_Handle_t I2C1Handle;
static void mdelay(uint32_t cnt);
	/*
	 * ***************************************************************************
	 * Function        STM32 pin       AF         Connector             Pin number => Signal or Label
	 * ***************************************************************************
	 * I2C1_SCL          PB8            4            CN1                   ARD.D15-I2C1_SCL
	 * I2C1_SDA          PB9            4            CN1                   ARD.D14-I2C1_SDA
	 * ***************************************************************************
	 */
/*
 * ********************************************************< I2C1 >*****************************************************************
 *                                       Pin Name               Signal or Label                              Feature / Comment
 *  I2C1_SCL                             PB8                    ARD.D15-I2C1_SCL                             I2C1_SCL
 *  I2C1_SDA                             PB9                    ARD.D14-I2C1_SDA                             I2C1_SDA
 * ALT_FUNCTION MODE  (AF4) ->        4
 * **********************************************************************************************************************************
 */
/*
void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;
	//memset(&I2CPins,0,sizeof(I2CPins));												/*  (GpioLedI => GPIO Led Internal from the board),*/

	//I2CPins.pGPIOx = GPIOB;

	//I2CPins.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_ALTFN;
	//I2CPins.GPIO_PinConf.GPIO_PinAltFunMode = 4;
	//I2CPins.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//I2CPins.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//I2CPins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//I2C1_SCL
	//I2CPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_8;
	//GPIO_PeriClockControl(GPIOB, ENABLE);
	//GPIO_Init(&I2CPins);
	//I2C1_SDA
	//I2CPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_9;
	//GPIO_PeriClockControl(GPIOB, ENABLE);
	//GPIO_Init(&I2CPins);

//}
void RCC_config(void){
	RCC -> CR |= ( 1 << 0);
	RCC -> CR |= ( 1 << 1);
	RCC -> CR |= ( 1 << 8);

	RCC -> CFGR |= ( 0x1 << 0);
	RCC -> CFGR |= ( 0x1 << 2);

	RCC -> APB1ENR[0] |= ( 0x1 << 28);
	RCC -> APB2ENR |= ( 0x1 << 0);
	//RCC -> CR |=( 1 << 10);
}

void I2C1_Inits(void){

	memset(&I2C1Handle,0,sizeof(I2C1Handle));
	I2C1Handle.pI2Cx = I2C1;

	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	//I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}
void GPIO_LCDInit(void){
	/**********************Internal Button => GpioBtnI*************************/
// LCD_GPIO_RS           GPIO_PIN_2   //PB2   ARD.D8
// LCD_GPIO_RW           GPIO_PIN_4   //PA4   ARD.D7
// LCD_GPIO_EN           GPIO_PIN_3   //PA3   ARD.D4
// LCD_GPIO_D4           GPIO_PIN_4   //PC4   ARD.A1
// LCD_GPIO_D5           GPIO_PIN_3   //PC3   ARD.A2
// LCD_GPIO_D6           GPIO_PIN_2   //PC2   ARD.A3
// LCD_GPIO_D7           GPIO_PIN_1   //PC1   ARD.A4
	/*************************************************************************/
	GPIO_Handle_t GpioPA, GpioPB, GpioPC;
	memset(&GpioPA,0,sizeof(GpioPA));
	memset(&GpioPB,0,sizeof(GpioPB));
	memset(&GpioPC,0,sizeof(GpioPC));
    /************    PORT B************************/
	GpioPB.pGPIOx = GPIOB;

	GpioPB.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioPB.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioPB.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioPB.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioPB.GPIO_PinConf.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioPB);
	/************    PORT A************************/
	GpioPA.pGPIOx = GPIOA;

	GpioPA.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioPA.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioPA.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioPA.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioPA.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_RW;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioPA);

	GpioPA.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_EN;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioPA);
	/************    PORT C************************/
	GpioPC.pGPIOx = GPIOC;

	GpioPC.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioPC.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioPC.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioPC.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioPC.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_D4;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioPC);

	GpioPC.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_D5;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioPC);

	GpioPC.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_D6;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioPC);

	GpioPC.GPIO_PinConf.GPIO_PinNumber =LCD_GPIO_D7;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioPC);
}

#define SYSTICK_TIM_CLK 16000000UL
//The SysTick calibration value is set to 0x4000270F, which gives a reference time base of
//1 ms with the SysTick clock set to 10 MHz (max fHCLK/8).
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    //calculation of reload value
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
   *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
   *pSRVR |= count_value;

    //do some settings
   *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
   *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
   *pSCSR |= ( 1 << 0); //enables the counter

}

char * get_day_of_week(uint8_t i)
{
	char* days[] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
	return days[i-1];
}

void number_to_string(uint8_t num, char* buf)
{
	if(num < 10){
		buf[0] = '0';
		buf[1] = num + 48;

	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num / 10) + 48;
		buf[1] = (num % 10) + 48;
	}
}
//hh:mm:ss
char* time_to_string(RTC_time_t * rtc_time)
{
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time -> hours, buf);
	number_to_string(rtc_time -> minutes, &buf[3]);
	number_to_string(rtc_time -> seconds, &buf[6]);
	buf[8] = '\0';

	return buf;
}

//dd/mm/yy
char* date_to_string(RTC_date_t * rtc_date)
{
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(rtc_date -> date, buf);
	number_to_string(rtc_date -> month, &buf[3]);
	number_to_string(rtc_date -> year, &buf[6]);
	buf[8] = '\0';

	return buf;
}
static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000 ); i++);
}

int main (void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	RCC_config();
	GPIO_LCDInit();
	lcd_initx();


	mdelay(2000);
	ds3231_init();

	current_date.day = THURSDAY;
	current_date.date = 11;
	current_date.month = 4;
	current_date.year = 24;

	current_time.hours = 4;
	current_time.minutes = 25;
	current_time.seconds = 41;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds3231_set_current_date(&current_date);
	ds3231_set_current_time(&current_time);

	while(1){

	 /* Infinite loop */

		lcd_print_string(" ");
		lcd_print_string("Nicthe Jimenez");
		mdelay(2000);
		lcd_display_clear();
		lcd_display_return_home();
	    //#endif

		//printf("RTC test \n");
		//if(ds3231_init()){
			//printf("RTC init has failed\n");
		//	while(1);
		//}

		//init_systick_timer(1);

		//ds3231_get_current_date(&current_date);
		//ds3231_get_current_time(&current_time);

		//char *am_pm;
/*
		if(current_time.time_format |= TIME_FORMAT_24HRS)
		{
			am_pm = (current_time.time_format) ? "PM" : "AM";
			//printf("Current time = %s %s\n", time_to_string(&current_time),am_pm); // 04:25:41 PM
		}else{
		//	printf("Current time = %s \n", time_to_string(&current_time)); // 04:25:41
		}

		//11/04/24 <thursday>
		//printf("Current date = %s <%s> \n",date_to_string(&current_date), get_day_of_week(current_date.day));
*/
	}

		return 0;
}

	void SysTick_Handler(void)
	{
		RTC_time_t current_time;
		RTC_date_t current_date;

		ds3231_get_current_time(&current_time);

		char *am_pm;

		if(current_time.time_format |= TIME_FORMAT_24HRS)
		{
			am_pm = (current_time.time_format) ? "PM" : "AM";
			printf("Current time = %s %s\n", time_to_string(&current_time),am_pm); // 04:25:41 PM
		}else{
			printf("Current time = %s \n", time_to_string(&current_time)); // 04:25:41
		}

		ds3231_get_current_date(&current_date);
			//15/01/21 <friday>
		printf("Current date = %s <%s> \n",date_to_string(&current_date), get_day_of_week(current_date.day));

}
