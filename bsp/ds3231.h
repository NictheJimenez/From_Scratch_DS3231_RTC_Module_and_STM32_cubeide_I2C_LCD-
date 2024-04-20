/*
 * DS3231.h
 * DS3231 serial time clock is a low power,
it's a fully binary coded decimal clock and calendar.
Full binary-coded decimal(BCD)
 */

#ifndef DS3231_H_
#define DS3231_H_

#include "stm32l475xx.h"
/*
 *   Application configurable items pines GPIOs with AF
	 * ***************************************************************************
	 * Function        STM32 pin       AF         Connector             Pin number => Signal or Label
	 * ***************************************************************************
	 * I2C1_SCL          PB8            4            CN1                   ARD.D15-I2C1_SCL
	 * I2C1_SDA          PB9            4            CN1                   ARD.D14-I2C1_SDA
	 * ***************************************************************************
 */
#define DS3231_I2C                    I2C1
#define DS3231_I2C_GPIO_PORT          GPIOB
#define DS3231_I2C_SDA_PIN            GPIO_PIN_NO_9
#define DS3231_I2C_SCL_PIN            GPIO_PIN_NO_8
#define DS3231_I2C_SPEED              I2C_SCL_SPEED_SM
#define DS3231_I2C_PUPD               GPIO_PU //Yes internal pull ups
//#define DS3231_I2C_PUPD             GPIO_NO_PUPD //No internal pull ups then I use external pull ups R=3.3kohm
/*
 * Registers addresses
 */
#define DS3231_ADDR_SEC                0X00
#define DS3231_ADDR_MIN                0X01
#define DS3231_ADDR_HRS                0X02
#define DS3231_ADDR_DAY                0X03
#define DS3231_ADDR_DATE               0X04
#define DS3231_ADDR_MONTH              0X05
#define DS3231_ADDR_YEAR               0X06
#define DS3231_ADDR_ALARM_1_SEC        0X07
#define DS3231_ADDR_ALARM_1_MIN        0X08
#define DS3231_ADDR_ALARM_1_HRS        0X09
#define DS3231_ADDR_ALARM_1_DY_DT      0X0A
#define DS3231_ADDR_ALARM_2_MIN        0X0B
#define DS3231_ADDR_ALARM_2_HRS        0X0C
#define DS3231_ADDR_ALARM_2_DY_DT      0X0D
#define DS3231_ADDR_CONTROL            0X0E
#define DS3231_ADDR_CONTROL_STATUS     0X0F
#define DS3231_ADDR_AGING_OFFSET       0X10
#define DS3231_ADDR_MSB_TEMP           0X11
#define DS3231_ADDR_LSB_TEMP           0X12

/*
 * Control Register (0Eh)
 */
#define CTL_EOSC                        7      //Control Register (0Eh) Bit 7: Enable Oscillator (EOSC). When set to logic 0 the oscillator is started
#define CTL_BBSQW                       6      //Battery-Backed Square-Wave Enable When set to logic 1 with INTCN = 0 and VCC < VPF this bit enables the square wave.
#define CTL_CONV                        5      //Setting this bit to 1 forces the temperature sensor to convert the temperature into digital code and execute the TCXO algorithm to update the capacitance array to the oscillator
#define CTL_RS2                         4      //SQUARE-WAVE OUTPUT FREQUENCY
#define CTL_RS1                         3
#define CTL_INTCN                       2      // Interrupt Control (INTCN)
#define CTL_A2IE                        1      //Bit 1: Alarm 2 Interrupt Enable (A2IE)
#define CTL_A1IE                        0      //Bit 0: Alarm 1 Interrupt Enable (A1IE)

/*
 * Status Register (0Fh)
 */
#define STATUS_OSF                      7
#define STATUS_EN32KHZ                  3
#define STATUS_BSY                      2
#define STATUS_A2F                      1
#define STATUS_A1F                      0

#define TIME_FORMAT_12HRS_AM            0
#define TIME_FORMAT_12HRS_PM            1
#define TIME_FORMAT_24HRS               2

#define DS3231_I2C_ADDRESS              0XD0 // 0x68 DS3231 ADDRESSES IS 1101000

#define SUNDAY                          1
#define MONDAY                          2
#define TUESDAY                         3
#define WEDNESDAY                       4
#define THURSDAY                        5
#define FRIDAY                          6
#define SATURDAY                        7

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
	uint8_t day_date;
	uint8_t day_date_format;
}RTC_alarm1_time_date_t;

typedef struct
{
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
	uint8_t day_date;
	uint8_t day_date_format;
}RTC_alarm2_time_date_t;

/*
 * Functions prototypes DS3231
 */

uint8_t ds3231_init(void);

void ds3231_set_current_time(RTC_time_t *);
void ds3231_get_current_time(RTC_time_t *);

void ds3231_set_current_date(RTC_date_t *);
void ds3231_get_current_date(RTC_date_t *);


#endif /* DS3231_H_ */
