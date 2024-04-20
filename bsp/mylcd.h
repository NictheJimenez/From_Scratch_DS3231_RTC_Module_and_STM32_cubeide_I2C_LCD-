/*
 * lcd.h
 */

#ifndef MYLCD_H_
#define MYLCD_H_

#include "stm32l475xx.h"

/*bps exposed apis*/
void lcd_init(void);
void lcd_send_command(uint8_t);
void lcd_print_char(uint8_t);
void lcd_display_clear(void);
void lcd_display_return_home(void);
void lcd_print_string(char*);

/*Application configurable items*/
#define LCD_GPIO_PORTA         GPIOA
#define LCD_GPIO_PORTB         GPIOB
#define LCD_GPIO_RS           GPIO_PIN_NO_2   //PB2   ARD.D8
#define LCD_GPIO_RW           GPIO_PIN_NO_4   //PA4   ARD.D7
#define LCD_GPIO_EN           GPIO_PIN_NO_3   //PA3   ARD.D4
#define LCD_GPIO_D4           GPIO_PIN_NO_4   //PC4   ARD.A1
#define LCD_GPIO_D5           GPIO_PIN_NO_3   //PC3   ARD.A2
#define LCD_GPIO_D6           GPIO_PIN_NO_2   //PC2   ARD.A3
#define LCD_GPIO_D7           GPIO_PIN_NO_1   //PC1   ARD.A4

/*LCD commands */
#define LCD_CMD_4DL_2N_5X8F        0X28
#define LCD_CMD_DON_CURON          0X0E
#define LCD_CMD_INCADD             0X06
#define LCD_CMD_DIS_CLEAR          0X01
#define LCD_CMD_DIS_RETURN_HOME    0X02
#endif /* MYLCD_H_ */
