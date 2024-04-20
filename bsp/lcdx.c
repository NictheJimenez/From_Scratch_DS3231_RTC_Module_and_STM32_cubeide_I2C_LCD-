/*
 * lcd.c
 */
#include <lcdx.h>


static void write_4_bits(uint8_t);
static void lcd_enable(void);
static void mdelay(uint8_t);
static void udelay(uint8_t);

void lcd_send_command(uint8_t cmd)
{
	/* RS = 0 for LCD command */
	//HAL_GPIO_WritePin(GPIOB, LCD_GPIO_RS, RESET);
	GPIO_WriteToOuputPin(GPIOB, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RnW = 0, writing to ldc */
	//HAL_GPIO_WritePin(GPIOA, LCD_GPIO_RW, RESET);
	GPIO_WriteToOuputPin(LCD_GPIO_PORTA, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(cmd >> 4); /* Higher nibble */

	write_4_bits(cmd & 0x0F); /* Lower nibble */
}
/*
 * This function sends a character to the LCD
 * Here we used 4 bit parallel data transmission.
 * First higher nibble of the data will be sent on to the data lines D4,D5,D6,D7
 * Then lower nibble of the data will be set on the data lines D4,D5,D6,D7
 *
 */
void lcd_print_char(uint8_t data)
{
	/* RS = 1 for LCD user data */
	//HAL_GPIO_WritePin(GPIOB, LCD_GPIO_RS, SET);
	/* RnW = 0, writing to ldc */
	//HAL_GPIO_WritePin(GPIOA, LCD_GPIO_RW, RESET);
	GPIO_WriteToOuputPin(LCD_GPIO_PORTA, LCD_GPIO_RS, GPIO_PIN_SET);
	/* RnW = 0, writing to ldc */
	GPIO_WriteToOuputPin(LCD_GPIO_PORTA, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data >> 4); /* Higher nibble */

	write_4_bits(data & 0x0F); /* Lower nibble */
}
void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t)*message++);
	}
	while (*message != '\0');
}
void lcd_initx(void)
{
	/*Application configurable items*/
	/*
	#define LCD_GPIO_RS           GPIO_PIN_NO_2   //PB2   ARD.D8
	#define LCD_GPIO_RW           GPIO_PIN_NO_4   //PA4   ARD.D7
	#define LCD_GPIO_EN           GPIO_PIN_NO_3   //PA3   ARD.D4
	#define LCD_GPIO_D4           GPIO_PIN_NO_4   //PC4   ARD.A1
	#define LCD_GPIO_D5           GPIO_PIN_NO_3   //PC3   ARD.A2
	#define LCD_GPIO_D6           GPIO_PIN_NO_2   //PC2   ARD.A3
	#define LCD_GPIO_D7           GPIO_PIN_NO_1   //PC1   ARD.A4
		*/
	//1. Configure the gpios pins which are used for lcd connections
	/*
	HAL_GPIO_WritePin(GPIOB, LCD_GPIO_RS, RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_GPIO_RW, RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_GPIO_EN, RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D4, RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D5, RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D6, RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D7, RESET);
    */
	GPIO_WriteToOuputPin(GPIOB, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOA, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOA, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. Do the LCD initialization

	mdelay(40);
	//HAL_Delay(50);

	/* RS = 0 for LCD command */
	//HAL_GPIO_WritePin(GPIOB, LCD_GPIO_RS, RESET);

	/* RnW = 0, writing to ldc */
	//HAL_GPIO_WritePin(GPIOA, LCD_GPIO_RW, RESET);
	/* RS = 0, for LCD command */
	GPIO_WriteToOuputPin(GPIOB, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RnW = 0, writing to ldc */
	GPIO_WriteToOuputPin(GPIOA, LCD_GPIO_RW, GPIO_PIN_RESET);
                        // B7 B6 B5 B4
	write_4_bits(0x3); // 0  0  1   1

	mdelay(5);

	write_4_bits(0x3); // 0  0  1   1

	udelay(150);

	write_4_bits(0x3); // 0  0  1   1

	write_4_bits(0x2); // 0  0  1   0

	//function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	//display ON and cursor ON
	lcd_send_command(LCD_CMD_DON_CURON);

	lcd_display_clear();

	lcd_send_command(LCD_CMD_INCADD);
}
/**
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      lcd_send_command((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_command((column |= 0x80));
      break;
    default:
      break;
  }
}
void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}


/* Writes 4 bits of data/command on to D4,D5,D6,D7 lines*/
static void write_4_bits(uint8_t value)
{
	/*
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D4, ((value >> 0) & 0x1));
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D5, ((value >> 1) & 0x1));
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D6, ((value >> 2) & 0x1));
	HAL_GPIO_WritePin(GPIOC, LCD_GPIO_D7, ((value >> 3) & 0x1));
	*/
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D4, ((value >> 0) & 0x1));
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D5, ((value >> 1) & 0x1));
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D6, ((value >> 2) & 0x1));
	GPIO_WriteToOuputPin(GPIOC, LCD_GPIO_D7, ((value >> 3) & 0x1));

	lcd_enable();
}

void lcd_display_clear(void)
{
	//display clear
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}
 /* Cursor returns to home position */
/*void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

	mdelay(2);
}
*/
static void lcd_enable(void)
{
	//HAL_GPIO_WritePin(GPIOA, LCD_GPIO_EN, GPIO_PIN_SET);
	GPIO_WriteToOuputPin(LCD_GPIO_PORTA, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	//HAL_GPIO_WritePin(GPIOA, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOuputPin(LCD_GPIO_PORTA, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100); /* Execution time > 37 micro seconds */
}
static void mdelay(uint8_t cnt)
{
	for(uint32_t i = 0; i< (cnt * 1000); i++);

}
static void udelay(uint8_t cnt)
{
	for(uint32_t i = 0; i< (cnt * 1); i++);

}

