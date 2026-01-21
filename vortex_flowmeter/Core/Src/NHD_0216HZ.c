/*----------------------------------------------------------------------------
 NHD0216HZ LCD C/C++ file
 *----------------------------------------------------------------------------*/

#include "main.h"
#include "NHD_0216HZ.h"
#ifdef ENABLE_SPI
#define LCD_SPI &hspi2

// Define your LCD control pins
#define LCD_CS_GPIO_Port SPI2_SS_GPIO_Port
#define LCD_CS_Pin       SPI2_SS_Pin

void delay_us(uint16_t us);

// ----------- SPI + LCD Functions ------------//

/*This function is used to  initialize LCD
 * No parameter
 * No return type
 */
void init_spi(void) {
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS is high
}

/*This function is used to perform a SPI transfer
 * Data is the parameter
 * No return type
 */
void write_spi(uint8_t data) {
	HAL_SPI_Transmit(LCD_SPI, &data, 1, HAL_MAX_DELAY); //transfer data over SPI

}

/*This function is used to send a nibble(4bits) and mode(command or data)
 * Nibble and mode are the parameters
 * No return type
 */
void write_4bit(uint8_t nibble, uint8_t mode) {
	uint8_t data;
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); //CS low
	data = nibble | ENABLE | mode; //4bit nibble + enable bit + mode
	write_spi(data); //send data
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); //CS high

	delay_us(1);

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);//CS low
	data = nibble & ~ENABLE; //LCD latches data on high to low transition of enable pin
	write_spi(data);//send data
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);//CS high
}

/*This function is used send instructions to LCD
 * Command is the parameter
 * No return type
 */
void write_cmd(uint8_t cmd) {
	uint8_t hi_n = cmd & 0xF0; //extract the upper 4 bits of command
	uint8_t lo_n = (cmd << 4) & 0xF0;//extract the lower 4 bits of command
	write_4bit(hi_n, COMMAND_MODE);//send the upper nibble
	write_4bit(lo_n, COMMAND_MODE);//send the lower nibble
}

/*This function is used send data to LCD
 * Data is the parameter
 * No return type
 */
void write_data(char data) {
	uint8_t hi_n = data & 0xF0;//extract the upper 4 bits of data
	uint8_t lo_n = (data << 4) & 0xF0;//extract the lower 4 bits of data
	write_4bit(hi_n, DATA_MODE);//send the upper nibble
	write_4bit(lo_n, DATA_MODE);//send the lower nibble
}

/*This function is set the cursor on the LCD
 * Column and row are the parameters
 * No return type
 */
void set_cursor(uint8_t column, uint8_t row) {
	uint8_t addr = (row * LINE_LENGTH) + column; //calculate the address of given row/column
	addr |= TOT_LENGTH; //add total length
	write_cmd(addr); //send the address as a command
}

/*This function is used to print string on the LCD
 *String is the parameter
 * No return type
 */
void print_lcd(const char *str) {
	while (*str) {
		write_data(*str++); //write the string
	}
}

/*This function is used to initialize the LCD via
 * No parameter
 * No return type
 */
void init_lcd(void) {
	HAL_Delay(40);

	write_cmd(0x30); //wake up
	delay_us(40);

	write_cmd(0x20); //4bit mode
	delay_us(40);
	write_cmd(0x20);
	delay_us(40);

	write_cmd(0x0C); //display ON
	delay_us(40);

	write_cmd(0x01); //clear
	HAL_Delay(2);

	write_cmd(0x06); //entry mode
	delay_us(40);

	write_cmd(0x28); //function set
	HAL_Delay(40);

	set_cursor(0, 0); //set cursor at start
}

/*This function is used for microsecond delay
 * The micro second value is the parameter
 * No return type
 */
void delay_us(uint16_t us) {
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim1);  // Use a timer TIM1
	while ((__HAL_TIM_GET_COUNTER(&htim1) - start) < us);
}
#endif
