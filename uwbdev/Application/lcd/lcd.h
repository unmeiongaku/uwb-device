/*
 * lcd.h
 *
 *  Created on: Jun 15, 2023
 *      Author: nguye
 */
#include "ssd1306.h"
#include "fonts.h"

#ifndef APPLICATION_LCD_LCD_H_
#define APPLICATION_LCD_LCD_H_

typedef enum{
	DISPLAY_MODE_ANCHOR,
	DISPLAY_MODE_TAG,
}lcd_mode_display_t;

typedef enum{
	DW_ERROR_POLLING,
	DW_ERROR_STS,
	DW_ERROR_NO_MATCHING_MSG,
	DW_ERROR_OVERLOAD_MSG,
}lcd_dw_error_code_t;

void lcd_clear();
void lcd_display_APP_NAME_(lcd_mode_display_t lcd_mode);
void lcd_display_MENU_(lcd_mode_display_t lcd_mode);
void lcd_display_parameters(float temp, float press, float humi, float rssi, float distance);
void lcd_display_bme_parameters(float temp, float press, float humi);
void lcd_display_status(char* str);
void lcd_display_polling(char* str);
void lcd_display_ret(char* str);
void lcd_display_error(lcd_dw_error_code_t dw_error);

void reset_decrease_number();
void lcd_display_decrease_number();
#endif /* APPLICATION_LCD_LCD_H_ */
