/*
 * lcd.c
 *
 *  Created on: Jun 16, 2024
 *      Author: nguye
 */

#include "lcd.h"
#include "ssd1306.h"
#include "fonts.h"
//#include "mode_anchor_active_object.h"
#include "stdio.h"


void lcd_clear(){
	SSD1306_Clear();
}

void lcd_display_APP_NAME_(lcd_mode_display_t lcd_mode){
switch(lcd_mode){
	case DISPLAY_MODE_ANCHOR:
	{
		SSD1306_GotoXY(0, 0);
		#if (ANCHOR_TYPE == 'A')
		SSD1306_Puts("UWB-ANA",&Font_7x10, 1);
		#elif (ANCHOR_TYPE == 'B')
		SSD1306_Puts("UWB-ANB",&Font_7x10, 1);
		#elif (ANCHOR_TYPE == 'C')
		SSD1306_Puts("UWB-ANC",&Font_7x10, 1);
		#elif (ANCHOR_TYPE == 'D')
		SSD1306_Puts("UWB-ANC",&Font_7x10, 1);
		#endif
		SSD1306_UpdateScreen();
	}
		break;
	case DISPLAY_MODE_TAG:
	{
		SSD1306_GotoXY(0, 0);
		#if (TAG_TYPE == 'TA')
		SSD1306_Puts("UWB-TGA",&Font_7x10, 1);
		#elif (TAG_TYPE == 'TB')
		SSD1306_Puts("UWB-TGB",&Font_7x10, 1);
		#elif (TAG_TYPE == 'TC')
		SSD1306_Puts("UWB-TGC",&Font_7x10, 1);
		#elif (TAG_TYPE == 'TD')
		SSD1306_Puts("UWB-TGD",&Font_7x10, 1);
		#endif
		SSD1306_UpdateScreen();
	}
		break;
}

}


void lcd_display_MENU_(lcd_mode_display_t lcd_mode){
	lcd_display_APP_NAME_(lcd_mode);
	switch(lcd_mode){
		case DISPLAY_MODE_ANCHOR:
		{
			SSD1306_GotoXY(60, 0);
			SSD1306_Puts("RSI: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 20);
			SSD1306_Puts("TE: ",&Font_7x10, 1);
			SSD1306_GotoXY(60, 20);
			SSD1306_Puts("HU: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 35);
			SSD1306_Puts("PE: ",&Font_7x10, 1);
			SSD1306_GotoXY(60, 35);
			SSD1306_Puts("DI: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 50);
			SSD1306_Puts("ST: ",&Font_7x10, 1);
			SSD1306_UpdateScreen();
		}
			break;
		case DISPLAY_MODE_TAG:
		{
			SSD1306_GotoXY(60, 0);
			SSD1306_Puts("RSI: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 20);
			SSD1306_Puts("TE: ",&Font_7x10, 1);
			SSD1306_GotoXY(60, 20);
			SSD1306_Puts("HU: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 35);
			SSD1306_Puts("PE: ",&Font_7x10, 1);
			SSD1306_GotoXY(60, 35);
			SSD1306_Puts("DI: ",&Font_7x10, 1);
			SSD1306_GotoXY(0, 50);
			SSD1306_Puts("ST: ",&Font_7x10, 1);
			SSD1306_UpdateScreen();
		}
			break;
	}

}

void lcd_display_parameters(float temp, float press, float humi, float rssi, float distance){
	char so_str[20];
	static float tt;
	SSD1306_GotoXY(20, 20);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_GotoXY(20, 35);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_GotoXY(80, 20);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_UpdateScreen();

	SSD1306_GotoXY(20, 20);
	sprintf(so_str, "%.2f", temp);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_GotoXY(80, 20);
	sprintf(so_str, "%.2f", humi);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_GotoXY(20, 35);
	sprintf(so_str, "%.1f", press);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_UpdateScreen();
	tt+=1.1;
}

void lcd_display_bme_parameters(float temp, float press, float humi){
	char so_str[20];
	SSD1306_GotoXY(20, 20);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_GotoXY(20, 35);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_GotoXY(80, 20);
	SSD1306_Puts("    ",&Font_7x10, 1);
	SSD1306_UpdateScreen();

	SSD1306_GotoXY(20, 20);
	sprintf(so_str, "%.2f", temp);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_GotoXY(80, 20);
	sprintf(so_str, "%.2f", humi);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_GotoXY(20, 35);
	sprintf(so_str, "%.1f", press);
	SSD1306_Puts(so_str,&Font_7x10, 1);

	SSD1306_UpdateScreen();
}

void lcd_display_status(char* str){
	SSD1306_GotoXY(20, 50);
	SSD1306_Puts("                                            ",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(20, 50);
	SSD1306_Puts(str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void lcd_display_polling(char* str){
	SSD1306_GotoXY(70, 50);
	SSD1306_Puts("                ",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(70, 50);
	SSD1306_Puts(str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}


void lcd_display_ret(char* str){
	SSD1306_GotoXY(104, 50);
	SSD1306_Puts("                ",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(104, 50);
	SSD1306_Puts(str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}


void lcd_display_error(lcd_dw_error_code_t dw_error){
	switch(dw_error){
	case DW_ERROR_POLLING:
	{
		static uint8_t failpollingcnt=0;
		static uint8_t textrxpollingblind = 0;
		if(failpollingcnt==5){
			switch(textrxpollingblind){
			case 0:
				lcd_display_status("Polling False");
				textrxpollingblind++;
				break;
			case 1:
				lcd_display_status("No Rx Device");
				textrxpollingblind++;
				break;
			case 2:
				lcd_display_status("Press Button 1");
				textrxpollingblind++;
				break;
			case 3:
				lcd_display_status("To Try Again");
				textrxpollingblind=0;
				break;
			}
			failpollingcnt = 0;
		}
		else{
			failpollingcnt++;
		}
	}
		break;
	case DW_ERROR_STS:
	{
		static uint8_t failstscnt=0;
		static uint8_t textstsblind = 0;
		if(failstscnt==5){
			switch(textstsblind){
			case 0:
				lcd_display_status("Bad STS");
				textstsblind++;
				break;
			case 1:
				lcd_display_status("Press Button 1");
				textstsblind++;
				break;
			case 2:
				lcd_display_status("To Try Again");
				textstsblind=0;
				break;
			}
			failstscnt = 0;
		}
		else{
			failstscnt++;
		}
	}
		break;
	case DW_ERROR_NO_MATCHING_MSG:
	{
		static uint8_t nomatchingcnt=0;
		static uint8_t textnomatchingblind = 0;
		if(nomatchingcnt==5){
			switch(textnomatchingblind){
			case 0:
				lcd_display_status("No Matching");
				textnomatchingblind++;
				break;
			case 1:
				lcd_display_status("MSG ID");
				textnomatchingblind=0;
				break;
			}
			nomatchingcnt = 0;
		}
		else{
			nomatchingcnt++;
		}
	}
		break;
	case DW_ERROR_OVERLOAD_MSG:
	{
		static uint8_t overloadmsgcnt=0;
		static uint8_t textoverloadmsgblind = 0;
		if(overloadmsgcnt==5){
			switch(textoverloadmsgblind){
			case 0:
				lcd_display_status("Overload MSG");
				textoverloadmsgblind++;
				break;
			case 1:
				lcd_display_status("MSG ID");
				textoverloadmsgblind=0;
				break;
			}
			overloadmsgcnt = 0;
		}
		else{
			overloadmsgcnt++;
		}
	}
		break;
	}
}

static uint8_t decrease_number = 0;

void reset_decrease_number(){
	decrease_number = 4;
}

void lcd_display_decrease_number(){
	char so_str[20];
	sprintf(so_str, "%d",decrease_number);
	SSD1306_GotoXY(80, 0);
	SSD1306_Puts("  ",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(80, 0);
	SSD1306_Puts(so_str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	decrease_number--;
	if(decrease_number == 0){
		SSD1306_GotoXY(80, 0);
		SSD1306_Puts("  ",&Font_7x10, 1);
	}
}

void lcd_sending_polling_tx(char* str){
	SSD1306_GotoXY(90, 50);
	SSD1306_Puts("                ",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(90, 50);
	SSD1306_Puts(str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

