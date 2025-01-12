/*
 * app_main.c
 *
 *  Created on: Jan 12, 2025
 *      Author: nguye
 */


#include "lcd.h"
#include "user_define.h"
#include "delay_us.h"
#include "app_main.h"
#include "buzzer.h"
#include "timer.h"
#include "bme_callback.h"

TID(gtid_led_callback);
TID(gtid_bme280);
TID(gtid_lcd_display);

static bme_proobject_t bme_A0s;

static void led_callback(void* ctx);
static void bme280_callback(void* ctx);
static void bme_proobject_event_dispatcher(bme_proobject_t *const bme_mobj,bme_event_t const *const bme_e);
static void lcd_display_callback(void* ctx);

/*Logo Drone*/
const unsigned char drone_logo[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0xE0, 0x40, 0xFC, 0x00, 0x00, 0x3F, 0x02, 0x07, 0xE0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x0F, 0xF0, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x0F, 0xFF, 0xFF, 0xF0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x7F, 0xFE, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x3F, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x0F, 0xF0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x03, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x03, 0x80, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x03, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x1F, 0xF8, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x1F, 0xF8, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x1F, 0xF8, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x1D, 0xB8, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x1F, 0xF8, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x1F, 0xF0, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x30, 0x73, 0xC3, 0x8E, 0x0F, 0x1F, 0xE7, 0x1F, 0x00, 0xF8, 0x3E, 0x01, 0xF1, 0xFF, 0x80,
0x00, 0x70, 0x63, 0xC3, 0x9E, 0x1F, 0x1F, 0xE7, 0x11, 0xC3, 0x1C, 0x33, 0x87, 0x1C, 0x18, 0x00,
0x00, 0x70, 0x63, 0xE3, 0x1E, 0x1F, 0x1C, 0x06, 0x10, 0xC6, 0x04, 0x21, 0x8C, 0x0C, 0x18, 0x00,
0x00, 0x70, 0xE3, 0xE3, 0x1E, 0x36, 0x1C, 0x06, 0x30, 0xCC, 0x06, 0x21, 0x98, 0x06, 0x10, 0x00,
0x00, 0x60, 0xE3, 0x63, 0x3F, 0x36, 0x18, 0x0E, 0x30, 0xCC, 0x06, 0x63, 0x18, 0x06, 0x10, 0x00,
0x00, 0x60, 0xE7, 0x77, 0x33, 0x76, 0x3F, 0xCE, 0x33, 0x88, 0x06, 0x7E, 0x10, 0x06, 0x30, 0x00,
0x00, 0xE0, 0xC7, 0x37, 0x33, 0x66, 0x3F, 0xCE, 0x3E, 0x18, 0x06, 0x63, 0x30, 0x06, 0x30, 0x00,
0x00, 0xE1, 0xC6, 0x36, 0x33, 0x4E, 0x38, 0x0C, 0x63, 0x18, 0x04, 0x41, 0xB0, 0x0C, 0x30, 0x00,
0x00, 0xE1, 0xC6, 0x3E, 0x73, 0xCE, 0x30, 0x0C, 0x61, 0x18, 0x0C, 0x41, 0xB0, 0x0C, 0x20, 0x00,
0x00, 0xE3, 0x86, 0x1E, 0x63, 0x8C, 0x30, 0x1C, 0x61, 0x0C, 0x18, 0x41, 0x18, 0x18, 0x60, 0x00,
0x00, 0x7F, 0x0E, 0x1E, 0x63, 0x8C, 0x7F, 0x9C, 0x41, 0x0E, 0x30, 0xC7, 0x0C, 0x70, 0x60, 0x00,
0x00, 0x3E, 0x0E, 0x1E, 0xE3, 0x0C, 0x7F, 0x9C, 0x41, 0x83, 0xE0, 0xFC, 0x07, 0xC0, 0x60, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void led_callback(void* ctx){
	HAL_GPIO_TogglePin(MCU_LED_0_GPIO_Port, MCU_LED_0_Pin);
}

static void bme280_callback(void* ctx){
	static uint8_t bme_proevent;
	static uint16_t bmetickcnt = 0;
	bme_proobject_user_event_t bme_ue;
	static bme_proobject_tick_event_t bme_te;
	bme_proevent = bmegetnextstatesig();
	/*Make Event*/
	switch(bme_proevent){
	case BME_NEXT_SIG_DEFINE:
	{
		bme_ue.super.sig = BME_NEXT_SIG;
	}
		break;
	}
	bmeresetnextstatesig();
	bme_proobject_event_dispatcher(&bme_A0s,&bme_ue.super);
	//4. dispatch the time tick event for every  UWB_PERIOD_CALLBACK*5ms
	if(bmetickcnt==1){
		bme_te.super.sig = BME_TICK_SIG;
		bme_proobject_event_dispatcher(&bme_A0s,&bme_te.super);
		bmetickcnt = 0;
	}
	else{
		bmetickcnt++;
	}
}

static void bme_proobject_event_dispatcher(bme_proobject_t *const bme_mobj,bme_event_t const *const bme_e){

  bme_event_status_t bme_status;
  bme_proobject_state_t bme_source, bme_target;

  bme_source = bme_mobj->bme_active_state;
  bme_status = bme_proobject_state_machine(bme_mobj,bme_e);

  if(bme_status == BME_EVENT_TRANSITION){
	bme_target = bme_mobj->bme_active_state;
    bme_event_t bme_ee;
    //1. run the exit action for the source state
    bme_ee.sig = BME_EXIT;
    bme_mobj->bme_active_state = bme_source;
    bme_proobject_state_machine(bme_mobj,&bme_ee);

    //2. run the entry action for the target state
    bme_ee.sig = BME_ENTRY;
    bme_mobj->bme_active_state = bme_target;
    bme_proobject_state_machine(bme_mobj,&bme_ee);
  }
}

static void lcd_display_callback(void* ctx){
	static uint8_t lcdcnt;
	if(lcdcnt==20){
		lcdcnt = 0;
		lcd_display_bme_parameters(bme_A0s.temperature, bme_A0s.pressure, bme_A0s.humidity);
	}
	else{
		lcdcnt++;
	}
}

void app_init(){
	  /*init Delay timer*/
		HAL_TIM_Base_Start(&TIM_DELAY_US);
		buzzer_sys_start();

		/*LCD*/
		SSD1306_Init();
		SSD1306_Clear();
		SSD1306_DrawBitmap(0, 0, drone_logo, 128, 64, 1);
		SSD1306_UpdateScreen();
		delay_ms(500);

		HAL_GPIO_WritePin(MCU_LED_0_GPIO_Port, MCU_LED_0_Pin, RESET);
		/*LCD Display Menu*/
		HAL_GPIO_WritePin(MCU_BUZZER_GPIO_Port, MCU_BUZZER_Pin, SET);
		delay_ms(50);
		HAL_GPIO_WritePin(MCU_BUZZER_GPIO_Port, MCU_BUZZER_Pin, RESET);
		lcd_clear();
#if DEFAULT_MODE_SELECT == 0
		lcd_display_MENU_(DISPLAY_MODE_ANCHOR);
#elif DEFAULT_MODE_SELECT == 1
		lcd_display_MENU_(DISPLAY_MODE_TAG);
#endif

		/*Init BME280*/
		bme_proobject_init(&bme_A0s);
		gtid_bme280 = timer_register_callback(bme280_callback, BMP280_PERIOD_CALLBACK, 0, TIMER_MODE_REPEAT);

		/*Init LCD*/
		gtid_lcd_display = timer_register_callback(lcd_display_callback, LCD_PERIOD_CALLBACK, 0, TIMER_MODE_REPEAT);

		/*Stating App*/
		gtid_led_callback = timer_register_callback(led_callback, 50, 0, TIMER_MODE_REPEAT);
}

