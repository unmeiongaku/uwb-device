/*
 * bme_callback.c
 *
 *  Created on: Jul 29, 2024
 *      Author: nguye
 */


#include "bme280.h"
#include "bme_callback.h"
#include "stm32f405xx.h"
#include "i2c.h"
#include <stdlib.h>
#include "delay_us.h"
#include "string.h"
#include "user_define.h"

static struct bme280_dev dev;
static struct bme280_data comp_data;

static uint8_t bme_global_signals;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  delay_ms(period);
}


int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

int8_t bme_set_sensor_settings(){
	  /* BME280 설정 */
	  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	  dev.settings.filter = BME280_FILTER_COEFF_16;
	  return bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
}

int8_t  bme_init(){
	  /* BME280 초기화 */
	  int8_t result;
	  dev.dev_id = BME280_I2C_ADDR_PRIM;
	  dev.intf = BME280_I2C_INTF;
	  dev.read = user_i2c_read;
	  dev.write = user_i2c_write;
	  dev.delay_ms = user_delay_ms;
	  result =  bme280_init(&dev);
	  result = bme_set_sensor_settings();
	  return result;

}

uint8_t bmegetnextstatesig(){
	return bme_global_signals;
}
void bmeresetnextstatesig(){
	bme_global_signals = 0;
}

static bme_event_status_t proobject_state_handle_BME_CALLBACK_SM(bme_proobject_t *const bme_mobj, bme_event_t const *const bme_e);
static bme_event_status_t proobject_state_handle_BME_READ_DATA_SM(bme_proobject_t *const bme_mobj, bme_event_t const *const bme_e);


void bme_proobject_init(bme_proobject_t *const bme_mobj){
	bme_event_t ee;
	ee.sig = BME_ENTRY;
	bme_mobj->bme_active_state = BME_CALLBACK_SM;
	bme_proobject_state_machine(bme_mobj,&ee);
	bme_mobj->result = bme_init();
}
bme_event_status_t bme_proobject_state_machine(bme_proobject_t *const bme_mobj, bme_event_t const * const bme_e){
	switch (bme_mobj->bme_active_state){
	case BME_CALLBACK_SM:
	{
		return proobject_state_handle_BME_CALLBACK_SM(bme_mobj,bme_e);
	}
	case BME_READ_DATA_SM:
	{
		return proobject_state_handle_BME_READ_DATA_SM(bme_mobj,bme_e);
	}
	case BME_MAX_SM:
		return BME_EVENT_IGNORED;
		break;
	}
	return BME_EVENT_IGNORED;
}


static bme_event_status_t proobject_state_handle_BME_CALLBACK_SM(bme_proobject_t *const bme_mobj, bme_event_t const *const bme_e){
	switch(bme_e->sig){
		case BME_ENTRY:
		{
			bme_mobj->tickcnt = 0;
			bme_mobj->result = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
			return BME_EVENT_HANDLED;
		}
		case BME_EXIT:
		{
			return BME_EVENT_HANDLED;
		}
		case BME_TICK_SIG:
		{
			if(bme_mobj->tickcnt == 2){
				bme_global_signals = BME_NEXT_SIG_DEFINE;
				bme_mobj->tickcnt = 0;
			}
			else{
				bme_mobj->tickcnt++;
			}
			return BME_EVENT_HANDLED;
		}
		case BME_NEXT_SIG:
		{
			bme_mobj->bme_active_state = BME_READ_DATA_SM;
			return BME_EVENT_TRANSITION;
		}
	}
	return BME_EVENT_IGNORED;
}

static bme_event_status_t proobject_state_handle_BME_READ_DATA_SM(bme_proobject_t *const bme_mobj, bme_event_t const *const bme_e){
	switch(bme_e->sig){
		case BME_ENTRY:
		{
			bme_mobj->result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
			return BME_EVENT_HANDLED;
		}
		case BME_EXIT:
		{
			return BME_EVENT_HANDLED;
		}
		case BME_TICK_SIG:
		{
			if(bme_mobj->result== BME280_OK){
				bme_mobj->temperature = comp_data.temperature / 100.0;
				bme_mobj->pressure = comp_data.pressure/ 10000.0;
				bme_mobj->humidity = comp_data.humidity/ 1024.0;
			}
			bme_global_signals = BME_NEXT_SIG_DEFINE;
			return BME_EVENT_HANDLED;
		}
		case BME_NEXT_SIG:
		{
			bme_mobj->bme_active_state = BME_CALLBACK_SM;
			return BME_EVENT_TRANSITION;
		}
	}
	return BME_EVENT_IGNORED;
}

