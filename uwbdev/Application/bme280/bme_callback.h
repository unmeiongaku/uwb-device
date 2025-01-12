/*
 * bme_callback.h
 *
 *  Created on: Jul 29, 2024
 *      Author: nguye
 */


typedef enum {
    BME_EVENT_HANDLED,
	BME_EVENT_IGNORED,
	BME_EVENT_TRANSITION
}bme_event_status_t;


/* Signals of the application*/
typedef enum{
/* Internal activity signals */
	BME_TICK_SIG  = 30,
	BME_NEXT_SIG = 31,
	BME_ENTRY,
	BME_EXIT
}bme_proobject_signal_t;

typedef enum{
	BME_CALLBACK_SM,
	BME_READ_DATA_SM,
	BME_MAX_SM,
}bme_proobject_state_t;


//forward declarations
struct bme_proobject_tag;
struct bme_event_tag;

typedef struct bme_proobject_tag {
	float temperature;
	float humidity;
	float pressure;
	int8_t result;
	uint8_t tickcnt;
	bme_proobject_state_t bme_active_state;
}bme_proobject_t;

/*Generic(Super) event structure */
typedef struct bme_event_tag{
    uint8_t sig;
}bme_event_t;

/* For user generated events */
typedef struct{
	bme_event_t super;
    uint8_t ss;
}bme_proobject_user_event_t;

/* For tick event */
typedef struct{
	bme_event_t super;
    uint8_t ss;
}bme_proobject_tick_event_t;

uint8_t bmegetnextstatesig();
void bmeresetnextstatesig();

void bme_proobject_init(bme_proobject_t *const bme_mobj);
bme_event_status_t bme_proobject_state_machine(bme_proobject_t *const bme_mobj, bme_event_t const * const bme_e);
