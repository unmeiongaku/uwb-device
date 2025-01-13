/*
 * mode_anchor.h
 *
 *  Created on: Jan 12, 2025
 *      Author: nguye
 */

#ifndef MODE_MODE_ANCHOR_MODE_ANCHOR_H_
#define MODE_MODE_ANCHOR_MODE_ANCHOR_H_


#include "stdint.h"

#if DEFAULT_MODE_SELECT == 0

typedef struct{
	/* This will contain STS quality index and status */
	int16_t stsQual;
	/* Used for checking STS quality in received signal */
	int goodSts;
	uint8_t loopCount;
	/* Used to track whether STS count should be reinitialised or not */
	uint8_t messageFlag;
	uint32_t status_reg;
	uint8_t allMSGCOMMONLEN;
	/*Addr*/
	uint8_t initiatorAdress[2];
	/* Timestamps of frames transmission/reception. */
	uint64_t poll_rx_ts;
	uint64_t resp_tx_ts;
	uint8_t frame_seq_nb;
}uwb_anchor_t;


void uwb_mode_anchor_init(uwb_anchor_t *uwba);
void mode_anchor_run(uwb_anchor_t *uwba);
#endif


#endif /* MODE_MODE_ANCHOR_MODE_ANCHOR_H_ */
