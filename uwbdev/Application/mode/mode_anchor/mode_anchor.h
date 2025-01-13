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
	/*Final rx msg*/
    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
	uint64_t final_rx_ts;
	/*Data*/
	double Ra;
	double Rb;
	double Da;
	double Db;
	double tof;
	double distance;
	int64_t tof_dtu;
	float rssi;
	/*Error*/
	uint32_t errors[23];

}uwb_anchor_t;


void uwb_mode_anchor_init(uwb_anchor_t *uwba);
int8_t mode_anchor_run(uwb_anchor_t *uwba);
#endif


#endif /* MODE_MODE_ANCHOR_MODE_ANCHOR_H_ */
