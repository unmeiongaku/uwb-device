/*
 * mode_anchor.c
 *
 *  Created on: Jan 12, 2025
 *      Author: nguye
 */

#include "mode_anchor.h"
#include "port.h"
#include <stdlib.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include <config_options.h>
#include "string.h"
#include "stdio.h"
#include "lcd.h"
#include "deca_vals.h"
#include "user_define.h"
#include "delay_us.h"
#include "myFunctions.h"

#if DEFAULT_MODE_SELECT == 0

// msg to TAG
char dist_str_to_uav[16] = {0};

static dwt_rxdiag_t rx_diag;
uint32_t C; // Channel Impulse Response Power; ipatovPower
uint16_t N; // Preamble Accumulation Count; ipatovAccumCount
uint8_t D; // DGC_DECISION (0 to 7)
const float A = 121.7;
char rssi_str[16] = {'\0'};

/* Hold the amount of errors that have occurred */
static uint32_t errors[23] = { 0 };

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Index in tx_dist_to_PC to where put distance */
#define DISTANCE_IDX 11

#if (ANCHOR_TYPE == 'A')
		/*Create a tx_poll_msg send from Anchor A to Tag*/
		static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 1,'T', 'U', 0xE0, 0, 0};
		/*After send tx_poll_msg tp tag, Anchor A receive a rx_resp_msg from Tag*/
		static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'A', 1, 0xE1, 0, 0};
		/*Receive final  Distance and Tof of Tag to Anchor A to display on LCD */
		static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 1, 'T', 'U', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static uint8_t tx_send_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'A', 1, 0xE2, 'A', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif (ANCHOR_TYPE == 'B')
		/*Create a tx_poll_msg send from Anchor B to Tag*/
		static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2,'T', 'U', 0xE0, 0, 0};
		/*After send tx_poll_msg tp tag, Anchor B receive a rx_resp_msg from Tag*/
		static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'B', 2, 0xE1, 0, 0};
		/*Receive final  Distance and Tof of Tag to Anchor B display on LCD */
		static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2, 'T', 'U', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static uint8_t tx_send_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'B', 2, 0xE2, 'B', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif (ANCHOR_TYPE == 'C')
		/*Create a tx_poll_msg send from Anchor B to Tag*/
		static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'C', 3,'T', 'U', 0xE0, 0, 0};
		/*After send tx_poll_msg tp tag, Anchor B receive a rx_resp_msg from Tag*/
		static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'C', 3, 0xE1, 0, 0};
		/*Receive final  Distance and Tof of Tag to Anchor C display on LCD */
		static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'C', 3, 'T', 'U', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static uint8_t tx_send_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'C', 3, 0xE2, 'C', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif (ANCHOR_TYPE == 'D')
		/*Create a tx_poll_msg send from Anchor D to Tag*/
		static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 4,'T', 'U', 0xE0, 0, 0};
		/*After send tx_poll_msg tp tag, Anchor D receive a rx_resp_msg from Tag*/
		static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'D', 4, 0xE1, 0, 0};
		/*Receive final  Distance and Tof of Tag to Anchor D display on LCD */
		static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 4, 'T', 'U', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static uint8_t tx_send_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'U', 'D', 4, 0xE2, 'D', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18


#define RX_BUF_LEN 30//Must be less than FRAME_LEN_MAX_EX
static uint8_t rx_buffer[RX_BUF_LEN];

#if NUMBER_OF_TAG!=1
/* for take the initiator adress from poll msg */
		static uint8_t allMSGCOMMONLEN = 7; //tutaj
		static uint8_t initiatorAdress[] = {0,0};
#endif
/*
 * 128-bit STS key to be programmed into CP_KEY register.
 *
 * This key needs to be known and programmed the same at both units performing the SS-TWR.
 * In a real application for security this would be private and unique to the two communicating units
 * and chosen/assigned in a secure manner lasting just for the period of their association.
 *
 * Here we use a default KEY as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_key_t cp_key = { 0x14EB220F, 0xF86050A8, 0xD1D336AA, 0x14148674 };

/*
 * 128-bit initial value for the nonce to be programmed into the CP_IV register.
 *
 * The IV, like the key, needs to be known and programmed the same at both units performing the SS-TWR.
 * It can be considered as an extension of the KEY. The low 32 bits of the IV is the counter.
 * In a real application for any particular key the value of the IV including the count should not be reused,
 * i.e. if the counter value wraps the upper 96-bits of the IV should be changed, e.g. incremented.
 *
 * Here we use a default IV as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_iv_t cp_iv = { 0x1F9A3DE4, 0xD37EC3CA, 0xC44FA8FB, 0x362EEB34 };


/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#if  ANCHOR_TYPE == 'A'
	#define ANT_DELAY 16385//16550
#elif ANCHOR_TYPE == 'B'
	#define ANT_DELAY 16385//16550
#elif ANCHOR_TYPE == 'C'
	#define ANT_DELAY 16385
#elif ANCHOR_TYPE == 'D'
	#define ANT_DELAY 16385
#endif

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY  ANT_DELAY  //16535//16525
#define RX_ANT_DLY  ANT_DELAY //16535//16525

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS (500 + CPU_PROCESSING_TIME)

#define RESP_TX_TO_FINAL_RX_DLY_UUS (100 + CPU_PROCESSING_TIME)

void uwb_mode_anchor_init(uwb_anchor_t *uwba){
	HAL_GPIO_WritePin(DW_PWR_GPIO_Port, DW_PWR_Pin, SET);
	uwba->frame_seq_nb = 0;
	uwba->allMSGCOMMONLEN = 7;
	uwba->goodSts = 0;
	uwba->loopCount = 0;
	uwba->messageFlag = 0;
	uwba->initiatorAdress[0] = 0;
	uwba->initiatorAdress[1] = 0;
    /* Configure SPI rate, DW3000 supports up to 36 MHz */
#ifdef CONFIG_SPI_FAST_RATE
    port_set_dw_ic_spi_fastrate();
#endif /* CONFIG_SPI_FAST_RATE */
#ifdef CONFIG_SPI_SLOW_RATE
    port_set_dw_ic_spi_slowrate();
#endif /* CONFIG_SPI_SLOW_RATE */
    port_DisableEXT_IRQ();
	/*Reset DWIC*/
    lcd_display_status("Reset DWIC");
    HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
    delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
	reset_DWIC();
	while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
	 {
		lcd_display_status("Reset False");
	 };
	lcd_display_status("Successful");
	HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
	delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
	lcd_display_status("Initiation DWIC");
	HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
	delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
	/*Init DW3000*/
	if(dwt_initialise(DWT_DW_IDLE) == DWT_ERROR){
		/*False Init*/
		/*Print to LCD Status*/
		lcd_display_status("Init False");
		while(1)
		{ };
	}
	else{
		lcd_display_status("Successful");
		delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
	}
	HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
	/* Configure DW IC. See NOTE 14 below. */
	lcd_display_status("Configuration");
	if(!dwt_configure(&config_options) == DWT_SUCCESS){
		lcd_display_status("Config False");
        while (1)
        { };
	}
	delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
    if(config_options.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
        lcd_display_status("ch5 Success");
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
        lcd_display_status("ch9 Success");
    }
    delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
    HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
    lcd_display_status("Enable Filter");
	/*ENABLE FRAME FILTERING*/
	#if ANCHOR_TYPE == 'A'
		dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
		dwt_setpanid(0xDECA);
		dwt_setaddress16(0x141);
	#elif ANCHOR_TYPE == 'B'
		dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
		dwt_setpanid(0xDECA);
		dwt_setaddress16(0x242);
	#elif ANCHOR_TYPE == 'C'
		dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
		dwt_setpanid(0xDECA);
		dwt_setaddress16(0x444);
	#elif ANCHOR_TYPE == 'D'
		dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
		dwt_setpanid(0xDECA);
		dwt_setaddress16(0x646);
	#endif
	/*Apply default antenna delay value. See Note 2 */
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_setrxantennadelay(TX_ANT_DLY);
	    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help diagnostics, and also TX/RX LEDs */
	    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
		/*Delay Between the response frame and final frame*/
		dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
		/*Enable IC diagnostic calculation and logging*/
		dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
		lcd_display_status("Config Success");
}

int8_t mode_anchor_run(uwb_anchor_t *uwba){
	if(!uwba->messageFlag){
        if (!uwba->loopCount)
        {
            /*
             * On first loop, configure the STS key & IV, then load them.
             */
            dwt_configurestskey(&cp_key);
            dwt_configurestsiv(&cp_iv);
            dwt_configurestsloadiv();
        }
        else
        {
            /*
             * On subsequent loops, we only need to reload the lower 32 bits of STS IV.
             */
            dwt_writetodevice(STS_IV0_ID, 0, 4, (uint8_t *)&cp_iv);
            dwt_configurestsloadiv();
        }
	}
    if(!uwba->messageFlag)  // Responder will enable the receive when waiting for Poll message,
                      // the receiver will be automatically enabled (DWT_RESPONSE_EXPECTED) when waiting for Final message
    {
    	uwba->loopCount++;  // increment the loop count only when starting ranging exchange
		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    lcd_display_status("RX Polling");
    HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
    delay_ms(DELAY_MS_FOR_LCD_FUNCTION);
    /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
    while (!((uwba->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };
    /*
     * Need to check the STS has been received and is good.
     */
    lcd_display_status("Checking goodSts");
    uwba->goodSts = dwt_readstsquality(&uwba->stsQual);
    /*
     * Check for a good frame and STS count.
     */
    if ((uwba->status_reg & SYS_STATUS_RXFCG_BIT_MASK) && (uwba->goodSts >= 0)){
    	uint32_t frame_len;
        /* Policzenie i wydobycie parametrów RSSI zajmuje bardzo dużo czasu w us.
         * Jeżeli nie chcemy spowolnić wymiany, obliczamy to po finalnej wiadomości
         */
        /* Clear good RX frame event in the DW IC status register. */
    	 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
         /* A frame has been received, read it into the local buffer. */
         frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
         if (frame_len <= sizeof(rx_buffer)){
             	 dwt_readrxdata(rx_buffer, frame_len, 0);
             	/* Check that the frame is a poll sent by "SS TWR initiator STS" example.
             				  * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
             	 rx_buffer[ALL_MSG_SN_IDX] = 0;
				 if (memcmp(rx_buffer, rx_poll_msg, uwba->allMSGCOMMONLEN) == 0) //tutaj
				 {
					 lcd_display_status("Rx Poll MSG");
					 HAL_GPIO_TogglePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin);
					/* Take adrress of initiator from frame, which was first */
					 uwba->initiatorAdress[0] = rx_buffer[7]; //tutaj
					 uwba->initiatorAdress[1] = rx_buffer[8];
					 uwba->allMSGCOMMONLEN = 10;
					 // set current initiator address
					tx_resp_msg[5] = uwba->initiatorAdress[0];
					tx_resp_msg[6] = uwba->initiatorAdress[1];
					//
					rx_poll_msg[7] = uwba->initiatorAdress[0];
					rx_poll_msg[8] = uwba->initiatorAdress[1];
					//
					rx_final_msg[7] = uwba->initiatorAdress[0];
					rx_final_msg[8] = uwba->initiatorAdress[1];
					uint32_t resp_tx_time;
					int ret;
					/* Retrieve poll reception timestamp. */
					uwba->poll_rx_ts = get_rx_timestamp_u64();
					resp_tx_time = (uwba->poll_rx_ts                               /* Received timestamp value */
							+ ((POLL_RX_TO_RESP_TX_DLY_UUS                   /* Set delay time */
									+ get_rx_delay_time_data_rate()          /* Added delay time for data rate set */
									+ get_rx_delay_time_txpreamble()         /* Added delay for TX preamble length */
									+ ((1<<(config_options.stsLength+2))*8)) /* Added delay for STS length */
									* UUS_TO_DWT_TIME)) >> 8;                /* Converted to time units for chip */
					dwt_setdelayedtrxtime(resp_tx_time);

					/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
					uwba->resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

					/* Write and send the response message. See NOTE 9 below. */
					tx_resp_msg[ALL_MSG_SN_IDX] = uwba->frame_seq_nb;
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

					dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

					/*
					 * As described above, we will be delaying the transmission of the RESP message
					 * with a set value that is also with reference to the timestamp of the received
					 * POLL message.
					 */
					dwt_setrxaftertxdelay(100); // receiver can be delayed as Final message will not come immediately

					ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
					if (ret == DWT_SUCCESS)
					{
						/* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
						{ };

						/* Clear TXFRS event. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

						/* Increment frame sequence number after transmission of the poll message (modulo 256). */
						uwba->frame_seq_nb++;

						/*
						 * This flag is set high here so that we do not reset the STS count before receiving
						 * the final message from the initiator. Otherwise, the STS count would be bad and
						 * we would be unable to receive it.
						 */
						uwba->messageFlag = 1;
					}
				 }
                 else if(memcmp(rx_buffer, rx_final_msg, uwba->allMSGCOMMONLEN) == 0) //tutaj
                 {
                	 lcd_display_status("Rx Final MSG");
                     int ret;
                     /* Retrieve response transmission and final reception timestamps. */
                     uwba->resp_tx_ts = get_tx_timestamp_u64();
                     uwba->final_rx_ts = get_rx_timestamp_u64();
                     lcd_display_status("CalTof");
                     /* Get timestamps embedded in the final message. */
                     final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &uwba->poll_tx_ts);
                     final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &uwba->resp_rx_ts);
                     final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &uwba->final_tx_ts);
                     /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 15 below. */
                     uwba->poll_rx_ts_32 = (uint32_t)uwba->poll_rx_ts;
                     uwba->resp_tx_ts_32 = (uint32_t)uwba->resp_tx_ts;
                     uwba->final_rx_ts_32 = (uint32_t)uwba->final_rx_ts;
                     uwba->Ra = (double)(uwba->resp_rx_ts - uwba->poll_tx_ts);
                     uwba->Rb = (double)(uwba->final_rx_ts_32 - uwba->resp_tx_ts_32);
                     uwba->Da = (double)(uwba->final_tx_ts - uwba->resp_rx_ts);
                     uwba->Db = (double)(uwba->resp_tx_ts_32 - uwba->poll_rx_ts_32);
                     uwba->tof_dtu = (int64_t)((uwba->Ra * uwba->Rb - uwba->Da * uwba->Db) / (uwba->Ra + uwba->Rb + uwba->Da + uwba->Db));
                     uwba->tof = uwba->tof_dtu * DWT_TIME_UNITS;
                     lcd_display_status("CalDistance");
                     uwba->distance = uwba->tof * SPEED_OF_LIGHT;
                     // set back allMSGCOMMONLEN to 7 for get msg form another initiator
                     uwba->allMSGCOMMONLEN = 7; // tutaj
                     /* ====> Get params for diagnostic (in progress) <==== */
 					//C = STS_DIAG_1 to jest pole ipatovPower
 					//N = STS_DIAG_12 to jest pole ipatovAccumCount
 					dwt_readdiagnostics(&rx_diag);
 					C = rx_diag.ipatovPower;
 					N = rx_diag.ipatovAccumCount;
 					/* ====> Read DGC here <==== */
 					D = dwt_get_dgcdecision();
 					/* ====> Calculate RSSI <==== */
 					lcd_display_status("CalRSSI");
 					uwba->rssi = getRSSI(C, N, D, A);
 					sprintf(rssi_str, "%3.2fd" ,uwba->rssi);
 					sprintf(dist_str_to_uav, "%3.2fm" ,uwba->distance);
 					add_distANDrssi_to_PCmsg(tx_send_final_msg, dist_str_to_uav, rssi_str,DISTANCE_IDX, uwba->initiatorAdress[1]);
					dwt_writetxdata(sizeof(tx_send_final_msg), tx_send_final_msg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(tx_send_final_msg), 0, 0); /* Zero offset in TX buffer, ranging. */
					ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
					memset(&tx_send_final_msg[DISTANCE_IDX], 0, 7*sizeof(uint8_t));
					memset(dist_str_to_uav, '\0', 16*sizeof(char));
					memset(rssi_str, '\0', 16*sizeof(char));
					   /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
					   if (ret == DWT_SUCCESS)
					   {
						   /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
						   while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
						   { };

						   /* Clear TXFRS event. */
						   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
					   }
					   else
					   {
						   lcd_display_status("NW");
					   }
					   uwba->messageFlag = 0;
                 }
                 else
                 {
                	 uwba->errors[BAD_FRAME_ERR_IDX] += 1;
                     /*
                      * If any error occurs, we can reset the STS count back to default value.
                      */
                	 uwba->messageFlag = 0;
                	 uwba->allMSGCOMMONLEN = 7;
                 }
    	}
         else{
        	 uwba->errors[BAD_FRAME_ERR_IDX] += 1;
             /*
              * If any error occurs, we can reset the STS count back to default value.
              */
        	 uwba->messageFlag = 0;
        	 uwba->allMSGCOMMONLEN = 7;
         }
    }
    else{
        /* Clear RX error events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        /*
         * If any error occurs, we can reset the STS count back to default value.
         */
        uwba->messageFlag = 0;
        uwba->allMSGCOMMONLEN = 7;
    }
    return DWT_SUCCESS;
}


#endif
