#ifndef _SHARE_FUNC_
#define _SHARE_FUNC_

//#include "mode_anchor_active_object.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn check_for_status_errors()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
 *
 * @param reg: uint32_t value representing the current status register value.
 * @param errors: pointer to a uint32_t buffer that contains the sum of different errors logged during program operation.
 *
 * @return none
 */
void check_for_status_errors(uint32_t reg, uint32_t * errors);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_delay_time_txpreamble()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
 *
 * @param None
 *
 * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
 */
uint32_t get_rx_delay_time_txpreamble(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_delay_time_data_rate()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current data rate set.
 *
 * @param None
 *
 * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
 */
uint32_t get_rx_delay_time_data_rate(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn set_delayed_rx_time()
 *
 * @brief This function is used to set the delayed RX time before running dwt_rxenable()
 *
 * @param delay - This is a defined delay value (usually POLL_TX_TO_RESP_RX_DLY_UUS)
 * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
 *                         is called.
 *
 * @return None
 */
void set_delayed_rx_time(uint32_t delay, dwt_config_t *config_options);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn set_resp_rx_timeout()
 *
 * @brief This function is used to set the RX timeout value
 *
 * @param delay - This is a defined delay value (usually RESP_RX_TIMEOUT_UUS)
 * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
 *                         is called.
 *
 * @return None
 */
void set_resp_rx_timeout(uint32_t delay, dwt_config_t *config_options);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resync_sts()
 *
 * @brief Resync the current device's STS value the given value
 *
 * @param newCount - The 32 bit value to set the STS to.
 *
 * @return None
 */
void resync_sts(uint32_t newCount);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will continuously read the system status register until it matches the bits set in the mask
 *        input parameter. It will then exit the function.
 *        This is useful to use when waiting on particular events to occurs. For example, the user could wait for a
 *        good UWB frame to be received and/or no receive errors have occurred.
 *        The lower 32-bits of the system status register will be read in a while loop. Each iteration of the loop will check if a matching
 *        mask value for the higher 32-bits of the system status register is set. If the mask value is set in the higher 32-bits of the system
 *        status register, the function will return that value along with the last recorded value of the lower 32-bits of the system status
 *        register. Thus, the user should be aware that this function will not wait for high and low mask values to be set in both the low and high
 *        system status registers. Alternatively, the user can call this function to *only* check the higher or lower system status registers.
 *
 * input parameters
 * @param lo_result - A pointer to a uint32_t that will contain the final value of the system status register (lower 32 bits).
 *                    Pass in a NULL pointer to ignore returning this value.
 * @param hi_result - A pointer to a uint32_t that will contain the final value of the system status register (higher 32 bits).
 *                    Pass in a NULL pointer to ignore returning this value.
 * @param lo_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (lower 32 bits).
 *               Example values to use are as follows:
 *               DWT_INT_TXFRS_BIT_MASK - Wait for a TX frame to be sent.
 *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no reception errors.
 *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no receive timeout errors
 *                                                                                          and no reception errors.
 *               SYS_STATUS_RXFR_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_ND_RX_ERR - Wait for packet to be received and no receive timeout errors
 *                                                                                            and no reception errors.
 *                                                                                            These flags are useful when polling for STS Mode 4 (no data)
 *                                                                                            packets.
 *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
 * @param hi_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (higher 32 bits).
 *               Example values to use are as follows:
 *               SYS_STATUS_HI_CCA_FAIL_BIT_MASK - Check for CCA fail status.
 *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
 *
 * return None
 */

#ifdef __cplusplus
}
#endif


#endif
