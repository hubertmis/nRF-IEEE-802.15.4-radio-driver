/* Copyright (c) 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements Finite State Machine of nRF 802.15.4 radio driver.
 *
 */

#include "nrf_drv_radio802154_fsm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_radio802154.h"
#include "nrf_drv_radio802154_ack_pending_bit.h"
#include "nrf_drv_radio802154_config.h"
#include "nrf_drv_radio802154_const.h"
#include "nrf_drv_radio802154_critical_section.h"
#include "nrf_drv_radio802154_debug.h"
#include "nrf_drv_radio802154_fsm_hooks.h"
#include "nrf_drv_radio802154_notification.h"
#include "nrf_drv_radio802154_pib.h"
#include "nrf_drv_radio802154_priority_drop.h"
#include "nrf_drv_radio802154_procedures_duration.h"
#include "nrf_drv_radio802154_revision.h"
#include "nrf_drv_radio802154_rx_buffer.h"
#include "nrf_drv_radio802154_types.h"
#include "fem/nrf_fem_control_api.h"
#include "hal/nrf_egu.h"
#include "hal/nrf_ppi.h"
#include "hal/nrf_radio.h"
#include "hal/nrf_timer.h"
#include "mac_features/nrf_drv_radio802154_filter.h"
#include "raal/nrf_raal_api.h"


#define EGU_EVENT           NRF_EGU_EVENT_TRIGGERED0
#define EGU_TASK            NRF_EGU_TASK_TRIGGER0
#define PPI_CH0             NRF_PPI_CHANNEL6
#define PPI_CH1             NRF_PPI_CHANNEL7
#define PPI_CH2             NRF_PPI_CHANNEL8
#define PPI_CH3             NRF_PPI_CHANNEL9
#define PPI_CH4             NRF_PPI_CHANNEL10
#define PPI_CH5             NRF_PPI_CHANNEL11
#define PPI_CH6             NRF_PPI_CHANNEL12
#define PPI_CH7             NRF_PPI_CHANNEL13
#define PPI_CHGRP0          NRF_PPI_CHANNEL_GROUP0
#define PPI_CHGRP0_DIS_TASK NRF_PPI_TASK_CHG0_DIS
#define PPI_CHGRP0_EN_TASK  NRF_PPI_TASK_CHG0_EN

#define PPI_DISABLED_EGU    PPI_CH0
#define PPI_EGU_RAMP_UP     PPI_CH1
#define PPI_EGU_TIMER_START PPI_CH2
#define PPI_CRCERROR_CLEAR  PPI_CH3
#define PPI_TIMER_TX_ACK    PPI_CH3
#define PPI_CRCOK_DIS_PPI   PPI_CH4

/// Workaround for missing PHYEND event in older chip revision.
static inline uint32_t short_phyend_disable_mask_get(void)
{
    if (nrf_drv_radio802154_revision_has_phyend_event())
    {
        return NRF_RADIO_SHORT_PHYEND_DISABLE_MASK;
    }

    return NRF_RADIO_SHORT_END_DISABLE_MASK;
}

#if NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
#define SHORT_FRAMESTART_BCSTART    0UL
#else // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
#define SHORT_FRAMESTART_BCSTART    NRF_RADIO_SHORT_FRAMESTART_BCSTART_MASK
#endif // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

#if NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
#define SHORT_ADDRESS_BCSTART    0UL
#else // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
#define SHORT_ADDRESS_BCSTART    NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK
#endif // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

/// Value set to SHORTS register when no shorts should be enabled.
#define SHORTS_IDLE         0
/// Value set to SHORTS register for RX operation.
#define SHORTS_RX             (NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK  |                           \
                               NRF_RADIO_SHORT_END_DISABLE_MASK |                                  \
                               SHORT_ADDRESS_BCSTART)

#define SHORTS_RX_FREE_BUFFER (NRF_RADIO_SHORT_RXREADY_START_MASK)

#define SHORTS_TX_ACK         (NRF_RADIO_SHORT_TXREADY_START_MASK |                                \
                               short_phyend_disable_mask_get())

#define SHORTS_CCA_TX         (NRF_RADIO_SHORT_RXREADY_CCASTART_MASK |                             \
                               NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK  |                             \
                               NRF_RADIO_SHORT_CCAIDLE_TXEN_MASK     |                             \
                               NRF_RADIO_SHORT_TXREADY_START_MASK    |                             \
                               short_phyend_disable_mask_get())

#define SHORTS_TX             (NRF_RADIO_SHORT_TXREADY_START_MASK |                                \
                               short_phyend_disable_mask_get())

#define SHORTS_RX_ACK         (NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |                            \
                               NRF_RADIO_SHORT_END_DISABLE_MASK)

#define SHORTS_ED             (NRF_RADIO_SHORT_READY_EDSTART_MASK)

#define SHORTS_CCA            (NRF_RADIO_SHORT_RXREADY_CCASTART_MASK |                             \
                               NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK)

/// Value set to SHORTS register when receiver is waiting for incoming frame.
#define SHORTS_RX_INITIAL   (NRF_RADIO_SHORT_END_DISABLE_MASK | NRF_RADIO_SHORT_DISABLED_TXEN_MASK | \
                             SHORT_FRAMESTART_BCSTART)

/// Value set to SHORTS register when receiver started receiving a frame.
#define SHORTS_RX_FOLLOWING (short_phyend_disable_mask_get() | NRF_RADIO_SHORT_READY_START_MASK | \
                             SHORT_FRAMESTART_BCSTART)
/// Value set to SHORTS register when received frame should be acknowledged.
//#define SHORTS_TX_ACK       (short_phyend_disable_mask_get())
#if NRF_DRV_RADIO802154_SHORT_CCAIDLE_TXEN
/// Value set to SHORTS register during transmission of a frame
#define SHORTS_TX_FRAME     (short_phyend_disable_mask_get() | NRF_RADIO_SHORT_READY_START_MASK | \
                             NRF_RADIO_SHORT_CCAIDLE_TXEN_MASK)
#else
/// Value set to SHORTS register during transmission of a frame
#define SHORTS_TX_FRAME     (short_phyend_disable_mask_get() | NRF_RADIO_SHORT_READY_START_MASK)
#endif

/// Delay before sending ACK (aTurnaroundTime)
#define TIFS_ACK_US         TURNAROUND_TIME
/// Delay before first check of received frame: 24 bits is PHY header and MAC Frame Control field.
#define BCC_INIT            (3 * 8)

/// Duration of single iteration of Energy Detection procedure
#define ED_ITER_DURATION   128U
/// Overhead of hardware preparation for ED procedure (aTurnaroundTime) [number of iterations]
#define ED_ITERS_OVERHEAD  2U

#define CRC_LENGTH      2         ///< Length of CRC in 802.15.4 frames [bytes]
#define CRC_POLYNOMIAL  0x011021  ///< Polynomial used for CRC calculation in 802.15.4 frames

#define MHMU_MASK               0xff000700  ///< Mask of known bytes in ACK packet
#define MHMU_PATTERN            0x00000200  ///< Values of known bytes in ACK packet
#define MHMU_PATTERN_DSN_OFFSET 24          ///< Offset of DSN in MHMU_PATTER [bits]

/** Get LQI of given received packet. If CRC is calculated by hardware LQI is included instead of CRC
 *  in the frame. Length is stored in byte with index 0; CRC is 2 last bytes.
 */
#define RX_FRAME_LQI(psdu)  ((psdu)[(psdu)[0] - 1])

#if NRF_DRV_RADIO802154_RX_BUFFERS > 1
/// Pointer to currently used receive buffer.
static rx_buffer_t * mp_current_rx_buffer;
#else
/// If there is only one buffer use const pointer to the receive buffer.
static rx_buffer_t * const mp_current_rx_buffer = &nrf_drv_radio802154_rx_buffers[0];
#endif

static uint8_t         m_ack_psdu[ACK_LENGTH + 1]; ///< Ack frame buffer.
static const uint8_t * mp_tx_data;                 ///< Pointer to data to transmit.
static uint32_t        m_ed_time_left;             ///< Remaining time of current energy detection procedure [us].
static uint8_t         m_ed_result;                ///< Result of current energy detection procedure.

static volatile radio_state_t m_state = RADIO_STATE_SLEEP;  ///< State of the radio driver

typedef struct
{
    bool frame_filtered        :1;  ///< If frame being received passed filtering operation.
    bool rx_timeslot_requested :1;  ///< If timeslot for the frame being received is already requested.
} nrf_radio802154_flags_t;
static nrf_radio802154_flags_t m_flags;  ///< Flags used to store current driver state.


/***************************************************************************************************
 * @section FSM common operations
 **************************************************************************************************/

/** Set driver state.
 *
 * @param[in]  state  Driver state to set.
 */
static inline void state_set(radio_state_t state)
{
    m_state = state;

#if 0
    if (m_state == RADIO_STATE_SLEEP)
    {
        nrf_fem_control_deactivate();
    }
    else
    {
        nrf_fem_control_activate();
    }
#endif

    nrf_drv_radio802154_log(EVENT_SET_STATE, (uint32_t)state);
}

/// Common procedure when the driver enters SLEEP state.
static inline void sleep_start(void)
{
    nrf_drv_radio802154_priority_drop_timeslot_exit();
}

/// Start receiver to wait for frames.
static inline void rx_start(void)
{
    nrf_radio_packet_ptr_set(mp_current_rx_buffer->psdu);
    nrf_radio_task_trigger(NRF_RADIO_TASK_START);
}

/// Start receiver to wait for frame that can be acknowledged.
static inline void rx_frame_start(void)
{
    m_flags.frame_filtered        = false;
    m_flags.rx_timeslot_requested = false;

    rx_start();

    // Just after starting receiving to receive buffer set packet pointer to ACK frame that can be
    // sent automatically.
    nrf_radio_packet_ptr_set(m_ack_psdu);
}

/** Get result of last RSSI measurement.
 *
 * @returns  Result of last RSSI measurement [dBm].
 */
static inline int8_t rssi_last_measurement_get(void)
{
    return -((int8_t)nrf_radio_rssi_sample_get());
}

/// Notify MAC layer that a frame was received.
static inline void received_frame_notify(uint8_t * p_psdu)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    nrf_drv_radio802154_notify_received(p_psdu,                       // data
                                        rssi_last_measurement_get(),  // rssi
                                        RX_FRAME_LQI(p_psdu));        // lqi

    nrf_drv_radio802154_critical_section_nesting_deny();
}

/// Notify MAC layer that receive procedure failed.
static void receive_failed_notify(nrf_drv_radio802154_rx_error_t error)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    nrf_drv_radio802154_notify_receive_failed(error);

    nrf_drv_radio802154_critical_section_nesting_deny();
}

static inline void transmit_started_notify(void)
{
    if (nrf_drv_radio802154_fsm_hooks_tx_started(mp_tx_data))
    {
        nrf_drv_radio802154_tx_started(mp_tx_data);
    }

}

/// Notify MAC layer that a frame was transmitted.
static inline void transmitted_frame_notify(uint8_t * p_ack, int8_t power, int8_t lqi)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    nrf_drv_radio802154_fsm_hooks_transmitted(mp_tx_data);
    nrf_drv_radio802154_notify_transmitted(mp_tx_data, p_ack, power, lqi);

    nrf_drv_radio802154_critical_section_nesting_deny();
}

/// Notify MAC layer that transmission procedure failed.
static void transmit_failed_notify(nrf_drv_radio802154_tx_error_t error)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    if (nrf_drv_radio802154_fsm_hooks_tx_failed(mp_tx_data, error))
    {
        nrf_drv_radio802154_notify_transmit_failed(mp_tx_data, error);
    }

    nrf_drv_radio802154_critical_section_nesting_deny();
}

/// Notify MAC layer that energy detection procedure ended.
static inline void energy_detected_notify(uint8_t result)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    nrf_drv_radio802154_notify_energy_detected(result);

    nrf_drv_radio802154_critical_section_nesting_deny();
}

/// Notify MAC layer that CCA procedure ended.
static inline void cca_notify(bool result)
{
    nrf_drv_radio802154_critical_section_nesting_allow();

    nrf_drv_radio802154_notify_cca(result);

    nrf_drv_radio802154_critical_section_nesting_deny();
}

/** Set currently used rx buffer to given address.
 *
 * @param[in]  p_rx_buffer  Pointer to receive buffer that should be used now.
 */
static inline void rx_buffer_in_use_set(rx_buffer_t * p_rx_buffer)
{
#if NRF_DRV_RADIO802154_RX_BUFFERS > 1
    mp_current_rx_buffer = p_rx_buffer;
#else
    (void) p_rx_buffer;
#endif
}

/** Check if currently there is available rx buffer.
 *
 * @retval true   There is available rx buffer.
 * @retval false  Currently there is no available rx buffer.
 */
static bool rx_buffer_is_available(void)
{
    return (mp_current_rx_buffer != NULL) && (mp_current_rx_buffer->free);
}

/** Get pointer to available rx buffer.
 *
 * @returns Pointer to available rx buffer or NULL if rx buffer is not available.
 */
static uint8_t * rx_buffer_get(void)
{
    return rx_buffer_is_available() ? mp_current_rx_buffer->psdu : NULL;
}

/** Update CCA configuration in RADIO registers. */
static void cca_configuration_update(void)
{
    nrf_drv_radio802154_cca_cfg_t cca_cfg;

    nrf_drv_radio802154_pib_cca_cfg_get(&cca_cfg);
    nrf_radio_cca_mode_set(cca_cfg.mode);
    nrf_radio_cca_ed_threshold_set(cca_cfg.ed_threshold);
    nrf_radio_cca_corr_threshold_set(cca_cfg.corr_threshold);
    nrf_radio_cca_corr_counter_set(cca_cfg.corr_limit);
}

/** Trigger RX task. */
static void rx_enable(void)
{
    nrf_fem_control_time_latch();
    nrf_radio_task_trigger(NRF_RADIO_TASK_RXEN);
    nrf_fem_control_lna_set(false);
}

/** Trigger TX task. */
static void tx_enable(void)
{
    nrf_fem_control_time_latch();
    nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);
    nrf_fem_control_pa_set(false, false);
}


/***************************************************************************************************
 * @section Radio parameters calculators
 **************************************************************************************************/

/** Set radio channel
 *
 *  @param[in]  channel  Channel number to set (11-26).
 */
static void channel_set(uint8_t channel)
{
    assert(channel >= 11 && channel <= 26);

    nrf_radio_frequency_set(5 + (5 * (channel - 11)));
}


/***************************************************************************************************
 * @section Shorts management
 **************************************************************************************************/

/// Disable peripheral shorts.
static inline void shorts_disable(void)
{
    nrf_radio_shorts_set(SHORTS_IDLE);
    nrf_radio_ifs_set(0);
    nrf_radio_ramp_up_mode_set(NRF_RADIO_RAMP_UP_MODE_FAST);
}

/// Enable peripheral shorts used during data frame transmission.
static inline void shorts_tx_frame_set(void)
{
    nrf_radio_shorts_set(SHORTS_TX_FRAME);
}

/// Enable peripheral shorts used in receive state to enable automatic ACK procedure.
static inline void shorts_rx_initial_set(void)
{
    nrf_radio_ifs_set(TIFS_ACK_US);
//    nrf_radio_ramp_up_mode_set(NRF_RADIO_RAMP_UP_MODE_DEFAULT);
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_bcc_set(BCC_INIT);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    nrf_radio_shorts_set(SHORTS_RX_INITIAL);
}

/// Enable peripheral shorts used during automatic ACK transmission.
static inline void shorts_rx_following_set(void)
{
    nrf_radio_shorts_set(SHORTS_RX_FOLLOWING);
}

/// Disable peripheral shorts used during automatic ACK transmission when ACK is being transmitted
static inline void shorts_tx_ack_set(void)
{
    // If ACK is sent PHYEND_DISABLE short should persist to disable transmitter automatically.
    nrf_radio_shorts_set(SHORTS_TX_ACK);
    nrf_radio_ifs_set(0);
    nrf_radio_ramp_up_mode_set(NRF_RADIO_RAMP_UP_MODE_FAST);
}

/***************************************************************************************************
 * @section ACK transmission management
 **************************************************************************************************/

/// Set valid sequence number in ACK frame.
static inline void ack_prepare(void)
{
    // Copy sequence number from received frame to ACK frame.
    m_ack_psdu[DSN_OFFSET] = mp_current_rx_buffer->psdu[DSN_OFFSET];
}

/// Set pending bit in ACK frame.
static inline void ack_pending_bit_set(void)
{
    m_ack_psdu[FRAME_PENDING_OFFSET] = ACK_HEADER_WITH_PENDING;

    if (!nrf_drv_radio802154_ack_pending_bit_should_be_set(mp_current_rx_buffer->psdu))
    {
        m_ack_psdu[FRAME_PENDING_OFFSET] = ACK_HEADER_WITHOUT_PENDING;
    }
}

/** Check if ACK is requested in given frame.
 *
 * @param[in]  p_frame  Pointer to a frame to check.
 *
 * @retval  true   ACK is requested in given frame.
 * @retval  false  ACK is not requested in given frame.
 */
static inline bool ack_is_requested(const uint8_t * p_frame)
{
    return (p_frame[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT) ? true : false;
}

/** Abort automatic ACK procedure.
 *
 * @param[in]  state_to_set  Driver state that shall be set after the procedure is aborted.
 */
static void auto_ack_abort(radio_state_t state_to_set)
{
    nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_AUTO_ACK_ABORT);

    shorts_disable();

    switch (nrf_radio_state_get())
    {
        case NRF_RADIO_STATE_RX:     // When stopping before whole frame received.
        case NRF_RADIO_STATE_RX_RU:  // When transmission is initialized during receiver ramp up.
        case NRF_RADIO_STATE_RX_IDLE:
        case NRF_RADIO_STATE_TX_RU:
        case NRF_RADIO_STATE_TX_IDLE:
        case NRF_RADIO_STATE_TX:
            nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED); // Clear disabled event that was set by short.
            state_set(state_to_set);
            nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
            break;

        case NRF_RADIO_STATE_RX_DISABLE:
        case NRF_RADIO_STATE_DISABLED:
        case NRF_RADIO_STATE_TX_DISABLE:
            // Do not trigger DISABLE task in those states to prevent double DISABLED events.
            state_set(state_to_set);
            break;

        default:
            assert(false);
    }

    nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_AUTO_ACK_ABORT);
}

/***************************************************************************************************
 * @section ACK receiving management
 **************************************************************************************************/

/// Enable hardware ACK matching accelerator.
static inline void ack_matching_enable(void)
{
    nrf_radio_event_clear(NRF_RADIO_EVENT_MHRMATCH);
    nrf_radio_mhmu_search_pattern_set(MHMU_PATTERN |
                                      ((uint32_t) mp_tx_data[DSN_OFFSET] <<
                                       MHMU_PATTERN_DSN_OFFSET));
}

/// Disable hardware ACK matching accelerator.
static inline void ack_matching_disable(void)
{
    nrf_radio_mhmu_search_pattern_set(0);
    nrf_radio_event_clear(NRF_RADIO_EVENT_MHRMATCH);
}

/** Check if hardware ACK matching accelerator matched ACK pattern in received frame.
 *
 * @retval  true   ACK matching accelerator matched ACK pattern.
 * @retval  false  ACK matching accelerator did not match ACK pattern.
 */
static inline bool ack_is_matched(void)
{
    return (nrf_radio_event_get(NRF_RADIO_EVENT_MHRMATCH)) &&
            (nrf_radio_crc_status_get() == NRF_RADIO_CRC_STATUS_OK);
}

/** Start receiver to receive data after receiving of ACK frame.
 *
 * @param[in] error  Error value that should be notified to the MAC layer in case there was an error
 *                   when ACK frame was received. If there was no error @p error shall be set to 0
 *                   and error will not be notified.
 */
static inline void frame_rx_start_after_ack_rx(nrf_drv_radio802154_rx_error_t error)
{
    state_set(RADIO_STATE_RX);
    nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE); // Errata [110]

    if (error)
    {
        transmit_failed_notify(error);
    }

    ack_matching_disable();
}

/***************************************************************************************************
 * @section RADIO peripheral management
 **************************************************************************************************/

/// Initialize radio peripheral
static void nrf_radio_init(void)
{
    nrf_radio_mode_set(NRF_RADIO_MODE_IEEE802154_250KBIT);
    nrf_radio_config_length_field_length_set(8);
    nrf_radio_config_preamble_length_set(NRF_RADIO_PREAMBLE_LENGTH_32BIT_ZERO);
    nrf_radio_config_crc_included_set(true);
    nrf_radio_config_max_length_set(MAX_PACKET_SIZE);
    nrf_radio_ramp_up_mode_set(NRF_RADIO_RAMP_UP_MODE_FAST);

    // Configure CRC
    nrf_radio_crc_length_set(CRC_LENGTH);
    nrf_radio_crc_includes_address_set(NRF_RADIO_CRC_INCLUDES_ADDR_IEEE802154);
    nrf_radio_crc_polynominal_set(CRC_POLYNOMIAL);

    // Configure CCA
    cca_configuration_update();

    // Configure MAC Header Match Unit
    nrf_radio_mhmu_search_pattern_set(0);
    nrf_radio_mhmu_pattern_mask_set(MHMU_MASK);

    // Set channel
    channel_set(nrf_drv_radio802154_pib_channel_get());

#if 0
    nrf_radio_int_enable(NRF_RADIO_INT_FRAMESTART_MASK);
    nrf_radio_int_enable(NRF_RADIO_INT_END_MASK);
    nrf_radio_int_enable(NRF_RADIO_INT_DISABLED_MASK);
    nrf_radio_int_enable(NRF_RADIO_INT_CCAIDLE_MASK);
    nrf_radio_int_enable(NRF_RADIO_INT_CCABUSY_MASK);
    nrf_radio_int_enable(NRF_RADIO_INT_READY_MASK);
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_int_enable(NRF_RADIO_INT_BCMATCH_MASK);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_int_enable(NRF_RADIO_INT_EDEND_MASK);

    /// Workaround for missing PHYEND event in older chip revision.
    if (nrf_drv_radio802154_revision_has_phyend_event())
    {
        nrf_radio_int_enable(NRF_RADIO_INT_PHYEND_MASK);
    }
#endif
}

/// Reset radio peripheral
static void nrf_radio_reset(void)
{
    nrf_radio_power_set(false);
    nrf_radio_power_set(true);

    nrf_drv_radio802154_log(EVENT_RADIO_RESET, 0);
}

/// Initialize interrupts for radio peripheral
static void irq_init(void)
{
    NVIC_SetPriority(RADIO_IRQn, NRF_DRV_RADIO802154_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}

/// Deinitialize interrupts for radio peripheral
static void irq_deinit(void)
{
    NVIC_DisableIRQ(RADIO_IRQn);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 0);

    __DSB();
    __ISB();
}


/***************************************************************************************************
 * @section TIMER peripheral management
 **************************************************************************************************/

static void nrf_timer_init(void)
{
    nrf_timer_mode_set(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_BIT_WIDTH_16);
    nrf_timer_frequency_set(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_FREQ_1MHz);
}

/***************************************************************************************************
 * @section Energy detection management
 **************************************************************************************************/

/** Setup next iteration of energy detection procedure.
 *
 *  Energy detection procedure is performed in iterations to make sure it is performed for requested
 *  time regardless radio arbitration.
 *
 *  @param[in]  Remaining time of energy detection procedure [us].
 *
 *  @retval  true   Next iteration of energy detection procedure will be performed now.
 *  @retval  false  Next iteration of energy detection procedure will not be performed now due to
 *                  ending timeslot.
 */
static inline bool ed_iter_setup(uint32_t time_us)
{
    uint32_t us_left_in_timeslot = nrf_raal_timeslot_us_left_get();
    uint32_t next_ed_iters       = us_left_in_timeslot / ED_ITER_DURATION;

    if (next_ed_iters > ED_ITERS_OVERHEAD)
    {
        next_ed_iters -= ED_ITERS_OVERHEAD;

        if ((time_us / ED_ITER_DURATION) < next_ed_iters)
        {
            m_ed_time_left = 0;
            next_ed_iters  = time_us / ED_ITER_DURATION;
        }
        else
        {
            m_ed_time_left = time_us - (next_ed_iters * ED_ITER_DURATION);
            next_ed_iters--; // Time of ED procedure is (next_ed_iters + 1) * 128us
        }

        nrf_radio_ed_loop_count_set(next_ed_iters);

        return true;
    }
    else
    {
        if (nrf_raal_timeslot_is_granted())
        {
            irq_deinit();
            nrf_radio_reset();
        }

        m_ed_time_left = time_us;

        return false;
    }
}


/***************************************************************************************************
 * @section FSM transition request sub-procedures
 **************************************************************************************************/

/** Terminate Sleep procedure. */
static void sleep_terminate(void)
{
    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(NRF_RADIO_INT_DISABLED_MASK);
    }
}

/** Terminate RX procedure. */
static void rx_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_CRCERROR_CLEAR);
    nrf_ppi_channel_disable(PPI_CRCOK_DIS_PPI);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_CLEAR);
    nrf_timer_shorts_disable(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK | NRF_TIMER_SHORT_COMPARE2_STOP_MASK);

    if (nrf_raal_timeslot_is_granted())
    {
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
        nrf_radio_int_disable(NRF_RADIO_INT_BCMATCH_MASK | NRF_RADIO_INT_CRCERROR_MASK);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
        nrf_radio_int_disable(NRF_RADIO_INT_CRCOK_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Terminate TX ACK procedure. */
static void tx_ack_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_CRCERROR_CLEAR);
    nrf_ppi_channel_disable(PPI_CRCOK_DIS_PPI);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_CLEAR);
    nrf_timer_shorts_disable(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK | NRF_TIMER_SHORT_COMPARE2_STOP_MASK);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(nrf_drv_radio802154_revision_has_phyend_event() ?
                              NRF_RADIO_INT_PHYEND_MASK : NRF_RADIO_INT_END_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Terminate TX procedure. */
static void tx_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup(PPI_EGU_RAMP_UP, 0);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(NRF_RADIO_INT_CCABUSY_MASK);
        nrf_radio_int_disable(nrf_drv_radio802154_revision_has_phyend_event() ?
                              NRF_RADIO_INT_PHYEND_MASK : NRF_RADIO_INT_END_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Terminate RX ACK procedure. */
static void rx_ack_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup(PPI_EGU_RAMP_UP, 0);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(NRF_RADIO_INT_END_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);

        ack_matching_disable();
    }
}

/** Terminate ED procedure. */
static void ed_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup(PPI_EGU_RAMP_UP, 0);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(NRF_RADIO_INT_EDEND_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Terminate CCA procedure. */
static void cca_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);

    nrf_ppi_channel_remove_from_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup(PPI_EGU_RAMP_UP, 0);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_int_disable(NRF_RADIO_INT_CCABUSY_MASK | NRF_RADIO_INT_CCAIDLE_MASK);
        nrf_radio_shorts_set(SHORTS_IDLE);
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Terminate Continuous Carrier procedure. */
static void continuous_carrier_terminate(void)
{
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable(PPI_EGU_RAMP_UP);

    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

/** Abort transmission procedure.
 *
 *  This function is called when MAC layer requests transition from transmit to receive state.
 */
static inline void tx_procedure_abort(radio_state_t state)
{
    shorts_disable();

    assert(nrf_radio_shorts_get() == SHORTS_IDLE);

    state_set(state);

    // Stop CCA and clear result. CCA may be performed regardless disabled receiver.
    nrf_radio_task_trigger(NRF_RADIO_TASK_CCASTOP);
    nrf_radio_event_clear(NRF_RADIO_EVENT_CCAIDLE);
    nrf_radio_event_clear(NRF_RADIO_EVENT_CCABUSY);

    switch (nrf_radio_state_get())
    {
        case NRF_RADIO_STATE_TX_DISABLE:
        case NRF_RADIO_STATE_RX_DISABLE:
            // Do not enabled receiver. It will be enabled in DISABLED handler.
            break;

        default:
            nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);
            nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }

    ack_matching_disable();

    // Clear events that could have happened in critical section due to receiving frame.
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
    nrf_radio_event_clear(NRF_RADIO_EVENT_END);
    nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
}

/** Terminate ongoing operation.
 *
 * This function is called when MAC layer requests transition to another operation.
 *
 * After calling this function RADIO should enter DISABLED state and RAAL should be in continuous
 * mode.
 *
 */
static bool current_operation_terminate(nrf_drv_radio802154_term_t term_lvl)
{
    bool result = nrf_drv_radio802154_fsm_hooks_terminate(term_lvl);

    if (result)
    {
        switch (m_state)
        {
            case RADIO_STATE_SLEEP:
                sleep_terminate();
                nrf_raal_continuous_mode_enter();
                break;

            case RADIO_STATE_RX:
                // TODO: Check if PSDU is being received. If it is term_lvl must be high enough to stop it.
                rx_terminate();
                break;

            case RADIO_STATE_TX_ACK:
                if (term_lvl >= NRF_DRV_RADIO802154_TERM_802154)
                {
                    tx_ack_terminate();
                }
                else
                {
                    result = false;
                }

                break;

            case RADIO_STATE_CCA_TX:
            case RADIO_STATE_TX:
                if (term_lvl >= NRF_DRV_RADIO802154_TERM_802154)
                {
                    tx_terminate();

                    nrf_drv_radio802154_notify_transmit_failed(
                            mp_tx_data,
                            NRF_DRV_RADIO802154_TX_ERROR_ABORTED);
                }
                else
                {
                    result = false;
                }

                break;

            case RADIO_STATE_RX_ACK:
                if (term_lvl >= NRF_DRV_RADIO802154_TERM_802154)
                {
                    rx_ack_terminate();

                    nrf_drv_radio802154_notify_transmit_failed(
                            mp_tx_data,
                            NRF_DRV_RADIO802154_TX_ERROR_ABORTED);
                }
                else
                {
                    result = false;
                }

                break;

            case RADIO_STATE_ED:
                if (term_lvl >= NRF_DRV_RADIO802154_TERM_802154)
                {
                    ed_terminate();

                    nrf_drv_radio802154_notify_energy_detection_failed(
                            NRF_DRV_RADIO802154_ED_ERROR_ABORTED);
                }
                else
                {
                    result = false;
                }

                break;

            case RADIO_STATE_CCA:
                if (term_lvl >= NRF_DRV_RADIO802154_TERM_802154)
                {
                    cca_terminate();

                    nrf_drv_radio802154_notify_cca_failed(
                            NRF_DRV_RADIO802154_CCA_ERROR_ABORTED);
                }
                else
                {
                    result = false;
                }
                break;

            case RADIO_STATE_CONTINUOUS_CARRIER:
                continuous_carrier_terminate();
                break;

            default:
                assert(false);
        }
    }

    return result;
}

/** Detect if PPI starting EGU for current operation worked.
 *
 * @retval  true   PPI worked.
 * @retval  false  PPI did not work. DISABLED task should be triggered.
 */
static bool ppi_egu_worked(void)
{
    // Detect if PPIs were set before DISABLED event was notified. If not trigger DISABLE
    if (nrf_radio_state_get() != NRF_RADIO_STATE_DISABLED)
    {
        // If RADIO state is not DISABLED, it means that RADIO is still ramping down or already
        // started ramping up.
        return true;
    }

    // Wait for PPIs
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");

    if (nrf_egu_event_check(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT))
    {
        // If EGU event is set, procedure is running.
        return true;
    }
    else
    {
        return false;
    }
}

/** Clear flags describing frame being received. */
static void rx_flags_clear(void)
{
    m_flags.frame_filtered        = false;
    m_flags.rx_timeslot_requested = false;
}

/** Prepare to enter Sleep state. */
static void sleep_begin(void)
{
    nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);
    nrf_radio_int_enable(NRF_RADIO_INT_DISABLED_MASK);

    if (nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED)
    {
        // Radio is already disabled. Enter sleep state directly.
        sleep_start();
    }
}

/** Begin RX operation. */
static void receive_begin(bool disabled_was_triggered)
{
    bool     free_buffer;
    uint32_t shorts;
    bool     trigger_task_disable;

    if (!nrf_raal_timeslot_is_granted())
    {
        return;
    }

    // Clear filtering flag
    rx_flags_clear();

    free_buffer = rx_buffer_is_available();
    shorts      = free_buffer ? (SHORTS_RX | SHORTS_RX_FREE_BUFFER) : (SHORTS_RX);

    nrf_radio_tx_power_set(nrf_drv_radio802154_pib_tx_power_get());

    if (free_buffer)
    {
        nrf_radio_packet_ptr_set(rx_buffer_get());
    }

    // Set shorts
    nrf_radio_shorts_set(shorts);

    // Set BCC
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_bcc_set(BCC_INIT);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    // Enable IRQs
    //nrf_radio_int_enable(NRF_RADIO_INT_FRAMESTART_MASK);
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH | NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_int_enable(NRF_RADIO_INT_BCMATCH_MASK | NRF_RADIO_INT_CRCERROR_MASK);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable(NRF_RADIO_INT_CRCOK_MASK);

    // TODO: Set FEM

    // Set TIMERs
    nrf_timer_shorts_enable(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK | NRF_TIMER_SHORT_COMPARE2_STOP_MASK);
    nrf_timer_cc_write(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, 1);
    nrf_timer_cc_write(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, 1 + 192 - 40 -23); // 40 is ramp up, 23 is event latency
    nrf_timer_cc_write(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL2, 1 + 192 - 40 -23 + 1);

    // Clr event EGU
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

    // Set PPIs
    nrf_ppi_channel_endpoint_setup(PPI_EGU_RAMP_UP,
                                   (uint32_t)nrf_egu_event_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           EGU_EVENT),
                                   (uint32_t)nrf_radio_task_address_get(NRF_RADIO_TASK_RXEN));
    nrf_ppi_channel_endpoint_setup(PPI_EGU_TIMER_START,
                                   (uint32_t)nrf_egu_event_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           EGU_EVENT),
                                   (uint32_t)nrf_timer_task_address_get(
                                           NRF_DRV_RADIO802154_TIMER_INSTANCE,
                                           NRF_TIMER_TASK_START));
    nrf_ppi_channel_endpoint_setup(PPI_CRCERROR_CLEAR,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_CRCERROR),
                                   (uint32_t)nrf_timer_task_address_get(
                                           NRF_DRV_RADIO802154_TIMER_INSTANCE,
                                           NRF_TIMER_TASK_CLEAR));
    nrf_ppi_channel_endpoint_setup(PPI_CRCOK_DIS_PPI,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_CRCOK),
                                   (uint32_t)nrf_ppi_task_address_get(PPI_CHGRP0_DIS_TASK));
    nrf_ppi_channel_include_in_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_endpoint_setup(PPI_DISABLED_EGU,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_DISABLED),
                                   (uint32_t)nrf_egu_task_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           NRF_EGU_TASK_TRIGGER0));

    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable(PPI_EGU_TIMER_START);
    nrf_ppi_channel_enable(PPI_CRCERROR_CLEAR);
    nrf_ppi_channel_enable(PPI_CRCOK_DIS_PPI);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    // Detect if PPIs were set before DISABLED event was notified. If not trigger DISABLE
    if (!disabled_was_triggered)
    {
        trigger_task_disable = true;
    }
    else if (!ppi_egu_worked())
    {
        trigger_task_disable = true;
    }
    else
    {
        trigger_task_disable = false;
    }

    if (trigger_task_disable)
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }

    if (!free_buffer)
    {
        rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());

        if (rx_buffer_is_available())
        {
            nrf_radio_packet_ptr_set(rx_buffer_get());
            nrf_radio_shorts_set(SHORTS_RX | SHORTS_RX_FREE_BUFFER);

            if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
            {
                nrf_radio_task_trigger(NRF_RADIO_TASK_START);
            }
        }
    }
}

/** Begin TX operation. */
static bool transmit_begin(const uint8_t * p_data, bool cca, bool disabled_was_triggered)
{
    bool trigger_task_disable;

    if (!nrf_raal_timeslot_request(
            nrf_drv_radio802154_tx_duration_get(p_data[0], cca, ack_is_requested(p_data))))
    {
        return false;
    }

    nrf_radio_tx_power_set(nrf_drv_radio802154_pib_tx_power_get());
    nrf_radio_packet_ptr_set(p_data);

    // Set shorts
    nrf_radio_shorts_set(cca ? SHORTS_CCA_TX : SHORTS_TX);

    // Enable IRQs
    if (nrf_drv_radio802154_revision_has_phyend_event())
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
        nrf_radio_int_enable(NRF_RADIO_INT_PHYEND_MASK);
    }
    else
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_END);
        nrf_radio_int_enable(NRF_RADIO_INT_END_MASK);
    }

    if (cca)
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_CCABUSY);
        nrf_radio_int_enable(NRF_RADIO_INT_CCABUSY_MASK);
    }

    // TODO: Set FEM
    // Clr event EGU
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

    // Set PPIs
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_EGU_RAMP_UP,
                                            (uint32_t)nrf_egu_event_address_get(
                                                    NRF_DRV_RADIO802154_EGU_INSTANCE,
                                                    EGU_EVENT),
                                            (uint32_t)nrf_radio_task_address_get(
                                                    cca ? NRF_RADIO_TASK_RXEN :
                                                          NRF_RADIO_TASK_TXEN),
                                            (uint32_t)nrf_ppi_task_address_get(
                                                    PPI_CHGRP0_DIS_TASK));

    nrf_ppi_channel_endpoint_setup(PPI_DISABLED_EGU,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_DISABLED),
                                   (uint32_t)nrf_egu_task_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           NRF_EGU_TASK_TRIGGER0));

    nrf_ppi_channel_include_in_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    // Detect if PPIs were set before DISABLED event was notified. If not trigger DISABLE
    //       if (!disabled_was_triggered) -> trigger and exit
    if (!disabled_was_triggered)
    {
        trigger_task_disable = true;
    }
    else if (!ppi_egu_worked())
    {
        trigger_task_disable = true;
    }
    else
    {
        trigger_task_disable = false;
    }

    if (trigger_task_disable)
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }

    return true;
}

/** Begin ED operation */
static void ed_begin(bool disabled_was_triggered)
{
    bool trigger_task_disable;

    if (!ed_iter_setup(m_ed_time_left))
    {
        // Just wait for next timeslot if there is not enough time in this one.
        return;
    }

    // Set shorts
    nrf_radio_shorts_set(SHORTS_ED);

    // Enable IRQs
    nrf_radio_event_clear(NRF_RADIO_EVENT_EDEND);
    nrf_radio_int_enable(NRF_RADIO_INT_EDEND_MASK);

    // TODO: Set FEM
    // Clr event EGU
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

    // Set PPIs
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_EGU_RAMP_UP,
                                            (uint32_t)nrf_egu_event_address_get(
                                                    NRF_DRV_RADIO802154_EGU_INSTANCE,
                                                    EGU_EVENT),
                                            (uint32_t)nrf_radio_task_address_get(
                                                    NRF_RADIO_TASK_RXEN),
                                            (uint32_t)nrf_ppi_task_address_get(
                                                    PPI_CHGRP0_DIS_TASK));
    nrf_ppi_channel_endpoint_setup(PPI_DISABLED_EGU,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_DISABLED),
                                   (uint32_t)nrf_egu_task_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           NRF_EGU_TASK_TRIGGER0));

    nrf_ppi_channel_include_in_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    if (!disabled_was_triggered)
    {
        trigger_task_disable = true;
    }
    else if (!ppi_egu_worked())
    {
        trigger_task_disable = true;
    }
    else
    {
        trigger_task_disable = false;
    }

    if (trigger_task_disable)
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

static void cca_begin(bool disabled_was_triggered)
{
    bool trigger_task_disable;

    if (!nrf_raal_timeslot_request(nrf_drv_radio802154_cca_duration_get()))
    {
        return;
    }

    // Set shorts
    nrf_radio_shorts_set(SHORTS_CCA);

    // Enable IRQs
    nrf_radio_event_clear(NRF_RADIO_EVENT_CCABUSY | NRF_RADIO_EVENT_CCAIDLE);
    nrf_radio_int_enable(NRF_RADIO_INT_CCABUSY_MASK | NRF_RADIO_INT_CCAIDLE_MASK);

    // TODO: Set FEM
    // Clr event EGU
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

    // Set PPIs
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_EGU_RAMP_UP,
                                            (uint32_t)nrf_egu_event_address_get(
                                                    NRF_DRV_RADIO802154_EGU_INSTANCE,
                                                    EGU_EVENT),
                                            (uint32_t)nrf_radio_task_address_get(
                                                    NRF_RADIO_TASK_RXEN),
                                            (uint32_t)nrf_ppi_task_address_get(
                                                    PPI_CHGRP0_DIS_TASK));
    nrf_ppi_channel_endpoint_setup(PPI_DISABLED_EGU,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_DISABLED),
                                   (uint32_t)nrf_egu_task_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           NRF_EGU_TASK_TRIGGER0));

    nrf_ppi_channel_include_in_group(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    if (!disabled_was_triggered)
    {
        trigger_task_disable = true;
    }
    else if (!ppi_egu_worked())
    {
        trigger_task_disable = true;
    }
    else
    {
        trigger_task_disable = false;
    }

    if (trigger_task_disable)
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}

static void continuous_carrier_begin(bool disabled_was_triggered)
{
    bool trigger_task_disable;

    if (!nrf_raal_timeslot_is_granted())
    {
        return;
    }

    // TODO: Set FEM
    // Clr event EGU
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

    // Set PPIs
    nrf_ppi_channel_endpoint_setup(PPI_EGU_RAMP_UP,
                                   (uint32_t)nrf_egu_event_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           EGU_EVENT),
                                   (uint32_t)nrf_radio_task_address_get(NRF_RADIO_TASK_TXEN));
    nrf_ppi_channel_endpoint_setup(PPI_DISABLED_EGU,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_DISABLED),
                                   (uint32_t)nrf_egu_task_address_get(
                                           NRF_DRV_RADIO802154_EGU_INSTANCE,
                                           NRF_EGU_TASK_TRIGGER0));

    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    if (!disabled_was_triggered)
    {
        trigger_task_disable = true;
    }
    else if (!ppi_egu_worked())
    {
        trigger_task_disable = true;
    }
    else
    {
        trigger_task_disable = false;
    }

    if (trigger_task_disable)
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }
}


/***************************************************************************************************
 * @section RAAL notification handlers
 **************************************************************************************************/

void nrf_raal_timeslot_started(void)
{
    nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_TIMESLOT_STARTED);

    // Prevent interrupting of this handler by requests from higher priority code.
    nrf_drv_radio802154_critical_section_forcefully_enter();

    nrf_radio_reset();
    nrf_radio_init();
    irq_init();

    assert(nrf_radio_shorts_get() == SHORTS_IDLE);

    switch (m_state)
    {
        case RADIO_STATE_RX:
            receive_begin(false);
            break;

        case RADIO_STATE_CCA_TX:
            (void)transmit_begin(mp_tx_data, true, false);
            break;

        case RADIO_STATE_TX:
            (void)transmit_begin(mp_tx_data, false, false);
            break;

        case RADIO_STATE_ED:
            ed_begin(false);
            break;

        case RADIO_STATE_CCA:
            cca_begin(false);
            break;

        case RADIO_STATE_CONTINUOUS_CARRIER:
            continuous_carrier_begin(false);
            break;

        case RADIO_STATE_SLEEP:
            // This case may happen when sleep is requested by the next higher layer right before
            // timeslot starts and the driver uses SWI for requests and notifications. In this case
            // RAAL may report timeslot start event when exiting sleep request critical section.
            // The driver is already in SLEEP state but did not request timeslot end yet - it will
            // be requested in the next SWI handler.
            break;

        default:
            assert(false);
    }

    nrf_drv_radio802154_critical_section_exit();

    nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_TIMESLOT_STARTED);
}

void nrf_raal_timeslot_ended(void)
{
    bool result;

    nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_TIMESLOT_ENDED);

    // Prevent interrupting of this handler by requests from higher priority code.
    nrf_drv_radio802154_critical_section_forcefully_enter();

    irq_deinit();
    nrf_radio_reset();
    nrf_fem_control_deactivate();

    result = current_operation_terminate(NRF_DRV_RADIO802154_TERM_802154);
    assert(result);

    switch (m_state)
    {
        case RADIO_STATE_SLEEP:
            // Detect if timeslot is incorrectly entered again by current_operation_terminate()
            assert(false);
            break;

        case RADIO_STATE_RX:
            // TODO: Indicate error if PSDU is being received.
            break;

        case RADIO_STATE_TX_ACK:
            state_set(RADIO_STATE_RX);
            receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_TIMESLOT_ENDED);
            break;

        case RADIO_STATE_CCA_TX:
        case RADIO_STATE_TX:
        case RADIO_STATE_RX_ACK:
            state_set(RADIO_STATE_RX);
            transmit_failed_notify(NRF_DRV_RADIO802154_TX_ERROR_TIMESLOT_ENDED);
            break;

        case RADIO_STATE_ED:
        case RADIO_STATE_CCA:
        case RADIO_STATE_CONTINUOUS_CARRIER:
            // Intentionally empty.
            break;

        default:
            assert(false);
    }

    nrf_drv_radio802154_critical_section_exit();

    nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_TIMESLOT_ENDED);
}


/***************************************************************************************************
 * @section RADIO interrupt handler
 **************************************************************************************************/

#if 0
/// This event is handled when the radio starts receiving a frame.
static inline void irq_framestart_state_waiting_rx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);

#if NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    state_set(RADIO_STATE_RX_FRAME);
    nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTART);
    nrf_drv_radio802154_rx_started();

#else // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    uint8_t num_psdu_bytes = PHR_SIZE;

    state_set(RADIO_STATE_RX_HEADER);
    //assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);

    if (nrf_drv_radio802154_filter_frame_part(mp_current_rx_buffer->psdu, &num_psdu_bytes))
    {
        if (num_psdu_bytes != PHR_SIZE)
        {
            nrf_radio_bcc_set((num_psdu_bytes - 1) * 8);
        }
        else
        {
            nrf_radio_bcc_set(FCF_SIZE * 8); // To request timeslot
            m_flags.frame_filtered = true;
        }

        nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTART);

        nrf_drv_radio802154_rx_started();
    }
    else
    {
        // TODO: Find good way to drop frame.
#if 0
        auto_ack_abort(RADIO_STATE_RX);
        nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
        nrf_radio_event_clear(NRF_RADIO_EVENT_END);
        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
        nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
        receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_INVALID_FRAME);
#endif
    }

#endif // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

#if 0
    switch (nrf_radio_state_get())
    {
        case NRF_RADIO_STATE_RX:

        // If the received frame was short the radio could have changed it's state.
        case NRF_RADIO_STATE_RX_IDLE:

        // The radio could have changed state to one of the following due to enabled shorts.
        case NRF_RADIO_STATE_RX_DISABLE:
        case NRF_RADIO_STATE_DISABLED:
        case NRF_RADIO_STATE_TX_RU:
            break;

        // If something had stopped the CPU too long. Try to recover radio state.
        case NRF_RADIO_STATE_TX_IDLE:
        case NRF_RADIO_STATE_TX_DISABLE:
            auto_ack_abort(RADIO_STATE_RX);
            nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
            nrf_radio_event_clear(NRF_RADIO_EVENT_END);
            nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
            nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
            receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_RUNTIME);
            break;

        default:
            assert(false);
    }
#endif
}
#endif

/// This event is handled when the radio starts receiving an ACK frame.
static inline void irq_framestart_state_rx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);

    if ((mp_current_rx_buffer->psdu[0] < ACK_LENGTH) ||
        (mp_current_rx_buffer->psdu[0] > MAX_PACKET_SIZE))
    {
        frame_rx_start_after_ack_rx(NRF_DRV_RADIO802154_TX_ERROR_INVALID_ACK);
        nrf_radio_event_clear(NRF_RADIO_EVENT_END); // In case frame ended before task DISABLE
        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
    }
    else
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTART);

        nrf_drv_radio802154_rx_ack_started();
    }
}

/// This event is handled when the radio starts transmitting a requested frame.
static inline void irq_framestart_state_tx_frame(void)
{
    transmit_started_notify();
}

/// This event is handled when the radio starts transmitting an ACK frame.
static inline void irq_framestart_state_tx_ack(void)
{
    nrf_drv_radio802154_tx_ack_started();
}

#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
/// This event is generated during frame reception to request RAAL timeslot and to filter frame
static inline void irq_bcmatch_state_rx(void)
{
    uint8_t prev_num_psdu_bytes;
    uint8_t num_psdu_bytes;

    num_psdu_bytes      = nrf_radio_bcc_get() / 8;
    prev_num_psdu_bytes = num_psdu_bytes;

    if (!m_flags.rx_timeslot_requested)
    {
        assert(num_psdu_bytes >= PHR_SIZE + FCF_SIZE);

        if (nrf_raal_timeslot_request(nrf_drv_radio802154_rx_duration_get(
                mp_current_rx_buffer->psdu[0],
                ack_is_requested(mp_current_rx_buffer->psdu))))
        {
            m_flags.rx_timeslot_requested = true;
        }
        else
        {
            irq_deinit();
            nrf_radio_reset();

            nrf_drv_radio802154_notify_receive_failed(NRF_DRV_RADIO802154_RX_ERROR_TIMESLOT_ENDED);

            return;
        }
    }

    if (!m_flags.frame_filtered)
    {
        if (nrf_drv_radio802154_filter_frame_part(mp_current_rx_buffer->psdu,
                                                  &num_psdu_bytes))
        {
            if (num_psdu_bytes != prev_num_psdu_bytes)
            {
                nrf_radio_bcc_set(num_psdu_bytes * 8);
            }
            else
            {
                m_flags.frame_filtered = true;
            }
        }
        else if (!nrf_drv_radio802154_pib_promiscuous_get())
        {
            rx_terminate();
            receive_begin(true);

            if ((mp_current_rx_buffer->psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) !=
                FRAME_TYPE_ACK)
            {
                receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_INVALID_FRAME);
            }
        }
        else
        {
            // Promiscuous mode, allow incorrect frames. Nothing to do here.
        }
    }
}

static inline void irq_crcerror_state_rx(void)
{
    rx_flags_clear();
    nrf_radio_bcc_set(BCC_INIT);
}
#endif //!NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

static inline void irq_crcok_state_rx(void)
{
    uint8_t * p_received_psdu = mp_current_rx_buffer->psdu;
#if NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    uint8_t num_psdu_bytes      = PHR_SIZE + FCF_SIZE;
    uint8_t prev_num_psdu_bytes = 0;

    // Frame filtering
    while (num_psdu_bytes != prev_num_psdu_bytes)
    {
        prev_num_psdu_bytes = num_psdu_bytes;

        // Kepp checking consecutive parts of the frame header.
        if (nrf_drv_radio802154_filter_frame_part(mp_current_rx_buffer->psdu,
                                                  &num_psdu_bytes))
        {
            if (num_psdu_bytes == prev_num_psdu_bytes)
            {
                m_flags.frame_filtered = true;
            }
        }
        else
        {
            // If checking of any of the parts fails, break the loop.
            if (!nrf_drv_radio802154_pib_promiscuous_get())
            {
                rx_terminate();
                receive_begin(true);

                if ((p_received_psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK)
                {
                    receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_INVALID_FRAME);
                }
            }

            break;
        }
    }

    // Timeslot request
    if (m_flags.frame_filtered &&
        ack_is_requested(p_received_psdu) &&
        !nrf_raal_timeslot_request(nrf_drv_radio802154_rx_duration_get(0, true)))
    {
        // Frame is destined to this node but there is no timeslot to transmit ACK
        irq_deinit();
        nrf_radio_reset();

        rx_flags_clear();

        // Filter out received ACK frame if promiscuous mode is disabled.
        if (((p_received_psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK) ||
            nrf_drv_radio802154_pib_promiscuous_get())
        {
            mp_current_rx_buffer->free = false;
            received_frame_notify(p_received_psdu);
        }

        return;
    }
#endif // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    if (m_flags.frame_filtered || nrf_drv_radio802154_pib_promiscuous_get())
    {
        if (m_flags.frame_filtered &&
            ack_is_requested(mp_current_rx_buffer->psdu) &&
            nrf_drv_radio802154_pib_auto_ack_get())
        {
            bool wait_for_phyend;

            // Prepare ACK
            ack_prepare();
            nrf_radio_packet_ptr_set(m_ack_psdu);

            // Set shorts
            nrf_radio_shorts_set(SHORTS_TX_ACK);

            // Clear TXREADY event to detect if PPI worked
            nrf_radio_event_clear(NRF_RADIO_EVENT_TXREADY);

            // Set PPIs
            nrf_ppi_channel_endpoint_setup(PPI_TIMER_TX_ACK,
                                           (uint32_t)nrf_timer_event_address_get(
                                                   NRF_DRV_RADIO802154_TIMER_INSTANCE,
                                                   NRF_TIMER_EVENT_COMPARE1),
                                           (uint32_t)nrf_radio_task_address_get(
                                                   NRF_RADIO_TASK_TXEN));
            // TODO: Set FEM PPIs

            // Detect if PPI worked (timer is counting or TIMER event is marked)
            nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
            if (nrf_timer_cc_read(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3) <
                nrf_timer_cc_read(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1))
            {
                wait_for_phyend = true;
            }
            // TODO: Wait for PPI and EGU
            else if (nrf_radio_state_get() == NRF_RADIO_STATE_TX_RU)
            {
                wait_for_phyend = true;
            }
            else if (nrf_radio_event_get(NRF_RADIO_EVENT_TXREADY))
            {
                wait_for_phyend = true;
            }
            else
            {
                wait_for_phyend = false;
            }

            if (wait_for_phyend)
            {
                ack_pending_bit_set();
                state_set(RADIO_STATE_TX_ACK);

                // Set event handlers
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
                nrf_radio_int_disable(NRF_RADIO_INT_BCMATCH_MASK | NRF_RADIO_INT_CRCERROR_MASK);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
                nrf_radio_int_disable(NRF_RADIO_INT_CRCOK_MASK);

                if (nrf_drv_radio802154_revision_has_phyend_event())
                {
                    nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
                    nrf_radio_int_enable(NRF_RADIO_INT_PHYEND_MASK);
                }
                else
                {
                    nrf_radio_event_clear(NRF_RADIO_EVENT_END);
                    nrf_radio_int_enable(NRF_RADIO_INT_END_MASK);
                }
            }
            else
            {
                mp_current_rx_buffer->free = false;

                // RX uses the same peripherals as TX_ACK until RADIO ints are updated.
                rx_terminate();
                receive_begin(true);

                received_frame_notify(p_received_psdu);
            }
        }
        else
        {
            // Disable PPIs on DISABLED event to control TIMER.
            nrf_ppi_channel_disable(PPI_DISABLED_EGU);

            nrf_radio_shorts_set(SHORTS_RX);

            // Restart TIMER.
            nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_CLEAR);

            // Enable PPI disabled by CRCOK
            nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);

            // Enable PPIs on DISABLED event and clear event to detect if PPI worked
            nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);
            nrf_ppi_channel_enable(PPI_DISABLED_EGU);

            if (!ppi_egu_worked())
            {
                nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
            }

            rx_flags_clear();

            // Filter out received ACK frame if promiscuous mode is disabled.
            if (((p_received_psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK) ||
                nrf_drv_radio802154_pib_promiscuous_get())
            {
                // Find new RX buffer
                mp_current_rx_buffer->free = false;
                rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());

                if (rx_buffer_is_available())
                {
                    nrf_radio_packet_ptr_set(rx_buffer_get());
                    nrf_radio_shorts_set(SHORTS_RX | SHORTS_RX_FREE_BUFFER);

                    if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
                    {
                        nrf_radio_task_trigger(NRF_RADIO_TASK_START);
                    }
                }

                received_frame_notify(p_received_psdu);
            }
            else
            {
                nrf_radio_shorts_set(SHORTS_RX | SHORTS_RX_FREE_BUFFER);

                if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
                {
                    nrf_radio_task_trigger(NRF_RADIO_TASK_START);
                }
            }
        }
    }
}

/** This event is generated when radio receives invalid frame with length set to 0.
 */
static inline void irq_end_state_waiting_rx_frame(void)
{
    // Radio state is not asserted here. It can be a lot of states due to shorts.
    assert(mp_current_rx_buffer->psdu[0] == 0);

    assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);
    auto_ack_abort(RADIO_STATE_RX);
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
}

/// This event is generated if frame being received is shorter than expected MAC header length.
static inline void irq_end_state_rx_header(void)
{
    // Frame ended before header was received.
    auto_ack_abort(RADIO_STATE_RX);
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
    receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_INVALID_FRAME);
}

#if 0
/// This event is generated when radio peripheral ends receiving of a complete frame.
static inline void irq_end_state_rx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);

    switch (nrf_radio_state_get())
    {
        case NRF_RADIO_STATE_RX_IDLE:
        case NRF_RADIO_STATE_RX_DISABLE:
        case NRF_RADIO_STATE_DISABLED:
        case NRF_RADIO_STATE_TX_RU:

            if (nrf_radio_crc_status_get() == NRF_RADIO_CRC_STATUS_OK)
            {
#if NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
                if (!ack_is_requested(mp_current_rx_buffer->psdu) ||
                    m_flags.rx_timeslot_requested ||
                    nrf_raal_timeslot_request(nrf_drv_radio802154_rx_duration_get(0, true)))
                {
                    m_flags.rx_timeslot_requested = true;

                    uint8_t num_psdu_bytes      = PHR_SIZE;
                    uint8_t prev_num_psdu_bytes = 0;

                    while (num_psdu_bytes != prev_num_psdu_bytes)
                    {
                        prev_num_psdu_bytes = num_psdu_bytes;

                        // Kepp checking consecutive parts of the frame header.
                        if (nrf_drv_radio802154_filter_frame_part(mp_current_rx_buffer->psdu,
                                                                  &num_psdu_bytes))
                        {
                            if (num_psdu_bytes == prev_num_psdu_bytes)
                            {
                                m_flags.frame_filtered = true;
                            }
                        }
                        else
                        {
                            // If checking of any of the parts fails, break the loop.
                            if (!nrf_drv_radio802154_pib_promiscuous_get())
                            {
                                auto_ack_abort(RADIO_STATE_RX);
                                nrf_radio_event_clear(NRF_RADIO_EVENT_END);
                                nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
                                nrf_radio_event_clear(NRF_RADIO_EVENT_READY);

                                if ((mp_current_rx_buffer->psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) !=
                                    FRAME_TYPE_ACK)
                                {
                                    nrf_drv_radio802154_notify_receive_failed(
                                            NRF_DRV_RADIO802154_RX_ERROR_INVALID_FRAME);
                                }
                            }

                            break;
                        }
                    }
                }
                else
                {
                    irq_deinit();
                    nrf_radio_reset();

                    state_set(RADIO_STATE_WAITING_TIMESLOT);

                    nrf_drv_radio802154_notify_receive_failed(
                            NRF_DRV_RADIO802154_RX_ERROR_TIMESLOT_ENDED);
                }

                if (m_flags.frame_filtered || nrf_drv_radio802154_pib_promiscuous_get())
#endif // NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
                {
                    if ((!ack_is_requested(mp_current_rx_buffer->psdu)) ||
                        (!nrf_drv_radio802154_pib_auto_ack_get()) ||
                        (!m_flags.frame_filtered && nrf_drv_radio802154_pib_promiscuous_get()))
                    {
                        auto_ack_abort(RADIO_STATE_RX);
                        nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
                        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);

                        // Filter out received ACK frame if promiscuous mode is disabled.
                        if (((mp_current_rx_buffer->psdu[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) !=
                             FRAME_TYPE_ACK) || nrf_drv_radio802154_pib_promiscuous_get())
                        {
                            mp_current_rx_buffer->free = false;
                            received_frame_notify(mp_current_rx_buffer->psdu);
                        }
                    }
                    else
                    {
                        ack_prepare();
                        state_set(RADIO_STATE_TX_ACK);
                    }
                }
            }
            else
            {
                auto_ack_abort(RADIO_STATE_RX);
                nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
                nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
                receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_INVALID_FCS);
            }

            break;

        case NRF_RADIO_STATE_TX_IDLE:
            // CPU was hold too long.
            auto_ack_abort(RADIO_STATE_RX);
            nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
            nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
            receive_failed_notify(NRF_DRV_RADIO802154_RX_ERROR_RUNTIME);
            break;

        default:
            assert(false);
    }
}
#endif

#if 0
/// This event is generated when the radio ends transmission of ACK frame.
static inline void irq_phyend_state_tx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_TX_ACK ||
           nrf_radio_shorts_get() == SHORTS_RX_FOLLOWING); // In case PHYEND is handled before READY
    shorts_disable();

    mp_current_rx_buffer->free = false;
    received_frame_notify(mp_current_rx_buffer->psdu);

    // Clear event READY in case CPU was halted and event PHYEND is handled before READY.
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    nrf_radio_event_clear(NRF_RADIO_EVENT_END);

    state_set(RADIO_STATE_RX);
    // Receiver is enabled by shorts.
}
#endif
static void irq_phyend_state_tx_ack(void)
{
    uint8_t * p_received_psdu = mp_current_rx_buffer->psdu;

    // Disable PPIs on DISABLED event to control TIMER.
    nrf_ppi_channel_disable(PPI_DISABLED_EGU);

    nrf_radio_shorts_set(SHORTS_RX);

    // Set BCC for next reception
#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_bcc_set(BCC_INIT);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING

    nrf_radio_int_disable(nrf_drv_radio802154_revision_has_phyend_event() ?
                          NRF_RADIO_INT_PHYEND_MASK : NRF_RADIO_INT_END_MASK);

#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH | NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_int_enable(NRF_RADIO_INT_BCMATCH_MASK | NRF_RADIO_INT_CRCERROR_MASK);
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable(NRF_RADIO_INT_CRCOK_MASK);

    // Restart TIMER.
    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_DRV_RADIO802154_TIMER_INSTANCE, NRF_TIMER_TASK_CLEAR);

    // Reset PPI for RX mode
#if PPI_TIMER_TX_ACK != PPI_CRCERROR_CLEAR
#error Invalid PPI configuration
#endif
    nrf_ppi_channel_endpoint_setup(PPI_CRCERROR_CLEAR,
                                   (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_CRCERROR),
                                   (uint32_t)nrf_timer_task_address_get(
                                           NRF_DRV_RADIO802154_TIMER_INSTANCE,
                                           NRF_TIMER_TASK_CLEAR));

    // Enable PPI disabled by CRCOK
    nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);

    // Enable PPIs on DISABLED event and clear event to detect if PPI worked
    nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable(PPI_DISABLED_EGU);

    if (!ppi_egu_worked())
    {
        nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
    }

    // Find new RX buffer
    mp_current_rx_buffer->free = false;
    rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());

    if (rx_buffer_is_available())
    {
        nrf_radio_packet_ptr_set(rx_buffer_get());
        nrf_radio_shorts_set(SHORTS_RX | SHORTS_RX_FREE_BUFFER);

        if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
        {
            nrf_radio_task_trigger(NRF_RADIO_TASK_START);
        }
    }

    state_set(RADIO_STATE_RX);

    rx_flags_clear();

    received_frame_notify(p_received_psdu);
}

static void irq_phyend_state_tx_frame(void)
{
    if (ack_is_requested(mp_tx_data))
    {
        bool     trigger_task_disable;
        bool     rx_buffer_free = rx_buffer_is_available();
        uint32_t shorts = rx_buffer_free ? SHORTS_RX_ACK | SHORTS_RX_FREE_BUFFER : SHORTS_RX_ACK;

        // Disable EGU PPI to prevent unsynchronized PPIs
        nrf_ppi_channel_disable(PPI_DISABLED_EGU);

        nrf_radio_shorts_set(shorts);

        if (rx_buffer_free)
        {
            nrf_radio_packet_ptr_set(rx_buffer_get());
        }

        nrf_radio_int_disable(NRF_RADIO_INT_CCABUSY_MASK);

        if (nrf_drv_radio802154_revision_has_phyend_event())
        {
            nrf_radio_int_disable(NRF_RADIO_INT_PHYEND_MASK);
            nrf_radio_event_clear(NRF_RADIO_EVENT_END);
            nrf_radio_int_enable(NRF_RADIO_INT_END_MASK);
        }

        // TODO: FEM reconfiguration for RX ACK

        nrf_ppi_channel_and_fork_endpoint_setup(PPI_EGU_RAMP_UP,
                                                (uint32_t)nrf_egu_event_address_get(
                                                        NRF_DRV_RADIO802154_EGU_INSTANCE,
                                                        EGU_EVENT),
                                                (uint32_t)nrf_radio_task_address_get(
                                                        NRF_RADIO_TASK_RXEN),
                                                (uint32_t)nrf_ppi_task_address_get(
                                                        PPI_CHGRP0_DIS_TASK));

        nrf_egu_event_clear(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT);

        // Enable PPI disabled by DISABLED event
        nrf_ppi_channel_enable(PPI_EGU_RAMP_UP);

        // Enable EGU PPI to start all PPIs synchronously
        nrf_ppi_channel_enable(PPI_DISABLED_EGU);

        state_set(RADIO_STATE_RX_ACK);

        if (nrf_radio_state_get() != NRF_RADIO_STATE_DISABLED)
        {
            // If state is not DISABLED, it means that RADIO is still ramping down or PPI worked.
            trigger_task_disable = false;
        }

        //       wait for PPIs (TODO: test how long)

        else if (nrf_egu_event_check(NRF_DRV_RADIO802154_EGU_INSTANCE, EGU_EVENT))
        {
            // If EGU event is set, procedure is running.
            trigger_task_disable = false;
        }
        else
        {
            trigger_task_disable = true;
        }

        if (trigger_task_disable)
        {
            nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
        }

        if (!rx_buffer_free)
        {
            rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());

            if (rx_buffer_is_available())
            {
                nrf_radio_packet_ptr_set(rx_buffer_get());
                nrf_radio_shorts_set(SHORTS_RX_ACK | SHORTS_RX_FREE_BUFFER);

                if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
                {
                    nrf_radio_task_trigger(NRF_RADIO_TASK_START);
                }
            }
        }

        ack_matching_enable();
    }
    else
    {
        tx_terminate();
        state_set(RADIO_STATE_RX);
        receive_begin(true);

        transmitted_frame_notify(NULL, 0, 0);
    }
}

/** This event may occur at the beginning of transmission procedure (the procedure already has
 *  disabled shorts).
 */
static inline void irq_end_state_cca_before_tx(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
}

#if 0
/// This event is generated when the radio ends transmission of a frame.
static inline void irq_phyend_state_tx_frame(void)
{
    shorts_disable();
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);

    // Clear event READY in case CPU was halted and event PHYEND is handled before READY.
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    nrf_radio_event_clear(NRF_RADIO_EVENT_END);

    if (!ack_is_requested(mp_tx_data))
    {
        transmitted_frame_notify(NULL, 0, 0);

        state_set(RADIO_STATE_RX);
    }
    else
    {
        state_set(RADIO_STATE_RX_ACK);

        ack_matching_enable();
    }

    // Task DISABLE is triggered by shorts.
}
#endif

static void irq_end_state_rx_ack(void)
{
    bool          ack_match = ack_is_matched();
    rx_buffer_t * p_ack_buffer;

    if (ack_match)
    {
        p_ack_buffer = mp_current_rx_buffer;
        mp_current_rx_buffer->free = false;
    }

    rx_ack_terminate();
    state_set(RADIO_STATE_RX);
    receive_begin(true);

    if (ack_match)
    {
        transmitted_frame_notify(p_ack_buffer->psdu,                // psdu
                                 rssi_last_measurement_get(),       // rssi
                                 RX_FRAME_LQI(p_ack_buffer->psdu)); // lqi;
    }
    else
    {
        transmit_failed_notify(NRF_DRV_RADIO802154_TX_ERROR_INVALID_ACK);
    }
}

#if 0
/// This event is generated when the radio ends receiving of ACK frame.
static inline void irq_end_state_rx_ack(void)
{
    nrf_drv_radio802154_tx_error_t error = NRF_DRV_RADIO802154_TX_ERROR_NONE;

    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    if (ack_is_matched())
    {
        mp_current_rx_buffer->free = false;
        transmitted_frame_notify(mp_current_rx_buffer->psdu,                // psdu
                                 rssi_last_measurement_get(),               // rssi
                                 RX_FRAME_LQI(mp_current_rx_buffer->psdu)); // lqi;
    }
    else
    {
        error = NRF_DRV_RADIO802154_TX_ERROR_INVALID_ACK;
    }

    frame_rx_start_after_ack_rx(error);
}
#endif

static void irq_disabled_state_sleep(void)
{
    sleep_start();
}

/// This event is generated when radio peripheral disables in order to enter sleep state.
static inline void irq_disabled_state_disabling(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);
    sleep_start();
}

/** This event is generated when the driver enters receive state.
 *
 *  The radio is now disabled and the driver starts enabling receiver.
 */
static inline void irq_disabled_state_waiting_rx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);

    while (nrf_radio_state_get() == NRF_RADIO_STATE_TX_DISABLE)
    {
        // This event can be handled in TXDISABLE state due to double DISABLE event (IC-15879).
        // This busy loop waits to the end of this state.
    }

    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    rx_enable();

    rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());
    nrf_radio_tx_power_set(nrf_drv_radio802154_pib_tx_power_get());

    // Clear this event after RXEN task in case event is triggered just before.
    nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);
}

/** This event is generated in case both END and DISABLED events are notified between
 *  checking if END event was notified and checking if DISABLED event was notified.
 *
 *  In this invalid and rare case just drop received frame.
 */
static inline void irq_disabled_state_rx_frame(void)
{
    assert(nrf_radio_event_get(NRF_RADIO_EVENT_END));
    assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);

    auto_ack_abort(RADIO_STATE_RX);
    nrf_radio_event_clear(NRF_RADIO_EVENT_END);
    nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
}

/** This event is generated during automatic ACK transmission.
 *
 *  Receiver is disabled and transmitter is being enabled by shorts.
 */
static inline void irq_disabled_state_tx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_RX_INITIAL);

    shorts_rx_following_set();
    ack_pending_bit_set();

    // IC-15879
    nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);

    if (nrf_radio_state_get() == NRF_RADIO_STATE_TX_IDLE)
    {
        // CPU was hold too long.
        auto_ack_abort(RADIO_STATE_RX);
        nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
    }
    else
    {
        // nrf_fem_control_time_latch was called at the beginning of the irq handler,
        // to capture the time as close to short occurence as possible
        nrf_fem_control_pa_set(true, false);
    }
}

/** This event is generated before CCA procedure starts.
 *
 *  The radio is disabled and the drivers enables receiver in order to start CCA procedure.
 */
static inline void irq_disabled_state_cca_before_tx(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    rx_enable();
}

/** This event is generated before transmission of a frame starts.
 *
 *  The radio is disabled and enabling transmitter is requested by shorts or by the driver.
 */
static inline void irq_disabled_state_tx_frame(void)
{
    if (nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED)
    {
        shorts_tx_frame_set();

        tx_enable();
    }

    assert(nrf_radio_shorts_get() == SHORTS_TX_FRAME);
}

/** This event is generated when radio is disabled after transmission of a frame with ACK request.
 *
 *  The driver enables receiver to receive ACK frame.
 */
static inline void irq_disabled_state_rx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    rx_enable();

    if (mp_current_rx_buffer == NULL || (!mp_current_rx_buffer->free))
    {
        rx_buffer_in_use_set(nrf_drv_radio802154_rx_buffer_free_find());
    }
}

/** This event is generated before energy detection procedure.
 *
 *  The radio is disabled and the driver enables receiver in order to start energy detection
 *  procedure.
 */
static inline void irq_disabled_state_ed(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    rx_enable();
}

/** This event is generated before stand-alone CCA procedure.
 *
 *  The radio is disabled and the driver enables receiver in order to start CCA procedure.
 */
static inline void irq_disabled_state_cca(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    rx_enable();
}

/** This event is generated before continuous-carrier procedure is started.
 *
 *  The radio is disabled and the driver enables transmitter in order to start emitting carrier
 *  continuously.
 */
static inline void irq_disabled_state_continuous_carrier(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_DISABLED);

    tx_enable();
}

/** This event is generated when receiver is ready to start receiving a frame.
 *
 *  Driver checks if buffer for a frame is available and starts receiver.
 */
static inline void irq_ready_state_waiting_rx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    if ((mp_current_rx_buffer != NULL) && (mp_current_rx_buffer->free))
    {
        shorts_rx_initial_set();
        rx_frame_start();
    }
}

/** This event is generated when transmitter is ready to transmit ACK frame.
 *
 *  Transmission is started by shorts. The driver sets shorts to gracefully end transmission of ACK.
 */
static inline void irq_ready_state_tx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_RX_FOLLOWING);
    shorts_tx_ack_set();
}

/** This event is generated when receiver is ready to start CCA procedure.
 *
 *  The driver prepares for transmission and starts CCA procedure.
 */
static inline void irq_ready_state_cca_before_tx(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    state_set(RADIO_STATE_TX);

    shorts_tx_frame_set();
    nrf_radio_task_trigger(NRF_RADIO_TASK_CCASTART);
}

/** This event is generated when transmitter is ready to transmit a frame.
 *
 *  Transmission is started by shorts.
 */
static inline void irq_ready_state_tx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_TX_FRAME);
}

/// This event is generated when receiver is ready to receive an ACK frame.
static inline void irq_ready_state_rx_ack(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    if (mp_current_rx_buffer == NULL || (!mp_current_rx_buffer->free))
    {
        frame_rx_start_after_ack_rx(NRF_DRV_RADIO802154_TX_ERROR_NO_MEM);
    }
    else
    {
        rx_start();
    }
}

/// This event is generated when receiver is ready to start energy detection procedure.
static inline void irq_ready_state_ed(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);
    nrf_radio_task_trigger(NRF_RADIO_TASK_EDSTART);
}

/// This event is generated when receiver is ready to start CCA procedure.
static inline void irq_ready_state_cca(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);
    nrf_radio_task_trigger(NRF_RADIO_TASK_CCASTART);
}

/// This event is generated when transmitter is emitting carrier continuously.
static inline void irq_ready_state_continuous_carrier(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_TX_IDLE);
}

#if !NRF_DRV_RADIO802154_SHORT_CCAIDLE_TXEN
/// This event is generated when CCA reports that channel is idle.
static inline void irq_ccaidle_state_tx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_TX_FRAME);

    nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
}
#endif // NRF_DRV_RADIO802154_SHORT_CCAIDLE_TXEN

/// This event is generated when CCA reports idle channel during stand-alone procedure.
static inline void irq_ccaidle_state_cca(void)
{
    cca_terminate();
    state_set(RADIO_STATE_RX);
    receive_begin(true);

    cca_notify(true);
}

static inline void irq_ccabusy_state_tx_frame(void)
{
    tx_terminate();
    state_set(RADIO_STATE_RX);
    receive_begin(true);

    transmit_failed_notify(NRF_DRV_RADIO802154_TX_ERROR_BUSY_CHANNEL);
}

static inline void irq_ccabusy_state_cca(void)
{
    cca_terminate();
    state_set(RADIO_STATE_RX);
    receive_begin(true);

    cca_notify(false);
}

#if 0
/// This event is generated when CCA reports busy channel prior to transmission.
static inline void irq_ccabusy_state_tx_frame(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_TX_FRAME);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    shorts_disable();
    state_set(RADIO_STATE_RX);
    nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);

    transmit_failed_notify(NRF_DRV_RADIO802154_TX_ERROR_BUSY_CHANNEL);
}

/// This event is generated when CCA reports busy channel during stand-alone procedure.
static inline void irq_ccabusy_state_cca(void)
{
    assert(nrf_radio_shorts_get() == SHORTS_IDLE);
    assert(nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE);

    state_set(RADIO_STATE_RX);
    nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);

    cca_notify(false);
}
#endif

/// This event is generated when energy detection procedure ends.
static inline void irq_edend_state_ed(void)
{
    uint32_t result = nrf_radio_ed_sample_get();
    m_ed_result     = result > m_ed_result ? result : m_ed_result;

    if (m_ed_time_left)
    {
        if (ed_iter_setup(m_ed_time_left))
        {
            nrf_radio_task_trigger(NRF_RADIO_TASK_EDSTART);
        }
    }
    else
    {
        // In case channel change was requested during energy detection procedure.
        channel_set(nrf_drv_radio802154_pib_channel_get());

        ed_terminate();
        state_set(RADIO_STATE_RX);
        receive_begin(true);

        energy_detected_notify(m_ed_result);
    }
}

/// Handler of radio interrupts.
static inline void irq_handler(void)
{
    nrf_fem_control_time_latch();

    nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_IRQ_HANDLER);

    // Prevent interrupting of this handler by requests from higher priority code.
    nrf_drv_radio802154_critical_section_forcefully_enter();

#if 0
    if (nrf_radio_event_get(NRF_RADIO_EVENT_FRAMESTART))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_FRAMESTART);
        nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);

        switch (m_state)
        {
            case RADIO_STATE_RX:
                irq_framestart_state_waiting_rx_frame();
                break;

            case RADIO_STATE_RX_ACK:
                irq_framestart_state_rx_ack();
                break;

            case RADIO_STATE_TX:
            case RADIO_STATE_CCA_BEFORE_TX: // This could happen at the beginning of transmission procedure.
                irq_framestart_state_tx_frame();
                break;

            case RADIO_STATE_TX_ACK:
                irq_framestart_state_tx_ack();
                break;

            case RADIO_STATE_WAITING_TIMESLOT:
                break;

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_FRAMESTART);
    }
#endif

#if !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING
    // Check MAC frame header.
    if (nrf_radio_int_get(NRF_RADIO_INT_BCMATCH_MASK) && nrf_radio_event_get(NRF_RADIO_EVENT_BCMATCH))
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);

        // TODO: switch states, add logs
        irq_bcmatch_state_rx();
    }

    if (nrf_radio_int_get(NRF_RADIO_INT_CRCERROR_MASK) && nrf_radio_event_get(NRF_RADIO_EVENT_CRCERROR))
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_CRCERROR);

        // TODO: switch states, add logs
        irq_crcerror_state_rx();
    }
#endif // !NRF_DRV_RADIO802154_DISABLE_BCC_MATCHING


    if (nrf_radio_int_get(NRF_RADIO_INT_CRCOK_MASK) && nrf_radio_event_get(NRF_RADIO_EVENT_CRCOK))
    {
        nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);

        // TODO: switch states, add logs
        irq_crcok_state_rx();
    }

    if (nrf_drv_radio802154_revision_has_phyend_event() &&
        nrf_radio_int_get(NRF_RADIO_INT_PHYEND_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_PHYEND))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_PHYEND);
        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);

        switch (m_state)
        {
#if 0
            case RADIO_STATE_RX:
            case RADIO_STATE_RX_HEADER:
            case RADIO_STATE_RX_FRAME:
                break;
#endif

            case RADIO_STATE_TX_ACK:
                 irq_phyend_state_tx_ack();
                 break;

#if 0
            case RADIO_STATE_CCA_BEFORE_TX:
                break;
#endif

            case RADIO_STATE_CCA_TX:
            case RADIO_STATE_TX:
                irq_phyend_state_tx_frame();
                break;

#if 0
            case RADIO_STATE_RX_ACK:
            case RADIO_STATE_WAITING_TIMESLOT:
                break;
#endif

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_PHYEND);
    }

    if (nrf_radio_int_get(NRF_RADIO_INT_END_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_END))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_END);
        nrf_radio_event_clear(NRF_RADIO_EVENT_END);

        switch (m_state)
        {
#if 0
            case RADIO_STATE_RX:
                irq_end_state_waiting_rx_frame();
                break;

            case RADIO_STATE_RX_HEADER:
                irq_end_state_rx_header();
                break;

            case RADIO_STATE_RX_FRAME:
                irq_end_state_rx_frame();
                break;
#endif

            case RADIO_STATE_TX_ACK:
                if (!nrf_drv_radio802154_revision_has_phyend_event())
                {
                    irq_phyend_state_tx_ack();
                }

                break;

#if 0
            case RADIO_STATE_CCA_BEFORE_TX: // This could happen at the beginning of transmission
                                            // procedure (the procedure already has disabled shorts)
                irq_end_state_cca_before_tx();
                break;
#endif

            case RADIO_STATE_CCA_TX:
            case RADIO_STATE_TX:
                if (!nrf_drv_radio802154_revision_has_phyend_event())
                {
                    irq_phyend_state_tx_frame();
                }

                break;

            case RADIO_STATE_RX_ACK: // Ended receiving of ACK.
                irq_end_state_rx_ack();
                break;

#if 0
            case RADIO_STATE_WAITING_TIMESLOT:
                // Exit as soon as possible when waiting for timeslot.
                break;
#endif

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_END);
    }

    if (nrf_radio_int_get(NRF_RADIO_INT_DISABLED_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_DISABLED))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_DISABLED);
        nrf_radio_event_clear(NRF_RADIO_EVENT_DISABLED);

        switch (m_state)
        {
            case RADIO_STATE_SLEEP:
                irq_disabled_state_sleep();
                break;
#if 0
            case RADIO_STATE_RX:
                irq_disabled_state_waiting_rx_frame();
                break;

            case RADIO_STATE_RX_HEADER:
            case RADIO_STATE_RX_FRAME:
                irq_disabled_state_rx_frame();
                break;

            case RADIO_STATE_TX_ACK:
                irq_disabled_state_tx_ack();
                break;

            case RADIO_STATE_CCA_BEFORE_TX:
                irq_disabled_state_cca_before_tx();
                break;

            case RADIO_STATE_TX:
                irq_disabled_state_tx_frame();
                break;

            case RADIO_STATE_RX_ACK:
                irq_disabled_state_rx_ack();
                break;

            case RADIO_STATE_ED:
                irq_disabled_state_ed();
                break;

            case RADIO_STATE_CCA:
                irq_disabled_state_cca();
                break;

            case RADIO_STATE_CONTINUOUS_CARRIER:
                irq_disabled_state_continuous_carrier();
                break;

            case RADIO_STATE_WAITING_TIMESLOT:
                // Exit as soon as possible when waiting for timeslot.
                break;
#endif
            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_DISABLED);
    }
#if 0
    if (nrf_radio_event_get(NRF_RADIO_EVENT_READY))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_READY);
        nrf_radio_event_clear(NRF_RADIO_EVENT_READY);

        switch (m_state)
        {
            case RADIO_STATE_RX:
                irq_ready_state_waiting_rx_frame();
                break;

            case RADIO_STATE_TX_ACK:
                irq_ready_state_tx_ack();
                break;

            case RADIO_STATE_CCA_BEFORE_TX:
                irq_ready_state_cca_before_tx();
                break;

            case RADIO_STATE_TX:
                irq_ready_state_tx_frame();
                break;

            case RADIO_STATE_RX_ACK:
                irq_ready_state_rx_ack();
                break;

            case RADIO_STATE_ED:
                irq_ready_state_ed();
                break;

            case RADIO_STATE_CCA:
                irq_ready_state_cca();
                break;

            case RADIO_STATE_CONTINUOUS_CARRIER:
                irq_ready_state_continuous_carrier();
                break;

            case RADIO_STATE_WAITING_TIMESLOT:
                // Exit as soon as possible when waiting for timeslot.
                break;

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_READY);
    }
#endif

    if (nrf_radio_int_get(NRF_RADIO_INT_CCAIDLE_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_CCAIDLE))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_CCAIDLE);
        nrf_radio_event_clear(NRF_RADIO_EVENT_CCAIDLE);

        switch (m_state)
        {
#if 0
            case RADIO_STATE_TX:
#if !NRF_DRV_RADIO802154_SHORT_CCAIDLE_TXEN
                irq_ccaidle_state_tx_frame();
#else
                // nrf_fem_control_time_latch was called at the beginning of the irq handler,
                // to capture the time as close to short occurence as possible
                nrf_fem_control_pa_set(true, true);
#endif
                break;
#endif

            case RADIO_STATE_CCA:
                irq_ccaidle_state_cca();
                break;

#if 0
#if NRF_DRV_RADIO802154_SHORT_CCAIDLE_TXEN
            case RADIO_STATE_RX:
            case RADIO_STATE_RX_ACK:
                // If CCAIDLE->TXEN short is enabled, this event may be handled after event END.
                break;
#endif

            case RADIO_STATE_WAITING_TIMESLOT:
                // Exit as soon as possible when waiting for timeslot.
                break;
#endif

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_CCAIDLE);
    }

    if (nrf_radio_int_get(NRF_RADIO_INT_CCABUSY_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_CCABUSY))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_CCABUSY);
        nrf_radio_event_clear(NRF_RADIO_EVENT_CCABUSY);

        switch (m_state)
        {
            case RADIO_STATE_CCA_TX:
            case RADIO_STATE_TX:
                irq_ccabusy_state_tx_frame();
                break;

            case RADIO_STATE_CCA:
                irq_ccabusy_state_cca();
                break;

#if 0
            case RADIO_STATE_WAITING_TIMESLOT:
                // Exit as soon as possible when waiting for timeslot.
                break;
#endif

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_CCABUSY);
    }

    if (nrf_radio_int_get(NRF_RADIO_INT_EDEND_MASK) &&
        nrf_radio_event_get(NRF_RADIO_EVENT_EDEND))
    {
        nrf_drv_radio802154_log(EVENT_TRACE_ENTER, FUNCTION_EVENT_EDEND);
        nrf_radio_event_clear(NRF_RADIO_EVENT_EDEND);

        switch (m_state)
        {
            case RADIO_STATE_ED:
                irq_edend_state_ed();
                break;

            default:
                assert(false);
        }

        nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_EVENT_EDEND);
    }

    nrf_drv_radio802154_critical_section_exit();

    nrf_drv_radio802154_log(EVENT_TRACE_EXIT, FUNCTION_IRQ_HANDLER);
}


/***************************************************************************************************
 * @section API functions
 **************************************************************************************************/

void nrf_drv_radio802154_fsm_init(void)
{
    const uint8_t ack_psdu[] = {0x05, ACK_HEADER_WITH_PENDING, 0x00, 0x00, 0x00, 0x00};
    memcpy(m_ack_psdu, ack_psdu, sizeof(ack_psdu));

    nrf_timer_init();
}

void nrf_drv_radio802154_fsm_deinit(void)
{
    if (nrf_raal_timeslot_is_granted())
    {
        nrf_radio_reset();
    }

    irq_deinit();
}

radio_state_t nrf_drv_radio802154_fsm_state_get(void)
{
    return m_state;
}

bool nrf_drv_radio802154_fsm_sleep(nrf_drv_radio802154_term_t term_lvl)
{
    bool result = true;

    if (m_state != RADIO_STATE_SLEEP)
    {
        result = current_operation_terminate(term_lvl);

        if (result)
        {
            state_set(RADIO_STATE_SLEEP);
            sleep_begin();
        }
    }

    return result;
}

bool nrf_drv_radio802154_fsm_receive(nrf_drv_radio802154_term_t term_lvl)
{
    bool result = true;

    if ((m_state != RADIO_STATE_RX) && (m_state != RADIO_STATE_TX_ACK))
    {
        result = current_operation_terminate(term_lvl);

        if (result)
        {
            state_set(RADIO_STATE_RX);
            receive_begin(true);
        }
    }

    return result;
}

bool nrf_drv_radio802154_fsm_transmit(nrf_drv_radio802154_term_t term_lvl,
                                      const uint8_t            * p_data,
                                      bool                       cca)
{
    bool result = current_operation_terminate(term_lvl);

    if (result)
    {
        mp_tx_data = p_data;
        result     = transmit_begin(p_data, cca, true);
    }

    if (result)
    {
        state_set(cca ? RADIO_STATE_CCA_TX : RADIO_STATE_TX);
    }

    return result;
#if 0
    result = false;
    mp_tx_data  = p_data;

    if (m_state == RADIO_STATE_RX)
    {
        if (nrf_raal_timeslot_request(nrf_drv_radio802154_tx_duration_get(p_data[0],
                                                                          cca,
                                                                          ack_is_requested(p_data))))
        {
            auto_ack_abort(cca ? RADIO_STATE_CCA_BEFORE_TX : RADIO_STATE_TX);

            nrf_radio_tx_power_set(nrf_drv_radio802154_pib_tx_power_get());
            nrf_radio_packet_ptr_set(p_data);

            // Clear events that could have happened in critical section due to receiving frame or RX ramp up.
            nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
            nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
            nrf_radio_event_clear(NRF_RADIO_EVENT_END);
            nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
            nrf_radio_event_clear(NRF_RADIO_EVENT_READY);

            // Check 2nd time in case this procedure was interrupted.
            if (nrf_raal_timeslot_request(
                    nrf_drv_radio802154_tx_duration_get(p_data[0], cca, ack_is_requested(p_data))))
            {
                result = true;
            }
            else
            {
                irq_deinit();
                nrf_radio_reset();

                state_set(RADIO_STATE_WAITING_TIMESLOT);
            }
        }
    }

    return result;
#endif
}

bool nrf_drv_radio802154_fsm_energy_detection(nrf_drv_radio802154_term_t term_lvl,
                                              uint32_t                   time_us)
{
    bool result = current_operation_terminate(term_lvl);

    if (result)
    {
        state_set(RADIO_STATE_ED);
        m_ed_time_left = time_us;
        m_ed_result    = 0;
        ed_begin(true);
    }

    return result;
#if 0
    result = false;

    switch (m_state)
    {
        case RADIO_STATE_SLEEP:
            state_set(RADIO_STATE_ED);
            m_ed_time_left = time_us;
            m_ed_result    = 0;

            nrf_raal_continuous_mode_enter();

            result = true;

            break;

        case RADIO_STATE_RX:
            m_ed_result = 0;

            if (ed_iter_setup(time_us))
            {
                auto_ack_abort(RADIO_STATE_ED);

                assert(nrf_radio_shorts_get() == SHORTS_IDLE);

                // Clear events that could have happened in critical section due to receiving
                // frame or RX ramp up.
                nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
                nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
                nrf_radio_event_clear(NRF_RADIO_EVENT_END);
                nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
                nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
            }
            else
            {
                state_set(RADIO_STATE_ED);
            }

            result = true;

            break;

        case RADIO_STATE_DISABLING:
        case RADIO_STATE_RX_HEADER:
        case RADIO_STATE_RX_FRAME:
        case RADIO_STATE_TX_ACK:
        case RADIO_STATE_WAITING_TIMESLOT:
            break;

        default:
            assert(false); // This should not happen.
    }

    return result;
#endif
}

bool nrf_drv_radio802154_fsm_cca(nrf_drv_radio802154_term_t term_lvl)
{
    bool result = current_operation_terminate(term_lvl);

    if (result)
    {
        state_set(RADIO_STATE_CCA);
        cca_begin(true);
    }

    return result;
#if 0
    result = false;

    switch (m_state)
    {
        case RADIO_STATE_SLEEP:
            state_set(RADIO_STATE_CCA);

            nrf_raal_continuous_mode_enter();

            result = true;

            break;

        case RADIO_STATE_RX:
            if (nrf_raal_timeslot_request(nrf_drv_radio802154_cca_duration_get()))
            {
                auto_ack_abort(RADIO_STATE_CCA);

                // Clear events that could have happened in critical section due to receiving
                // frame or RX ramp up.
                nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
                nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
                nrf_radio_event_clear(NRF_RADIO_EVENT_END);
                nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
                nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
            }
            else
            {
                state_set(RADIO_STATE_CCA);
            }

            result = true;

            break;

        case RADIO_STATE_DISABLING:
        case RADIO_STATE_RX_HEADER:
        case RADIO_STATE_RX_FRAME:
        case RADIO_STATE_TX_ACK:
        case RADIO_STATE_WAITING_TIMESLOT:
            break;

        default:
            assert(false); // This should not happen.
    }

    return result;
#endif
}

bool nrf_drv_radio802154_fsm_continuous_carrier(nrf_drv_radio802154_term_t term_lvl)
{
    bool result = current_operation_terminate(term_lvl);

    if (result)
    {
        state_set(RADIO_STATE_CONTINUOUS_CARRIER);
        continuous_carrier_begin(true);
    }

    return result;
#if 0
    // TODO: Use priority to abort ongoing operations with current_operation_abort()
    (void)term_lvl;

    bool result = false;

    if (m_state == RADIO_STATE_RX ||
        m_state == RADIO_STATE_SLEEP)
    {
        auto_ack_abort(RADIO_STATE_CONTINUOUS_CARRIER);

        // Clear events that could have happened in critical section due to receiving frame or RX ramp up.
        nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
        nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
        nrf_radio_event_clear(NRF_RADIO_EVENT_END);
        nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
        nrf_radio_event_clear(NRF_RADIO_EVENT_READY);

        result = true;
    }

    return result;
#endif
}

bool nrf_drv_radio802154_fsm_notify_buffer_free(uint8_t * p_data)
{
    rx_buffer_t * p_buffer = (rx_buffer_t *)p_data;

    p_buffer->free = true;

    if (!nrf_raal_timeslot_is_granted())
    {
        return true;
    }

    switch (m_state)
    {
        case RADIO_STATE_RX:
            if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
            {
                assert(nrf_radio_shorts_get() == SHORTS_RX);

                rx_buffer_in_use_set(p_buffer);

                nrf_radio_packet_ptr_set(rx_buffer_get());
                nrf_radio_shorts_set(SHORTS_RX | SHORTS_RX_FREE_BUFFER);

                nrf_radio_task_trigger(NRF_RADIO_TASK_START);
            }

            break;

        case RADIO_STATE_RX_ACK:
            if (nrf_radio_state_get() == NRF_RADIO_STATE_RX_IDLE)
            {
                assert(nrf_radio_shorts_get() == SHORTS_RX_ACK);

                rx_buffer_in_use_set(p_buffer);

                nrf_radio_packet_ptr_set(rx_buffer_get());
                nrf_radio_shorts_set(SHORTS_RX_ACK | SHORTS_RX_FREE_BUFFER);

                nrf_radio_task_trigger(NRF_RADIO_TASK_START);
            }

            break;

        default:
            // Don't perform any action in any other state (receiver should not be started).
            break;
    }

    return true;
}

bool nrf_drv_radio802154_fsm_channel_update(void)
{
    switch (m_state)
    {
        case RADIO_STATE_RX:
        {
            bool result;

            if (nrf_raal_timeslot_is_granted())
            {
                channel_set(nrf_drv_radio802154_pib_channel_get());
            }

            result = current_operation_terminate(NRF_DRV_RADIO802154_TERM_NONE);

            if (result)
            {
                receive_begin(true);
            }
        }
#if 0
            auto_ack_abort(RADIO_STATE_RX);

            // Clear events that could have happened in critical section due to receiving
            // frame or RX ramp up.
            nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
            nrf_radio_event_clear(NRF_RADIO_EVENT_BCMATCH);
            nrf_radio_event_clear(NRF_RADIO_EVENT_END);
            nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
            nrf_radio_event_clear(NRF_RADIO_EVENT_READY);
#endif
            break;

        case RADIO_STATE_CONTINUOUS_CARRIER:
            if (nrf_raal_timeslot_is_granted())
            {
                channel_set(nrf_drv_radio802154_pib_channel_get());
                nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
            }

            break;

        case RADIO_STATE_TX_ACK:
        case RADIO_STATE_CCA_TX:
        case RADIO_STATE_TX:
        case RADIO_STATE_RX_ACK:
        case RADIO_STATE_CCA:
            if (nrf_raal_timeslot_is_granted())
            {
                channel_set(nrf_drv_radio802154_pib_channel_get());
            }

            break;

        case RADIO_STATE_SLEEP:
        case RADIO_STATE_ED:
            // Don't perform any action in these states (channel will be updated on state change).
            break;
    }

    return true;
}

bool nrf_drv_radio802154_fsm_cca_cfg_update(void)
{
    if (nrf_raal_timeslot_is_granted())
    {
        cca_configuration_update();
    }

    return true;
}

#if NRF_DRV_RADIO802154_INTERNAL_IRQ_HANDLING
void RADIO_IRQHandler(void)
#else // NRF_DRV_RADIO802154_INTERNAL_IRQ_HANDLING
void nrf_drv_radio802154_fsm_irq_handler(void)
#endif // NRF_DRV_RADIO802154_INTERNAL_IRQ_HANDLING
{
    irq_handler();
}
