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
 *   This file implements delayed transmission and reception features.
 *
 */

#include "nrf_802154_delayed_trx.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "nrf_802154_notification.h"
#include "nrf_802154_pib.h"
#include "nrf_802154_procedures_duration.h"
#include "nrf_802154_request.h"
#include "nrf_802154_rsch.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

#define TX_SETUP_TIME 190u ///< Time [us] needed to change channel, stop rx and setup tx procedure.
#define RX_SETUP_TIME 93u  ///< Time [us] needed to change channel, stop tx and setup rx procedure.

/**
 * @brief States of delayed operations.
 */
typedef enum
{
    DELAYED_TRX_OP_STATE_STOPPED, ///< Delayed operation stopped.
    DELAYED_TRX_OP_STATE_PENDING, ///< Delayed operation scheduled and waiting for timeslot.
    DELAYED_TRX_OP_STATE_ONGOING, ///< Delayed operation ongoing (during timeslot).
    DELAYED_TRX_OP_STATE_NB       ///< Number of delayed operation states.
} delayed_trx_op_state_t;

/**
 * @brief TX delayed operation configuration.
 */
static const uint8_t * mp_tx_psdu;         ///< Pointer to PHR + PSDU of the frame requested to transmit.
static bool            m_tx_cca;           ///< If CCA should be performed prior to transmission.
static uint8_t         m_tx_channel;       ///< Channel number on which transmission should be performed.

/**
 * @brief RX delayed operation configuration.
 */
static nrf_802154_timer_t m_timeout_timer; ///< Timer for delayed RX timeout handling.
static uint8_t            m_rx_channel;    ///< Channel number on which reception should be performed.

/**
 * @brief State of delayed operations.
 */
static delayed_trx_op_state_t m_dly_op_state[RSCH_DLY_TS_NUM];

/**
 * Set state of a delayed operation.
 *
 * @param[in]  dly_ts_id    Delayed timeslot ID.
 * @param[in]  dly_op_state Delayed operation state.
 */
static void dly_op_state_set(rsch_dly_ts_id_t dly_ts_id, delayed_trx_op_state_t dly_op_state)
{
    assert(dly_ts_id < RSCH_DLY_TS_NUM);
    assert(dly_op_state < DELAYED_TRX_OP_STATE_NB);

    m_dly_op_state[dly_ts_id] = dly_op_state;
}

/**
 * Get state of a delayed operation.
 *
 * @param[in]  dly_ts_id  Delayed timeslot ID.
 *
 * @retval     State of delayed operation.
 */
static delayed_trx_op_state_t dly_op_state_get(rsch_dly_ts_id_t dly_ts_id)
{
    assert(dly_ts_id < RSCH_DLY_TS_NUM);

    return m_dly_op_state[dly_ts_id];
}

/**
 * Start delayed operation.
 *
 * @param[in]  t0      Base time of the timestamp of the timeslot start [us].
 * @param[in]  dt      Time delta between @p t0 and the timestamp of the timeslot start [us].
 * @param[in]  length  Requested radio timeslot length [us].
 * @param[in]  dly_ts  Delayed timeslot ID.
 */
static bool dly_op_request(uint32_t         t0,
                           uint32_t         dt,
                           uint32_t         length,
                           rsch_dly_ts_id_t dly_ts_id)
{
    bool result;

    assert(dly_op_state_get(dly_ts_id) == DELAYED_TRX_OP_STATE_STOPPED);

    // Set PENDING state before timeslot request, in case timeslot starts
    // immediatly and interrupts current function execution.
    dly_op_state_set(dly_ts_id, DELAYED_TRX_OP_STATE_PENDING);

    result = nrf_802154_rsch_delayed_timeslot_request(t0,
                                                      dt,
                                                      length,
                                                      RSCH_PRIO_MAX,
                                                      dly_ts_id);

    if (!result)
    {
        dly_op_state_set(dly_ts_id, DELAYED_TRX_OP_STATE_STOPPED);
    }

    return result;
}

/**
 * Transmit request result callback.
 *
 * @param[in]  dly_ts_id  Delayed timeslot ID.
 * @param[in]  result     Result of delayed operation start:
 *                        - true: start succeeded (PENDING -> ONGOING)
 *                        - false: start failed (PENDING -> STOPPED)
 */
static void dly_op_timeslot_started_callback(rsch_dly_ts_id_t dly_ts_id, bool result)
{
    if (result)
    {
        dly_op_state_set(dly_ts_id, DELAYED_TRX_OP_STATE_ONGOING);
    }
    else
    {
        dly_op_state_set(dly_ts_id, DELAYED_TRX_OP_STATE_STOPPED);
    }
}

/**
 * Notify MAC layer that requested timeslot is not granted if tx request failed.
 */
static void notify_tx_timeslot_denied(void)
{
    nrf_802154_notify_transmit_failed(mp_tx_psdu, NRF_802154_TX_ERROR_TIMESLOT_DENIED);
}

/**
 * Notify MAC layer that requested timeslot is not granted if rx request failed.
 */
static void notify_rx_timeslot_denied(void)
{
    nrf_802154_notify_receive_failed(NRF_802154_RX_ERROR_DELAYED_TIMESLOT_DENIED);
}

/**
 * Notify MAC layer that delayed operation was aborted.
 */
static void notify_rx_aborted(void)
{
    nrf_802154_notify_receive_failed(NRF_802154_RX_ERROR_DELAYED_ABORTED);
}

/**
 * Notify MAC layer that no frame was received before timeout.
 *
 * @param[in]  p_context  Not used.
 */
static void notify_rx_timeout(void * p_context)
{
    (void)p_context;

    dly_op_state_set(RSCH_DLY_RX, DELAYED_TRX_OP_STATE_STOPPED);
    nrf_802154_notify_receive_failed(NRF_802154_RX_ERROR_DELAYED_TIMEOUT);
}

/**
 * Transmit request result callback.
 *
 * @param[in]  result  Result of TX request.
 */
static void tx_timeslot_started_callback(bool result)
{
    (void)result;

    // To avoid attaching to every possible transmit hook, in order to be able
    // to switch from ONGOING to STOPPED state, ONGOING state is not used at all
    // and state is changed to STOPPED right after transmit request.
    dly_op_timeslot_started_callback(RSCH_DLY_TX, false);

    if (!result)
    {
        notify_tx_timeslot_denied();
    }
}

/**
 * Receive request result callback.
 *
 * @param[in]  result  Result of RX request.
 */
static void rx_timeslot_started_callback(bool result)
{
    dly_op_timeslot_started_callback(RSCH_DLY_RX, result);

    if (result)
    {
        m_timeout_timer.t0 = nrf_802154_timer_sched_time_get();

        nrf_802154_timer_sched_add(&m_timeout_timer, true);
    }
    else
    {
        notify_rx_timeslot_denied();
    }
}

/**
 * Handle TX timeslot start.
 */
static void tx_timeslot_started_callout(void)
{
    bool result;

    nrf_802154_pib_channel_set(m_tx_channel);
    result = nrf_802154_request_channel_update();

    if (result)
    {
        (void)nrf_802154_request_transmit(NRF_802154_TERM_802154,
                                          REQ_ORIG_DELAYED_TRX,
                                          mp_tx_psdu,
                                          m_tx_cca,
                                          true,
                                          tx_timeslot_started_callback);
    }
    else
    {
        tx_timeslot_started_callback(result);
    }
}

/**
 * Handle RX timeslot start.
 */
static void rx_timeslot_started_callout(void)
{
    bool result;

    nrf_802154_pib_channel_set(m_rx_channel);
    result = nrf_802154_request_channel_update();

    if (result)
    {
        (void)nrf_802154_request_receive(NRF_802154_TERM_802154,
                                         REQ_ORIG_DELAYED_TRX,
                                         rx_timeslot_started_callback,
                                         true);
    }
    else
    {
        rx_timeslot_started_callback(result);
    }
}

bool nrf_802154_delayed_trx_transmit(const uint8_t * p_data,
                                     bool            cca,
                                     uint32_t        t0,
                                     uint32_t        dt,
                                     uint8_t         channel)
{
    bool     result;
    uint16_t timeslot_length;
    bool     ack;

    result = dly_op_state_get(RSCH_DLY_TX) == DELAYED_TRX_OP_STATE_STOPPED;

    if (result)
    {
        dt -= TX_SETUP_TIME;
        dt -= TX_RAMP_UP_TIME;

        if (cca)
        {
            dt -= nrf_802154_cca_before_tx_duration_get();
        }

        ack             = p_data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;
        timeslot_length = nrf_802154_tx_duration_get(p_data[0], cca, ack);

        mp_tx_psdu   = p_data;
        m_tx_cca     = cca;
        m_tx_channel = channel;

        result = dly_op_request(t0, dt, timeslot_length, RSCH_DLY_TX);
    }

    return result;
}

bool nrf_802154_delayed_trx_receive(uint32_t t0,
                                    uint32_t dt,
                                    uint32_t timeout,
                                    uint8_t  channel)
{
    bool     result;
    uint16_t timeslot_length;

    result = dly_op_state_get(RSCH_DLY_RX) == DELAYED_TRX_OP_STATE_STOPPED;

    if (result)
    {

        dt -= RX_SETUP_TIME;
        dt -= RX_RAMP_UP_TIME;

        timeslot_length = timeout + nrf_802154_rx_duration_get(MAX_PACKET_SIZE, true);

        m_timeout_timer.dt        = timeout;
        m_timeout_timer.callback  = notify_rx_timeout;
        m_timeout_timer.p_context = NULL;

        m_rx_channel = channel;

        result = dly_op_request(t0, dt, timeslot_length, RSCH_DLY_RX);
    }

    return result;
}

void nrf_802154_rsch_delayed_timeslot_started(rsch_dly_ts_id_t dly_ts_id)
{
    assert(dly_ts_id < RSCH_DLY_TS_NUM);
    assert(dly_op_state_get(dly_ts_id) == DELAYED_TRX_OP_STATE_PENDING);

    switch (dly_ts_id)
    {
        case RSCH_DLY_TX:
            tx_timeslot_started_callout();
            break;

        case RSCH_DLY_RX:
            rx_timeslot_started_callout();
            break;

        default:
            break;
    }
}

bool nrf_802154_delayed_trx_abort(nrf_802154_term_t term_lvl, req_originator_t req_orig)
{
    bool result = true;

    if (dly_op_state_get(RSCH_DLY_RX) == DELAYED_TRX_OP_STATE_ONGOING)
    {
        if (term_lvl >= NRF_802154_TERM_802154)
        {
            dly_op_state_set(RSCH_DLY_RX, DELAYED_TRX_OP_STATE_STOPPED);
            nrf_802154_timer_sched_remove(&m_timeout_timer);
            notify_rx_aborted();
        }
        else
        {
            result = false;
        }
    }

    return result;
}

void nrf_802154_delayed_trx_rx_started_hook(void)
{
    if (dly_op_state_get(RSCH_DLY_RX) == DELAYED_TRX_OP_STATE_ONGOING)
    {
        // @TODO protect against infinite extensions - allow only one timer extension
        if (nrf_802154_timer_sched_remaining_time_get(&m_timeout_timer)
            < nrf_802154_rx_duration_get(MAX_PACKET_SIZE, true))
        {
            m_timeout_timer.t0 = nrf_802154_timer_sched_time_get();
            m_timeout_timer.dt = nrf_802154_rx_duration_get(MAX_PACKET_SIZE, true);

            nrf_802154_timer_sched_add(&m_timeout_timer, true);
        }
    }
}
