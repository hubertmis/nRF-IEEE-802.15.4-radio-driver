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
 *   This file implements ACK timeout procedure for the 802.15.4 driver.
 *
 */

#include "nrf_802154_ack_timeout.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_802154_notification.h"
#include "nrf_802154_request.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

#define NUM_TIMERS      2       ///< Number of timers used to make sure that no timer is added from its own handler.
#define RETRY_DELAY     500     ///< Procedure is delayed by this time if cannot be performed at the moment.
#define MAX_RETRY_DELAY 100000  ///< Maximal allowed delay of procedure retry.

static void timeout_timer_retry(nrf_802154_timer_t * p_prev_timer);

static uint32_t           m_timeout = NRF_802154_ACK_TIMEOUT_DEFAULT_TIMEOUT;  ///< ACK timeout in us.
static nrf_802154_timer_t m_timers[NUM_TIMERS];                                ///< Timer used to notify when we are waiting too long for ACK.
static volatile uint8_t   m_timer_idx;
static volatile bool      m_procedure_is_active;
static const uint8_t    * mp_frame;

static void notify_tx_error(bool result)
{
    if (result)
    {
        nrf_802154_notify_transmit_failed(mp_frame, NRF_802154_TX_ERROR_NO_ACK);
    }
}

static void timeout_timer_fired(void * p_context)
{
    if (m_procedure_is_active)
    {
        if (nrf_802154_request_receive(NRF_802154_TERM_802154,
                                       REQ_ORIG_ACK_TIMEOUT,
                                       notify_tx_error,
                                       false))
        {
            m_procedure_is_active = false;
        }
        else
        {
            timeout_timer_retry(p_context);
        }
    }
}

static uint8_t next_timer_idx(void)
{
    uint8_t next_idx = m_timer_idx + 1;

    if (next_idx >= NUM_TIMERS)
    {
        next_idx = 0;
    }

    return next_idx;
}

static void next_timer_start(uint32_t t0, uint32_t dt)
{
    nrf_802154_timer_t * p_timer  = &m_timers[m_timer_idx];
    uint8_t              next_idx = next_timer_idx();

    if (next_idx >= NUM_TIMERS)
    {
        next_idx = 0;
    }

    p_timer->callback  = timeout_timer_fired;
    p_timer->p_context = p_timer;
    p_timer->t0        = t0;
    p_timer->dt        = dt;

    assert(!nrf_802154_timer_sched_is_running(p_timer));
    assert(!nrf_802154_timer_sched_is_running(p_timer->p_context));
    assert(next_idx == next_timer_idx());

    m_timer_idx = next_idx;

    nrf_802154_timer_sched_add(p_timer, true);
}

static void timeout_timer_retry(nrf_802154_timer_t * p_prev_timer)
{
    uint32_t dt = p_prev_timer->dt + RETRY_DELAY;

    assert(dt <= MAX_RETRY_DELAY);
    next_timer_start(p_prev_timer->t0, dt);
}

static void timeout_timer_start(void)
{
    m_procedure_is_active = true;
    next_timer_start(nrf_802154_timer_sched_time_get(), m_timeout);
}

static void timeout_timer_stop(void)
{
    m_procedure_is_active = false;

    for (uint8_t i = 0; i < NUM_TIMERS; i++)
    {
        nrf_802154_timer_sched_remove(&m_timers[i]);
    }
}

void nrf_802154_ack_timeout_time_set(uint32_t time)
{
    m_timeout = time;
}

bool nrf_802154_ack_timeout_tx_started_hook(const uint8_t * p_frame)
{
    mp_frame = p_frame;
    timeout_timer_start();

    return true;
}

bool nrf_802154_ack_timeout_abort(nrf_802154_term_t term_lvl, req_originator_t req_orig)
{
    bool result;

    if (!m_procedure_is_active || req_orig == REQ_ORIG_ACK_TIMEOUT)
    {
        // Ignore if procedure is not running or self-request.
        result = true;
    }
    else if (term_lvl >= NRF_802154_TERM_802154)
    {
        // Stop procedure only if termination level is high enough.
        timeout_timer_stop();

        result = true;
    }
    else
    {
        result = false;
    }

    return result;
}

void nrf_802154_ack_timeout_transmitted_hook(const uint8_t * p_frame)
{
    assert((p_frame == mp_frame) || (!m_procedure_is_active));

    timeout_timer_stop();
}

bool nrf_802154_ack_timeout_tx_failed_hook(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
    (void)error;
    assert((p_frame == mp_frame) || (!m_procedure_is_active));

    timeout_timer_stop();

    return true;
}
