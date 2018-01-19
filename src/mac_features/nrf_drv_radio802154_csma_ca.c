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
 *   This file implements CSMA-CA procedure for the 802.15.4 driver.
 *
 */

#include "nrf_drv_radio802154_csma_ca.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "nrf_drv_radio802154_config.h"
#include "nrf_drv_radio802154_const.h"
#include "nrf_drv_radio802154_notification.h"
#include "nrf_drv_radio802154_request.h"
#include "timer_scheduler/nrf_drv_radio802154_timer_sched.h"

static uint8_t m_nb;  ///< The number of times the CSMA-CA algorithm was required to back off while attempting the current transmission.
static uint8_t m_be;  ///< Backoff exponent, which is related to how many backoff periods a device shall wait before attempting to assess a channel.

static const uint8_t             * mp_psdu;  ///< Pointer to PSDU of the frame being transmitted.
static nrf_drv_radio802154_timer_t m_timer;  ///< Timer used to back off during CSMA-CA procedure.

/**
 * @brief Perform appropriate actions for busy channel conditions.
 *
 * According to CSMA-CA description in 802.15.4 specification, when channel is busy NB and BE shall
 * be incremented and the device shall wait random delay before next CCA procedure. If NB reaches
 * macMaxCsmaBackoffs procedure fails.
 *
 * @retval true   Procedure failed and TX failure should be notified to the next higher layer.
 * @retval false  Procedure is still ongoing and TX failure should be handled internally.
 */
static bool channel_busy(void);

/**
 * @brief Perform CCA procedure followed by frame transmission.
 *
 * If transmission is requested, CSMA-CA module waits for notification from the FSM module.
 * If transmission request fails, CSMA-CA module performs procedure for busy channel condition
 * @sa channel_busy().
 *
 * @param[in] p_context  Unused variable passed from the Timer Scheduler module.
 */
static void frame_transmit(void * p_context)
{
    (void)p_context;

    bool error_shall_be_notified = false;

    if (!nrf_drv_radio802154_request_transmit(mp_psdu, true))
    {
        error_shall_be_notified = channel_busy();
    }

    if (error_shall_be_notified)
    {
        nrf_drv_radio802154_notify_transmit_failed(NRF_DRV_RADIO802154_TX_ERROR_BUSY_CHANNEL);
    }
}

/**
 * @brief Delay CCA procedure for random (2^BE - 1) unit backoff periods.
 */
static void random_backoff_start(void)
{
    uint8_t backoff_periods = rand() % (1 << m_be);

    m_timer.callback  = frame_transmit;
    m_timer.p_context = NULL;
    m_timer.t0        = nrf_drv_radio802154_timer_sched_time_get();
    m_timer.dt        = backoff_periods * UNIT_BACKOFF_PERIOD;

    nrf_drv_radio802154_timer_sched_add(&m_timer, false);
}

/**
 * @brief Check if CSMA-CA is ongoing.
 *
 * @retval true   CSMA-CA is running.
 * @retval false  CSMA-CA is not running currently.
 */
static bool procedure_is_running(void)
{
    return mp_psdu != NULL;
}

/**
 * @brief Stop CSMA-CA procedure.
 */
static void procedure_stop(void)
{
    mp_psdu = NULL;
}

static bool channel_busy(void)
{
    bool result = true;

    if (procedure_is_running())
    {
        m_nb++;

        if (m_be < NRF_DRV_RADIO802154_CSMA_CA_MAX_BE)
        {
            m_be++;
        }

        if (m_nb < NRF_DRV_RADIO802154_CSMA_CA_MAX_CSMA_BACKOFFS)
        {
            random_backoff_start();
            result = false;
        }
        else
        {
            procedure_stop();
        }
    }

    return result;
}

void nrf_drv_radio802154_csma_ca_start(const uint8_t * p_data)
{
    assert(!procedure_is_running());

    mp_psdu = p_data;
    m_nb    = 0;
    m_be    = NRF_DRV_RADIO802154_CSMA_CA_MIN_BE;

    random_backoff_start();
}

void nrf_drv_radio802154_csma_ca_abort(void)
{
    nrf_drv_radio802154_timer_sched_remove(&m_timer);
    procedure_stop();
}

bool nrf_drv_radio802154_csma_ca_tx_failed_hook(nrf_drv_radio802154_tx_error_t error)
{
    (void)error;

    return channel_busy();
}

bool nrf_drv_radio802154_csma_ca_tx_started_hook(void)
{
    assert(!nrf_drv_radio802154_timer_sched_is_running(&m_timer));

    procedure_stop();

    return true;
}

