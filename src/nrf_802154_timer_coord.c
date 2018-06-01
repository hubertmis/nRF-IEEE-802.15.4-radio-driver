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
 *   This file implements Timer Coordinator module.
 *
 */

#include "nrf_802154_timer_coord.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal/nrf_ppi.h"
#include "platform/hp_timer/nrf_802154_hp_timer.h"
#include "platform/lp_timer/nrf_802154_lp_timer.h"

#define RESYNC_TIME (5 * 60 * 1000000UL)

#define PPI_CH0    NRF_PPI_CHANNEL13
#define PPI_CH1    NRF_PPI_CHANNEL14
#define PPI_CHGRP0 NRF_PPI_CHANNEL_GROUP1

#define PPI_SYNC            PPI_CH0
#define PPI_TIMESTAMP       PPI_CH1
#define PPI_TIMESTAMP_GROUP PPI_CHGRP0

// Structure holding common timepoint from both timers.
typedef struct
{
    uint32_t lp_timer_time; ///< LP Timer time of common timepoint.
    uint32_t hp_timer_time; ///< HP Timer time of common timepoint.
} common_timepoint_t;

static common_timepoint_t m_last_sync;    ///< Common timepoint of last synchronization event.
static volatile bool      m_synchronized; ///< If timers were synchronized since last start.

void nrf_802154_timer_coord_init(void)
{
    uint32_t sync_event;
    uint32_t sync_task;

    nrf_802154_hp_timer_init();

    sync_event = nrf_802154_lp_timer_sync_event_get();
    sync_task  = nrf_802154_hp_timer_sync_task_get();

    nrf_ppi_channel_endpoint_setup(PPI_SYNC, sync_event, sync_task);
    nrf_ppi_channel_enable(PPI_SYNC);

    nrf_ppi_channel_include_in_group(PPI_TIMESTAMP, PPI_TIMESTAMP_GROUP);
}

void nrf_802154_timer_coord_uninit(void)
{
    nrf_802154_hp_timer_deinit();

    nrf_ppi_channel_disable(PPI_SYNC);
    nrf_ppi_channel_endpoint_setup(PPI_SYNC, 0, 0);

    nrf_ppi_group_disable(PPI_TIMESTAMP_GROUP);
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_TIMESTAMP, 0, 0, 0);
}

void nrf_802154_timer_coord_start(void)
{
    m_synchronized = false;
    nrf_802154_hp_timer_start();
    nrf_802154_hp_timer_sync_prepare();
    nrf_802154_lp_timer_sync_start_now();
}

void nrf_802154_timer_coord_stop(void)
{
    nrf_802154_hp_timer_stop();
    nrf_802154_lp_timer_sync_stop();
}

void nrf_802154_timer_coord_timestamp_prepare(uint32_t event_addr)
{
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_TIMESTAMP,
                                            event_addr,
                                            nrf_802154_hp_timer_timestamp_task_get(),
                                            (uint32_t)nrf_ppi_task_group_disable_address_get(PPI_TIMESTAMP_GROUP));

    nrf_ppi_group_enable(PPI_TIMESTAMP_GROUP);
}

bool nrf_802154_timer_coord_timestamp_get(uint32_t * p_timestamp)
{
    uint32_t hp_timestamp;
    uint32_t hp_delta;

    assert(p_timestamp != NULL);

    if (!m_synchronized)
    {
        return false;
    }

    hp_timestamp = nrf_802154_hp_timer_timestamp_get();
    hp_delta     = hp_timestamp - m_last_sync.hp_timer_time;
    *p_timestamp = hp_delta + m_last_sync.lp_timer_time;

    return true;
}

void nrf_802154_lp_timer_synchronized(void)
{
    common_timepoint_t sync_time;

    if (nrf_802154_hp_timer_sync_time_get(&sync_time.hp_timer_time))
    {
        sync_time.lp_timer_time = nrf_802154_lp_timer_sync_time_get();

        /* To avoid possible race when nrf_802154_timer_coord_timestamp_get
         * is called when m_last_sync is being assigned report that we are not synchronized
         * during assignment.
         * This is naive solution that can be improved if needed with double buffering.
         */
        m_synchronized = false;
        __DMB();
        m_last_sync = sync_time;
        __DMB();
        m_synchronized = true;

        nrf_802154_hp_timer_sync_prepare();
        nrf_802154_lp_timer_sync_start_at(m_last_sync.lp_timer_time, RESYNC_TIME);
    }
    else
    {
        nrf_802154_hp_timer_sync_prepare();
        nrf_802154_lp_timer_sync_start_now();
    }
}
