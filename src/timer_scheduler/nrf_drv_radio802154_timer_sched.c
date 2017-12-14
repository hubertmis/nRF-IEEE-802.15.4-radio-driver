/* Copyright (c) 2017, Nordic Semiconductor ASA
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
 *   This file implements timer scheduler for the nrf 802.15.4 driver.
 *
 * Note that current implementation supports just single one-shot timer. Scheduling is not
 * implemented yet.
 *
 */

#include "nrf_drv_radio802154_timer_sched.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform/timer/nrf_drv_radio802154_timer.h"

static nrf_drv_radio802154_timer_t * mp_timer;  ///< Pointer to currently running timer.

uint32_t nrf_drv_radio802154_timer_sched_time_get(void)
{
    return nrf_drv_radio802154_timer_time_get();
}

bool nrf_drv_radio802154_timer_sched_time_is_in_future(uint32_t now, uint32_t t0, uint32_t dt)
{
    uint32_t target_time = t0 + dt;
    int32_t  difference  = target_time - now;

    return difference > 0;
}

void nrf_drv_radio802154_timer_sched_add(nrf_drv_radio802154_timer_t * p_timer, bool round_up)
{
    assert(!nrf_drv_radio802154_timer_is_running()); // Currently only one timer can be running at a time
    assert(p_timer != NULL);
    assert(p_timer->callback != NULL);

    uint32_t dt = p_timer->dt;

    if (round_up)
    {
        dt += nrf_drv_radio802154_timer_granularity_get() - 1;
    }

    mp_timer = p_timer;

    nrf_drv_radio802154_timer_start(p_timer->t0, dt);
}

void nrf_drv_radio802154_timer_sched_remove(nrf_drv_radio802154_timer_t * p_timer)
{
    assert(p_timer != NULL);

    nrf_drv_radio802154_timer_stop();

    mp_timer = NULL;
}

void nrf_drv_radio802154_timer_fired(void)
{
    assert(mp_timer != NULL);
    assert(mp_timer->callback != NULL);

    mp_timer->callback(mp_timer->p_context);
}

