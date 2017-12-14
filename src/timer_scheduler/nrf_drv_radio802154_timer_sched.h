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
 * @brief This module provides timer scheduling functionality for the 802.15.4 driver.
 *
 * Timer module should be used to implement strict timing features specified in IEEE 802.15.4 like:
 * * CSL,
 * * Timing out waiting for ACK frames,
 * * CSMA/CA,
 * * Inter-frame spacing: SIFS, LIFS (note that AIFS is implemented without using timer module).
 *
 * @note Current implementation supports only single one-shot timer. It may be extended to support
 *       timer scheduling and repetitive timers if needed.
 */

#ifndef NRF_DRV_RADIO802154_TIMER_SCHED_H_
#define NRF_DRV_RADIO802154_TIMER_SCHED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_drv_radio802154_timer_sched Timer Scheduler module for the 802.15.4 driver
 * @{
 * @ingroup nrf_drv_radio802154_timer_sched
 * @brief Timer Scheduler module for the 802.15.4 driver.
 *
 */

/**
 * @brief Type of function called when timer fires.
 *
 * @param[inout]  p_context  Pointer to user-defined memory location. May be NULL.
 */
typedef void (* nrf_drv_radio802154_timer_callback_t)(void * p_context);

/**
 * @brief Structure containing timer data used by timer module.
 */
typedef struct
{
    uint32_t                             t0;         ///< Base time of the timer [us]
    uint32_t                             dt;         ///< Timer expiration delta from @p t0 [us]
    nrf_drv_radio802154_timer_callback_t callback;   ///< Callback function called when timer expires
    void                               * p_context;  ///< User-defined context passed to callback function
} nrf_drv_radio802154_timer_t;


/**
 * @brief Get current time.
 *
 * This function is useful to set base time in @sa nrf_drv_radio802154_timer_t structure.
 *
 * @return Current time in microseconds [us].
 */
uint32_t nrf_drv_radio802154_timer_sched_time_get(void);

/**
 * @brief Check if given time is in future.
 *
 * @param[in]  now  Current time. @sa nrf_drv_radio802154_timer_sched_time_get()
 * @param[in]  t0   Base of time compared with @p now.
 * @param[in]  dt   Time delta from @p t0 compared with @p now.
 *
 * @retval true   Given time @p t0 @p dt is in future (compared to given @p now).
 * @retval false  Given time @p t0 @p dt is not in future (compared to given @p now).
 */
bool nrf_drv_radio802154_timer_sched_time_is_in_future(uint32_t now, uint32_t t0, uint32_t dt);

/**
 * @brief Start given timer and add it to the scheduler.
 *
 * @note Fields t0, dt, callback and p_context should be filled in @p p_timer prior to calling this
 *       function; callback field cannot be NULL.
 *
 * @note Due to timer granularity the callback function cannot be called exactly at specified time.
 *       Use @p round_up to specify if given timer should be expired before or after time given in
 *       the @p p_timer structure.
 *
 * @param[inout]  p_timer   Pointer to the timer to start and add to the scheduler.
 * @param[in]     round_up  True if timer should expire after specified time, false if it should
 *                          expire before.
 */
void nrf_drv_radio802154_timer_sched_add(nrf_drv_radio802154_timer_t * p_timer, bool round_up);

/**
 * @brief Stop given timer and remove it from the scheduler.
 *
 * @param[inout]  p_timer  Pointer to the timer to stop and remove from the scheduler.
 */
void nrf_drv_radio802154_timer_sched_remove(nrf_drv_radio802154_timer_t * p_timer);

/**
 *@}
 **/

#ifdef __cplusplus
}
#endif

#endif /* NRF_DRV_RADIO802154_TIMER_SCHED_H_ */
