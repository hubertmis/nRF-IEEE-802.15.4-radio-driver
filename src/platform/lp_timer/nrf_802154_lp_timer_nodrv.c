/* Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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
 *   This file contains standalone implementation of the nRF 802.15.4 timer abstraction.
 *
 * This implementation is built on top of the RTC peripheral.
 *
 */

#include "nrf_802154_lp_timer.h"

#include <assert.h>

#include <hal/nrf_rtc.h>
#include <nrf.h>

#include "platform/clock/nrf_802154_clock.h"
#include "nrf_802154_config.h"

#define RTC_LP_TIMER_COMPARE_CHANNEL    0
#define RTC_LP_TIMER_COMPARE_INT_MASK   NRF_RTC_INT_COMPARE0_MASK
#define RTC_LP_TIMER_COMPARE_EVENT      NRF_RTC_EVENT_COMPARE_0
#define RTC_LP_TIMER_COMPARE_EVENT_MASK RTC_EVTEN_COMPARE0_Msk

#define RTC_SYNC_COMPARE_CHANNEL        1
#define RTC_SYNC_COMPARE_INT_MASK       NRF_RTC_INT_COMPARE1_MASK
#define RTC_SYNC_COMPARE_EVENT          NRF_RTC_EVENT_COMPARE_1
#define RTC_SYNC_COMPARE_EVENT_MASK     RTC_EVTEN_COMPARE1_Msk

#define RTC_FREQUENCY                   32768ULL

#define US_PER_S                        1000000ULL
#define US_PER_TICK                     CEIL_DIV(US_PER_S, RTC_FREQUENCY)
#define US_PER_OVERFLOW                 (512UL * US_PER_S)                  ///< Time that has passed between overflow events. On full RTC speed, it occurs every 512 s.
#define MIN_RTC_COMPARE_EVENT_DT        (2 * US_PER_TICK)                     ///< Minimum time delta from now before RTC compare event is guaranteed to fire.


#define FREQUENCY_US_PER_S_GDD_BITS     6                                   ///< Number of bits to shift RTC_FREQUENCY and US_PER_S to achieve division by greatest common divisor.

#define CEIL_DIV(A, B)                  (((A) + (B) - 1) / (B))

// Struct holding information about compare channel.
typedef struct
{
    uint32_t        channel;    ///< Channel number
    uint32_t        int_mask;   ///< Interrupt mask
    nrf_rtc_event_t event;      ///< Event
    uint32_t        event_mask; ///< Event mask
} compare_channel_descriptor_t;

// Enum holding all used compare channels.
typedef enum {LP_TIMER_CHANNEL, SYNC_CHANNEL, CHANNEL_CNT} compare_channel_t;

// Descriptors of all used compare channels.
static const compare_channel_descriptor_t m_cmp_ch[CHANNEL_CNT] = {{RTC_LP_TIMER_COMPARE_CHANNEL,
                                                                    RTC_LP_TIMER_COMPARE_INT_MASK,
                                                                    RTC_LP_TIMER_COMPARE_EVENT,
                                                                    RTC_LP_TIMER_COMPARE_EVENT_MASK},
                                                                   {RTC_SYNC_COMPARE_CHANNEL,
                                                                    RTC_SYNC_COMPARE_INT_MASK,
                                                                    RTC_SYNC_COMPARE_EVENT,
                                                                    RTC_SYNC_COMPARE_EVENT_MASK}};

static uint64_t m_target_times[CHANNEL_CNT]; ///< Target time of given channel [us].


static volatile uint32_t m_offset_counter; ///< Counter of RTC overflows, incremented by 2 on each OVERFLOW event.
static volatile uint8_t  m_mutex;          ///< Mutex for write access to @ref m_offset_counter.
static volatile bool     m_clock_ready;    ///< Information that LFCLK is ready.

static uint32_t overflow_counter_get(void);

/** @brief Non-blocking mutex for mutual write access to @ref m_offset_counter variable.
 *
 *  @retval  true   Mutex was acquired.
 *  @retval  false  Mutex could not be acquired.
 */
static inline bool mutex_get(void)
{
    do
    {
        volatile uint8_t mutex_value = __LDREXB(&m_mutex);

        if (mutex_value)
        {
            __CLREX();
            return false;
        }
    }
    while (__STREXB(1, &m_mutex));

    // Disable OVERFLOW interrupt to prevent lock-up in interrupt context while mutex is locked from lower priority context
    // and OVERFLOW event flag is stil up.
    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, NRF_RTC_INT_OVERFLOW_MASK);

    __DMB();

    return true;
}

/** @brief Release mutex. */
static inline void mutex_release(void)
{
    // Re-enable OVERFLOW interrupt.
    nrf_rtc_int_enable(NRF_802154_RTC_INSTANCE, NRF_RTC_INT_OVERFLOW_MASK);

    __DMB();
    m_mutex = 0;
}

/** @brief Check if timer shall strike.
 *
 *  @param[in]  now  Current time.
 *
 *  @retval  true   Timer shall strike now.
 *  @retval  false  Timer shall not strike now.
 */
static inline bool shall_strike(uint64_t now)
{
    return now >= m_target_times[LP_TIMER_CHANNEL];
}

/** @brief Convert time in [us] to RTC ticks.
 *
 *  @param[in]  time  Time to convert.
 *
 *  @return  Time value in RTC ticks.
 */
static inline uint32_t time_to_ticks(uint64_t time)
{
    // Divide the divider and the divident by the greatest common divisor to increase capacity of the multiplication.
    return (uint32_t)CEIL_DIV((time * (RTC_FREQUENCY >> FREQUENCY_US_PER_S_GDD_BITS)),
                              (US_PER_S >> FREQUENCY_US_PER_S_GDD_BITS)) & RTC_CC_COMPARE_Msk;
}

/** @brief Convert RTC ticks to time in [us].
 *
 *  @param[in]  ticks  RTC ticks to convert.
 *
 *  @return  Time value in [us].
 */
static inline uint64_t ticks_to_time(uint32_t ticks)
{
    return CEIL_DIV((US_PER_S * (uint64_t)ticks), RTC_FREQUENCY);
}

/** @brief Get current time.
 *
 *  @return  Current time in [us].
 */
static uint64_t time_get(void)
{
    uint32_t offset_1 = overflow_counter_get();

    __DMB();

    uint32_t rtc_value_1 = nrf_rtc_counter_get(NRF_802154_RTC_INSTANCE);

    __DMB();

    uint32_t offset_2 = overflow_counter_get();

    __DMB();

    uint32_t rtc_value_2 = nrf_rtc_counter_get(NRF_802154_RTC_INSTANCE);

    if (offset_1 == offset_2)
    {
        return (uint64_t)offset_1 * US_PER_OVERFLOW + ticks_to_time(rtc_value_1);
    }
    else
    {
        return (uint64_t)offset_2 * US_PER_OVERFLOW + ticks_to_time(rtc_value_2);
    }
}

/** @brief Get current time plus 2 RTC ticks to prevent RTC compare event miss.
 *
 *  @return  Current time with RTC protection in [us].
 */
static inline uint64_t rtc_protected_time_get(void)
{
    return time_get() + MIN_RTC_COMPARE_EVENT_DT;
}

/** @brief Get current overflow counter and handle OVERFLOW event if present.
 *
 *  This function returns current value of m_overflow_counter variable. If OVERFLOW event is present
 *  while calling this function, it is handled within it.
 *
 *  @return  Current number of OVERFLOW events since platform start.
 */
static uint32_t overflow_counter_get(void)
{
    uint32_t offset;

    // Get mutual access for writing to m_offset_counter variable.
    if (mutex_get())
    {
        bool increasing = false;

        // Check if interrupt was handled already.
        if (nrf_rtc_event_pending(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW))
        {
            m_offset_counter++;
            increasing = true;

            __DMB();

            // Mark that interrupt was handled.
            nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW);

            // Result should be incremented. m_offset_counter will be incremented after mutex is released.
        }
        else
        {
            // Either overflow handling is not needed OR we acquired the mutex just after it was released.
            // Overflow is handled after mutex is released, but it cannot be assured that m_offset_counter
            // was incremented for the second time, so we increment the result here.
        }

        offset = (m_offset_counter + 1) / 2;

        mutex_release();

        if (increasing)
        {
            // It's virtually impossible that overflow event is pending again before next instruction is performed. It is an error condition.
            assert(m_offset_counter & 0x01);

            // Increment the counter for the second time, to alloww instructions from other context get correct value of the counter.
            m_offset_counter++;
        }
    }
    else
    {
        // Failed to acquire mutex.
        if (nrf_rtc_event_pending(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW) || (m_offset_counter & 0x01))
        {
            // Lower priority context is currently incrementing m_offset_counter variable.
            offset = (m_offset_counter + 2) / 2;
        }
        else
        {
            // Lower priority context has already incremented m_offset_counter variable or incrementing is not needed now.
            offset = m_offset_counter / 2;
        }
    }

    return offset;
}

/** @brief Handle COMPARE event. */
static void handle_compare_match(bool skip_check)
{
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event);

    // In case the target time was larger than single overflow,
    // we should only strike the timer on final compare event.
    if (skip_check || shall_strike(time_get()))
    {
        nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event_mask);
        nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);

        nrf_802154_lp_timer_fired();
    }
}

/**
 * @brief Convert t0 and dt to 64 bit time.
 *
 * @note This function takes into account possible overflow of first 32 bits in current time.
 *
 * @return  Converted time in [us].
 */
static uint64_t convert_to_64bit_time(uint32_t t0, uint32_t dt)
{
    uint64_t now;

    now = time_get();

    // Check if 32 LSB of `now` overflowed between getting t0 and loading `now` value.
    if ((uint32_t)now < t0)
    {
        now -= 0x0000000100000000;
    }

    return (now & 0xffffffff00000000) + t0 + dt;
}

/**
 * @brief Start one-shot timer that expires at specified time on desired channel.
 *
 * Start one-shot timer that will expire @p dt microseconds after @p t0 time on channel @p channel.
 *
 * @param[in]  channel  Comapre channel on which timer will be started.
 * @param[in]  t0       Number of microseconds representing timer start time.
 * @param[in]  dt       Time of timer expiration as time elapsed from @p t0 [us].
 */
static void timer_start_at(compare_channel_t channel, uint32_t t0, uint32_t dt)
{
    uint32_t target_counter;
    uint64_t target_time;

    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[channel].int_mask);
    nrf_rtc_event_enable(NRF_802154_RTC_INSTANCE, m_cmp_ch[channel].event_mask);

    target_time    = convert_to_64bit_time(t0, dt);
    target_counter = time_to_ticks(target_time);

    m_target_times[channel] = target_time;

    nrf_rtc_cc_set(NRF_802154_RTC_INSTANCE, m_cmp_ch[channel].channel, target_counter);
}

void nrf_802154_lp_timer_init(void)
{
    m_offset_counter                 = 0;
    m_target_times[LP_TIMER_CHANNEL] = 0;
    m_clock_ready                    = false;

    // Setup low frequency clock.
    nrf_802154_clock_lfclk_start();

    while (!m_clock_ready) { }

    // Setup RTC timer.
    NVIC_SetPriority(NRF_802154_RTC_IRQN, NRF_802154_RTC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(NRF_802154_RTC_IRQN);
    NVIC_EnableIRQ(NRF_802154_RTC_IRQN);

    nrf_rtc_prescaler_set(NRF_802154_RTC_INSTANCE, 0);

    // Setup RTC events.
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW);
    nrf_rtc_event_enable(NRF_802154_RTC_INSTANCE, RTC_EVTEN_OVRFLW_Msk);
    nrf_rtc_int_enable(NRF_802154_RTC_INSTANCE, NRF_RTC_INT_OVERFLOW_MASK);

    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);
    nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event_mask);
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event);

    // Start RTC timer.
    nrf_rtc_task_trigger(NRF_802154_RTC_INSTANCE, NRF_RTC_TASK_START);
}

void nrf_802154_lp_timer_deinit(void)
{
    nrf_rtc_task_trigger(NRF_802154_RTC_INSTANCE, NRF_RTC_TASK_STOP);

    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);
    nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event_mask);
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event);

    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, NRF_RTC_INT_OVERFLOW_MASK);
    nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, RTC_EVTEN_OVRFLW_Msk);
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW);

    nrf_802154_lp_timer_sync_stop();

    NVIC_DisableIRQ(NRF_802154_RTC_IRQN);
    NVIC_ClearPendingIRQ(NRF_802154_RTC_IRQN);
    NVIC_SetPriority(NRF_802154_RTC_IRQN, 0);

    nrf_802154_clock_lfclk_stop();
}

void nrf_802154_lp_timer_critical_section_enter(void)
{
    NVIC_DisableIRQ(NRF_802154_RTC_IRQN);
    __DSB();
    __ISB();
}

void nrf_802154_lp_timer_critical_section_exit(void)
{
    NVIC_EnableIRQ(NRF_802154_RTC_IRQN);
}

uint32_t nrf_802154_lp_timer_time_get(void)
{
    return (uint32_t)time_get();
}

uint32_t nrf_802154_lp_timer_granularity_get(void)
{
    return US_PER_TICK;
}

void nrf_802154_lp_timer_start(uint32_t t0, uint32_t dt)
{
    uint64_t now;

    timer_start_at(LP_TIMER_CHANNEL, t0, dt);

    now = rtc_protected_time_get();

    if (shall_strike(now))
    {
        handle_compare_match(true);
    }
    else
    {
        nrf_rtc_int_enable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);
    }
}

bool nrf_802154_lp_timer_is_running(void)
{
    return nrf_rtc_int_is_enabled(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);
}

void nrf_802154_lp_timer_stop(void)
{
    nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event_mask);
    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask);
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event);
}

void nrf_802154_lp_timer_sync_start_now(void)
{
    do
    {
        nrf_802154_lp_timer_sync_start_at(time_get(), MIN_RTC_COMPARE_EVENT_DT);
    } while (nrf_802154_lp_timer_sync_time_get() != (uint32_t)rtc_protected_time_get());
}

void nrf_802154_lp_timer_sync_start_at(uint32_t t0, uint32_t dt)
{
    timer_start_at(SYNC_CHANNEL, t0, dt);

    nrf_rtc_int_enable(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].int_mask);
}

void nrf_802154_lp_timer_sync_stop(void)
{
    nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event_mask);
    nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].int_mask);
    nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event);
}

uint32_t nrf_802154_lp_timer_sync_event_get(void)
{
    return (uint32_t)nrf_rtc_event_address_get(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event);
}

uint32_t nrf_802154_lp_timer_sync_time_get(void)
{
    return (uint32_t)m_target_times[SYNC_CHANNEL];
}

void nrf_802154_clock_lfclk_ready(void)
{
    m_clock_ready = true;
}

void NRF_802154_RTC_IRQ_HANDLER(void)
{
    // Handle overflow.
    if (nrf_rtc_event_pending(NRF_802154_RTC_INSTANCE, NRF_RTC_EVENT_OVERFLOW))
    {
        // Disable OVERFLOW interrupt to prevent lock-up in interrupt context while mutex is locked from lower priority context
        // and OVERFLOW event flag is stil up.
        // OVERFLOW interrupt will be re-enabled when mutex is released - either from this handler, or from lower priority context,
        // that locked the mutex.
        nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, NRF_RTC_INT_OVERFLOW_MASK);

        // Handle OVERFLOW event by reading current value of overflow counter.
        (void)overflow_counter_get();
    }

    // Handle compare match.
    if (nrf_rtc_int_is_enabled(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].int_mask) &&
        nrf_rtc_event_pending(NRF_802154_RTC_INSTANCE, m_cmp_ch[LP_TIMER_CHANNEL].event))
    {
        handle_compare_match(false);
    }

    if (nrf_rtc_int_is_enabled(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].int_mask) &&
        nrf_rtc_event_pending(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event))
    {
        nrf_rtc_event_clear(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event);
        nrf_rtc_event_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].event_mask);
        nrf_rtc_int_disable(NRF_802154_RTC_INSTANCE, m_cmp_ch[SYNC_CHANNEL].int_mask);
        nrf_802154_lp_timer_synchronized();
    }
}
