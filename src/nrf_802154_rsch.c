#include "nrf_802154_rsch.h"

#include <assert.h>
#include <nrf.h>

#include "nrf_802154_debug.h"
#include "platform/clock/nrf_802154_clock.h"
#include "raal/nrf_raal_api.h"

typedef enum
{
    RSCH_PREC_STATE_IDLE,
    RSCH_PREC_STATE_REQUESTED,
    RSCH_PREC_STATE_APPROVED,
} rsch_prec_state_t;

static volatile uint8_t           m_mutex;                      ///< Mutex for notyfying core.
static volatile uint8_t           m_mutex_monitor;              ///< Mutex monitor, incremented every failed mutex lock.
static volatile bool              m_last_notified_approved;     ///< Last reported state was approved.
static volatile rsch_prec_state_t m_prec_states[RSCH_PREC_CNT]; ///< State of all preconditions.

/** @brief Non-blocking mutex for notifying core.
 *
 *  @retval  true   Mutex was acquired.
 *  @retval  false  Mutex could not be acquired.
 */
static inline bool mutex_trylock(void)
{
    do
    {
        uint8_t mutex_value = __LDREXB(&m_mutex);

        if (mutex_value)
        {
            __CLREX();

            m_mutex_monitor++;
            return false;
        }
    } while (__STREXB(1, &m_mutex));

    __DMB();

    return true;
}

/** @brief Release mutex. */
static inline void mutex_unlock(void)
{
    __DMB();
    m_mutex = 0;
}

/** @brief Set RSCH_PREC_STATE_APPROVED state on given precondition @p prec only if
 *         its current state is other than RSCH_PREC_STATE_IDLE.
 * 
 * @param[in]  prec    Precondition which state will be changed.
 */
static inline void prec_approved_set(rsch_prec_t prec)
{
    do
    {
        rsch_prec_state_t old_state = (rsch_prec_state_t) __LDREXB((uint8_t*)&m_prec_states[prec]);

        assert(old_state != RSCH_PREC_STATE_APPROVED);

        if (old_state == RSCH_PREC_STATE_IDLE)
        {
            __CLREX();
            return;
        }
    } while (__STREXB((uint8_t)RSCH_PREC_STATE_APPROVED, (uint8_t*)&m_prec_states[prec]));
}

/** @brief Set RSCH_PREC_STATE_REQUESTED state on given precondition @p prec only if
 *         its current state is other than RSCH_PREC_STATE_IDLE or @p force is true.
 * 
 * @param[in]  prec    Precondition which state will be changed.
 * @param[in]  force   When true state will be forced to RSCH_PREC_STATE_REQUESTED.
 *                     When false state will be changed to RSCH_PREC_STATE_REQUESTED
 *                     only when current state is other than RSCH_PREC_STATE_IDLE.
 */
static inline void prec_requested_set(rsch_prec_t prec, bool force)
{
    if (force)
    {
        assert(m_prec_states[prec] != RSCH_PREC_STATE_REQUESTED);
        m_prec_states[prec] = RSCH_PREC_STATE_REQUESTED;
        __CLREX();
    }
    else
    {
        do
        {
            rsch_prec_state_t old_state = (rsch_prec_state_t) __LDREXB((uint8_t*)&m_prec_states[prec]);

            assert(old_state != RSCH_PREC_STATE_REQUESTED);

            if (old_state == RSCH_PREC_STATE_IDLE)
            {
                __CLREX();
                return;
            }
        } while (__STREXB((uint8_t)RSCH_PREC_STATE_REQUESTED, (uint8_t*)&m_prec_states[prec]));
    }
}

/** @brief Set RSCH_PREC_STATE_IDLE state on given precondition @p precE.
 * 
 * @param[in]  prec    Precondition which state will be changed.
 */
static inline void prec_idle_set(rsch_prec_t prec)
{
    assert(m_prec_states[prec] != RSCH_PREC_STATE_IDLE);
    m_prec_states[prec] = RSCH_PREC_STATE_IDLE;
    __CLREX();
}

static inline bool all_prec_are_approved(void)
{
    for (uint32_t i = 0; i < RSCH_PREC_CNT; i++)
    {
        if (m_prec_states[i] != RSCH_PREC_STATE_APPROVED)
        {
            return false;
        }
    }

    return true;
}

/** @brief Notify core if preconditions are approved or denied if current state differs from last reported.
 */
static inline void notify_core(void)
{
    bool   notify_approved;
    int8_t temp_mon;

    do
    {
        if (!mutex_trylock())
        {
            return;
        }

        /* It is possible that preemption is not detected (m_mutex_monitor is read after acquiring mutex).
         * It is not a problem because we will call proper handler function requested by preempting context.
         * Avoiding this race would generate one additional iteration without any effect.
         */
        temp_mon        = m_mutex_monitor;
        notify_approved = all_prec_are_approved();

        if (m_last_notified_approved != notify_approved)
        {
            m_last_notified_approved = notify_approved;

            if (notify_approved)
            {
                nrf_802154_rsch_prec_approved();
            }
            else
            {
                nrf_802154_rsch_prec_denied();
            }
        }

        mutex_unlock();
    } while(temp_mon != m_mutex_monitor);
}

void nrf_802154_rsch_init(void)
{
    nrf_raal_init();
}

void nrf_802154_rsch_uninit(void)
{
    nrf_raal_uninit();
}

void nrf_802154_rsch_continuous_mode_enter(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_CONTINUOUS_ENTER);

    prec_requested_set(RSCH_PREC_HFCLK, true);
    nrf_802154_clock_hfclk_start();

    prec_requested_set(RSCH_PREC_RAAL, true);
    nrf_raal_continuous_mode_enter();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_CONTINUOUS_ENTER);
}

void nrf_802154_rsch_continuous_mode_exit(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_CONTINUOUS_EXIT);

    prec_idle_set(RSCH_PREC_HFCLK);
    nrf_802154_clock_hfclk_stop();

    prec_idle_set(RSCH_PREC_RAAL);
    nrf_raal_continuous_mode_exit();

    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_CONTINUOUS_EXIT);
}

bool nrf_802154_rsch_prec_is_approved(rsch_prec_t prec)
{
    assert(prec < RSCH_PREC_CNT);
    return m_prec_states[prec] == RSCH_PREC_STATE_APPROVED;
}

bool nrf_802154_rsch_timeslot_request(uint32_t length_us)
{
    return nrf_raal_timeslot_request(length_us);
}

uint32_t nrf_802154_rsch_timeslot_us_left_get(void)
{
    return nrf_raal_timeslot_us_left_get();
}

// External handlers

void nrf_raal_timeslot_started(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMESLOT_STARTED);

    prec_approved_set(RSCH_PREC_RAAL);

    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMESLOT_STARTED);
}

void nrf_raal_timeslot_ended(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMESLOT_ENDED);

    prec_requested_set(RSCH_PREC_RAAL, false);

    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMESLOT_ENDED);
}

void nrf_802154_clock_hfclk_ready(void)
{
    prec_approved_set(RSCH_PREC_HFCLK);

    notify_core();
}
