#include "nrf_802154_rsch.h"

#include <assert.h>
#include <stddef.h>
#include <nrf.h>

#include "nrf_802154_debug.h"
#include "nrf_802154_priority_drop.h"
#include "platform/clock/nrf_802154_clock.h"
#include "raal/nrf_raal_api.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

#define PREC_RAMP_UP_TIME 300  ///< Ramp-up time of preconditions [us]. 300 is worst case for HFclock

static volatile uint8_t     m_ntf_mutex;                      ///< Mutex for notyfying core.
static volatile uint8_t     m_ntf_mutex_monitor;              ///< Mutex monitor, incremented every failed ntf mutex lock.
static volatile uint8_t     m_req_mutex;                      ///< Mutex for requesting preconditions.
static volatile uint8_t     m_req_mutex_monitor;              ///< Mutex monitor, incremented every failed req mutex lock.
static volatile rsch_prio_t m_last_notified_prio;             ///< Last reported approved priority level.
static volatile rsch_prio_t m_approved_prios[RSCH_PREC_CNT];  ///< Priority levels approved by each precondition.
static rsch_prio_t          m_requested_prio;                 ///< Priority requested from all preconditions.
static bool                 m_in_cont_mode;                   ///< If RSCH operates in continuous mode.

static bool               m_delayed_timeslot_is_scheduled;  ///< If delayed timeslot is scheduled at the moment.
static uint32_t           m_delayed_timeslot_t0;            ///< Time base of the delayed timeslot trigger time.
static uint32_t           m_delayed_timeslot_dt;            ///< Time delta of the delayed timeslot trigger time.
static nrf_802154_timer_t m_timer;                          ///< Timer used to trigger delayed timeslot.

/** @brief Non-blocking mutex for notifying core.
 *
 *  @param[inout]  p_mutex          Pointer to the mutex data.
 *  @param[inout]  p_mutex_monitor  Pointer to the mutex monitor counter.
 *
 *  @retval  true   Mutex was acquired.
 *  @retval  false  Mutex could not be acquired.
 */
static inline bool mutex_trylock(volatile uint8_t * p_mutex, volatile uint8_t * p_mutex_monitor)
{
    do
    {
        uint8_t mutex_value = __LDREXB(p_mutex);

        if (mutex_value)
        {
            __CLREX();

            (*p_mutex_monitor)++;
            return false;
        }
    } while (__STREXB(1, p_mutex));

    __DMB();

    return true;
}

/** @brief Release mutex. */
static inline void mutex_unlock(volatile uint8_t * p_mutex)
{
    __DMB();
    *p_mutex = 0;
}

/** @brief Check if any precondition should be requested at the moment for delayed timeslot.
 *
 * To meet delayed timeslot timing requirements there is a time window in which radio
 * preconditions should be requested. This function is used to prevent releasing preconditions
 * in this time window.
 *
 * @retval true   A precondition should be requested at the moment for delayed timeslot feature.
 * @retval false  None of preconditions should be requested at the moment for delayed timeslot.
 */
static bool any_prec_should_be_requested_for_delayed_timeslot(void)
{
    uint32_t now = nrf_802154_timer_sched_time_get();
    uint32_t t0  = m_delayed_timeslot_t0;
    uint32_t dt  = m_delayed_timeslot_dt - PREC_RAMP_UP_TIME -
            nrf_802154_timer_sched_granularity_get();

    return (m_delayed_timeslot_is_scheduled &&
            !nrf_802154_timer_sched_time_is_in_future(now, t0, dt));
}

static rsch_prio_t required_prio_lvl_get(void)
{
    rsch_prio_t result;

    if (any_prec_should_be_requested_for_delayed_timeslot())
    {
        result = RSCH_PRIO_MAX;
    }
    else if (m_in_cont_mode)
    {
        result = RSCH_PRIO_MAX;
    }
    else
    {
        result = RSCH_PRIO_IDLE;
    }

    return result;
}

/** @brief Set approved priority level @p prio on given precondition @p prec.
 *
 * When requested priority level equals to the @ref RSCH_PRIO_IDLE this function will approve only
 * the @ref RSCH_PRIO_IDLE priority level and drop other approved levels silently.
 * 
 * @param[in]  prec    Precondition which state will be changed.
 * @param[in]  prio    Approved priority level for given precondition.
 */
static inline void prec_approved_prio_set(rsch_prec_t prec, rsch_prio_t prio)
{
    assert(prec <= RSCH_PREC_CNT);

    if ((m_requested_prio == RSCH_PRIO_IDLE) && (prio != RSCH_PRIO_IDLE))
    {
        // Ignore approved precondition - it was not requested.
        return;
    }

    assert((m_approved_prios[prec] != prio) || (prio == RSCH_PRIO_IDLE));

    m_approved_prios[prec] = prio;
}

/** @brief Request all preconditions.
 */
static inline void all_prec_request(void)
{
    rsch_prio_t prev_prio;
    rsch_prio_t new_prio;
    uint8_t     monitor;

    do
    {
        if (!mutex_trylock(&m_req_mutex, &m_req_mutex_monitor))
        {
            return;
        }

        monitor   = m_req_mutex_monitor;
        prev_prio = m_requested_prio;
        new_prio  = required_prio_lvl_get();

        if (prev_prio != new_prio)
        {
            m_requested_prio = new_prio;

            if (new_prio == RSCH_PRIO_IDLE)
            {
                nrf_802154_priority_drop_hfclk_stop();
                prec_approved_prio_set(RSCH_PREC_HFCLK, RSCH_PRIO_IDLE);

                nrf_raal_continuous_mode_exit();
                prec_approved_prio_set(RSCH_PREC_RAAL, RSCH_PRIO_IDLE);
            }
            else
            {
                nrf_802154_priority_drop_hfclk_stop_terminate();
                nrf_802154_clock_hfclk_start();
                nrf_raal_continuous_mode_enter();
            }
        }

        mutex_unlock(&m_req_mutex);
    } while (monitor != m_req_mutex_monitor);
}

/** @brief Release all preconditions if not needed.
 *
 * If RSCH is not in continuous mode and delayed timeslot is not expected all preconditions are
 * released.
 */
static inline void all_prec_release(void)
{
    all_prec_request();
}

/** @brief Get currently approved priority level.
 *
 * @return Maximal priority level approved by all radio preconditions.
 */
static inline rsch_prio_t approved_prio_lvl_get(void)
{
    rsch_prio_t result = RSCH_PRIO_MAX;

    for (uint32_t i = 0; i < RSCH_PREC_CNT; i++)
    {
        if (m_approved_prios[i] < result)
        {
            result = m_approved_prios[i];
        }
    }

    return result;
}

/** @brief Check if all preconditions are requested or met at given priority level or higher.
 *
 * @param[in]  prio  Minimal priority level requested from preconditions.
 *
 * @retval true   All preconditions are requested or met at given or higher level.
 * @retval false  At least one precondition is requested at lower level than required.
 */
static inline bool requested_prio_lvl_is_at_least(rsch_prio_t prio)
{
    return m_requested_prio >= prio;
}


/** @brief Notify core if preconditions are approved or denied if current state differs from last reported.
 */
static inline void notify_core(void)
{
    rsch_prio_t approved_prio_lvl;
    uint8_t     temp_mon;

    do
    {
        if (!mutex_trylock(&m_ntf_mutex, &m_ntf_mutex_monitor))
        {
            return;
        }

        /* It is possible that preemption is not detected (m_ntf_mutex_monitor is read after
         * acquiring mutex). It is not a problem because we will call proper handler function
         * requested by preempting context. Avoiding this race would generate one additional
         * iteration without any effect.
         */
        temp_mon          = m_ntf_mutex_monitor;
        approved_prio_lvl = approved_prio_lvl_get();

        if (m_in_cont_mode && (m_last_notified_prio != approved_prio_lvl))
        {
            m_last_notified_prio = approved_prio_lvl;

            if (approved_prio_lvl > RSCH_PRIO_IDLE)
            {
                nrf_802154_rsch_prec_approved();
            }
            else
            {
                nrf_802154_rsch_prec_denied();
            }
        }

        mutex_unlock(&m_ntf_mutex);
    } while(temp_mon != m_ntf_mutex_monitor);
}

/** Timer callback used to trigger delayed timeslot.
 *
 * @param[in]  p_context  Unused parameter.
 */
static void delayed_timeslot_start(void * p_context)
{
    (void)p_context;

    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMER_DELAYED_START);

    m_delayed_timeslot_is_scheduled = false;

    if (approved_prio_lvl_get() > RSCH_PRIO_IDLE)
    {
        nrf_802154_rsch_delayed_timeslot_started();
    }
    else
    {
        nrf_802154_rsch_delayed_timeslot_failed();
    }

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMER_DELAYED_START);
}

/** Timer callback used to request preconditions for delayed timeslot.
 *
 * @param[in]  p_context  Unused parameter.
 */
static void delayed_timeslot_prec_request(void * p_context)
{
    (void)p_context;

    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMER_DELAYED_PREC);

    all_prec_request();

    m_timer.t0        = m_delayed_timeslot_t0;
    m_timer.dt        = m_delayed_timeslot_dt;
    m_timer.callback  = delayed_timeslot_start;
    m_timer.p_context = NULL;

    nrf_802154_timer_sched_add(&m_timer, true);

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMER_DELAYED_PREC);
}

/***************************************************************************************************
 * Public API
 **************************************************************************************************/

void nrf_802154_rsch_init(void)
{
    nrf_raal_init();

    m_ntf_mutex                     = 0;
    m_req_mutex                     = 0;
    m_last_notified_prio            = RSCH_PRIO_IDLE;
    m_in_cont_mode                  = false;
    m_delayed_timeslot_is_scheduled = false;
    m_requested_prio                = RSCH_PRIO_IDLE;

    for (uint32_t i = 0; i < RSCH_PREC_CNT; i++)
    {
        m_approved_prios[i] = RSCH_PRIO_IDLE;
    }
}

void nrf_802154_rsch_uninit(void)
{
    nrf_802154_timer_sched_remove(&m_timer);
    nrf_raal_uninit();
}

void nrf_802154_rsch_continuous_mode_enter(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_CONTINUOUS_ENTER);

    m_in_cont_mode = true;
    __DMB();

    all_prec_request();
    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_CONTINUOUS_ENTER);
}

void nrf_802154_rsch_continuous_mode_exit(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_CONTINUOUS_EXIT);

    __DMB();
    m_in_cont_mode = false;

    all_prec_release();
    notify_core();
    m_last_notified_prio = RSCH_PRIO_IDLE;

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_CONTINUOUS_EXIT);
}

bool nrf_802154_rsch_prec_is_approved(rsch_prec_t prec)
{
    assert(prec < RSCH_PREC_CNT);
    return m_approved_prios[prec] > RSCH_PRIO_IDLE;
}

bool nrf_802154_rsch_timeslot_request(uint32_t length_us)
{
    return nrf_raal_timeslot_request(length_us);
}

bool nrf_802154_rsch_delayed_timeslot_request(uint32_t t0, uint32_t dt, uint32_t length)
{
    (void)length;

    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_DELAYED_TIMESLOT_REQ);

    uint32_t now    = nrf_802154_timer_sched_time_get();
    uint32_t req_dt = dt - PREC_RAMP_UP_TIME;
    bool     result;

    assert(!nrf_802154_timer_sched_is_running(&m_timer));
    assert(!m_delayed_timeslot_is_scheduled);

    if (nrf_802154_timer_sched_time_is_in_future(now, t0, req_dt))
    {
        m_delayed_timeslot_is_scheduled = true;
        m_delayed_timeslot_t0           = t0;
        m_delayed_timeslot_dt           = dt;

        m_timer.t0        = t0;
        m_timer.dt        = req_dt;
        m_timer.callback  = delayed_timeslot_prec_request;
        m_timer.p_context = NULL;

        nrf_802154_timer_sched_add(&m_timer, false);

        result = true;
    }
    else if (requested_prio_lvl_is_at_least(RSCH_PRIO_MAX) &&
             nrf_802154_timer_sched_time_is_in_future(now, t0, dt))
    {
        m_delayed_timeslot_is_scheduled = true;
        m_delayed_timeslot_t0           = t0;
        m_delayed_timeslot_dt           = dt;

        m_timer.t0        = t0;
        m_timer.dt        = dt;
        m_timer.callback  = delayed_timeslot_start;
        m_timer.p_context = NULL;

        nrf_802154_timer_sched_add(&m_timer, true);

        result = true;
    }
    else
    {
        result = false;
    }

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_DELAYED_TIMESLOT_REQ);

    return result;
}

uint32_t nrf_802154_rsch_timeslot_us_left_get(void)
{
    return nrf_raal_timeslot_us_left_get();
}

// External handlers

void nrf_raal_timeslot_started(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMESLOT_STARTED);

    prec_approved_prio_set(RSCH_PREC_RAAL, RSCH_PRIO_MAX);
    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMESLOT_STARTED);
}

void nrf_raal_timeslot_ended(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RSCH_TIMESLOT_ENDED);

    prec_approved_prio_set(RSCH_PREC_RAAL, RSCH_PRIO_IDLE);
    notify_core();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RSCH_TIMESLOT_ENDED);
}

void nrf_802154_clock_hfclk_ready(void)
{
    prec_approved_prio_set(RSCH_PREC_HFCLK, RSCH_PRIO_MAX);
    notify_core();
}
