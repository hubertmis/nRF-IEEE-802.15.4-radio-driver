#include "nrf_802154_rsch.h"

#include "raal/nrf_raal_api.h"

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
    nrf_raal_continuous_mode_enter();
}

void nrf_802154_rsch_continuous_mode_exit(void)
{
    nrf_raal_continuous_mode_exit();
}

bool nrf_802154_rsch_timeslot_request(uint32_t length_us)
{
    return nrf_raal_timeslot_request(length_us);
}

bool nrf_802154_rsch_timeslot_is_granted(void)
{
    return nrf_raal_timeslot_is_granted();
}

uint32_t nrf_802154_rsch_timeslot_us_left_get(void)
{
    return nrf_raal_timeslot_us_left_get();
}

void nrf_802154_rsch_critical_section_enter(void)
{
    nrf_raal_critical_section_enter();
}

void nrf_802154_rsch_critical_section_exit(void)
{
    nrf_raal_critical_section_exit();
}

void nrf_raal_timeslot_started(void)
{
    nrf_802154_rsch_timeslot_started();
}

void nrf_raal_timeslot_ended(void)
{
    nrf_802154_rsch_timeslot_ended();
}
