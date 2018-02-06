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
 *   This file implements common function for Front End Module control of the nRF 802.15.4 radio driver.
 *
 */

#include "nrf_fem_control_api.h"

#include "compiler_abstraction.h"
#include "nrf_fem_control_config.h"
#include "nrf_drv_radio802154_config.h"
#include "nrf.h"
#include "hal/nrf_gpio.h"
#include "hal/nrf_gpiote.h"
#include "hal/nrf_ppi.h"
#include "hal/nrf_radio.h"

#define NRF_FEM_TIMER_INSTANCE         NRF_DRV_RADIO802154_TIMER_INSTANCE
#define NRF_FEM_TIMER_LNA_CC_CHANNEL   0
#define NRF_FEM_TIMER_PA_CC_CHANNEL    2

static nrf_fem_control_cfg_t m_nrf_fem_control_cfg;     /**< FEM controller configuration. */

/**
 * @section GPIO control.
 */

/** Initialize GPIO according to configuration provided. */
static void gpio_init(void)
{
    if (m_nrf_fem_control_cfg.pa_cfg.enable)
    {
        nrf_gpio_cfg_output(m_nrf_fem_control_cfg.pa_cfg.gpio_pin);
        nrf_gpio_pin_write(m_nrf_fem_control_cfg.pa_cfg.gpio_pin,
                           !m_nrf_fem_control_cfg.pa_cfg.active_high);
    }

    if (m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        nrf_gpio_cfg_output(m_nrf_fem_control_cfg.lna_cfg.gpio_pin);
        nrf_gpio_pin_write(m_nrf_fem_control_cfg.lna_cfg.gpio_pin,
                           !m_nrf_fem_control_cfg.lna_cfg.active_high);
    }
}

static void gpiote_configure(void)
{
    nrf_gpiote_task_configure(m_nrf_fem_control_cfg.lna_gpiote_ch_id,
                              m_nrf_fem_control_cfg.lna_cfg.gpio_pin,
                              (nrf_gpiote_polarity_t)GPIOTE_CONFIG_POLARITY_None,
                              (nrf_gpiote_outinit_t)!m_nrf_fem_control_cfg.lna_cfg.active_high);

    nrf_gpiote_task_enable(m_nrf_fem_control_cfg.lna_gpiote_ch_id);

    nrf_gpiote_task_configure(m_nrf_fem_control_cfg.pa_gpiote_ch_id,
                              m_nrf_fem_control_cfg.pa_cfg.gpio_pin,
                              (nrf_gpiote_polarity_t)GPIOTE_CONFIG_POLARITY_None,
                              (nrf_gpiote_outinit_t)!m_nrf_fem_control_cfg.pa_cfg.active_high);

    nrf_gpiote_task_enable(m_nrf_fem_control_cfg.pa_gpiote_ch_id);
}

/**
 * @section PPI control.
 */

/** Initialize PPI according to configuration provided. */
static void ppi_init(void)
{
    /* RADIO DISABLED --> clr LNA & clr PA PPI */
    nrf_ppi_channel_and_fork_endpoint_setup(
        (nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_clr,
        (uint32_t)(&NRF_RADIO->EVENTS_DISABLED),
        (m_nrf_fem_control_cfg.lna_cfg.active_high ? 
            (uint32_t)(&NRF_GPIOTE->TASKS_CLR[m_nrf_fem_control_cfg.lna_gpiote_ch_id]) : 
            (uint32_t)(&NRF_GPIOTE->TASKS_SET[m_nrf_fem_control_cfg.lna_gpiote_ch_id])),
        (m_nrf_fem_control_cfg.pa_cfg.active_high ? 
            (uint32_t)(&NRF_GPIOTE->TASKS_CLR[m_nrf_fem_control_cfg.pa_gpiote_ch_id]) : 
            (uint32_t)(&NRF_GPIOTE->TASKS_SET[m_nrf_fem_control_cfg.pa_gpiote_ch_id])));
}

/** Setup PPI to set LNA pin on a timer event. */
static void ppi_lna_enable_setup(void)
{
    /* TIMER0->COMPARE --> set LNA PPI */
    nrf_ppi_channel_endpoint_setup(
        (nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set,
        (uint32_t)(&NRF_FEM_TIMER_INSTANCE->EVENTS_COMPARE[NRF_FEM_TIMER_LNA_CC_CHANNEL]),
        (m_nrf_fem_control_cfg.lna_cfg.active_high ?
            (uint32_t)(&NRF_GPIOTE->TASKS_SET[m_nrf_fem_control_cfg.lna_gpiote_ch_id]) :
            (uint32_t)(&NRF_GPIOTE->TASKS_CLR[m_nrf_fem_control_cfg.lna_gpiote_ch_id])));
}

/** Setup PPI to set PA pin on a timer event. */
static void ppi_pa_enable_setup(void)
{
    /* TIMER2->COMPARE --> set PA PPI */
    nrf_ppi_channel_endpoint_setup(
        (nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set,
        (uint32_t)(&NRF_FEM_TIMER_INSTANCE->EVENTS_COMPARE[NRF_FEM_TIMER_PA_CC_CHANNEL]),
        (m_nrf_fem_control_cfg.pa_cfg.active_high ?
            (uint32_t)(&NRF_GPIOTE->TASKS_SET[m_nrf_fem_control_cfg.pa_gpiote_ch_id]) :
            (uint32_t)(&NRF_GPIOTE->TASKS_CLR[m_nrf_fem_control_cfg.pa_gpiote_ch_id])));
}

/**
 * @section FEM API functions.
 */

void nrf_fem_control_cfg_set(const nrf_fem_control_cfg_t * p_cfg)
{
    m_nrf_fem_control_cfg = *p_cfg;

    if (m_nrf_fem_control_cfg.pa_cfg.enable || m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        gpio_init();
        gpiote_configure();
        ppi_init();

        nrf_ppi_channel_enable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_clr);
    }
}

void nrf_fem_control_cfg_get(nrf_fem_control_cfg_t * p_cfg)
{
    *p_cfg = m_nrf_fem_control_cfg;
}

void nrf_fem_control_activate(void)
{
    if (m_nrf_fem_control_cfg.pa_cfg.enable || m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        nrf_ppi_channel_enable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_clr);
    }
}

void nrf_fem_control_deactivate(void)
{
    if (m_nrf_fem_control_cfg.pa_cfg.enable || m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        nrf_ppi_channel_disable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_clr);
        nrf_ppi_channel_disable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set);
    }
}

void nrf_fem_control_pa_ppi_enable(void)
{
    if (m_nrf_fem_control_cfg.pa_cfg.enable)
    {
        /* PPI configuration. */
        ppi_pa_enable_setup();

        /* Enable the PPI. */
        nrf_ppi_channel_enable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set);
    }
}

void nrf_fem_control_lna_ppi_enable(void)
{
    if (m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        /* PPI configuration. */
        ppi_lna_enable_setup();

        /* Enable the PPI. */
        nrf_ppi_channel_enable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set);
    }
}

void nrf_fem_control_pa_ppi_disable(void)
{
    nrf_ppi_channel_disable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set);
}

void nrf_fem_control_lna_ppi_disable(void)
{
    nrf_ppi_channel_disable((nrf_ppi_channel_t)m_nrf_fem_control_cfg.ppi_ch_id_set);
}

uint32_t nrf_fem_control_pa_delay_get(void)
{
    uint32_t target_time = 0;

    if (m_nrf_fem_control_cfg.pa_cfg.enable)
    {
        /* Calculate timer target time. */        
        target_time = NRF_FEM_RADIO_TX_STARTUP_LATENCY_US - NRF_FEM_PA_TIME_IN_ADVANCE;
    }

    return target_time;
}

uint32_t nrf_fem_control_lna_delay_get(void)
{
    uint32_t target_time = 1;

    if (m_nrf_fem_control_cfg.lna_cfg.enable)
    {
        /* Calculate timer target time. */
        target_time = NRF_FEM_RADIO_RX_STARTUP_LATENCY_US - NRF_FEM_LNA_TIME_IN_ADVANCE;
    }

    return target_time;
}

void nrf_fem_control_pa_lna_clear(void)
{
    nrf_gpio_pin_write(m_nrf_fem_control_cfg.pa_cfg.gpio_pin,
                       !m_nrf_fem_control_cfg.pa_cfg.active_high);

    nrf_gpio_pin_write(m_nrf_fem_control_cfg.lna_cfg.gpio_pin,
                       !m_nrf_fem_control_cfg.lna_cfg.active_high);
}
