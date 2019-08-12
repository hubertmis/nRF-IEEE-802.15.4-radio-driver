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

#ifndef NRF_FEM_CONTROL_CONFIG_H_
#define NRF_FEM_CONTROL_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration parameters for pins that enable or disable (or both) either Power Amplifier (PA) or Low Noise Amplifier (LNA).
 */
typedef struct
{
    bool    enable;       /* Enable toggling for this pin. */
    bool    active_high;  /* If true, the pin will be active high. Otherwise, the pin will be active low. */
    uint8_t gpio_pin;     /* GPIO pin number for the pin. */
    uint8_t gpiote_ch_id; /* GPIOTE channel to be used for toggling pins. */
} nrf_fem_gpiote_pin_config_t;

/**
 * @brief Configuration parameters for the PA/LNA interface.
 */
typedef struct
{
    struct
    {
        uint32_t pa_time_gap_us;                /* Time between the activation of the PA pin and the start of the radio transmission. */
        uint32_t lna_time_gap_us;               /* Time between the activation of the LNA pin and the start of the radio reception. */
        int8_t   pa_gain_db;                    /* Configurable PA gain. Ignored if the amplifier is not supporting this feature. */
        int8_t   lna_gain_db;                   /* Configurable LNA gain. Ignored if the amplifier is not supporting this feature. */
    }                           fem_config;

    nrf_fem_gpiote_pin_config_t pa_pin_config;  /* Power Amplifier pin configuration. */
    nrf_fem_gpiote_pin_config_t lna_pin_config; /* Low Noise Amplifier pin configuration. */

    int                         ppi_ch_id_set;  /* PPI channel to be used for setting pins. */
    int                         ppi_ch_id_clr;  /* PPI channel to be used for clearing pins. */
} nrf_fem_interface_config_t;

/**
 * @section Timings.
 */

/** Time in microseconds when PA GPIO is activated before the radio is ready for transmission. */
#define NRF_FEM_PA_TIME_IN_ADVANCE          23

/** Time in microseconds when LNA GPIO is activated before the radio is ready for reception. */
#define NRF_FEM_LNA_TIME_IN_ADVANCE         5

/** Radio ramp-up time in TX mode, in microseconds. */
#define NRF_FEM_RADIO_TX_STARTUP_LATENCY_US 40

/** Radio ramp-up time in RX mode, in microseconds. */
#define NRF_FEM_RADIO_RX_STARTUP_LATENCY_US 40

#ifdef NRF52811_XXAA
/** Default Power Amplifier pin. */
#define NRF_FEM_CONTROL_DEFAULT_PA_PIN  19

/** Default Low Noise Amplifier pin. */
#define NRF_FEM_CONTROL_DEFAULT_LNA_PIN 20

#else

/** Default Power Amplifier pin. */
#define NRF_FEM_CONTROL_DEFAULT_PA_PIN  15

/** Default Low Noise Amplifier pin. */
#define NRF_FEM_CONTROL_DEFAULT_LNA_PIN 16
#endif

/** Default PPI channel for pin setting. */
#define NRF_FEM_CONTROL_DEFAULT_SET_PPI_CHANNEL    15

/** Default PPI channel for pin clearing. */
#define NRF_FEM_CONTROL_DEFAULT_CLR_PPI_CHANNEL    16

/** Default GPIOTE channel for FEM control. */
#define NRF_FEM_CONTROL_DEFAULT_LNA_GPIOTE_CHANNEL 6

/** Default GPIOTE channel for FEM control. */
#define NRF_FEM_CONTROL_DEFAULT_PA_GPIOTE_CHANNEL  7

/**
 * @section Configuration
 */

#if ENABLE_FEM

/**
 * @brief Configures the PA and LNA device interface.
 *
 * This function sets device interface parameters for the PA/LNA module.
 * The module can then be used to control a power amplifier or a low noise amplifier (or both) through the given interface and resources.
 *
 * The function also sets the PPI and GPIOTE channels to be configured for the PA/LNA interface.
 *
 * @param[in] p_config Pointer to the interface parameters for the PA/LNA device.
 *
 * @retval   ::NRF_SUCCESS                 PA/LNA control successfully configured.
 * @retval   ::NRF_ERROR_NOT_SUPPORTED     PA/LNA is not available.
 *
 */
int32_t nrf_fem_interface_configuration_set(nrf_fem_interface_config_t const * const p_config);

/**
 * @brief Retrieves the configuration of PA and LNA device interface.
 *
 * This function gets device interface parameters for the PA/LNA module.
 * The module can then be used to control a power amplifier or a low noise amplifier (or both) through the given interface and resources.
 *
 *
 * @param[in] p_config Pointer to the interface parameters for the PA/LNA device to be populated.
 *
 * @retval   ::NRF_SUCCESS                 PA/LNA control successfully configured.
 * @retval   ::NRF_ERROR_NOT_SUPPORTED     PA/LNA is not available.
 *
 */
int32_t nrf_fem_interface_configuration_get(nrf_fem_interface_config_t * p_config);

#else // ENABLE_FEM

static inline int32_t nrf_fem_interface_configuration_set(
    nrf_fem_interface_config_t const * const p_config)
{
    (void)p_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

static inline int32_t nrf_fem_interface_configuration_get(nrf_fem_interface_config_t * p_config)
{
    (void)p_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

#endif // ENABLE_FEM

#ifdef __cplusplus
}
#endif

#endif /* NRF_FEM_CONTROL_CONFIG_H_ */
