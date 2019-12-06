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

#include <stdlib.h>

#include "unity.h"

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "mock_nrf_802154.h"
#include "mock_nrf_802154_ack_data.h"
#include "mock_nrf_802154_ack_generator.h"
#include "mock_nrf_802154_core_hooks.h"
#include "mock_nrf_802154_critical_section.h"
#include "mock_nrf_802154_debug.h"
#include "mock_nrf_802154_filter.h"
#include "mock_nrf_802154_frame_parser.h"
#include "mock_nrf_802154_notification.h"
#include "mock_nrf_802154_pib.h"
#include "mock_nrf_802154_priority_drop.h"
#include "mock_nrf_802154_procedures_duration.h"
#include "mock_nrf_802154_rsch.h"
#include "mock_nrf_802154_rssi.h"
#include "mock_nrf_802154_rx_buffer.h"
#include "mock_nrf_802154_timer_coord.h"
#include "mock_nrf_egu.h"
#include "mock_nrf_fem_protocol_api.h"
#include "mock_nrf_ppi.h"
#include "mock_nrf_radio.h"
#include "mock_nrf_timer.h"

#define __ISB()
#define __LDREXB(ptr)           (*ptr)
#define __STREXB(value, ptr)    (((*ptr) = value) != value)

#include "nrf_802154_core.c"

/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

#define TEST_FRAME_SIZE     10
#define TEST_FRAME_DURATION 2000

void nrf_802154_rx_started(void)
{
    // Intentionally empty.
}

void nrf_802154_tx_started(const uint8_t * p_frame)
{
    (void)p_frame;
    // Intentionally empty.
}

void nrf_802154_rx_ack_started(void)
{
    // Intentionally empty.
}

void nrf_802154_tx_ack_started(const uint8_t * p_data)
{
    // Intentionally empty.
}

static rx_buffer_t m_test_rx_buffer;
static uint8_t m_test_tx_buffer[128];

void setUp(void)
{
    memset(&m_test_rx_buffer, 0, sizeof(m_test_rx_buffer));
    m_test_rx_buffer.data[0] = TEST_FRAME_SIZE;
    m_test_rx_buffer.free    = true;

    mp_current_rx_buffer = &m_test_rx_buffer;
    mp_tx_data           = m_test_tx_buffer;

    m_flags.frame_filtered        = false;
    m_flags.rx_timeslot_requested = false;
}

void tearDown(void)
{
    // Intentionally empty.
}

static void ack_requested_set_rx(void)
{
    m_test_rx_buffer.data[ACK_REQUEST_OFFSET] |= ACK_REQUEST_BIT;
}

static void ack_not_requested_set_rx(void)
{
    m_test_rx_buffer.data[ACK_REQUEST_OFFSET] &= ~ACK_REQUEST_BIT;
}

static void ack_requested_set_tx(void)
{
    m_test_tx_buffer[ACK_REQUEST_OFFSET] |= ACK_REQUEST_BIT;
}

static void ack_not_requested_set_tx(void)
{
    m_test_tx_buffer[ACK_REQUEST_OFFSET] &= ~ACK_REQUEST_BIT;
}

static void frame_type_ack_set(void)
{
    m_test_rx_buffer.data[FRAME_TYPE_OFFSET] |= FRAME_TYPE_ACK;
}

static void mock_received_frame_notify(void)
{
    int8_t rssi = rand();
    int8_t corrected_rssi = rand();
    uint8_t corrected_lqi = rand();
    uint32_t expected_lqi = corrected_lqi * 4;

    if (expected_lqi > UINT8_MAX)
    {
        expected_lqi = UINT8_MAX;
    }

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_radio_rssi_sample_get_ExpectAndReturn(rssi);
    nrf_802154_rssi_sample_corrected_get_ExpectAndReturn(rssi, corrected_rssi);
    nrf_802154_rssi_lqi_corrected_get_IgnoreAndReturn(corrected_lqi);
    nrf_802154_notify_received_Expect(m_test_rx_buffer.data, -corrected_rssi, expected_lqi);
    nrf_802154_critical_section_nesting_deny_Expect();
}

static void mock_receive_failed_notify(nrf_802154_rx_error_t error)
{
    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_802154_notify_receive_failed_Expect(error);
    nrf_802154_critical_section_nesting_deny_Expect();
}

static void mock_transmitted_frame_notify(uint8_t * p_expected_ack, int8_t expected_power, int8_t expected_lqi)
{
    m_test_rx_buffer.data[TEST_FRAME_SIZE - 1] = expected_lqi;

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_802154_core_hooks_transmitted_Expect(m_test_tx_buffer);
    nrf_802154_notify_transmitted_Expect(m_test_tx_buffer, p_expected_ack, expected_power, expected_lqi);
    nrf_802154_critical_section_nesting_deny_Expect();
}

static void mock_transmit_failed_notify(nrf_802154_tx_error_t expected_error)
{
    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_802154_core_hooks_tx_failed_ExpectAndReturn(m_test_tx_buffer, expected_error, true);
    nrf_802154_notify_transmit_failed_Expect(m_test_tx_buffer, expected_error);
    nrf_802154_critical_section_nesting_deny_Expect();
}

static void mock_ppi_egu_worked(bool result)
{
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, result);
}

static void mock_rx_terminate(void)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                                    NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    m_rsch_timeslot_is_granted = true;

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_shorts_set_Expect(0);
    nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
}

static void mock_rx_ack_terminate(void)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    m_rsch_timeslot_is_granted = true;

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_END_MASK |
                                 NRF_RADIO_INT_ADDRESS_MASK);
    nrf_radio_shorts_set_Expect(SHORTS_IDLE);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    nrf_radio_mhmu_search_pattern_set_Expect(0);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_MHRMATCH);
}

static void mock_tx_terminate(void)
{
    m_state = RADIO_STATE_TX;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                                    NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                                    NRF_TIMER_SHORT_COMPARE1_STOP_MASK);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);    

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    m_rsch_timeslot_is_granted = true;

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_PHYEND_MASK  |
                                 NRF_RADIO_INT_CCABUSY_MASK |
                                 NRF_RADIO_INT_ADDRESS_MASK);
    nrf_radio_shorts_set_Expect(SHORTS_IDLE);
    nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_CCASTOP);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
}

static void mock_receive_begin(bool free_buffer, uint32_t shorts)
{
    uint32_t event_addr;
    uint32_t task_addr;
    uint32_t task2_addr;
    uint32_t delta_time;

    m_rsch_timeslot_is_granted = true;

    int8_t power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(power);
    nrf_radio_txpower_set_Expect(power);

    if (free_buffer)
    {
        nrf_radio_packetptr_set_Expect(m_test_rx_buffer.data);
    }

    // RADIO setup
    nrf_radio_shorts_set_Expect(shorts);
    nrf_radio_bcc_set_Expect(8 * (PHR_SIZE + FCF_SIZE));
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                NRF_RADIO_INT_BCMATCH_MASK  |
                                NRF_RADIO_INT_CRCOK_MASK);

    // FEM setup
    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    delta_time = rand();
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, delta_time);
    nrf_timer_cc_write_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, delta_time + ACK_IFS - TXRU_TIME - EVENT_LAT);

    // Clear EVENTS that are checked later
    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    // PPIs setup
    task2_addr = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK,
                                             (uint32_t *)task2_addr);
    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, task_addr);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE,
                                              EGU_EVENT,
                                              (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr, task2_addr);

    task_addr = rand();
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE,
                                               NRF_TIMER_TASK_START,
                                               (uint32_t *)task_addr);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE,
                                              EGU_EVENT,
                                              (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr);

    nrf_ppi_channel_include_in_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    task_addr = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

    mock_ppi_egu_worked(true);

    if (!free_buffer)
    {
        nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    }
}

static void mock_ack_requested_rx(void)
{
    uint8_t * p_ack = (uint8_t *)rand();
    uint32_t  task_addr;
    uint32_t  event_addr;
    uint32_t  time_to_pa = rand();

    time_to_pa = (time_to_pa > 0) ? time_to_pa : 1; // Time to ramp up must be greater than 0
    time_to_pa = (time_to_pa < UINT32_MAX) ? time_to_pa : UINT32_MAX - 1; // Time to ramp up must be lower than max

    nrf_802154_ack_generator_create_ExpectAndReturn(m_test_rx_buffer.data, p_ack);
    nrf_radio_packetptr_set_Expect(p_ack);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_TXREADY_START_MASK |
                                NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_TXREADY);

    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_TXEN, task_addr);
    event_addr = rand();
    nrf_timer_event_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE,
                                                NRF_TIMER_EVENT_COMPARE1,
                                               (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_TIMER_TX_ACK, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_TIMER_TX_ACK);

    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, time_to_pa);
    nrf_802154_fal_pa_configuration_set_IgnoreAndReturn(NRF_SUCCESS);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);

    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, time_to_pa - 1);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, time_to_pa + 1);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);
}

static void mock_ack_not_requested_rx(void)
{
    uint32_t event_addr;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_bcc_set_Expect(BCC_INIT);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);
}

static void mock_ack_requested_tx(void)
{
    uint32_t task_addr_1;
    uint32_t task_addr_2;
    uint32_t event_addr;

    m_state = RADIO_STATE_TX;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK);

    nrf_radio_packetptr_set_Expect(m_test_rx_buffer.data);


    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_END);
    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CCABUSY_MASK |
                                 NRF_RADIO_INT_ADDRESS_MASK |
                                 NRF_RADIO_INT_PHYEND_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_END_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                             NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                             NRF_TIMER_SHORT_COMPARE1_STOP_MASK);
    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    event_addr = rand();
    task_addr_1 = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr_1);
    
    nrf_timer_shorts_enable_Expect(m_activate_rx_cc0.event.timer.p_timer_instance,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr_1);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    event_addr = rand();
    task_addr_1 = rand();
    task_addr_2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr_2);
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, task_addr_1);
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE,
                                              EGU_EVENT,
                                              (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr_1, task_addr_2);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);
}

/***************************************************************************************************
 * @section FSM frame filtering tests
 **************************************************************************************************/

#if 0 // Disabled until timeslots are implemented
void test_OnBcmatchEventStateRxHeader_ShallSkipTimeslotRequestingIfTimeslotWasAlreadyRequested(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE - 1) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_shorts_get_ExpectAndReturn(SHORTS_RX_INITIAL);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RX);
    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);

    nrf_802154_filter_frame_part_IgnoreAndReturn(true);
    nrf_802154_filter_frame_part_ReturnThruPtr_p_num_bytes(&expected_size);

    mock_state_set(RADIO_STATE_RX_FRAME);

    irq_bcmatch_state_rx_header();
}

void test_OnBcmatchEventStateRxHeader_ShallRequestTimeslotAndSetTimeslotRequestedFlagToTrueIfFrameIsLongerThanMinimumSize(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE - 1) * 8;
    ack_requested_set_rx();

    nrf_radio_shorts_get_ExpectAndReturn(SHORTS_RX_INITIAL);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RX);
    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);

    nrf_802154_rx_duration_get_ExpectAndReturn(TEST_FRAME_SIZE, true, TEST_FRAME_DURATION);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(TEST_FRAME_DURATION, true);

    nrf_802154_filter_frame_part_IgnoreAndReturn(true);
    nrf_802154_filter_frame_part_ReturnThruPtr_p_num_bytes(&expected_size);

    mock_state_set(RADIO_STATE_RX_FRAME);

    irq_bcmatch_state_rx_header();
}

void test_OnBcmatchEventStateRxHeader_TransactionShallBeAbortedIfTimeslotRequestFails(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE - 1) * 8;
    ack_requested_set_rx();

    nrf_radio_shorts_get_ExpectAndReturn(SHORTS_RX_INITIAL);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RX);
    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);

    nrf_802154_rx_duration_get_ExpectAndReturn(TEST_FRAME_SIZE, true, TEST_FRAME_DURATION);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(TEST_FRAME_DURATION, false);

    mock_irq_deinit();
    mock_nrf_radio_reset();
    mock_state_set(RADIO_STATE_WAITING_TIMESLOT);

    nrf_802154_notify_receive_failed_Expect(NRF_802154_RX_ERROR_TIMESLOT_ENDED);

    irq_bcmatch_state_rx_header();
}
#endif // 0

void test_OnBcmatchEventStateRx_ShallDoNothingIfEventCrcerrorIsTriggered(void)
{
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, true);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_TransactionShallBeAbortedIfHeaderPartFilteringFails(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_FRAME);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(false);

    mock_rx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_receive_failed_notify(NRF_802154_RX_ERROR_INVALID_FRAME);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_TransactionShallBeAbortedIfDstAddrDoesNotMatch(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_DEST_ADDR);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(false);

    mock_rx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_receive_failed_notify(NRF_802154_RX_ERROR_INVALID_DEST_ADDR);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_TransactionShallBeAbortedIfFrameLengthInvalidInPromiscuousMode(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_LENGTH);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    mock_rx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_receive_failed_notify(NRF_802154_RX_ERROR_INVALID_LENGTH);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_ShallSetStateToRxFrameIfHeaderPartFilteringFailsInPromiscuousMode(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_DEST_ADDR);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(true);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_ShallSkipFrameFilteringIfFrameWasAlreadyFiltered(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.frame_filtered        = true;
    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    irq_bcmatch_state_rx();
}

void test_OnBcmatchEventStateRx_BccShallBeSetAndFrameFilteredFlagShallBeFalseIfHeaderPartFilteringSucceedsAndThereIsMoreDataToFilter(void)
{
    uint8_t expected_initial_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_updated_size = expected_initial_size + 4;
    uint8_t expected_initial_bcc  = expected_initial_size * 8;
    uint8_t expected_updated_bcc  = expected_updated_size * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_initial_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_NONE);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();
    nrf_802154_filter_frame_part_ReturnThruPtr_p_num_bytes(&expected_updated_size);

    nrf_radio_bcc_set_Expect(expected_updated_bcc);

    irq_bcmatch_state_rx();

    TEST_ASSERT_FALSE(m_flags.frame_filtered);
}

void test_OnBcmatchEventStateRx_FrameFilteredFlagShallBeTrueAndStateSetToRxFrameIfHeaderPartFilteringSucceedsAndThereIsNoMoreDataToFilter(void)
{
    uint8_t expected_initial_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_initial_bcc  = expected_initial_size * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_initial_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_NONE);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();
    nrf_802154_filter_frame_part_ReturnThruPtr_p_num_bytes(&expected_initial_size);

    irq_bcmatch_state_rx();

    TEST_ASSERT_TRUE(m_flags.frame_filtered);
}

void test_OnBcmatchEventStateRx_TimeslotShouldBeRequested(void)
{
    uint8_t  expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t  expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;
    uint16_t duration      = rand();
    bool     ack_req       = m_test_rx_buffer.data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_DEST_ADDR);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(true);

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, ack_req);
    nrf_802154_rx_duration_get_ExpectAndReturn(m_test_rx_buffer.data[0], false, duration);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(duration, true);

    nrf_802154_core_hooks_rx_started_Expect(m_test_rx_buffer.data);

    irq_bcmatch_state_rx();

    TEST_ASSERT_TRUE(m_flags.rx_timeslot_requested);
}

void test_OnBcmatchEventStateRx_ShallResetRadioIfTimeslotNotGranted(void)
{
    uint8_t  expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;
    uint16_t duration      = rand();
    bool     ack_req       = m_test_rx_buffer.data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_DEST_ADDR);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(true);

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, ack_req);
    nrf_802154_rx_duration_get_ExpectAndReturn(m_test_rx_buffer.data[0], false, duration);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(duration, false);

    mock_rx_terminate();
    nrf_802154_notify_receive_failed_Expect(NRF_802154_RX_ERROR_TIMESLOT_ENDED);

    irq_bcmatch_state_rx();

    TEST_ASSERT_FALSE(m_flags.rx_timeslot_requested);
    TEST_ASSERT_FALSE(m_flags.frame_filtered);
}

void test_OnBcmatchEventStateRx_ShallNotRequestTimeslotIfAlreadyGranted(void)
{
    uint8_t  expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t  expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_DEST_ADDR);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(true);

    irq_bcmatch_state_rx();

    TEST_ASSERT_TRUE(m_flags.rx_timeslot_requested);
}

void test_OnCrcOkEventStateRx_ShallResetRadioAndReportRuntimeErrorIfFrameWasNotCorrectlyFilteredAndNotInPromiscuousMode(void)
{
    nrf_802154_pib_promiscuous_get_ExpectAndReturn(false);

    mock_rx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    mock_receive_failed_notify(NRF_802154_RX_ERROR_RUNTIME);

    irq_crcok_state_rx();

    TEST_ASSERT_FALSE(m_flags.frame_filtered);
}

void test_OnCrcOkEventStateRx_ShallNotifyReceivedFrameIfAckIsNotRequested(void)
{
    m_flags.frame_filtered = true;

    ack_not_requested_set_rx();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, false);

    mock_ack_not_requested_rx();

    mock_received_frame_notify();

    irq_crcok_state_rx();

    TEST_ASSERT_FALSE(m_flags.frame_filtered);
}

void test_OnCrcOkEventStateRx_ShallNotifyReceivedFrameIfAckIsRequestedAndAutoAckIsDisabled(void)
{
    m_flags.frame_filtered = true;

    ack_requested_set_rx();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, true);
    nrf_802154_pib_auto_ack_get_ExpectAndReturn(false);

    mock_ack_not_requested_rx();

    mock_received_frame_notify();

    irq_crcok_state_rx();

    TEST_ASSERT_FALSE(m_flags.frame_filtered);
}

void test_OnCrcOkEventStateRx_ShallPrepareAckAndSetStateToTxAckIfAckIsRequested(void)
{
    m_flags.frame_filtered = true;

    ack_requested_set_rx();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, true);
    nrf_802154_pib_auto_ack_get_ExpectAndReturn(true);

    mock_ack_requested_rx();

    irq_crcok_state_rx();

    TEST_ASSERT_EQUAL(RADIO_STATE_TX_ACK, m_state);
}

/***************************************************************************************************
 * @section FSM ACK filtering tests
 **************************************************************************************************/

void test_OnPhyendEventStateTx_ShallSetupAckMatchAcceleratorToCorrectPattern(void)
{
    ack_requested_set_tx();
    m_flags.tx_started = true;

    uint8_t sequence_number = rand();
    m_test_tx_buffer[DSN_OFFSET] = sequence_number;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_tx_buffer, true);

    mock_ack_requested_tx();

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_MHRMATCH);
    nrf_radio_mhmu_search_pattern_set_Expect(MHMU_PATTERN |
                                             ((uint32_t)sequence_number << MHMU_PATTERN_DSN_OFFSET));

    irq_phyend_state_tx_frame();
}

void test_OnPhyendEventStateTx_ShallNotifyFrameAndNotSetupAckMatchAcceleratorIfAckWasNotRequested(void)
{
    ack_not_requested_set_tx();
    m_flags.tx_started = true;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_tx_buffer, false);

    mock_tx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_transmitted_frame_notify(NULL, 0, 0);

    irq_phyend_state_tx_frame();
}

void test_OnEndEventStateRxAck_ShallCallTransmitFailedOnAckMismatch(void)
{
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_MHRMATCH, false);

    mock_rx_ack_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_transmit_failed_notify(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
}

void test_OnEndEventStateRxAck_ShallNotifyFrameTransmittedOnAckMatch(void)
{
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_MHRMATCH, true);
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    mock_rx_ack_terminate();
    mock_receive_begin(false, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                              NRF_RADIO_SHORT_END_DISABLE_MASK |
                              NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    int8_t lqi = rand();
    int8_t rssi = rand();
    int8_t corrected_rssi = rand();
    uint8_t corrected_lqi = rand();
    uint32_t expected_lqi = corrected_lqi * 4;

    if (expected_lqi > UINT8_MAX)
    {
        expected_lqi = UINT8_MAX;
    }

    nrf_radio_rssi_sample_get_ExpectAndReturn(-rssi);
    nrf_802154_rssi_sample_corrected_get_ExpectAndReturn(-rssi, -corrected_rssi);
    nrf_802154_rssi_lqi_corrected_get_IgnoreAndReturn(corrected_lqi);
    mock_transmitted_frame_notify(m_test_rx_buffer.data, corrected_rssi, expected_lqi);
    irq_end_state_rx_ack();
}

/***************************************************************************************************
 * @section FSM PSDU receive notification
 **************************************************************************************************/

void test_OnBcmatchEventStateRx_ShallSetPsduBeingReceivedOnSuccessfullFiltration(void)
{
    uint8_t expected_initial_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_initial_bcc  = expected_initial_size * 8;

    m_flags.rx_timeslot_requested = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_initial_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_NONE);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();
    nrf_802154_filter_frame_part_ReturnThruPtr_p_num_bytes(&expected_initial_size);

    irq_bcmatch_state_rx();

    TEST_ASSERT_TRUE(psdu_is_being_received());
}

void test_OnBcmatchEventStateRx_ShallClearPsduBeingReceivedFlagOnFailedFiltering(void)
{
    uint8_t expected_size = PHR_SIZE + FCF_SIZE;
    uint8_t expected_bcc  = (PHR_SIZE + FCF_SIZE) * 8;

    m_flags.rx_timeslot_requested = true;
    m_flags.psdu_being_received   = true;

    nrf_radio_bcc_get_ExpectAndReturn(expected_bcc);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_CRCERROR, false);

    nrf_802154_filter_frame_part_ExpectAndReturn(m_test_rx_buffer.data, NULL, NRF_802154_RX_ERROR_INVALID_FRAME);
    nrf_802154_filter_frame_part_IgnoreArg_p_num_bytes();

    nrf_802154_pib_promiscuous_get_ExpectAndReturn(false);

    mock_rx_terminate();
    mock_receive_begin(true, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                             NRF_RADIO_SHORT_END_DISABLE_MASK |
                             NRF_RADIO_SHORT_RXREADY_START_MASK |
                             NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    mock_receive_failed_notify(NRF_802154_RX_ERROR_INVALID_FRAME);

    irq_bcmatch_state_rx();

    TEST_ASSERT_FALSE(psdu_is_being_received());
}

void test_OnCrcErrorEventStateRx_ShallClearPsduBeingReceivedFlag(void)
{
    uint32_t event_addr;

    m_flags.psdu_being_received = true;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_bcc_set_Expect(BCC_INIT);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_802154_notify_receive_failed_Expect(NRF_802154_RX_ERROR_INVALID_FCS);
    nrf_802154_critical_section_nesting_deny_Expect();

    irq_crcerror_state_rx();

    TEST_ASSERT_FALSE(psdu_is_being_received());
}

void test_OnCrcOkEventStateRx_ShallClearPsduBeingReceivedFlagWhenAckIsNotRequested(void)
{
    m_flags.frame_filtered      = true;
    m_flags.psdu_being_received = true;

    ack_not_requested_set_rx();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, false);

    mock_ack_not_requested_rx();

    mock_received_frame_notify();

    irq_crcok_state_rx();

    TEST_ASSERT_FALSE(psdu_is_being_received());
}

void test_OnCrcOkEventStateRx_ShallNotClearPsduBeingReceivedFlagWhenAckIsRequested(void)
{
    m_flags.frame_filtered      = true;
    m_flags.psdu_being_received = true;

    ack_requested_set_rx();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_test_rx_buffer.data, true);
    nrf_802154_pib_auto_ack_get_ExpectAndReturn(true);

    mock_ack_requested_rx();

    irq_crcok_state_rx();

    TEST_ASSERT_TRUE(psdu_is_being_received());
}
