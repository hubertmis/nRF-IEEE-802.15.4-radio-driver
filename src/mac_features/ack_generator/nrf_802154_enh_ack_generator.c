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
 *   This file implements an enhanced acknowledgement (Enh-Ack) generator for 802.15.4 radio driver.
 *
 */

#include "nrf_802154_enh_ack_generator.h"

#include <assert.h>
#include <string.h>

#include "mac_features/nrf_802154_frame_parser.h"
#include "nrf_802154_ack_data.h"
#include "nrf_802154_ack_pending_bit.h"
#include "nrf_802154_const.h"

#define ENH_ACK_MAX_SIZE     MAX_PACKET_SIZE

static uint8_t m_ack_psdu[ENH_ACK_MAX_SIZE + PHR_SIZE];

static void ack_buffer_clear(void)
{
    memset(m_ack_psdu, 0, FCF_SIZE + PHR_SIZE);
}

static void sequence_number_set(const uint8_t * p_frame)
{
    if (!nrf_802154_frame_parser_dsn_suppress_bit_is_set(p_frame))
    {
        m_ack_psdu[DSN_OFFSET] = p_frame[DSN_OFFSET];
    }
}

/***************************************************************************************************
 * @section Frame control field functions
 **************************************************************************************************/

static void fcf_frame_type_set(void)
{
    m_ack_psdu[FRAME_TYPE_OFFSET] |= FRAME_TYPE_ACK;
}

static void fcf_security_enabled_set(const uint8_t * p_frame)
{
    m_ack_psdu[SECURITY_ENABLED_OFFSET] |= (p_frame[SECURITY_ENABLED_OFFSET] & SECURITY_ENABLED_BIT);
}

static void fcf_frame_pending_set(const uint8_t * p_frame)
{
    if (nrf_802154_ack_pending_bit_should_be_set(p_frame))
    {
        m_ack_psdu[FRAME_PENDING_OFFSET] |= FRAME_PENDING_BIT;
    }
}

static void fcf_panid_compression_set(const uint8_t * p_frame)
{
    if (p_frame[PAN_ID_COMPR_OFFSET] & PAN_ID_COMPR_MASK)
    {
        m_ack_psdu[PAN_ID_COMPR_OFFSET] |= PAN_ID_COMPR_MASK;
        m_ack_psdu[0] += PAN_ID_SIZE;
    }
    else
    {
        // Handle special 802.15.4 case: source address extended, destination address extended,
        // destination PAN ID present, source PAN ID not present, PAN ID compression not set.
        if (nrf_802154_frame_parser_dst_addr_is_extended(p_frame) && nrf_802154_frame_parser_src_addr_is_extended(p_frame))
        {
            m_ack_psdu[0] += PAN_ID_SIZE;
        }
        else
        {
            m_ack_psdu[0] += 2 * PAN_ID_SIZE;
        }
    }
}

static void fcf_sequence_number_suppression_set(const uint8_t * p_frame)
{
    if (nrf_802154_frame_parser_dsn_suppress_bit_is_set(p_frame))
    {
        m_ack_psdu[DSN_SUPPRESS_OFFSET] |= DSN_SUPPRESS_BIT;
    }
    else
    {
        m_ack_psdu[0] += DSN_SIZE;
    }
}

static void fcf_ie_present_set(const uint8_t * p_frame, const uint8_t * p_ie_data)
{
    if (p_ie_data != NULL)
    {
        m_ack_psdu[IE_PRESENT_OFFSET] |= IE_PRESENT_BIT;
    }
}

static void fcf_dst_addressing_mode_set(const uint8_t * p_frame)
{
    if (nrf_802154_frame_parser_src_addr_is_extended(p_frame))
    {
        m_ack_psdu[DEST_ADDR_TYPE_OFFSET] |= DEST_ADDR_TYPE_EXTENDED;
        m_ack_psdu[0] += EXTENDED_ADDRESS_SIZE;
    }
    else
    {
        m_ack_psdu[DEST_ADDR_TYPE_OFFSET] |= DEST_ADDR_TYPE_SHORT;
        m_ack_psdu[0] += SHORT_ADDRESS_SIZE;
    }
}

static void fcf_src_addressing_mode_set(const uint8_t * p_frame)
{
    // TODO: Ack destination addressing mode should not be related to frame source addressing mode.
    if (nrf_802154_frame_parser_dst_addr_is_extended(p_frame))
    {
        m_ack_psdu[SRC_ADDR_TYPE_OFFSET] |= SRC_ADDR_TYPE_EXTENDED;
        m_ack_psdu[0] += EXTENDED_ADDRESS_SIZE;
    }
    else
    {
        m_ack_psdu[SRC_ADDR_TYPE_OFFSET] |= SRC_ADDR_TYPE_SHORT;
        m_ack_psdu[0] += SHORT_ADDRESS_SIZE;
    }
}

static void fcf_frame_version_set(void)
{
    m_ack_psdu[FRAME_VERSION_OFFSET] |= FRAME_VERSION_2;
}

static void frame_control_set(const uint8_t * p_frame, const uint8_t * p_ie_data)
{  
    fcf_frame_type_set();
    fcf_security_enabled_set(p_frame);
    fcf_frame_pending_set(p_frame);
    fcf_panid_compression_set(p_frame);
    fcf_sequence_number_suppression_set(p_frame);
    fcf_ie_present_set(p_frame, p_ie_data);
    fcf_dst_addressing_mode_set(p_frame);
    fcf_frame_version_set();
    fcf_src_addressing_mode_set(p_frame);

    m_ack_psdu[0] += FCF_SIZE;
    m_ack_psdu[0] += FCS_SIZE;
}

/***************************************************************************************************
 * @section Addressing fields functions
 **************************************************************************************************/

static void destination_set(const uint8_t * p_frame)
{
    bool            frame_src_addr_extended;
    const uint8_t * p_frame_src_panid = nrf_802154_frame_parser_src_panid_get(p_frame);
    const uint8_t * p_frame_src_addr  = nrf_802154_frame_parser_src_addr_get(p_frame, &frame_src_addr_extended);

    // Fill the Ack destination PAN ID field accordingly.
    if (p_frame_src_panid == NULL)
    {
        // Frame source PAN ID is compressed. Frame destination PAN ID can be used.
        memcpy(&m_ack_psdu[nrf_802154_frame_parser_dst_panid_offset_get(m_ack_psdu)],
               nrf_802154_frame_parser_dst_panid_get(p_frame),
               PAN_ID_SIZE);
    }
    else
    {
        // Frame source PAN ID is not compressed and must be used.
        memcpy(&m_ack_psdu[nrf_802154_frame_parser_dst_panid_offset_get(m_ack_psdu)],
               p_frame_src_panid,
               PAN_ID_SIZE);
    }

    // Fill the Ack destination address field.
    memcpy(&m_ack_psdu[nrf_802154_frame_parser_dst_addr_offset_get(m_ack_psdu)],
           p_frame_src_addr,
           frame_src_addr_extended ? EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE);
}

static void source_set(const uint8_t * p_frame)
{
    bool    frame_dst_addr_extended;
    uint8_t ack_src_panid_offset = nrf_802154_frame_parser_src_panid_offset_get(m_ack_psdu);

    // Ack source PAN ID is not compressed and should be set.
    if (0 != ack_src_panid_offset)
    {
        memcpy(&m_ack_psdu[ack_src_panid_offset],
               nrf_802154_frame_parser_dst_panid_get(p_frame),
               PAN_ID_SIZE);
    }

    // Ack source address should be always set.
    memcpy(&m_ack_psdu[nrf_802154_frame_parser_src_addr_offset_get(m_ack_psdu)],
           nrf_802154_frame_parser_dst_addr_get(p_frame, &frame_dst_addr_extended),
           frame_dst_addr_extended ? EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE);
}

/***************************************************************************************************
 * @section Auxiliary security header functions
 **************************************************************************************************/

static void security_control_set(const uint8_t * p_frame, uint8_t sec_ctrl_offset)
{
    // All the bits in the security control byte can be copied.
    m_ack_psdu[sec_ctrl_offset] = *nrf_802154_frame_parser_sec_ctrl_get(p_frame);

    m_ack_psdu[0] += SECURITY_CONTROL_SIZE;
}

static void security_key_id_set(const uint8_t * p_frame, uint8_t sec_ctrl_offset)
{
    uint8_t key_id_mode_size = 0;

    switch(m_ack_psdu[sec_ctrl_offset] & KEY_ID_MODE_MASK)
    {
        case KEY_ID_MODE_1:
            key_id_mode_size = KEY_ID_MODE_1_SIZE;
            break;

        case KEY_ID_MODE_2:
            key_id_mode_size = KEY_ID_MODE_2_SIZE;
            break;

        case KEY_ID_MODE_3:
            key_id_mode_size = KEY_ID_MODE_3_SIZE;
            break;

        default:
            break;
    }

    if (0 != key_id_mode_size)
    {
        memcpy(&m_ack_psdu[nrf_802154_frame_parser_key_id_offset_get(m_ack_psdu)],
               nrf_802154_frame_parser_key_id_get(p_frame),
               key_id_mode_size);
        m_ack_psdu[0] += key_id_mode_size;
    }

    switch (m_ack_psdu[sec_ctrl_offset] & SECURITY_LEVEL_MASK)
    {
        case SECURITY_LEVEL_MIC_32:
        case SECURITY_LEVEL_ENC_MIC_32:
            m_ack_psdu[0] += MIC_32_SIZE;
            break;

        case SECURITY_LEVEL_MIC_64:
        case SECURITY_LEVEL_ENC_MIC_64:
            m_ack_psdu[0] += MIC_64_SIZE;
            break;

        case SECURITY_LEVEL_MIC_128:
        case SECURITY_LEVEL_ENC_MIC_128:
            m_ack_psdu[0] += MIC_128_SIZE;
            break;

        default:
            break;
    }
}

static void security_header_set(const uint8_t * p_frame)
{
    if (0 == (m_ack_psdu[SECURITY_ENABLED_OFFSET] & SECURITY_ENABLED_BIT))
    {
        return;
    }

    // Latch security control offset for performance.
    uint8_t sec_ctrl_offset = nrf_802154_frame_parser_sec_ctrl_offset_get(m_ack_psdu);

    security_control_set(p_frame, sec_ctrl_offset);

    // Frame counter is set by MAC layer when the frame is encrypted.
    if (0 == (m_ack_psdu[sec_ctrl_offset] & FRAME_COUNTER_SUPPRESS_BIT))
    {
        m_ack_psdu[0] += FRAME_COUNTER_SIZE;
    }

    security_key_id_set(p_frame, sec_ctrl_offset);
}

/***************************************************************************************************
 * @section Information Elements
 **************************************************************************************************/

static void ie_header_set(const uint8_t * p_frame, const uint8_t * p_ie_data, uint8_t ie_data_len)
{
    if (p_ie_data == NULL)
    {
        return;
    }

    memcpy(&m_ack_psdu[nrf_802154_frame_parser_ie_header_offset_get(m_ack_psdu)], p_ie_data, ie_data_len);
    m_ack_psdu[0] += ie_data_len;
}

/***************************************************************************************************
 * @section Public API implementation
 **************************************************************************************************/

void nrf_802154_enh_ack_generator_init(void)
{
    // Intentionally empty.
}

const uint8_t * nrf_802154_enh_ack_generator_create(const uint8_t * p_frame)
{
    uint8_t         ie_data_len;
    const uint8_t * p_ie_data = nrf_802154_ack_data_ie_get(p_frame, &ie_data_len);

    // Clear previously created ACK.
    ack_buffer_clear();

    // Set Frame Control field bits.
    frame_control_set(p_frame, p_ie_data);

    // Set valid sequence number in ACK frame.
    sequence_number_set(p_frame);

    // Set destination address and PAN ID.
    destination_set(p_frame);

    // Set source address and PAN ID.
    source_set(p_frame);

    // Set auxiliary security header.
    security_header_set(p_frame);

    // Set IE header.
    ie_header_set(p_frame, p_ie_data, ie_data_len);

    return m_ack_psdu;
}
