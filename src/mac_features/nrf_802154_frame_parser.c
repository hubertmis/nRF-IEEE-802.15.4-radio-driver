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
 *   This file implements frame parsing utilities for 802.15.4 radio driver.
 *
 * @note This module is based on following assumptions:
 *       a. All received frames contain both source and destination address.
 *       b. All received frames contain destination PAN ID field.
 *       Frames that don't meet these assumptions are dropped.
 */

#include "nrf_802154_frame_parser.h"

#include <stdlib.h>

#include "nrf_802154_const.h"


/***************************************************************************************************
 * @section Helper functions
 **************************************************************************************************/

static bool addressing_is_valid(const uint8_t * p_frame)
{
    if ((p_frame[SRC_ADDR_TYPE_OFFSET] & SRC_ADDR_TYPE_MASK)
        || (p_frame[DEST_ADDR_TYPE_OFFSET] & DEST_ADDR_TYPE_MASK))
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool security_is_enabled(const uint8_t * p_frame)
{
    return p_frame[SECURITY_ENABLED_OFFSET] & SECURITY_ENABLED_BIT ? true : false;
}

static uint8_t key_id_size_get(const uint8_t * p_frame)
{
    switch(*nrf_802154_frame_parser_sec_ctrl_get(p_frame) & KEY_ID_MODE_MASK)
    {
        case KEY_ID_MODE_1:
            return KEY_ID_MODE_1_SIZE;

        case KEY_ID_MODE_2:
            return KEY_ID_MODE_2_SIZE;

        case KEY_ID_MODE_3:
            return KEY_ID_MODE_3_SIZE;

        default:
            return 0;
    }
}

/***************************************************************************************************
 * @section Frame format functions
 **************************************************************************************************/

bool nrf_802154_frame_parser_dst_addr_is_extended(const uint8_t * p_frame)
{
    switch (p_frame[DEST_ADDR_TYPE_OFFSET] & DEST_ADDR_TYPE_MASK)
    {
        case DEST_ADDR_TYPE_EXTENDED:
            return true;

        default:
            return false;
    }
}

bool nrf_802154_frame_parser_src_addr_is_extended(const uint8_t * p_frame)
{
    switch (p_frame[SRC_ADDR_TYPE_OFFSET] & SRC_ADDR_TYPE_MASK)
    {
        case SRC_ADDR_TYPE_EXTENDED:
            return true;

        default:
            return false;
    }
}

bool nrf_802154_frame_parser_src_panid_is_compressed(const uint8_t * p_frame)
{
    bool panid_compression = (p_frame[PAN_ID_COMPR_OFFSET] & PAN_ID_COMPR_MASK) ? true : false;
    bool src_panid_omitted = nrf_802154_frame_parser_dst_addr_is_extended(p_frame)
                             && nrf_802154_frame_parser_src_addr_is_extended(p_frame);

    return panid_compression || src_panid_omitted;
}

bool nrf_802154_frame_parser_dsn_suppress_bit_is_set(const uint8_t * p_frame)
{
    return (p_frame[DSN_SUPPRESS_OFFSET] & DSN_SUPPRESS_BIT) ? true : false;
}

bool nrf_802154_frame_parser_ie_present_bit_is_set(const uint8_t * p_frame)
{
    return (p_frame[IE_PRESENT_OFFSET] & IE_PRESENT_BIT) ? true : false;
}

/***************************************************************************************************
 * @section Offset functions
 **************************************************************************************************/

uint8_t nrf_802154_frame_parser_dst_panid_offset_get(const uint8_t * p_frame)
{
    if (addressing_is_valid(p_frame))
    {
        return nrf_802154_frame_parser_dsn_suppress_bit_is_set(p_frame) ? PAN_ID_OFFSET - 1 : PAN_ID_OFFSET;
    }
    else
    {
        return 0;
    }
}

uint8_t nrf_802154_frame_parser_dst_addr_offset_get(const uint8_t * p_frame)
{
    uint8_t dst_panid_offset = nrf_802154_frame_parser_dst_panid_offset_get(p_frame);

    return (0 == dst_panid_offset) ? 0 : dst_panid_offset + PAN_ID_SIZE;
}

uint8_t nrf_802154_frame_parser_src_panid_offset_get(const uint8_t * p_frame)
{
    if (nrf_802154_frame_parser_src_panid_is_compressed(p_frame))
    {
        // TODO: Provide also a smarter implementation, which would distinguish between compressed
        // PAN ID and invalid addressing and return pointer to destination PAN ID in the first case.
        return 0;
    }
    else
    {
        uint8_t dst_addr_len = nrf_802154_frame_parser_dst_addr_is_extended(p_frame) ?
                               EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE;

        return nrf_802154_frame_parser_dst_addr_offset_get(p_frame) + dst_addr_len;
    }
}

uint8_t nrf_802154_frame_parser_src_addr_offset_get(const uint8_t * p_frame)
{
    uint8_t src_panid_offset = nrf_802154_frame_parser_src_panid_offset_get(p_frame);
    if (0 == src_panid_offset)
    {
        uint8_t dst_addr_len = nrf_802154_frame_parser_dst_addr_is_extended(p_frame) ?
                               EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE;

        uint8_t dst_addr_offset = nrf_802154_frame_parser_dst_addr_offset_get(p_frame);

        return (0 == dst_addr_offset) ? 0 : dst_addr_offset + dst_addr_len;
    }
    else
    {
        return src_panid_offset + PAN_ID_SIZE;
    }
}

uint8_t nrf_802154_frame_parser_sec_ctrl_offset_get(const uint8_t * p_frame)
{
    if (!security_is_enabled(p_frame))
    {
        return 0;
    }
    else
    {
        uint8_t src_addr_len = nrf_802154_frame_parser_src_addr_is_extended(p_frame) ?
                               EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE;

        return nrf_802154_frame_parser_src_addr_offset_get(p_frame) + src_addr_len;
    }
}

uint8_t nrf_802154_frame_parser_key_id_offset_get(const uint8_t * p_frame)
{
    uint8_t sec_ctrl_offset = nrf_802154_frame_parser_sec_ctrl_offset_get(p_frame);
    if (0 == sec_ctrl_offset)
    {
        return 0;
    }

    if (p_frame[sec_ctrl_offset] & FRAME_COUNTER_SUPPRESS_BIT)
    {
        return sec_ctrl_offset + SECURITY_CONTROL_SIZE;
    }
    else
    {
        return sec_ctrl_offset + SECURITY_CONTROL_SIZE + FRAME_COUNTER_SIZE;
    }
}

uint8_t nrf_802154_frame_parser_ie_header_offset_get(const uint8_t * p_frame)
{
    if (!nrf_802154_frame_parser_ie_present_bit_is_set(p_frame))
    {
        return 0;
    }
    else
    {
        if (!security_is_enabled(p_frame))
        {
            uint8_t src_addr_size = nrf_802154_frame_parser_src_addr_is_extended(p_frame) ?
                                    EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE;
            return nrf_802154_frame_parser_src_addr_offset_get(p_frame) + src_addr_size;
        }
        else
        {
            return nrf_802154_frame_parser_key_id_offset_get(p_frame) + key_id_size_get(p_frame); 
        }
    }
}

/***************************************************************************************************
 * @section Get functions
 **************************************************************************************************/

const uint8_t * nrf_802154_frame_parser_dst_addr_get(const uint8_t * p_frame, bool * p_dst_addr_extended)
{
    if (!addressing_is_valid(p_frame))
    {
        *p_dst_addr_extended = false;
        return NULL;
    }
    else
    {
        const uint8_t * p_dst_addr = &p_frame[nrf_802154_frame_parser_dst_addr_offset_get(p_frame)];
        *p_dst_addr_extended = NULL == p_dst_addr ?
                               false : nrf_802154_frame_parser_dst_addr_is_extended(p_frame);
        return p_dst_addr;
    }
}

const uint8_t * nrf_802154_frame_parser_dst_panid_get(const uint8_t * p_frame)
{
    uint8_t dst_panid_offset = nrf_802154_frame_parser_dst_panid_offset_get(p_frame);

    return (0 == dst_panid_offset) ? NULL : &p_frame[dst_panid_offset];
}

const uint8_t * nrf_802154_frame_parser_src_panid_get(const uint8_t * p_frame)
{
    uint8_t src_panid_offset = nrf_802154_frame_parser_src_panid_offset_get(p_frame);

    return (0 == src_panid_offset) ? NULL : &p_frame[src_panid_offset];
}

const uint8_t * nrf_802154_frame_parser_src_addr_get(const uint8_t * p_frame, bool * p_src_addr_extended)
{
    if (!addressing_is_valid(p_frame))
    {
        *p_src_addr_extended = false;
        return NULL;
    }
    else
    {
        const uint8_t * p_src_addr = &p_frame[nrf_802154_frame_parser_src_addr_offset_get(p_frame)];
        *p_src_addr_extended = NULL == p_src_addr ? 
                               false : nrf_802154_frame_parser_src_addr_is_extended(p_frame);
        return p_src_addr;
    }
}

const uint8_t * nrf_802154_frame_parser_sec_ctrl_get(const uint8_t * p_frame)
{
    uint8_t sec_ctrl_offset = nrf_802154_frame_parser_sec_ctrl_offset_get(p_frame);

    return (0 == sec_ctrl_offset) ? NULL : &p_frame[sec_ctrl_offset];
}

const uint8_t * nrf_802154_frame_parser_key_id_get(const uint8_t * p_frame)
{
    uint8_t key_id_offset = nrf_802154_frame_parser_key_id_offset_get(p_frame);

    return (0 == key_id_offset) ? NULL : &p_frame[key_id_offset];
}

const uint8_t * nrf_802154_frame_parser_ie_header_get(const uint8_t * p_frame, uint8_t * p_length)
{
    uint8_t ie_header_offset = nrf_802154_frame_parser_ie_header_offset_get(p_frame);

    if (0 == ie_header_offset)
    {
        return NULL;
    }

    // TODO: Only a single IE header is supported
    *p_length = (p_frame[ie_header_offset] & IE_HEADER_LENGTH_MASK) + IE_HEADER_SIZE;

    return &p_frame[ie_header_offset];
}
