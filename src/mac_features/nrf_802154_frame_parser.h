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
 * @brief This module contatins frame parsing utilities for 802.15.4 radio driver.
 * 
 */

#ifndef NRF_802154_FRAME_PARSER_H
#define NRF_802154_FRAME_PARSER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Determine if destination address is extended.
 *
 * @param[in]   p_frame   Pointer to a frame to check.
 *
 * @retval  true   If destination address is extended.
 * @retval  false  Otherwise.
 */
bool nrf_802154_frame_parser_dst_addr_is_extended(const uint8_t * p_frame);

/**
 * @brief Get destination address from provided frame.
 *
 * @param[in]   p_frame             Pointer to a frame.
 * @param[out]  p_dst_addr_extended Pointer to a value which is true if destination address is extended.
 *                                  Otherwise it is false.
 *
 * @returns  Pointer to the first byte of destination address in @p p_frame.
 *           NULL if destination address cannot be retrieved.
 */
const uint8_t * nrf_802154_frame_parser_dst_addr_get(const uint8_t * p_frame, bool * p_dst_addr_extended);

/**
 * @brief Get offset of destination address field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of destination address field including one byte of frame length.
 *           Zero if destination address cannot be retrieved.
 */
uint8_t nrf_802154_frame_parser_dst_addr_offset_get(const uint8_t * p_frame);

/**
 * @brief Get destination PAN ID from provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Pointer to the first byte of destination PAN ID in @p p_frame.
 *           NULL if destination PAN ID cannot be retrieved.
 */
const uint8_t * nrf_802154_frame_parser_dst_panid_get(const uint8_t * p_frame);

/**
 * @brief Get offset of destination PAN ID field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of destination PAN ID field including one byte of frame length.
 *           Zero in case the destination PAN ID cannot be retrieved.
 */
uint8_t nrf_802154_frame_parser_dst_panid_offset_get(const uint8_t * p_frame);

/**
 * @brief Determine if source address is extended.
 *
 * @param[in]   p_frame   Pointer to a frame to check.
 *
 * @retval  true   If source address is extended.
 * @retval  false  Otherwise.
 */
bool nrf_802154_frame_parser_src_addr_is_extended(const uint8_t * p_frame);

/**
 * @brief Get source address from provided frame.
 *
 * @param[in]   p_frame             Pointer to a frame.
 * @param[out]  p_src_addr_extended Pointer to a value which is true if source address is extended.
 *                                  Otherwise it is false.
 *
 * @returns  Pointer to the first byte of source address in @p p_frame.
 *           NULL if source address cannot be retrieved.
 */
const uint8_t * nrf_802154_frame_parser_src_addr_get(const uint8_t * p_frame, bool * p_src_addr_extended);

/**
 * @brief Get offset of source address field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of source address field including one byte of frame length.
 *           Zero if source address cannot be retrieved.
 */
uint8_t nrf_802154_frame_parser_src_addr_offset_get(const uint8_t * p_frame);

/**
 * @brief Determine if source PAN ID is compressed.
 *
 * @param[in]   p_frame   Pointer to a frame to check.
 *
 * @retval  true   If source PAN ID is compressed.
 * @retval  false  Otherwise.
 */
bool nrf_802154_frame_parser_src_panid_is_compressed(const uint8_t * p_frame);

/**
 * @brief Get source PAN ID from provided frame.
 *
 * @param[in]   p_frame   Pointer to a frame.
 *
 * @returns  Pointer to the first byte of source PAN ID in @p p_frame.
 *           NULL if source PAN ID cannot be retrieved or if it is compressed.
 */
const uint8_t * nrf_802154_frame_parser_src_panid_get(const uint8_t * p_frame);

/**
 * @brief Get offset of source PAN ID field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of source PAN ID field including one byte of frame length.
 *           Zero in case the source PAN ID cannot be retrieved or it is compressed.
 */
uint8_t nrf_802154_frame_parser_src_panid_offset_get(const uint8_t * p_frame);

/**
 * @brief Get security control field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Pointer to the first byte of security control field in @p p_frame.
 *           NULL if security control cannot be retrieved (security not enabled).
 */
const uint8_t * nrf_802154_frame_parser_sec_ctrl_get(const uint8_t * p_frame);

/**
 * @brief Get offset of security control field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of security control field including one byte of frame length.
 *           Zero if security control cannot be retrieved (security not enabled).
 */
uint8_t nrf_802154_frame_parser_sec_ctrl_offset_get(const uint8_t * p_frame);

/**
 * @brief Get key identifier field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Pointer to the first byte of key identifier field in @p p_frame.
 *           NULL if key identifier cannot be retrieved (security not enabled).
 */
const uint8_t * nrf_802154_frame_parser_key_id_get(const uint8_t * p_frame);

/**
 * @brief Get offset of key identifier field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of key identifier field including one byte of frame length.
 *           Zero if key identifier cannot be retrieved (security not enabled).
 */
uint8_t nrf_802154_frame_parser_key_id_offset_get(const uint8_t * p_frame);

/**
 * @brief Determine if sequence number suppression bit is set.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @retval  true   If sequence number suppression bit is set.
 * @retval  false  Otherwise.
 */
bool nrf_802154_frame_parser_dsn_suppress_bit_is_set(const uint8_t * p_frame);

/**
 * @brief Determine if IE present bit is set.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @retval  true   If IE present bit is set.
 * @retval  false  Otherwise.
 */
bool nrf_802154_frame_parser_ie_present_bit_is_set(const uint8_t * p_frame);

/**
 * @brief Get IE header field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 * @param[out]  p_length Length of the IE header field.
 *
 * @returns  Pointer to the first byte of IE header field in @p p_frame.
 *           NULL if IE header cannot be retrieved (IE not present).
 */
const uint8_t * nrf_802154_frame_parser_ie_header_get(const uint8_t * p_frame, uint8_t * p_length);

/**
 * @brief Get offset of IE header field in provided frame.
 *
 * @param[in]   p_frame  Pointer to a frame.
 *
 * @returns  Offset in bytes of IE header field including one byte of frame length.
 *           Zero if IE header cannot be retrieved (IE not present).
 */
uint8_t nrf_802154_frame_parser_ie_header_offset_get(const uint8_t * p_frame);

#endif // NRF_802154_FRAME_PARSER_H
