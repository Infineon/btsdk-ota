/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * WICED Bluetooth OTA Upgrade using either SPP or BLE OTP protocol.
 *
 * This file provides function required to support Over the Air WICED Upgrade.
 * Both secure and none secure services are supported.  In the none-secure
 * case the software is only protected by the checksum.  In the secure case
 * the image is protected with a digital signature which is verified using
 * the private key passed by the application.
 *
 * To download host sends command to download with length of the patch to be
 * transmitted.  SPP Write Requests are used to send commands and portions of
 * data.  In case of an error Error Response indicates failure to the host.
 * Host sends fixed chunks of data.  After all the bytes has been downloaded
 * and acknowledged host sends verify command that includes CRC32 of the
 * whole patch.  During the download device saves data directly to the
 * serial flash.  At the verification stage device reads data back from the
 * NVRAM and calculates checksum of the data stored there.  Result of the
 * verification is indicated using SPP data write.
 *
 */
#include "spp_ota_fw_upgrade.h"
#include "bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_bt_trace.h"
#include "wiced_gki.h"
#include <wiced_firmware_upgrade.h>
#include "wiced_timer.h"
#include "sha256.h"
#include "p_256_multprecision.h"
#include "wiced_bt_spp.h"
#include "ota_fw_upgrade.h"

/******************************************************
 *                      Constants
 ******************************************************/

#define OTA_FW_UPGRADE_CONTROL_COMMAND              01
#define OTA_FW_UPGRADE_DATA                         02
#define OTA_FW_UPGRADE_EVENT                        03

static wiced_bool_t ota_fw_upgrade_command_handler(uint16_t conn_id, uint8_t command, uint8_t *data, int32_t len);
static void ota_fw_upgrade_reset_timer_timeout(uint32_t param);
static wiced_result_t wiced_spp_ota_firmware_upgrade_send_status(uint16_t handle, uint8_t status);
static wiced_result_t wiced_spp_ota_firmware_upgrade_command_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static wiced_result_t wiced_spp_ota_firmware_upgrade_data_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static wiced_result_t wiced_ota_firmware_upgrade_send_status(uint16_t handle, uint8_t status);
static wiced_bool_t wiced_ota_indication_enabled(void);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/*
 * Process timeout started after the last notification to perform restart
 */
void ota_fw_upgrade_reset_timer_timeout(uint32_t param)
{
    wiced_firmware_upgrade_finish();
}

/*
* Sends the resulted status on handling peer OTA command requests received over SPP.
*/
wiced_result_t  wiced_spp_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t data[3];
    data[0] = OTA_FW_UPGRADE_EVENT << 4 | status;
    data[1] = 0;
    data[2] = 0; // Length 2 bytes; set to 0 as there is no payload attached to event at the moment

    if (!wiced_bt_spp_send_session_data( handle, data, 3 ) )
    {
        result = WICED_ERROR;
    }
    return result;
}

/*
* Utility to extract the OTA command code from received SPP data and calls OTA commands handler.
*/
wiced_result_t wiced_spp_ota_firmware_upgrade_command_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t       len = 0;
    uint8_t        command = *p_data & 0x0F;
    len = p_data[1] + (p_data[2] << 8);

    WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_COMMAND packet %x data_len:%d  payload len: %d\n", *p_data, data_len, len );

    if( len == ( data_len - 3 ))
    {
       if ( ota_fw_upgrade_command_handler( handle, command, p_data+3, len ) )
       {
           result = WICED_SUCCESS;
       }
    }
    else
    {
        WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_COMMAND payload length does not match");
    }
    return result;
}

/*
 * handle OTA upgrade commands
 */
wiced_bool_t ota_fw_upgrade_command_handler(uint16_t conn_id, uint8_t command, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;
    int32_t verified = WICED_FALSE;

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("OTA handle cmd:%d, state:%d\n", command, p_state->state);
#endif
    if (command == WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD)
    {
        p_state->state = OTA_STATE_READY_FOR_DOWNLOAD;
        wiced_ota_firmware_upgrade_send_status(conn_id, value);

        if (ota_fw_upgrade_status_callback)
        {
            (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_STARTED);
        }
        return WICED_TRUE;
    }
    if (command == WICED_OTA_UPGRADE_COMMAND_ABORT)
    {
        p_state->state = OTA_STATE_ABORTED;
        wiced_ota_firmware_upgrade_send_status(conn_id, value);

        if (ota_fw_upgrade_status_callback)
        {
            (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
        }
        return WICED_FALSE;
    }

    switch (p_state->state)
    {
    case OTA_STATE_IDLE:
        return WICED_TRUE;

    case OTA_STATE_READY_FOR_DOWNLOAD:
        if (command == WICED_OTA_UPGRADE_COMMAND_DOWNLOAD)
        {
            // command to start upgrade should be accompanied by 4 bytes with the image size
            if (len < 4)
            {
                WICED_BT_TRACE("Bad Download len: %d \n", len);
                return WICED_FALSE;
            }

            if (!wiced_firmware_upgrade_init_nv_locations())
            {
                WICED_BT_TRACE("failed init nv locations\n");
                value = WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);
                return WICED_FALSE;
            }

            p_state->state                = OTA_STATE_DATA_TRANSFER;
            p_state->current_offset       = 0;
            p_state->current_block_offset = 0;
            p_state->total_offset         = 0;
            p_state->total_len            = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
#if OTA_UPGRADE_DEBUG
            p_state->recv_crc32           = 0xffffffff;
#endif

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20735B0) || defined(CYW20735B1) || defined(CYW20835B1) /*|| defined (CYW20819A1)*/)
            // if we are using Secure version the total length comes in the beginning of the image,
            // do not use the one from the downloader.
            if (p_ecdsa_public_key != NULL)
            {
                p_state->total_len            = 0;
            }
            else
            {
                p_state->total_len            = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
            }
#endif
            WICED_BT_TRACE("state %d total_len %d \n", p_state->state, data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
            wiced_ota_firmware_upgrade_send_status(conn_id, value);
            return WICED_TRUE;
        }
        break;

    case OTA_STATE_DATA_TRANSFER:
        if (command == WICED_OTA_UPGRADE_COMMAND_VERIFY)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 4)
            {
                WICED_BT_TRACE("Bad Verify len %d \n", len);
                return WICED_FALSE;
            }
            // command to perform verification.
            if (p_state->total_len != p_state->total_offset)
            {
                WICED_BT_TRACE("Verify failed received:%d out of %d\n", p_state->total_offset, p_state->total_len);
                p_state->state = OTA_STATE_ABORTED;
                value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);

                if (ota_fw_upgrade_status_callback)
                {
                    (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_FALSE;
            }

            // For none-secure case the command should have 4 bytes CRC32
            if (p_ecdsa_public_key == NULL)
            {
                p_state->crc32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                verified = ota_fw_upgrade_verify();
            }
            else
            {
                WICED_BT_TRACE("ota_sec_fw_upgrade_verify() \n");
                verified = ota_sec_fw_upgrade_verify();
            }

            if (!verified)
            {
                WICED_BT_TRACE("Verify failed\n");
                p_state->state = OTA_STATE_ABORTED;
                value = WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
                wiced_ota_firmware_upgrade_send_status(conn_id, value);

                if (ota_fw_upgrade_status_callback)
                {
                    (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_FALSE;
            }
            WICED_BT_TRACE("Verify success\n");
            p_state->state = OTA_STATE_VERIFIED;

            if ( WICED_SUCCESS == wiced_ota_firmware_upgrade_send_status(conn_id, value) )
            {
                if ( wiced_ota_indication_enabled() )
                {
                    return WICED_TRUE;
                }
                else
                {
                    //notify application that we are going down
                    if (ota_fw_upgrade_status_callback)
                    {
                        (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
                    }
                    // init timer for detect packet retransmission
                    wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
                    wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timer_timeout, 0, WICED_SECONDS_TIMER);
                    wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
                    return WICED_TRUE;
                }
            }

            WICED_BT_TRACE("failed to notify the peer\n");
            return WICED_FALSE;
        }
        break;

    case OTA_STATE_ABORTED:
    default:
        break;
    }

    value = WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
    wiced_ota_firmware_upgrade_send_status(conn_id, value);
    return WICED_FALSE;
}

wiced_result_t wiced_spp_ota_firmware_upgrade_data_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t       len = 0;
    len = p_data[1] + (p_data[2] << 8);
    WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_DATA packet data_len:%d  payload len: %d\n", data_len, len );
    if( len == ( data_len - 3 ))
    {
        if ( ota_fw_upgrade_image_data_handler(handle, p_data + 3, len ) )
        {
            result = WICED_SUCCESS;
            wiced_spp_ota_firmware_upgrade_send_status( handle, WICED_OTA_UPGRADE_STATUS_CONTINUE );
        }
    }
    else
    {
        WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_DATA payload length does not match");
    }
    return result;
}

wiced_result_t wiced_spp_ota_firmware_upgrade_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint8_t type          = *p_data >> 4;

#if OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("OTA Handler Type:%x\n", type );
#endif
    if ( type == OTA_FW_UPGRADE_DATA )
    {
        result = wiced_spp_ota_firmware_upgrade_data_handler(handle, p_data,data_len );
    }
    else if ( type == OTA_FW_UPGRADE_CONTROL_COMMAND )
    {
        result = wiced_spp_ota_firmware_upgrade_command_handler( handle, p_data, data_len );
    }
    return result;
}

wiced_result_t  wiced_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status )
{
    return wiced_spp_ota_firmware_upgrade_send_status(handle, status );
}

wiced_bool_t wiced_ota_indication_enabled( void )
{
    return 0;
}
