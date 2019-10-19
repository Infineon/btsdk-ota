/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 * WICED Bluetooth OTA Upgrade
 *
 * This file provides function required to support Over the Air WICED Upgrade.
 * Both secure and none secure services are supported.  In the none-secure
 * case the software is only protected by the checksum.  In the secure case
 * the image is protected with a digital signature which is verified using
 * the private key passed by the application.
 *
 * To download host sends command to download with length of the patch to be
 * transmitted.  GATT Write Requests are used to send commands and portions of
 * data.  In case of an error Error Response indicates failure to the host.
 * Host sends fixed chunks of data.  After all the bytes has been downloaded
 * and acknowledged host sends verify command that includes CRC32 of the
 * whole patch.  During the download device saves data directly to the
 * serial flash.  At the verification stage device reads data back from the
 * NVRAM and calculates checksum of the data stored there.  Result of the
 * verification is indicated in the Write Response or Error Response GATT message.
 *
 */
#include "bt_types.h"
#include "ota_fw_upgrade.h"
#include "wiced_bt_gatt.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_bt_trace.h"
#include "sha256.h"
#include "wiced_bt_l2c.h"

#include "wiced_platform.h"

#define OTA_UPGRADE_DEBUG 1

/******************************************************
 *                      Constants
 ******************************************************/


/******************************************************
 *               Variables Definitions
 ******************************************************/
uint16_t                                      ota_client_config_descriptor = 0;

extern uint32_t update_crc(uint32_t crc, uint8_t *buf, uint16_t len);

static void                   ota_fw_upgrade_set_client_configuration(uint16_t client_config);
static wiced_bool_t           ota_fw_upgrade_handle_data(uint16_t conn_id, uint8_t *data, int32_t len);
static wiced_bool_t           ota_fw_upgrade_handle_command(uint16_t conn_id, uint8_t *data, int32_t len);
static wiced_bt_gatt_status_t ota_fw_upgrade_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val);
static void                   ota_fw_upgrade_reset_timeout(uint32_t param);
extern UINT32 crc32_Update( UINT32 crc, UINT8 *buf, UINT16 len );
extern uint32_t update_crc32(uint32_t crc, uint8_t *buf, uint16_t len);
static uint16_t ota_conn_id = 0;
extern void allowSlaveLatency(BOOL8 allow);

/*
 * Process GATT Read request
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    switch (p_read_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        if (*p_read_data->p_val_len < 2)
            return WICED_BT_GATT_INVALID_ATTR_LEN;

        if (p_read_data->offset == 1)
        {
            p_read_data->p_val[0] = ota_client_config_descriptor >> 8;
            *p_read_data->p_val_len = 1;
        }
        else
        {
            p_read_data->p_val[0] = ota_client_config_descriptor & 0xff;
            p_read_data->p_val[1] = ota_client_config_descriptor >> 8;
            *p_read_data->p_val_len = 2;
        }
        return WICED_BT_GATT_SUCCESS;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process GATT Write request
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    switch (p_write_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        if (!ota_fw_upgrade_handle_command(conn_id, p_write_data->p_val, p_write_data->val_len))
        {
            WICED_BT_TRACE("ota_handle_command failed.\n");
            return WICED_BT_GATT_ERROR;
        }
        break;

    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_write_data->val_len != 2)
        {
            WICED_BT_TRACE("ota client config wrong len %d\n", p_write_data->val_len);
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        ota_fw_upgrade_set_client_configuration(p_write_data->p_val[0] + (p_write_data->p_val[1] << 8));
        break;

    case HANDLE_OTA_FW_UPGRADE_DATA:
        if (!ota_fw_upgrade_handle_data(conn_id, p_write_data->p_val, p_write_data->val_len))
        {
            WICED_BT_TRACE("ota_handle_data failed.\n");
            return WICED_BT_GATT_INTERNAL_ERROR;
        }
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT indication confirmation
 */
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;

    WICED_BT_TRACE("wiced_ota_fw_upgrade_indication_cfm_handler state:%d \n", p_state->state);
    WICED_BT_TRACE("wiced_ota_fw_upgrade_indication_cfm_handler, conn %d hdl %d\n", conn_id, handle);

    if (handle == HANDLE_OTA_FW_UPGRADE_CONTROL_POINT)
    {
#ifdef WICED_OTA_VERSION_2
        if (p_state->state == OTA_STATE_APPLY)
#else
        if (p_state->state == OTA_STATE_VERIFIED)
#endif
        {
            if (ota_fw_upgrade_status_callback)
            {
                (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
            }

            // disconnect and start timer to trigger hardware reset 1 second later
            wiced_bt_gatt_disconnect(conn_id);
            wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
            wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, 0, WICED_SECONDS_TIMER);
            wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
        }
        return WICED_BT_GATT_SUCCESS;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}

wiced_bool_t wiced_ota_fw_upgrade_is_gatt_handle(uint16_t handle)
{
    return HANDLE_OTA_FW_UPGRADE_SERVICE <= handle && handle <= HANDLE_OTA_FW_UPGRADE_APP_INFO;
}

/*
 * handle commands received over the control point
 */
wiced_bool_t ota_fw_upgrade_handle_command(uint16_t conn_id, uint8_t *data, int32_t len)
{
    uint8_t command = data[0];
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;
    int32_t verified = WICED_FALSE;

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("OTA handle cmd:%d state:%d\n", command, p_state->state);
#endif
    if (command == WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD)
    {
        if (len == 11)
        {
            p_state->fw_cid = (data[1] << 8) + data[2];
            memcpy(p_state->fw_id, &data[3], OTA_FWID_LENGTH);
        }
#if (defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined (CYW20819A1))
#ifdef DISABLED_SLAVE_LATENCY_ONLY
        allowSlaveLatency(FALSE);
#else
        wiced_bt_l2cap_update_ble_conn_params(ota_fw_upgrade_state.bdaddr, 6, 6, 0, 200);
#endif
#elif ( defined(CYW20735B0) || defined(CYW20735B1) )
#ifndef OTA_SKIP_CONN_PARAM_UPDATE
        wiced_bt_l2cap_update_ble_conn_params(ota_fw_upgrade_state.bdaddr, 6, 6, 0, 200);
#endif
#else
        wiced_bt_l2cap_update_ble_conn_params(ota_fw_upgrade_state.bdaddr, 6, 6, 0, 200);
#endif
        p_state->state = OTA_STATE_READY_FOR_DOWNLOAD;
        ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);

        if (ota_fw_upgrade_status_callback)
        {
            (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_STARTED);
        }
        return WICED_TRUE;
    }
    if (command == WICED_OTA_UPGRADE_COMMAND_ABORT)
    {
#if (defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined (CYW20819A1))
#ifdef DISABLED_SLAVE_LATENCY_ONLY
        allowSlaveLatency(TRUE);
#endif
#endif
        p_state->state = OTA_STATE_ABORTED;
        ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);

        if (ota_fw_upgrade_status_callback)
        {
            (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
        }
        return WICED_FALSE;
    }
#ifdef WICED_OTA_VERSION_2
    if (command == WICED_OTA_UPGRADE_COMMAND_APPLY && p_state->fw_verified)
    {
        p_state->state = OTA_STATE_APPLY;

        // if we are able to send indication (good host) wait for the confirmation before the reboot
        if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
        {
            if (ota_fw_upgrade_send_data_callback != NULL)
            {
                if (ota_fw_upgrade_send_data_callback(WICED_FALSE, conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                {
                    return WICED_TRUE;
                }
            }
            else
            {
                if (wiced_bt_gatt_send_indication(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                {
                    return WICED_TRUE;
                }
            }
        }
        // if we are unable to send indication, try to send notification and start 1 sec timer
        if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            if (ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
            {
                // notify application that we are going down
                if (ota_fw_upgrade_status_callback)
                {
                    (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
                }
                // Start timer to disconnect in a second (parameter passed to the function is conn_id)
                ota_conn_id = conn_id;
                wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
                wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, conn_id, WICED_SECONDS_TIMER);
                wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
                return WICED_TRUE;
            }
        }
        WICED_BT_TRACE("failed to notify the app\n");
        return WICED_FALSE;
    }
#endif

    switch (p_state->state)
    {
    case OTA_STATE_IDLE:
        return WICED_TRUE;

    case OTA_STATE_READY_FOR_DOWNLOAD:
        if (command == WICED_OTA_UPGRADE_COMMAND_DOWNLOAD)
        {
            // command to start upgrade should be accompanied by 4 bytes with the image size
            if (len < 5)
            {
                WICED_BT_TRACE("Bad Download len: %d \n", len);
                return WICED_FALSE;
            }

            WICED_BT_TRACE("calling wiced_firmware_upgrade_init_nv_locations\n");
            if (!wiced_firmware_upgrade_init_nv_locations())
            {
                WICED_BT_TRACE("failed init nv locations\n");
                value = WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
                ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
                return WICED_FALSE;
            }
            WICED_BT_TRACE("done calling wiced_firmware_upgrade_init_nv_locations\n");

            p_state->state                = OTA_STATE_DATA_TRANSFER;
            p_state->current_offset       = 0;
            p_state->current_block_offset = 0;
            p_state->total_offset         = 0;
            p_state->total_len            = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
            p_state->fw_verified          = WICED_FALSE;
#if OTA_UPGRADE_DEBUG
            p_state->recv_crc32           = 0xffffffff;
#endif

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20735B0) || defined(CYW20735B1)/* || defined (CYW20819A1) */)
            // if we are using Secure version the total length comes in the beginning of the image,
            // do not use the one from the downloader.
            if (p_ecdsa_public_key != NULL)
            {
                p_state->total_len            = 0;
            }
            else
            {
                p_state->total_len            = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
            }
#endif
            WICED_BT_TRACE("state %d total_len %d \n", p_state->state, data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24));
            ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
            return WICED_TRUE;
        }
        break;

    case OTA_STATE_DATA_TRANSFER:
        if (command == WICED_OTA_UPGRADE_COMMAND_VERIFY)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 5)
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
                ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);

                if (ota_fw_upgrade_status_callback)
                {
                    (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_TRUE;
            }

            // For none-secure case the command should have 4 bytes CRC32
            if (p_ecdsa_public_key == NULL)
            {
                p_state->crc32 = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
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
                ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);

                if (ota_fw_upgrade_status_callback)
                {
                    (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
                }
                return WICED_TRUE;
            }
            WICED_BT_TRACE("Verify success\n");
            p_state->state = OTA_STATE_VERIFIED;
#ifdef WICED_OTA_VERSION_2
            p_state->fw_verified = WICED_TRUE;
            ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
            return WICED_TRUE;
#else
            // if we are able to send indication (good host) wait for the confirmation before the reboot
            if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
            {
                if (ota_fw_upgrade_send_data_callback != NULL)
                {
                    if (ota_fw_upgrade_send_data_callback(WICED_FALSE, conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                    {
                        return WICED_TRUE;
                    }
                }
                else
                {
                    if (wiced_bt_gatt_send_indication(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                    {
                        return WICED_TRUE;
                    }
                }
            }
            // if we are unable to send indication, try to send notification and start 1 sec timer
            if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
            {
                if (ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
                {
                    // notify application that we are going down
                    if (ota_fw_upgrade_status_callback)
                    {
                        (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
                    }
                    // Start timer to disconnect in a second (parameter passed to the function is conn_id)
                    ota_conn_id = conn_id;
                    wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
                    wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, conn_id, WICED_SECONDS_TIMER);
                    wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
                    return WICED_TRUE;
                }
            }
            WICED_BT_TRACE("failed to notify the app\n");
            return WICED_FALSE;
#endif
        }
        break;

    case OTA_STATE_ABORTED:
    default:
        break;
    }

    value = WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
    WICED_BT_TRACE("calling ota_fw_upgrade_send_notification before exit\n");
    ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
    WICED_BT_TRACE("exit ota_fw_upgrade_handle_command\n");
    return WICED_FALSE;
}

/*
 * Process data chunk received from the host.  If received num of bytes equals to
 * OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT, save the data to NV.
 *
 */
wiced_bool_t ota_fw_upgrade_handle_data(uint16_t conn_id, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t *p = data;
#ifndef CYW20706A2
#ifdef WICED_BT_TRACE_ENABLE
    uint16_t image_product_id;
    uint8_t  image_major, image_minor;
#endif
#endif

    if (p_state->state != OTA_STATE_DATA_TRANSFER)
        return FALSE;
    WICED_BT_TRACE("ota_fw_upgrade_handle_data\n");

// Image prefixes are supported on 20719xx and 20735
#ifndef CYW20706A2
    // For the Secure upgrade, verify the Product info
    if (p_ecdsa_public_key != NULL)
    {
        // If this is the first chunk of the image, we need to verify the header and extract the length
        // Following check is for the FW2
        if (p_state->total_len == 0)
        {
            if (memcmp(data, ds_image_prefix, sizeof(ds_image_prefix)) != 0)
            {
                WICED_BT_TRACE("Bad data start\n");
                return (FALSE);
            }
#ifdef WICED_BT_TRACE_ENABLE
            image_product_id = data[8] + (data[9] << 8);
            image_major = data[10];
            image_minor = data[11];
#endif

            // length store in the image does not include size of ds_image_prefix
            p_state->total_len = data[12] + (data[13] << 8) + (data[14] << 16) + (data[15] << 24);
            p_state->total_len += DS_IMAGE_PREFIX_LEN + SIGNATURE_LEN;

            // ToDo validate flash size
            // ToDo validate that product is the same as stored and major is >= the one that stored
#ifdef WICED_BT_TRACE_ENABLE
            WICED_BT_TRACE("Image for Product 0x%x %d.%d len:%d\n", image_product_id, image_major, image_minor, p_state->total_len);
#endif
        }
    }
    else
#endif
    {
#if OTA_UPGRADE_DEBUG
        // For testing calculate received CRC32 of the received data
#ifdef CYW20706A2
        p_state->recv_crc32 = update_crc(p_state->recv_crc32, data, len);
#else
#if (defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20735B1) || defined(CYW20819A1))
        p_state->recv_crc32 = crc32_Update(p_state->recv_crc32, data, len);
#else
        p_state->recv_crc32 = update_crc32(p_state->recv_crc32, data, len);
#endif
#endif
#endif
    }

    while (len)
    {
        int bytes_to_copy =
            (p_state->current_block_offset + len) < OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT ? len: (OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT - p_state->current_block_offset);

        if ((p_state->total_offset + p_state->current_block_offset + bytes_to_copy > p_state->total_len))
        {
            WICED_BT_TRACE("Too much data. size of the image %d offset %d, block offset %d len rcvd %d\n",
                    p_state->total_len, p_state->total_offset, p_state->current_block_offset, len);
            return (FALSE);
        }

        memcpy (&(p_state->read_buffer[p_state->current_block_offset]), p, bytes_to_copy);
        p_state->current_block_offset += bytes_to_copy;

        if ((p_state->current_block_offset == OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT) ||
            (p_state->total_offset + p_state->current_block_offset == p_state->total_len))
        {
            uint32_t written = 0;
#if OTA_UPGRADE_DEBUG
            // WICED_BT_TRACE("write offset:%x\n", p_state->total_offset);
            //dump_hex(p_state->read_buffer, p_state->current_block_offset);
#endif
            // write should be on the word boundary and in full words, we may write a bit more
            written = wiced_firmware_upgrade_store_to_nv(p_state->total_offset, p_state->read_buffer, (p_state->current_block_offset + 3) & 0xFFFFFFFC);
            if(written != ((p_state->current_block_offset + 3) & 0xFFFFFFFC))
            {
                WICED_BT_TRACE("write failed, returned %x\n", written);
            }
            p_state->total_offset        += p_state->current_block_offset;
            p_state->current_block_offset = 0;

#if OTA_UPGRADE_DEBUG
            if (p_state->total_offset == p_state->total_len)
            {
                p_state->recv_crc32 = p_state->recv_crc32 ^ 0xffffffff;
            }
#endif
        }

        len = len - bytes_to_copy;
        p = p + bytes_to_copy;

        //WICED_BT_TRACE("remaining len: %d \n", len);
    }
    return (TRUE);
}

/*
 * Set new value for client configuration descriptor
 */
void ota_fw_upgrade_set_client_configuration(uint16_t client_config)
{
    ota_client_config_descriptor = client_config;
}

/*
 * Send Notification if allowed, or indication if allowed, or return error
 */
wiced_bt_gatt_status_t ota_fw_upgrade_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val)
{
    if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        if (ota_fw_upgrade_send_data_callback != NULL)
            return ota_fw_upgrade_send_data_callback(WICED_TRUE, conn_id, attr_handle, val_len, p_val);
        else
            return wiced_bt_gatt_send_notification(conn_id, attr_handle, val_len, p_val);
    }
    else if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
    {
        if (ota_fw_upgrade_send_data_callback != NULL)
            return ota_fw_upgrade_send_data_callback(WICED_FALSE, conn_id, attr_handle, val_len, p_val);
        else
            return wiced_bt_gatt_send_indication(conn_id, attr_handle, val_len, p_val);
    }
    return WICED_BT_GATT_ERROR;
}

/*
 * Process timeout started after the last notification to perform restart
 */
void ota_fw_upgrade_reset_timeout(uint32_t param)
{
    uint16_t conn_id = ota_conn_id; // TODO : Should use param instead of  ota_conn_id

    // if conn_id is not zero, connection is still up, disconnect and start 1 second timer before reset
    if (conn_id)
    {
       wiced_bt_gatt_disconnect(conn_id);
       ota_conn_id = 0;
       wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
       wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, 0, WICED_SECONDS_TIMER);
       wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
    }
    else
       wiced_firmware_upgrade_finish();
}

wiced_bool_t wiced_ota_fw_upgrade_get_new_fw_info(uint16_t *company_id, uint8_t *fw_id_len, uint8_t *fw_id)
{
    if (ota_fw_upgrade_state.state != OTA_STATE_VERIFIED)
        return WICED_FALSE;

    if (*fw_id_len < OTA_FWID_LENGTH)
        return WICED_FALSE;

    *company_id = ota_fw_upgrade_state.fw_cid;
    *fw_id_len = OTA_FWID_LENGTH;
    memcpy(fw_id, ota_fw_upgrade_state.fw_id, OTA_FWID_LENGTH);
    return WICED_TRUE;
}
