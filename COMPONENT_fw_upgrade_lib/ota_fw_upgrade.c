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
#include "wiced_bt_event.h"
#include "sha256.h"
#include "wiced_bt_l2c.h"

#include "wiced_platform.h"

#define OTA_UPGRADE_DEBUG 1

/******************************************************
 *                      Constants
 ******************************************************/
#define VERIFY_CRC      0
#define VERIFY_ECDSA    1


/******************************************************
 *               Variables Definitions
 ******************************************************/
uint16_t                                      ota_client_config_descriptor = 0;

extern uint32_t update_crc(uint32_t crc, uint8_t *buf, uint16_t len);

static void                   ota_fw_upgrade_set_client_configuration(uint16_t client_config);
wiced_bool_t           ota_fw_upgrade_handle_data(uint16_t conn_id, uint8_t *data, int32_t len);
wiced_bool_t           ota_fw_upgrade_handle_command(uint16_t conn_id, uint8_t *data, int32_t len);
static wiced_bt_gatt_status_t ota_fw_upgrade_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val);
static void                   ota_fw_upgrade_reset_timeout(uint32_t param);
extern UINT32 crc32_Update( UINT32 crc, UINT8 *buf, UINT16 len );
extern uint32_t update_crc32(uint32_t crc, uint8_t *buf, uint16_t len);
extern void allowPeripheralLatency(BOOL8 allow);

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
        if (p_state->state == OTA_STATE_VERIFIED)
        {
            if (ota_fw_upgrade_status_callback)
            {
                (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
            }

            if (ota_fw_upgrade_state.transfer_only && ota_fw_upgrade_state.p_event_callback)
            {
                uint8_t status = 0;
                (*ota_fw_upgrade_state.p_event_callback)(OTA_FW_UPGRADE_EVENT_COMPLETED, &status);
            }
            else
            {
                // disconnect and start timer to trigger hardware reset 1 second later
                wiced_bt_gatt_disconnect(conn_id);
                wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
                wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, 0, WICED_SECONDS_TIMER);
                wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
            }
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
 * This function is executed after we received valid Write Req to perform verification and send Write Rsp
 */
int perform_verification(void *data)
{
    uint16_t conn_id = ota_fw_upgrade_state.conn_id;
    int method = (int)data;
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;
    int32_t verified;

    if (p_state->transfer_only)
    {
        // For transfer-only verify is done elsewhere
        verified = WICED_TRUE;
    }
    else
    {
        verified = (method == VERIFY_CRC) ? ota_fw_upgrade_verify() : ota_sec_fw_upgrade_verify();
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
        if (p_state->transfer_only && p_state->p_event_callback)
        {
            uint8_t status = 1;
            (*p_state->p_event_callback)(OTA_FW_UPGRADE_EVENT_COMPLETED, &status);
        }
        return WICED_TRUE;
    }
    WICED_BT_TRACE("Verify success\n");
    p_state->state = OTA_STATE_VERIFIED;

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
#if !defined(OTA_FW_LIB_HCI_DFU)
    if (ota_client_config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        if (ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value) == WICED_BT_GATT_SUCCESS)
        {
#endif
            // notify application that we are going down
            if (ota_fw_upgrade_status_callback)
            {
                (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_COMPLETED);
            }
            if (p_state->transfer_only)
            {
                uint8_t status = 0;
                if (p_state->p_event_callback)
                    (*p_state->p_event_callback)(OTA_FW_UPGRADE_EVENT_COMPLETED, &status);
            }
            else
            {
                // Start timer to disconnect in a second (parameter passed to the function is conn_id)
                wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
                wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, conn_id, WICED_SECONDS_TIMER);
                wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
            }
            return WICED_TRUE;
#if !defined(OTA_FW_LIB_HCI_DFU)
        }
    }
#endif
    WICED_BT_TRACE("failed to notify the app\n");
    return WICED_TRUE;
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
#if (defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined (CYW20819A1))
#ifdef DISABLED_PERIPHERAL_LATENCY_ONLY
        allowPeripheralLatency(FALSE);
#else
        wiced_bt_l2cap_update_ble_conn_params(ota_fw_upgrade_state.bdaddr, 6, 6, 0, 200);
#endif
#elif ( defined(CYW20835B1) )
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
        if (ota_fw_upgrade_state.transfer_only && ota_fw_upgrade_state.p_event_callback)
        {
            (*ota_fw_upgrade_state.p_event_callback)(OTA_FW_UPGRADE_EVENT_STARTED, NULL);
        }
        return WICED_TRUE;
    }
    if (command == WICED_OTA_UPGRADE_COMMAND_ABORT)
    {
#if (defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2) || defined (CYW20819A1))
#ifdef DISABLED_PERIPHERAL_LATENCY_ONLY
        allowPeripheralLatency(TRUE);
#endif
#endif
        p_state->state = OTA_STATE_ABORTED;
        ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);

        if (ota_fw_upgrade_status_callback)
        {
            (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_ABORTED);
        }
        if (ota_fw_upgrade_state.transfer_only && ota_fw_upgrade_state.p_event_callback)
        {
            uint8_t status = 1;
            (*ota_fw_upgrade_state.p_event_callback)(OTA_FW_UPGRADE_EVENT_COMPLETED, &status);
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
            if (len < 5)
            {
                WICED_BT_TRACE("Bad Download len: %d \n", len);
                return WICED_FALSE;
            }

            if (!ota_fw_upgrade_state.transfer_only)
            {
                WICED_BT_TRACE("calling wiced_firmware_upgrade_init_nv_locations\n");
                if (!wiced_firmware_upgrade_init_nv_locations())
                {
                    WICED_BT_TRACE("failed init nv locations\n");
                    value = WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
                    ota_fw_upgrade_send_notification(conn_id, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &value);
                    return WICED_FALSE;
                }
                WICED_BT_TRACE("done calling wiced_firmware_upgrade_init_nv_locations\n");
            }

            p_state->state                = OTA_STATE_DATA_TRANSFER;
            p_state->current_offset       = 0;
            p_state->current_block_offset = 0;
            p_state->total_offset         = 0;
#if OTA_UPGRADE_DEBUG
            p_state->recv_crc32           = 0xffffffff;
#endif

            // if we are using Secure version the total length comes in the beginning of the image,
            // do not use the one from the downloader.
#ifndef CYW20706A2
            if (p_ecdsa_public_key != NULL)
            {
                p_state->total_len            = 0;
            }
            else
#endif
            {
                p_state->total_len            = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
            }
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
                if (ota_fw_upgrade_state.transfer_only && ota_fw_upgrade_state.p_event_callback)
                {
                    uint8_t status = 1;
                    (*ota_fw_upgrade_state.p_event_callback)(OTA_FW_UPGRADE_EVENT_COMPLETED, &status);
                }
                return WICED_TRUE;
            }

            // Verification is a lengthy process.  We should serialize performing the verification, i.e.
            // after returning from this function, the stack will be able to send GATT Write Rsp to the client.
            // Result of the verification will be sent as a notification or indication
            if (p_ecdsa_public_key == NULL)
            {
                // For none-secure case the command should have 4 bytes CRC32
                p_state->crc32 = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
                wiced_app_event_serialize(perform_verification, (void *)VERIFY_CRC);
            }
            else
            {
                WICED_BT_TRACE("ota_sec_fw_upgrade_verify() \n");
                wiced_app_event_serialize(perform_verification, (void *)VERIFY_ECDSA);
            }

            return WICED_TRUE;
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
    wiced_bt_application_id_t app_id_and_ver;
    wiced_bt_application_id_t *image_id_and_ver;
#endif

    if (p_state->state != OTA_STATE_DATA_TRANSFER)
        return FALSE;
    WICED_BT_TRACE("ota_fw_upgrade_handle_data\n");

// Image prefixes are supported on 20719xx and 20835
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

            // validate application id and version
            image_id_and_ver = (wiced_bt_application_id_t *)&data[8];
            WICED_BT_TRACE("Image product ID 0x%x and version %d.%d\n", image_id_and_ver->product_id,
                                                image_id_and_ver->major, image_id_and_ver->minor);
            if(!wiced_get_current_app_id_and_version(&app_id_and_ver))
            {
                WICED_BT_TRACE("Failed to read current app version and id\n");
                return (FALSE);
            }
            if( (image_id_and_ver->product_id != app_id_and_ver.product_id) &&
                (app_id_and_ver.product_id != 0))
            {
                WICED_BT_TRACE("Image product ID (%x) does not match current (%x)\n",
                            image_id_and_ver->product_id, app_id_and_ver.product_id);
                return (FALSE);
            }
            if(image_id_and_ver->major < app_id_and_ver.major)
            {
                WICED_BT_TRACE("Image major version (%d) is behind current (%d)\n",
                            image_id_and_ver->major, app_id_and_ver.major);
                return (FALSE);
            }

            // length store in the image does not include size of ds_image_prefix
            p_state->total_len = data[12] + (data[13] << 8) + (data[14] << 16) + (data[15] << 24);
            p_state->total_len += DS_IMAGE_PREFIX_LEN + SIGNATURE_LEN;

            // ToDo validate flash size
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
#if (defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20835B1) || defined(CYW20819A1))
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
            if (ota_fw_upgrade_state.transfer_only && ota_fw_upgrade_state.p_event_callback)
            {
                wiced_bt_ota_fw_upgrad_event_data_t event_data;

                event_data.offset = p_state->total_offset;
                event_data.data_len = p_state->current_block_offset;
                event_data.p_data = p_state->read_buffer;
                (*ota_fw_upgrade_state.p_event_callback)(OTA_FW_UPGRADE_EVENT_DATA, &event_data);
            }
            else
            {
                // write should be on the word boundary and in full words, we may write a bit more
                written = wiced_firmware_upgrade_store_to_nv(p_state->total_offset, p_state->read_buffer, (p_state->current_block_offset + 3) & 0xFFFFFFFC);
                if(written != ((p_state->current_block_offset + 3) & 0xFFFFFFFC))
                {
                    WICED_BT_TRACE("write failed, returned %x\n", written);
                }
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
    // if conn_id is not zero, connection is still up, disconnect and start 1 second timer before reset
    if (ota_fw_upgrade_state.conn_id)
    {
        wiced_bt_gatt_disconnect(ota_fw_upgrade_state.conn_id);
        wiced_deinit_timer(&ota_fw_upgrade_state.reset_timer);
        wiced_init_timer(&ota_fw_upgrade_state.reset_timer, ota_fw_upgrade_reset_timeout, 0, WICED_SECONDS_TIMER);
        wiced_start_timer(&ota_fw_upgrade_state.reset_timer, 1);
    }
    else
       wiced_firmware_upgrade_finish();
}

wiced_bool_t wiced_ota_fw_upgrade_set_transfer_mode(wiced_bool_t transfer_only, wiced_ota_firmware_event_callback_t *p_event_callback)
{
    ota_fw_upgrade_state.transfer_only = transfer_only;
    ota_fw_upgrade_state.p_event_callback = p_event_callback;

    return WICED_TRUE;
}
