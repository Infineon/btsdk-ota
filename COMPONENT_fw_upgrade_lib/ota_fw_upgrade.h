/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
/*
 * ota_fw_upgrade.h
 *
 */

#ifndef OTA_FW_UPGRADE_H_
#define OTA_FW_UPGRADE_H_

#include "p_256_ecc_pp.h"
#include <wiced_bt_ota_firmware_upgrade.h>
#include <wiced_timer.h>

#define OTA_UPGRADE_DEBUG 1

#define OTA_FW_UPGRADE_READ_CHUNK                   512
#define OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT         512
#ifdef OTA_ENCRYPT_SFLASH_DATA
//block sizes must match for AES-CTR
#define OTA_SEC_FW_UPGRADE_READ_CHUNK               OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT
#else
#define OTA_SEC_FW_UPGRADE_READ_CHUNK               1024
#endif

#define KEY_LENGTH_BITS             256
#define KEY_LENGTH_BYTES            (KEY_LENGTH_BITS / 8)
#define SIGNATURE_LEN               (KEY_LENGTH_BYTES * 2)
#define DS_IMAGE_PREFIX_LEN         16

typedef struct
{
// device states during OTA FW upgrade
#define OTA_STATE_IDLE                   0
#define OTA_STATE_READY_FOR_DOWNLOAD     1
#define OTA_STATE_DATA_TRANSFER          2
#define OTA_STATE_VERIFIED               3
#define OTA_STATE_APPLY                  4
#define OTA_STATE_ABORTED                5
    int32_t         state;
    uint8_t         bdaddr[6];               // BDADDR of connected device
    uint16_t        client_configuration;    // characteristic client configuration descriptor
    uint16_t        conn_id;                 // none zero connection id when connected
    uint8_t         status;                  // Current status
    uint16_t        current_offset;          // Offset in the image to store the data
    int32_t         total_len;               // Total length expected from the host
    int32_t         current_block_offset;
    int32_t         total_offset;
    uint32_t        crc32;
#if OTA_UPGRADE_DEBUG
    uint32_t        recv_crc32;
#endif
    uint8_t         indication_sent;
    wiced_timer_t   reset_timer;
    uint8_t         read_buffer[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];

    wiced_bool_t    transfer_only;          // Transfer only, no saving to flash
    wiced_ota_firmware_event_callback_t *p_event_callback;
} ota_fw_upgrade_state_t;

extern const uint8_t                                          ds_image_prefix[8];
extern ota_fw_upgrade_state_t                           ota_fw_upgrade_state;
extern wiced_ota_firmware_upgrade_status_callback_t     *ota_fw_upgrade_status_callback;
extern wiced_ota_firmware_upgrade_send_data_callback_t  *ota_fw_upgrade_send_data_callback;
extern Point                                            *p_ecdsa_public_key;

int32_t ota_fw_upgrade_verify(void);
int32_t ota_sec_fw_upgrade_verify(void);
int32_t ota_fw_upgrade_calculate_checksum( int32_t offset, int32_t length );
wiced_bool_t ota_fw_upgrade_image_data_handler(uint16_t conn_id, uint8_t *data, int32_t len);

#endif /* OTA_FW_UPGRADE_H_ */
