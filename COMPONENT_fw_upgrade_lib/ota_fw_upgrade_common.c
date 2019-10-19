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
 * WICED Bluetooth OTA Upgrade common functionality.
 *
 * This file provides functions required for image verification
 * once the download is completed.
 *
 */
#include "bt_types.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_timer.h"
#include "ota_fw_upgrade.h"
#include "sha256.h"
#include "p_256_multprecision.h"
#include "wiced_hal_eflash.h"
#ifdef ENABLE_SFLASH_UPGRADE
#include "wiced_hal_sflash.h"
#endif
#include <wiced_platform.h>

// make distinction between download storage (could be external flash)
// and DS storage (could be on-chip flash)
#if defined(ENABLE_SFLASH_UPGRADE)
#if defined(USE_256K_SECTOR_SIZE)
#define DOWNLOAD_SECTOR_SIZE (256*1024)
#else
#define DOWNLOAD_SECTOR_SIZE (4*1024)
#endif
#else
#define DOWNLOAD_SECTOR_SIZE FLASH_SECTOR_SIZE
#endif

#define DS_SECTOR_SIZE FLASH_SECTOR_SIZE

// flash layout is passed in defines from makefile
#if !defined(SS_LOCATION)
  #define SS_LOCATION (flash_base_address)
#endif
#if !defined(VS_LOCATION)
  #define VS_LOCATION (SS_LOCATION + (DOWNLOAD_SECTOR_SIZE * 2))
#endif
#if !defined(VS_LENGTH)
  #if ( defined(CYW20819A1) && !defined(ENABLE_SFLASH_UPGRADE) )
    #define VS_LENGTH (DOWNLOAD_SECTOR_SIZE * 8)
  #else
    #define VS_LENGTH DOWNLOAD_SECTOR_SIZE
  #endif
#endif
#if !defined(VS2_LOCATION)
  #if ( defined(CYW20819A1) && !defined(ENABLE_SFLASH_UPGRADE) )
    #define VS2_LOCATION VS_LOCATION
  #else
    #define VS2_LOCATION (VS_LOCATION + VS_LENGTH)
  #endif
#endif
#if !defined(VS2_LENGTH)
  #define VS2_LENGTH VS_LENGTH
#endif
#if !defined(DS_LOCATION)
  #define DS_LOCATION (VS2_LOCATION + VS2_LENGTH)
#endif
#if !defined(DS2_LOCATION)
  #define DS2_LOCATION (DS_LOCATION + ((fw_upgrade_flash_size - nv_loc_len.ds1_loc + flash_base_address)/2))
#endif


static void ota_fw_upgrade_init_data(wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback);
extern uint32_t update_crc32( uint32_t crc, uint8_t *buf, uint16_t len );
extern uint32_t update_crc(uint32_t crc, uint8_t *buf, uint16_t len);
extern UINT32 crc32_Update( UINT32 crc, UINT8 *buf, UINT16 len );

#ifdef OTA_UPGRADE_DEBUG
void dump_hex(uint8_t *p, uint32_t len);
#endif

const uint8_t ds_image_prefix[8] = { 'B', 'R', 'C', 'M', 'c', 'f', 'g', 'D' };
static wiced_bool_t ota_fw_upgrade_initialized = WICED_FALSE;
ota_fw_upgrade_state_t ota_fw_upgrade_state;
wiced_ota_firmware_upgrade_status_callback_t *ota_fw_upgrade_status_callback = NULL;
wiced_ota_firmware_upgrade_send_data_callback_t *ota_fw_upgrade_send_data_callback = NULL;
Point *p_ecdsa_public_key = NULL;

/*
 * Initializes global data
 */
void ota_fw_upgrade_init_data(wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback)
{
    ota_fw_upgrade_status_callback       = p_status_callback;
    ota_fw_upgrade_send_data_callback    = p_send_data_callback;

    ota_fw_upgrade_initialized           = WICED_TRUE;
    ota_fw_upgrade_state.state           = OTA_STATE_IDLE;
    ota_fw_upgrade_state.indication_sent = WICED_FALSE;
    ota_fw_upgrade_state.fw_verified     = WICED_FALSE;

#if defined(OTA_FW_UPGRADE_SFLASH_COPY) || defined(OTA_FW_UPGRADE_EFLASH_COPY)
    {
        // this call is added to force the inclusion of code from fw_update_copy_sflash.c
        // as the linker can't be otherwise persuaded
        void init_wiced_firmware_update_copy_sflash(void);
        init_wiced_firmware_update_copy_sflash();
    }
#endif
}

/*
 * Return TRUE if status of the OTA Firmware Upgrade process is not IDLE or ABORTED
 */
wiced_bool_t wiced_ota_fw_upgrade_is_active(void)
{
    return (ota_fw_upgrade_initialized &&
           (ota_fw_upgrade_state.state != OTA_STATE_IDLE) &&
           (ota_fw_upgrade_state.state != OTA_STATE_ABORTED));
}

/*
 * Connection with peer device is established
 */
void wiced_ota_fw_upgrade_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
    // State is now idle
    ota_fw_upgrade_state.state = OTA_STATE_IDLE;
    if (p_status->connected)
    {
        memcpy(ota_fw_upgrade_state.bdaddr, p_status->bd_addr, BD_ADDR_LEN);
    }
}

/*
 * This function called by the application during OTA procedure init.
 * It initializes active and upgradable firmware FLASH address locations.
 * Also allows the application to provide public keys to perform secure OTA
 * and registers the application status callback, data callback to provide
 * FW upgrade status, to send any GATT notification/indiaction data to the peer
 * during FW upgrade.
 */
wiced_bool_t wiced_ota_fw_upgrade_init(void *p_public_key, wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback)
{
    wiced_fw_upgrade_nv_loc_len_t   nv_loc_len;
    //extern DWORD g_config_Info[];
    extern Point                    *p_ecdsa_public_key;
    uint32_t                        fw_upgrade_flash_size = FLASH_SIZE;
#if !(defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2))
    uint32_t                        flash_base_address = 0x00000000;
#if defined(FLASH_BASE_ADDRESS)
    flash_base_address = FLASH_BASE_ADDRESS;
#endif
#endif

    p_ecdsa_public_key = (Point *)p_public_key;

#ifndef APPLICATION_SPECIFIC_FLASH_RESERVATION
#define APPLICATION_SPECIFIC_FLASH_RESERVATION 0
#endif

    fw_upgrade_flash_size = (FLASH_SIZE - (APPLICATION_SPECIFIC_FLASH_RESERVATION * DS_SECTOR_SIZE));

    /* 20721 and 20719 uses SS to determine the location and length. */
#if !(defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2))
    /* In flash first sectors reserved for static section and for fail safe OTA block.
    Caution!!: Application should not modify these sections. */
    /* Remaining portion can be used for Vendor specific data followed by firmware
    It is applications choice to choose size for Vendor specific data and firmware.
    Apart from vendor specific portion, remaining area equally divided for active firmware and upgradable firmware.
    Below are the example offsets for 4M bit Serial flash
    Note: below configuration should not conflict with ConfigDSLocation, DLConfigVSLocation and DLConfigVSLength configured in
    platform sflash .btp file */

    /* Flash offsets for firmware upgrade. SS location always starts from
    zero offset of default base address. Default flash configuration comes from
    corresponding platform header file(i.e. either from wiced_platform.h */
    nv_loc_len.ss_loc  = SS_LOCATION;
    nv_loc_len.vs1_loc = VS_LOCATION;
    nv_loc_len.vs1_len = VS_LENGTH;
    nv_loc_len.vs2_loc = VS2_LOCATION;
    nv_loc_len.vs2_len = VS2_LENGTH;
    nv_loc_len.ds1_loc = DS_LOCATION;
    nv_loc_len.ds2_loc = DS2_LOCATION;
    nv_loc_len.ds1_len = DS2_LOCATION - DS_LOCATION;
    nv_loc_len.ds2_len = fw_upgrade_flash_size + flash_base_address - DS2_LOCATION;

    WICED_BT_TRACE("ds1:0x%08x, len:0x%08x\n", nv_loc_len.ds1_loc, nv_loc_len.ds1_len);
    WICED_BT_TRACE("ds2:0x%08x, len:0x%08x\n", nv_loc_len.ds2_loc, nv_loc_len.ds2_len);
    //WICED_BT_TRACE("Active partition:0x%08x\n", g_config_Info[4]);
#endif

    if (!wiced_firmware_upgrade_init(&nv_loc_len, fw_upgrade_flash_size))
    {
        WICED_BT_TRACE("WARNING: Upgrade will fail - active DS is not one of the expected locations\n");
        WICED_BT_TRACE("WARNING: Please build with OTA_FW_UPGRADE=1 and download \n");
        return WICED_FALSE;
    }
    ota_fw_upgrade_init_data(p_status_callback, p_send_data_callback);
    return WICED_TRUE;
}

/*
 * verify function is called after all the data has been received and stored
 * in the NV.  The function reads back data from the NV and calculates the checksum.
 * Function returns TRUE if calculated CRC matches the one calculated by the host
 */
int32_t ota_fw_upgrade_verify(void)
{
    uint32_t offset;
    uint32_t crc32 = 0xffffffff;

    if (ota_fw_upgrade_status_callback)
    {
        (*ota_fw_upgrade_status_callback)(OTA_FW_UPGRADE_STATUS_VERIFICATION_START);
    }

    for (offset = 0; offset < ota_fw_upgrade_state.total_len; offset += OTA_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read = ((offset + OTA_FW_UPGRADE_READ_CHUNK) < ota_fw_upgrade_state.total_len) ?
                                        OTA_FW_UPGRADE_READ_CHUNK : ota_fw_upgrade_state.total_len - offset;

        // read should be on the word boundary and in full words, we may read a bit more, but
        // include correct number of bytes in the CRC calculation
        wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC);
#ifdef CYW20706A2
        crc32 = update_crc(crc32, memory_chunk, bytes_to_read);
#else
#if (defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20735B1) || defined(CYW20819A1))
        crc32 = crc32_Update(crc32, memory_chunk, bytes_to_read);
#else
        crc32 = update_crc32(crc32, memory_chunk, bytes_to_read);
#endif
#endif

#ifdef OTA_UPGRADE_DEBUG
        //WICED_BT_TRACE("read offset:%x\n", offset);
        //dump_hex(memory_chunk, bytes_to_read);
#endif
    }
    crc32 = crc32 ^ 0xffffffff;

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("stored crc:%4x received bytes crc:%4x recvd crc:%4x\n", crc32, ota_fw_upgrade_state.recv_crc32, ota_fw_upgrade_state.crc32);
#endif
    return (crc32 == ota_fw_upgrade_state.crc32);
}

/*
 * verify function is called after all the data has been received and stored
 * in the NV.  The function reads back data from the NV and calculates the checksum.
 * Function returns TRUE if calculated CRC matches the one calculated by the host
 */
int32_t ota_sec_fw_upgrade_verify(void)
{
    mbedtls_sha256_context sha2_ctx;
    uint32_t     offset;
    uint32_t     nvram_len = ota_fw_upgrade_state.total_len - SIGNATURE_LEN;
    uint8_t      hash[32];
    uint8_t      signature[SIGNATURE_LEN + 4];
    uint8_t      res;

    mbedtls_sha256_init(&sha2_ctx);
    // initialize sha256 context
    mbedtls_sha256_starts_ret(&sha2_ctx, 0);

    for (offset = 0; offset < nvram_len; offset += OTA_SEC_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_SEC_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read = ((offset + OTA_SEC_FW_UPGRADE_READ_CHUNK) < nvram_len) ? OTA_SEC_FW_UPGRADE_READ_CHUNK : nvram_len - offset;

        // read should be on in full words, we may read a bit more, but include correct number of bytes in the hash calculation
        if (wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC) != ((bytes_to_read + 3) & 0xFFFFFFFC))
        {
            WICED_BT_TRACE("failed to read loc0:%x\n", offset);
        }

        // dump_hex(memory_chunk, bytes_to_read);
        mbedtls_sha256_update_ret(&sha2_ctx, memory_chunk, bytes_to_read);
    }
    mbedtls_sha256_finish_ret(&sha2_ctx, hash);

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("hash:\n");
    dump_hex(hash, sizeof(hash));

    WICED_BT_TRACE("public_key:%x\n", (uint8_t *)p_ecdsa_public_key);
    dump_hex((uint8_t *)p_ecdsa_public_key, sizeof(Point));
#endif
    // read should be on the word boundary and in full words. Need to adjust offset to full words and read a bit more.
    offset = ota_fw_upgrade_state.total_len - SIGNATURE_LEN;
    if (wiced_firmware_upgrade_retrieve_from_nv(offset -  (offset & 0x03), signature, SIGNATURE_LEN + 4) != SIGNATURE_LEN + 4)
    {
        WICED_BT_TRACE("failed to read loc1:%x\n", offset -  (offset & 0x03));
    }

#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("signature:\n");
    dump_hex(signature + (offset & 0x03), SIGNATURE_LEN);
#endif

    res = ecdsa_verify_(hash, signature + (offset & 0x03), p_ecdsa_public_key);
#ifdef OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("ecdsa_verify_:%d", res);
#endif
    return res;
}

/*
 * This function can be used if the server/client wants to calculate the checksum
 * of the OTA upgrade data
 */
int32_t ota_fw_upgrade_calculate_checksum( int32_t offset, int32_t length )
{

    uint32_t read_start = 0;
    uint32_t crc32 = 0xffffffff;
    uint32_t total_len = offset + length;

    for (; offset < total_len; offset += OTA_FW_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[OTA_FW_UPGRADE_READ_CHUNK];
        int32_t bytes_to_read;
        uint8_t mod_val = ( offset % OTA_FW_UPGRADE_READ_CHUNK );

        //To ensure aligned read
        if ( ( !read_start ) && ( mod_val ) )
        {
            bytes_to_read = ( ( OTA_FW_UPGRADE_READ_CHUNK ) < total_len ) ?
                    ( OTA_FW_UPGRADE_READ_CHUNK - mod_val ) : total_len - mod_val;
        }
        else
        {
            bytes_to_read = ((offset + OTA_FW_UPGRADE_READ_CHUNK) < total_len) ?
                                        OTA_FW_UPGRADE_READ_CHUNK : total_len - offset;
        }
        read_start = 1;

        // read should be on the word boundary and in full words, we may read a bit more, but
        // include correct number of bytes in the CRC calculation
        wiced_firmware_upgrade_retrieve_from_nv(offset, memory_chunk, (bytes_to_read + 3) & 0xFFFFFFFC);
#if OTA_CHIP == 20703
        crc32 = update_crc(crc32, memory_chunk, bytes_to_read);
#else
#if (defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20719B2) || defined(CYW20735B1) || defined(CYW20819A1))
        crc32 = crc32_Update(crc32, memory_chunk, bytes_to_read);
#else
        crc32 = update_crc32(crc32, memory_chunk, bytes_to_read);
#endif
#endif

#ifdef OTA_UPGRADE_DEBUG
        //WICED_BT_TRACE("read offset:%x\n", offset);
        //dump_hex(memory_chunk, bytes_to_read);
#endif
    }
    crc32 = crc32 ^ 0xffffffff;

    return crc32;
}

/*
 * Process data chunk received from the host.  If received num of bytes equals to
 * OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT, save the data to NV.
 *
 */
wiced_bool_t ota_fw_upgrade_image_data_handler(uint16_t conn_id, uint8_t *data, int32_t len)
{
    ota_fw_upgrade_state_t *p_state = &ota_fw_upgrade_state;
    uint8_t *p = data;

    if (p_state->state != OTA_STATE_DATA_TRANSFER)
        return FALSE;

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
            // length store in the image does not include size of ds_image_prefix
            p_state->total_len = data[12] + (data[13] << 8) + (data[14] << 16) + (data[15] << 24);
            p_state->total_len += DS_IMAGE_PREFIX_LEN + SIGNATURE_LEN;

            // ToDo validate flash size
            // ToDo validate that product is the same as stored and major is >= the one that stored
            WICED_BT_TRACE("Image for Product 0x%x Major:%d Minor:%d len:%d\n", data[8] + (data[9] << 8), data[10], data[11], p_state->total_len);
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
#if OTA_UPGRADE_DEBUG
            WICED_BT_TRACE("write offset:%x\n", p_state->total_offset);
            //dump_hex(p_state->read_buffer, p_state->current_block_offset);
#endif
            // write should be on the word boundary and in full words, we may write a bit more
            const int current_block_len = (p_state->current_block_offset + 3) & 0xfffffffc;
            if (current_block_len != wiced_firmware_upgrade_store_to_nv(
                        p_state->total_offset, p_state->read_buffer, current_block_len))
            {
                return FALSE;
            }
            p_state->total_offset        += p_state->current_block_offset;
            p_state->current_block_offset = 0;

#if OTA_UPGRADE_DEBUG
            if (p_state->total_offset == p_state->total_len)
            {
                p_state->recv_crc32 = p_state->recv_crc32 ^ 0xffffffff;
                WICED_BT_TRACE("recv_crc32:%x\n", p_state->recv_crc32);
            }
#endif
        }

        len = len - bytes_to_copy;
        p = p + bytes_to_copy;

        //WICED_BT_TRACE("remaining len: %d \n", len);
    }
    return (TRUE);
}

#ifdef OTA_UPGRADE_DEBUG
void dump_hex(uint8_t *p, uint32_t len)
{
    uint32_t i;
    char     buff1[100];

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 32; i++)
        {
            int s1 = (*p & 0xf0) >> 4;
            int s2 = *p & 0x0f;
            buff1[i * 3]     = (s1 >= 0 && s1 <= 9) ? s1 + '0' : s1 - 10 + 'A';
            buff1[i * 3 + 1] = (s2 >= 0 && s2 <= 9) ? s2 + '0' : s2 - 10 + 'A';
            buff1[i * 3 + 2] = ' ';
            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}
#endif
