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
* WICED Firmware Upgrade internal definitions specific to shim layer
*
* This file provides common functions required to support WICED Smart Ready Upgrade
* whether it is being done over the air, UART, or SPI.  Primarily the
* functionality is provided for storing and retrieving information from  Serial Flash
* The data being stored is DS portion of burn image generated from CGS.
*/
#if CYW20706A2
#include "bt_types.h"
#include "wiced.h"
#include "wiced_hal_sflash.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"
#include "string.h"
#include "sfi.h"

/******************************************************
 *                      Constants
 ******************************************************/
#define WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K             (4 * 1024) //Serial Flash Sector size
#define WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K           (256 * 1024) //Serial Flash Sector size

#define INDIRECT_MEM_MAP_MASK   0xFF000000
/// indirect memory map for serial flash Read/Write/Erase access
#define INDIRECT_MEM_MAP_SF     0xF8000000

/// Converts a given address into the NV virtual address and returns same.
#define CONVERT_TO_NV_VIRTUAL_ADDRESS(address) (address | INDIRECT_MEM_MAP_SF)

//#define WICED_FW_UPGRADE_NV_IS_EEPROM()  ((Config_and_Firmware_Status & CFA_CONFIG_LOCATION_MASK) == CFA_CONFIG_LOCATION_EEPROM)

/******************************************************
 *                     Structures
 ******************************************************/
//ws_upgrde global data
typedef struct
{
    uint32_t active_ds_location;
    uint32_t upgrade_ds_location;
    uint32_t upgrade_ds_length;

    uint32_t erase_start;
    uint32_t bytes_erased;
} wiced_fw_upgrade_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
/********************************************************************************************************
* Recommended firmware upgrade 4 MBit Serila flash offsets
 * -------------------------------------------------------------------------------------------------------------------
 * |  SS1 (4K @ 0)  |  Fail safe area(4K @ 0x1000)  |  VS1 (4K @ 0x2000)  | VS2 (4K @ 0x3000)  | DS1 (248K @ 0x4000)  | DS2 (248K @ 0x42000)
 *  -------------------------------------------------------------------------------------------------------------------
 *******************************************************************************************************/
wiced_fw_upgrade_nv_loc_len_t   g_nv_loc_len;
wiced_fw_upgrade_t              g_fw_upgrade;

/******************************************************
 *               External variables
 ******************************************************/
extern uint32_t Config_VS_Location;
extern uint32_t Config_DS_Location;
extern uint32_t Config_DS_End_Location;
extern uint8_t  sfi_sectorErase256K;

extern void ota_fw_upgrade_init_data(wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback);

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t wiced_firmware_upgrade_init(wiced_fw_upgrade_nv_loc_len_t *p_sflash_nv_loc_len, uint32_t sflash_size)
{
    wiced_fw_upgrade_nv_loc_len_t *p_active_nv = &g_nv_loc_len;

    if (p_sflash_nv_loc_len == NULL)
    {
        WICED_BT_TRACE("Error. can not upgrade. Please provide NV location and length \n");
        return WICED_FALSE;
    }

    // Enable SW write protect.
    *((unsigned char*)(0x00201a14)) = 1;

    WICED_BT_TRACE("platform btp: Config_DS_Location 0x%08x, Config_VS_Location:0x%08x \n", Config_DS_Location, Config_VS_Location);

    memcpy(p_active_nv, p_sflash_nv_loc_len, sizeof(wiced_fw_upgrade_nv_loc_len_t));

    if ((Config_DS_Location != p_active_nv->ds1_loc) &&
        (Config_DS_Location != p_active_nv->ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("WARNING: Upgrade will fail - active DS is not one of the expected locations\n");
        WICED_BT_TRACE("WARNING: Are ConfigDSLocation and DLConfigVSLocation in the .btp set up as in nv_loc_len[]?\n");
        return WICED_FALSE;
    }

    wiced_hal_sflash_set_size( sflash_size );

    return WICED_TRUE;
}


// setup nvram locations to be used during upgrade. if success returns 1, else fails return 0
uint32_t wiced_firmware_upgrade_init_nv_locations(void)
{
    wiced_fw_upgrade_t              *p_gdata        = &g_fw_upgrade;

    if ((Config_DS_Location != g_nv_loc_len.ds1_loc) &&
        (Config_DS_Location != g_nv_loc_len.ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("Active DS %x is not DS1:%x and not DS2:%x. Cannot upgrade.\n", Config_DS_Location, g_nv_loc_len.ds1_loc, g_nv_loc_len.ds2_loc);
        return 0;
    }

    p_gdata->active_ds_location  = Config_DS_Location;

    p_gdata->upgrade_ds_location = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_loc : g_nv_loc_len.ds1_loc;
    p_gdata->upgrade_ds_length   = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_len : g_nv_loc_len.ds1_len;

    WICED_BT_TRACE("Active: 0x%08X, Upgrade: 0x%08X, UG length: 0x%08X", p_gdata->active_ds_location, p_gdata->upgrade_ds_location, p_gdata->upgrade_ds_length);

    Config_DS_End_Location = p_gdata->active_ds_location +
                                    ((p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                      g_nv_loc_len.ds1_len : g_nv_loc_len.ds2_len);

    p_gdata->bytes_erased = 0;

    return 1;
}

//Erases the Serial Flash Sector
void fw_upgrade_erase_mem(uint32_t erase_addr)
{
    // if Write Serial Flash
    //f8000000
    if ((erase_addr & INDIRECT_MEM_MAP_MASK) == INDIRECT_MEM_MAP_SF)
    {
        erase_addr &= (~INDIRECT_MEM_MAP_MASK);
        if (sfi_sectorErase256K)
        {
            wiced_hal_sflash_erase(erase_addr, WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K);
        }
	else
        {
            wiced_hal_sflash_erase(erase_addr, WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K);
        }
    }
    else
    {
        WICED_BT_TRACE("Not Serial Flash. Can't Erase @addr %x\n", erase_addr);
    }
}

//Reads the given length of data from SF/EEPROM. If success returns len, else returns 0
uint32_t fw_upgrade_read_mem(uint32_t read_from, uint8_t *buf, uint32_t len)
{
    memset(buf, 0xAA, len); // fill with some "bad" value for debug

    // if read from Serial Flash
    // if readFrom & 0xff000000 == 0xf8000000
    if ((read_from & INDIRECT_MEM_MAP_MASK) == INDIRECT_MEM_MAP_SF)
    {
        read_from &= (~INDIRECT_MEM_MAP_MASK);
        // WICED_BT_TRACE("sflash_read from:%x len:%d\n", read_from, len);
        if (wiced_hal_sflash_read(read_from, len, buf) != len)
        {
            WICED_BT_TRACE("sflash_read failed\n");
            return 0;
        }
        return len;
    }

    return 0;
}

// Writes the given length of data to SF. If success returns len, else returns 0
uint32_t fw_upgrade_write_mem(uint32_t write_to, uint8_t *data, uint32_t len)
{
    // if Write Serial Flash
     if ((write_to & INDIRECT_MEM_MAP_MASK) == INDIRECT_MEM_MAP_SF)
    {
        write_to &= (~INDIRECT_MEM_MAP_MASK);

        // WICED_BT_TRACE("sflash_write to:%x len:%d\n", write_to, len);
        return wiced_hal_sflash_write(write_to, len, data);
    }

    return 0;
}

wiced_bool_t wiced_firmware_upgrade_erase_nv(uint32_t start, uint32_t size)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t sector_size = (sfi_sectorErase256K) ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
    uint32_t offset;

    if (start % sector_size)
        return WICED_FALSE;

    for (offset = 0; offset < size; offset += sector_size)
        fw_upgrade_erase_mem(start + offset);

    p_gdata->erase_start = start;
    p_gdata->bytes_erased = offset;
    return WICED_TRUE;
}

wiced_bool_t fw_upgrade_is_sector_erased(uint32_t offset)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;

    if (p_gdata->bytes_erased == 0
        || offset < p_gdata->erase_start
        || offset >= (p_gdata->erase_start + p_gdata->bytes_erased))
        return WICED_FALSE;

    return WICED_TRUE;
}

// Stores to the physical NV storage medium. if success, return len, else returns 0
uint32_t wiced_firmware_upgrade_store_to_nv(uint32_t offset, uint8_t *data, uint32_t len)
{
    uint32_t _offset = offset;
    uint32_t sector_size = (sfi_sectorErase256K) ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
    uint32_t bytes_written;

    // The real offset into the NV is the current offset + the upgrade DS location.
    offset += g_fw_upgrade.upgrade_ds_location;

    // Now add the NV virtual address to this offset based on the physical part.
    offset = CONVERT_TO_NV_VIRTUAL_ADDRESS(offset);

    // if this is a beginning of a new sector erase first.
    if ((offset % sector_size) == 0 && !fw_upgrade_is_sector_erased(offset))
    {
        fw_upgrade_erase_mem(offset);
    }

    bytes_written = fw_upgrade_write_mem(offset, data, len);

    return bytes_written;
}

// Retrieve chunk of data from the physical NV storage medium. if success returns len, else return 0
uint32_t wiced_firmware_upgrade_retrieve_from_nv(uint32_t offset, uint8_t *data, uint32_t len)
{
    // The real offset into the NV is the current offset + the upgrade DS location.
    offset += g_fw_upgrade.upgrade_ds_location;

    // Now add the NV virtual address to this offset based on the physical part.
    offset = CONVERT_TO_NV_VIRTUAL_ADDRESS(offset);

    return fw_upgrade_read_mem(offset, data, len);
}

// In fail safe OTA patch, boot will decide active DS location
// No access SS after OTA verification done
#define DS2_MAGIC_NUMBER_BUFFER_LEN     8
#define SS_ORIGINAL_DSVS_ADDR           0x1B

#pragma pack(1)
typedef union
{
    UINT8  buffer[DS2_MAGIC_NUMBER_BUFFER_LEN + sizeof(UINT32)];
    struct
    {
        UINT8   magicNumber[DS2_MAGIC_NUMBER_BUFFER_LEN];
        UINT32  dsLoc;
    } ds2Info;
} tDs2Record;

typedef struct
{
    UINT8 cfgID;
    UINT16 len;
    UINT32 DS_Loc;
}tDSInfo;
#pragma pack()

UINT8  magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN + 4] = { 0xAA, 0x55, 0xF0, 0x0F, 0x68, 0xE5, 0x97, 0xD2, 0,0,0,0};

uint8_t fw_upgrade_switch_active_ds(void)
{
    uint8_t dsDetect = 0;
    uint32_t magic_num_loc;
    wiced_fw_upgrade_nv_loc_len_t *p_active_nv = NULL;
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    tDSInfo DSInfo;

    p_active_nv = &g_nv_loc_len;

    if (sfi_read(SS_ORIGINAL_DSVS_ADDR, sizeof(tDSInfo), (uint8_t *)&DSInfo) != sizeof(tDSInfo))
    {
        WICED_BT_TRACE("dsinfo@SS_ORIGINAL_DSVS_ADDR read fail \n");
        return 0;
    }
    WICED_BT_TRACE("DSInfo id:%x DS_Loc:0x%x Len:%d cfgID:%d\n", DSInfo.cfgID, DSInfo.DS_Loc, DSInfo.len, DSInfo.cfgID);

    // fail safe OTA patch will not have 0x02 item
    if (DSInfo.cfgID != 0x02)
    {
        // Sector must be erased before writing to it.
        if (sfi_sectorErase256K)
        {
            // adjust location for non-4k erase sector size - this must match location used by fail safe download
            magic_num_loc = CONVERT_TO_NV_VIRTUAL_ADDRESS(2 * WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K) - sizeof(tDs2Record);
            sfi_erase(WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K, WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K);
        }
        else
        {
            magic_num_loc = CONVERT_TO_NV_VIRTUAL_ADDRESS(2 * WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K) - sizeof(tDs2Record);
            sfi_erase(WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K, WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K);
        }

        if (p_gdata->active_ds_location == p_active_nv->ds1_loc)
        {
            // Then update the DS with the upgrade DS.
            magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN]   = (p_gdata->upgrade_ds_location & 0xFF);
            magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN+1] = (p_gdata->upgrade_ds_location & 0xFF00) >> 8;
            magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN+2] = (p_gdata->upgrade_ds_location & 0xFF0000) >> 16;
            magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN+3] = (p_gdata->upgrade_ds_location & 0xFF000000) >> 24;

            // Write this to the upgrade SS.
            if (sfi_write(magic_num_loc , DS2_MAGIC_NUMBER_BUFFER_LEN + 4, magic_num_ds2) != (DS2_MAGIC_NUMBER_BUFFER_LEN + 4))
            {
                WICED_BT_TRACE("Could not update the 2nd SS block w/ magic number and DS location!\n");
                return 0;
            }
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

// After download is completed and verified this function is
// called to switch active partitions with the one that has been
// receiving the new image.
void wiced_firmware_upgrade_finish(void)
{
    if (!fw_upgrade_switch_active_ds())
    {
        WICED_BT_TRACE("No fail safe OTA patch. Cannot upgrade firmware \n");
    }
    wiced_hal_wdog_reset_system();
    // End of the world - will not return.
}

#define PARTITION_ACTIVE    0
#define PARTITION_UPGRADE   1

uint32_t wiced_bt_get_fw_image_size(uint8_t partition)
{
#define PARTITION_END_MARKER    0xFE
#define READ_CHUNK_SIZE         512

    uint8_t     memory_chunk[READ_CHUNK_SIZE];
    uint32_t    image_size = 0, bytes_read, i;
    uint32_t    offset;

    if (partition == PARTITION_ACTIVE)
        offset = Config_DS_Location;
    else
        offset = (Config_DS_Location == g_nv_loc_len.ds1_loc) ? g_nv_loc_len.ds2_loc : g_nv_loc_len.ds1_loc;

    /* DS config items are stored as type-length-value (1 byte - 2 bytes - length bytes) tuples and
    the last item has type = 0xFE, length = 0x0000. So you should be able to read 3 bytes, then
    use byte 1 and 2 from that, and use this as the length bytes (little endian) to the next item till you find type = 0xFE. */
    do{
        /* to reduce number of flash reads, read 512 chunks at a time */
        bytes_read = wiced_hal_sflash_read(CONVERT_TO_NV_VIRTUAL_ADDRESS(offset), READ_CHUNK_SIZE, memory_chunk);
//        WICED_BT_TRACE("br: %d offset: %08x j:%d \n", bytes_read, offset, j);
        if (bytes_read)
        {
            /* parse tlvs to get the length of partition. If tlv itself (i.e. 3 bytes) is
            not complete in this read, do not parse it so that it will be processed in next flash read */
            for (i = 0; (i+3) <= bytes_read; )
            {
                uint8_t *p = &memory_chunk[i];

                if (*p == PARTITION_END_MARKER)
                {
                    /* end marker is 3 byte tlv */
                    image_size += (i + 3);

                    return image_size;
                }
                else
                {
                    /* next tlv offset  = length of current value + 3 bytes for tlv itself */
                    p++;
                    i += (((uint16_t)(*p + ((*(p + 1)) << 8))) + 3);
                }
            }

            /* move offset to point to next tlv */
            offset += i;

            /* calculate image_size by sum of all tlv tupples and its value lengths */
            image_size += i;

            /* for safe side, feed watch dog to avoid fw restart in case of lengthy firmwares */
            wiced_hal_wdog_restart ();
        }
        else
        {
            break;
        }
    }while(TRUE);

    return image_size;
}

uint32_t wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    uint32_t base_offset;

    if (partition == PARTITION_ACTIVE)
        base_offset = Config_DS_Location;
    else
        base_offset = (Config_DS_Location == g_nv_loc_len.ds1_loc) ? g_nv_loc_len.ds2_loc : g_nv_loc_len.ds1_loc;

    return fw_upgrade_read_mem(CONVERT_TO_NV_VIRTUAL_ADDRESS((offset+base_offset)), p_data, data_len);
}

#define IMAGE_META_DATA_PREFIX_LEN      12
#define IMAGE_META_DATA_ID              0xABADCAFE

void wiced_bt_fw_save_meta_data(uint8_t partition, uint8_t *p_data, uint32_t len)
{
    uint32_t save_len = len + IMAGE_META_DATA_PREFIX_LEN;
    uint8_t *p_save_data;
    uint8_t *p;

    if (len == 0)       // Remove meta data
    {
        wiced_firmware_upgrade_erase_nv(0, 1);
        return;
    }

    p_save_data = wiced_bt_get_buffer(save_len);
    if (p_save_data == NULL)
        return;

    p = p_save_data;
    UINT32_TO_STREAM(p, 0xFFFFFFFF);
    UINT32_TO_STREAM(p, IMAGE_META_DATA_ID);
    UINT32_TO_STREAM(p, len);
    memcpy(p, p_data, len);
    if (partition == PARTITION_UPGRADE)
    {
        wiced_firmware_upgrade_store_to_nv(0, p_save_data, (save_len + 3) & 0xFFFFFFFC);
    }

    wiced_bt_free_buffer(p_save_data);
}

wiced_bool_t wiced_bt_fw_read_meta_data(uint8_t partition, uint8_t *p_data, uint32_t *p_len)
{
    uint8_t buffer[IMAGE_META_DATA_PREFIX_LEN];
    uint8_t *p = buffer;
    uint32_t temp;
    uint32_t data_len;
    wiced_bool_t got_data = WICED_FALSE;

    if (p_data == NULL || p_len == NULL)
        return WICED_FALSE;

    if (partition == PARTITION_UPGRADE)
    {
        if (wiced_firmware_upgrade_retrieve_from_nv(0, buffer, IMAGE_META_DATA_PREFIX_LEN) == IMAGE_META_DATA_PREFIX_LEN)
        {
            STREAM_TO_UINT32(temp, p);
            if (temp != 0xFFFFFFFF)
                return WICED_FALSE;
            STREAM_TO_UINT32(temp, p);
            if (temp != IMAGE_META_DATA_ID)
                return WICED_FALSE;
            STREAM_TO_UINT32(data_len, p);
            if (data_len > *p_len)
                return WICED_FALSE;

            if (wiced_firmware_upgrade_retrieve_from_nv(IMAGE_META_DATA_PREFIX_LEN, p_data, data_len) == data_len)
            {
                *p_len = data_len;
                got_data = WICED_TRUE;
            }
        }
    }

    return got_data;
}
#endif // CYW20706A2
