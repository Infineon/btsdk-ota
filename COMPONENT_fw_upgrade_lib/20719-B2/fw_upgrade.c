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
#if CYW20719B2
#include "bt_types.h"
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_hal_eflash.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"

/******************************************************
 *                      Constants
 ******************************************************/

/******************************************************
 *                     Structures
 ******************************************************/
//ws_upgrde global data
typedef struct
{
    uint32_t active_ds_location;
    uint32_t active_ds_length;
    uint32_t upgrade_ds_location;
    uint32_t upgrade_ds_length;
    uint32_t upgrade_ds_signature;

    uint32_t erase_start;
    uint32_t bytes_erased;
} wiced_fw_upgrade_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
wiced_fw_upgrade_t              g_fw_upgrade;

/******************************************************
 *               External declarations
 ******************************************************/
#define NVRAM_INTF_NONE           0
#define NVRAM_INTF_SERIAL_FLASH   1
#define NVRAM_INTF_I2C_EEPROM     2
#define NVRAM_INTF_PARRAL_FLASH   3
#define NVRAM_INTF_EFLASH         4
#define NVRAM_INTF_QUAD_FLASH     5

#define EF_BASE_ADDR                    (0x500000u)
#define EF_PAGE_SIZE                    (0x1000u)

//==================================================================================================
// Types
//==================================================================================================
//! Structure for FOUNDATION_CONFIG_ITEM_ID_CONFIG_LAYOUT.
#pragma pack(1)
typedef struct
{
    //! Base address or offset of the failsafe (not upgradable) dynamic section base.  This field
    //! must be present.
    UINT32 failsafe_ds_base;

    //! Base address or offset of the upgradable dynamic section base.  This field is optional for
    //! media types for which DFU is supported.
    UINT32 upgradable_ds_base;

    //! Base address or offset to the area reserved for volatile section copy 1.  Whether this is an
    //! address or offset depends on the media type, and is an internal detail of those media types'
    //! access functions.  Double-buffering of the volatile section alternates between the two
    //! copies when the active copy fills up and has to be consolidated to the other.  The volatile
    //! section stores information that is mutable at runtime, and is therefore subject to loss if a
    //! write operation is interrupted by loss of power.  Only an item that is currently being
    //! written is subject to loss.  Generally, NVRAM media with large page sizes (like flash) use
    //! double-buffering, while media with small page sizes (like EEPROM) allocate one or more
    //! complete pages per volatile section item.
    UINT32 vs_copy1_base;

    //! Base address or offset to the area reserved for volatile section copy 2.  Whether this is an
    //! address or offset depends on the media type, and is an internal detail of those media types'
    //! access functions.  See the documentation for vs_copy1_base, but note that not all media
    //! types use double-buffering.
    UINT32 vs_copy2_base;

    //! Length in bytes per copy of the area reserved for each volatile section copy.  If the target
    //! media uses double buffering to protect against loss, the total space used by the volatile
    //! section is twice this amount.  See the documentation for vs_copy1_base and vs_copy1_base.
    UINT32 vs_length_per_copy;

    //! Block size for volatile section items.  For media with small page sizes (like EEPROM) which
    //! allocate one or more pages per volatile section item, blocks must be a multiple of the media
    //! page size.
    UINT32 vs_block_size;

    //! Media page size.  This info is needed for managing volatile section contents.
    UINT32 media_page_size;
} FOUNDATION_CONFIG_ITEM_CONFIG_LAYOUT_t;
#pragma pack()

//! Enumeration used to specify one of the three sections of config data.
//!                                                                                         <br><br>
//! If config data is stored in NVRAM:
//!                                                                                         <br><br>
//! Static section is written once during manufacturing, and never again.  This section includes
//! per-device information like crystal trimming information and an assigned address like BD_ADDR
//! for Bluetooth devices or a MAC address for ethernet or WLAN devices.  The static section also
//! includes key layout information like whether a volatile section is present and if so, where it
//! is located.
//!                                                                                         <br><br>
//! Dynamic section is written during manufacturing.  This section might be subject to upgrades in
//! the field, by the end user.  An example of such an upgrade process is USB device firmware
//! upgrade.  If this section is subject to upgrade in the field, then a failsafe config must be
//! present, which if present would either force the device into an upgrade-only mode, or fall back
//! to the un-upgraded behavior it would have exhibited when it left the factory.
//!                                                                                         <br><br>
//! Volatile section is used to hold information that can change at runtime, for example storing
//! pairing information for pairing with other devices.  The volatile section is implemented as
//! failsafe as possible for the target media, such that the most recently written "nugget" of
//! information is subject to loss, but contents that were present before a given write operation
//! will be preserved.
//!                                                                                         <br><br>
//! The "volatile" nomenclature is somewhat misleading because this section is only ever present on
//! NVRAM (nonvolatile memory).  The "volatile" nomenclature is simply used to highlight the fact
//! that the contents are subject to loss.  This is generally a non-issue, but if multiple "nuggets"
//! of information are interdependent but written independently, then it is possible for one
//! "nugget" in the interdependent set to be lost, in which case the firmware that uses this
//! information needs to be ready to recognize that situation and take appropriate action to discard
//! or if possible repair the rest of the set.  If no "nuggets" of volatile information form
//! interdependent sets then loss of power during a write operation is functionally equivalent to
//! loss of power immediately before the write operation was initiated.
//!                                                                                         <br><br>
//! If config data is stored in RAM (downloaded by the host):
//!                                                                                         <br><br>
//! Only the static and dynamic sections are relevant.  The distinction between the two halves is
//! more or less irrelevant, merely being a reflection of the NVRAM organization.  Nonetheless, the
//! location in which certain pieces of information are stored is influenced by the NVRAM
//! organization.  A volatile section should never be specified for RAM config data.
typedef enum
{
    //! Configuration data section containing per-device information and key layout information.
    //! The layout information communicates to firmware where to find the rest of the configuration
    //! data.  See the documentation for the config_section_id_t enumeration as a whole for more
    //! complete info.
    CONFIG_STATIC,

    //! Configuration data section containing per-product or product family information.  See the
    //! documentation for the config_section_id_t enumeration as a whole for more complete info.
    CONFIG_DYNAMIC,

    //! Configuration data section in NVRAM containing information that can be changed at runtime.
    //! This refers to info that needs to be preserved across resets or power cycles.  See the
    //! documentation for the config_section_id_t enumeration as a whole for more complete info,
    //! including where the seemingly contradictory name comes from.
    CONFIG_VOLATILE
} config_section_id_t;

//! \internal
//! Structure used internally by the config module to achieve config media abstraction.  It stores
//! layout information for any supported config data media type, as well as media-specific function
//! pointers for various tasks.
typedef struct
{
    //! Access function pointer to read raw data from the media on which config data is stored.
    void    (*fp_ReadRaw)( int offset,
                            config_section_id_t which_section,
                            OUT BYTE* buffer,
                            int length);

    //! Access function pointer to write raw data to the media on which config data is stored.
    void    (*fp_WriteRaw)(int offset,
                            config_section_id_t which_section,
                            IN BYTE* buffer,
                            int length);

    //! Address of the static section.
    UINT32 ss_base;

    //! Function to handle when the layout config item below has been filled in.  It will have been
    //! filled in using content from the static section, then this function will be called.
    void    (*fp_ConfigLayoutHasBeenSet)(void);

    //! Address of the valid dynamic section (which might be the failsafe copy, or might be the
    //! upgradable copy).
    UINT32 active_ds_base;

    //! Access function pointer to read a volatile section item from config data.  The function is
    //! presented as being specific to the type of media, but it really reflects the partitioning
    //! scheme used by this media as dictated by its physical page size.  The truly media-specific
    //! access function is in fp_ReadRaw.
    UINT16  (*fp_ReadVolatileSectionItem)( UINT16 group_id,
                                            UINT16 sub_id_in_group,
                                            OUT BYTE* buffer,
                                            UINT16 max_length);

    //! Access function pointer to write a volatile section item to config data.  The function is
    //! presented as being specific to the type of media, but it really reflects the partitioning
    //! scheme used by this media as dictated by its physical page size.  The truly media-specific
    //! access function is in fp_WriteRaw.
    void    (*fp_WriteVolatileSectionItem)(UINT16 group_id,
                                            UINT16 sub_id_in_group,
                                            IN BYTE* buffer,
                                            UINT16 length);

    //! Layout info, retrieved from the static section.
    FOUNDATION_CONFIG_ITEM_CONFIG_LAYOUT_t layout;

    //! Checksum/CRC info for validating segment by segment in the dynamic section.
    UINT32 checksum;
    UINT32 crc32;
    BOOL8 valid_crc32;

    //! Used to allow faster acces to the config if it is memory mapped (not in serial flash for exmaple)
    BOOL8 direct_access;

    //! Whether a valid DS section was found or not.
    BOOL8 valid_ds_found;
} CONFIG_INFO_t;


extern CONFIG_INFO_t        g_config_Info;
extern UINT8                g_nvram_intf;

extern void ota_fw_upgrade_init_data(wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback);

/* internal functions */
uint8_t     firmware_upgrade_switch_active_ds(void);
uint8_t     firmware_upgrade_switch_eflash_active_ds(void);


#ifdef ENABLE_WICED_FW_DEBUG
UINT8 first_256_byte_dump[256];
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t wiced_firmware_upgrade_init(wiced_fw_upgrade_nv_loc_len_t *p_sflash_nv_loc_len, uint32_t sflash_size)
{
    const uint32_t ds1_location = g_config_Info.layout.failsafe_ds_base;
    const uint32_t ds1_length = g_config_Info.layout.upgradable_ds_base - g_config_Info.layout.failsafe_ds_base;
    const uint32_t ds2_location = g_config_Info.layout.upgradable_ds_base;
    const uint32_t ds2_length = EF_BASE_ADDR + sflash_size - g_config_Info.layout.upgradable_ds_base;

    WICED_BT_TRACE("ds1: 0x%08x, len: 0x%08x\n", ds1_location, ds1_length);
    WICED_BT_TRACE("ds2: 0x%08x, len: 0x%08x\n", ds2_location, ds2_length);
    WICED_BT_TRACE("Active: 0x%08x, vs1: 0x%08x, vs2: 0x%08x\n", g_config_Info.active_ds_base,
            g_config_Info.layout.vs_copy1_base, g_config_Info.layout.vs_copy2_base);

    if (g_config_Info.active_ds_base == ds1_location)
    {
        g_fw_upgrade.active_ds_location = ds1_location;
        g_fw_upgrade.active_ds_length = ds1_length;
        g_fw_upgrade.upgrade_ds_location = ds2_location;
        g_fw_upgrade.upgrade_ds_length = ds2_length;
    }
    else if (g_config_Info.active_ds_base == ds2_location)
    {
        g_fw_upgrade.active_ds_location = ds2_location;
        g_fw_upgrade.active_ds_length = ds2_length;
        g_fw_upgrade.upgrade_ds_location = ds1_location;
        g_fw_upgrade.upgrade_ds_length = ds1_length;
    }
    else
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        return WICED_FALSE;
    }

    g_fw_upgrade.upgrade_ds_signature = 0xfffffffful;

#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("ss: 0x%08x\n", g_config_Info.ss_base);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_config_Info.ss_base - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("vs:%x\n", g_config_Info.layout.vs_copy1_base);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_config_Info.layout.vs_copy1_base - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", ds1_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(ds1_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", ds2_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(ds2_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

    return WICED_TRUE;
}


// setup NVRAM locations to be used during upgrade. if success returns 1, else fails return 0
uint32_t wiced_firmware_upgrade_init_nv_locations(void)
{
    g_fw_upgrade.bytes_erased = 0;

    return 1;
}

wiced_bool_t wiced_firmware_upgrade_erase_nv(uint32_t start, uint32_t size)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t offset;

    if (start % EF_PAGE_SIZE)
        return WICED_FALSE;

    for (offset = 0; offset < size; offset += EF_PAGE_SIZE)
        wiced_hal_eflash_erase(start + offset + g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);

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
    if (g_nvram_intf == NVRAM_INTF_EFLASH)
    {
        // if this is a beginning of a new sector erase first.
        if ((offset % EF_PAGE_SIZE) == 0 && !fw_upgrade_is_sector_erased(offset))
        {
            wiced_hal_eflash_erase(offset + g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
        }
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        if(offset == 0)
        {
            memcpy(&g_fw_upgrade.upgrade_ds_signature, data, 4);
            offset += 4;
            data += 4;
            len -= 4;
        }
        offset += (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR);

        //WICED_BT_TRACE("write: offset:%x len:%d\n", offset, len);
        if (wiced_hal_eflash_write(offset, data, len) == WICED_SUCCESS)
        {
            if(offset == (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR + 4))
                len += 4;
            return len;
        }
        else
        {
            WICED_BT_TRACE("write: failed offset:%x len:%d\n", offset, len);
            return 0;
        }
    }

    return 0;
}

// Retrieve chunk of data from the physical NV storage medium. if success returns len, else return 0
uint32_t wiced_firmware_upgrade_retrieve_from_nv(uint32_t offset, uint8_t *data, uint32_t len)
{
    if (g_nvram_intf == NVRAM_INTF_EFLASH)
    {
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        if(offset == 0)
        {
            memcpy(data, &g_fw_upgrade.upgrade_ds_signature, 4);
            offset += 4;
            data += 4;
            len -= 4;
        }
        offset += (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR);

        if (wiced_hal_eflash_read(offset, data, len) == WICED_SUCCESS)
        {
            if(offset == (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR + 4))
                len += 4;
            return len;
        }
        else
        {
            WICED_BT_TRACE("failed to read offset:%x\n", offset);
            return 0;
        }
    }

    return 0;
}

// After download is completed and verified this function is
// called to switch active partitions with the one that has been
// receiving the new image.
void wiced_firmware_upgrade_finish(void)
{
#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("before switch active ds:%x-%x\n", g_fw_upgrade.active_ds_location, g_config_Info.active_ds_base);
    WICED_BT_TRACE("ss:\n");
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(0, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("active:%x\n", g_fw_upgrade.active_ds_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("upgrade:%x\n", g_fw_upgrade.upgrade_ds_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

    if (!firmware_upgrade_switch_active_ds())
    {
        // just maybe print message but don't return, still fall through to reset
    }

    WICED_BT_TRACE("FW upgrade completed\n");

#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("after switch\n");
    WICED_BT_TRACE("ss:\n");
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(0, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("active:%x\n", g_fw_upgrade.active_ds_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
    WICED_BT_TRACE("upgrade:%x\n", g_fw_upgrade.upgrade_ds_location);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

    wiced_hal_wdog_reset_system();

    // End of the world - will not return.
}

/* internal utility functions */
uint8_t firmware_upgrade_switch_active_ds(void)
{
    if (g_nvram_intf == NVRAM_INTF_EFLASH)
    {
        return firmware_upgrade_switch_eflash_active_ds();
    }
    return 0;
}

uint8_t firmware_upgrade_switch_eflash_active_ds(void)
{
    wiced_result_t result;
    uint32_t signature = 0;

    // commit reserved first 4 bytes of download to complete
    // this is done last and after crc in case of power loss during download
    // boot rom checks this signature to validate DS, checking DS1 first, then DS2
    wiced_hal_eflash_write(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, (uint8_t *)&g_fw_upgrade.upgrade_ds_signature, 4);

    // check that the write completed
    wiced_hal_eflash_read(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, (uint8_t *)&signature, 4);
    if(signature != g_fw_upgrade.upgrade_ds_signature)
    {
        return 0;
    }

    // clear first active DS sector in eflash, so that on next boot, CRC check will fail and ROM code boots from upgraded DS
    result = wiced_hal_eflash_erase(g_config_Info.active_ds_base - EF_BASE_ADDR, EF_PAGE_SIZE);
    WICED_BT_TRACE(" [%s] wiced_hal_eflash_erase status %d \n", __FUNCTION__, result);

    //WICED_BT_TRACE("active DS first sector erase result:%d active_ds:%x\n", result, g_config_Info.active_ds_base - EF_BASE_ADDR);
    return 1;
}




#define PARTITION_ACTIVE    0
#define PARTITION_UPGRADE   1
uint32_t wiced_bt_get_fw_image_size(uint8_t partition)
{
    uint8_t data[16];
    uint32_t image_size = 0;
    uint32_t offset = PARTITION_ACTIVE ? g_fw_upgrade.active_ds_location : g_fw_upgrade.upgrade_ds_location;
    if (wiced_hal_eflash_read(offset, data, 16) == WICED_SUCCESS)
    {
        image_size = data[12] + (data[13] << 8) + (data[14] << 16) + (data[15] << 24);
        WICED_BT_TRACE("image size:%d offset:%08x\n", image_size, offset);
    }
    else
    {
        WICED_BT_TRACE("failed to read header offset:%08x\n", offset);
    }
    return image_size;
}

void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
}

/*
 * wiced_firmware_upgrade_retrieve_from_active_nv
 * Retrieve chunk of data of the active Data Section from the physical NV storage medium.
 * if success returns length, else return 0
 */
uint32_t wiced_firmware_upgrade_retrieve_from_active_ds(uint32_t offset, uint8_t *data, uint32_t len)
{
    if (g_nvram_intf == NVRAM_INTF_EFLASH)
    {
        offset += (g_fw_upgrade.active_ds_location - EF_BASE_ADDR);

        if (wiced_hal_eflash_read(offset, data, len) == WICED_SUCCESS)
        {
            return len;
        }
        else
        {
            WICED_BT_TRACE("failed to read offset:%x\n", offset);
            return 0;
        }
    }

    return 0;
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
#endif // CYW20719B2
