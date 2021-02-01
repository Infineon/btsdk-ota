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

/** @file
*
* WICED Firmware Upgrade internal definitions specific to shim layer
*
* This file provides common functions required to support WICED Smart Ready Upgrade
* whether it is being done over the air, UART, or SPI.  Primarily the
* functionality is provided for storing and retrieving information from  Serial Flash
* The data being stored is DS portion of burn image generated from CGS.
*/
#if CYW20735B1
#include "bt_types.h"
#include "wiced.h"
#include "wiced_hal_sflash.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"

/******************************************************
 *                      Constants
 ******************************************************/
#define WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K             (4 * 1024) //Serial Flash Sector size
#define WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K           (256 * 1024) //Serial Flash Sector size

// In fail safe ota patch, boot will decide active DS location
// No access SS after OTA verification done
#define DS2_MAGIC_NUMBER_BUFFER_LEN     8

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

#pragma pack()
#define FAIL_SAFE_RESERVE_SECTOR_START  0x1000
#define FAIL_SAFE_RESERVE_SECTOR_END    0x2000

#define DS2_MAGIC_NUMBER_BUFFER_LOC (0x2000 - sizeof(tDs2Record))
UINT8  magic_num_ds2[DS2_MAGIC_NUMBER_BUFFER_LEN + 4] = { 0xAA, 0x55, 0xF0, 0x0F, 0x68, 0xE5, 0x97, 0xD2, 0,0,0,0};

/******************************************************
 *                     Structures
 ******************************************************/
//ws_upgrde global data
typedef struct
{
    uint32_t active_ds_location;
    uint32_t upgrade_ds_location;
    uint32_t upgrade_ds_length;
    uint32_t upgrade_ds_signature;

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
 *               External declarations
 ******************************************************/
#define NVRAM_INTF_NONE           0
#define NVRAM_INTF_SERIAL_FLASH   1
#define NVRAM_INTF_I2C_EEPROM     2
#define NVRAM_INTF_PARRAL_FLASH   3
#define NVRAM_INTF_EFLASH         4
#define NVRAM_INTF_QUAD_FLASH     5

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

// TODO
//extern uint32_t Config_DS_End_Location;

uint8_t  sfi_sectorErase256K = WICED_FALSE;

static void     fw_upgrade_erase_sector(uint32_t erase_addr);
static uint32_t fw_upgrade_write_mem(uint32_t write_to, uint8_t *data, uint32_t len);
static uint32_t fw_upgrade_read_mem(uint32_t read_from, uint8_t *buf, uint32_t len);
uint8_t     firmware_upgrade_switch_sflash_active_ds(void);
uint8_t     firmware_upgrade_switch_active_ds(void);

extern void ota_fw_upgrade_init_data();
/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t wiced_firmware_upgrade_init(wiced_fw_upgrade_nv_loc_len_t *p_sflash_nv_loc_len, uint32_t sflash_size)
{
    if (p_sflash_nv_loc_len == NULL)
    {
        WICED_BT_TRACE("Error. can not upgrade. Please provide NV location and length\n");
        return WICED_FALSE;
    }

    memcpy(&g_nv_loc_len, p_sflash_nv_loc_len, sizeof(wiced_fw_upgrade_nv_loc_len_t));

    WICED_BT_TRACE("Active DS:%x vs1:%x vs2:%x\n", g_config_Info.active_ds_base, g_config_Info.layout.vs_copy1_base, g_config_Info.layout.vs_copy2_base);

#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("ss:%x\n", g_nv_loc_len.ss_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ss_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("vs:%x\n", g_nv_loc_len.vs1_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.vs1_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", g_nv_loc_len.ds1_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds1_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", g_nv_loc_len.ds2_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds2_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

    wiced_hal_sflash_set_size( sflash_size );

    if ((g_config_Info.active_ds_base != p_sflash_nv_loc_len->ds1_loc) &&
        (g_config_Info.active_ds_base != p_sflash_nv_loc_len->ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("WARNING: Upgrade will fail - active DS is not one of the expected locations\n");
        WICED_BT_TRACE("WARNING: Are ConfigDSLocation and DLConfigVSLocation in the .btp set up as in nv_loc_len[]?\n");
        return WICED_FALSE;
    }

    return WICED_TRUE;
}


// setup NVRAM locations to be used during upgrade. if success returns 1, else fails return 0
uint32_t wiced_firmware_upgrade_init_nv_locations(void)
{
    wiced_fw_upgrade_t  *p_gdata = &g_fw_upgrade;

    if ((g_config_Info.active_ds_base != g_nv_loc_len.ds1_loc) &&
        (g_config_Info.active_ds_base != g_nv_loc_len.ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("Active DS %x is not DS1:%x and not DS2:%x. Cannot upgrade.\n", g_config_Info.active_ds_base, g_nv_loc_len.ds1_loc, g_nv_loc_len.ds2_loc);
        return 0;
    }

    p_gdata->active_ds_location = g_config_Info.active_ds_base;

    p_gdata->upgrade_ds_location = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_loc : g_nv_loc_len.ds1_loc;
    p_gdata->upgrade_ds_length   = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_len : g_nv_loc_len.ds1_len;
    p_gdata->upgrade_ds_signature = 0;

    WICED_BT_TRACE("Active: 0x%08X, Upgrade: 0x%08X, UG length: 0x%08X", p_gdata->active_ds_location, p_gdata->upgrade_ds_location, p_gdata->upgrade_ds_length);
      // TODO
//    Config_DS_End_Location = p_gdata->active_ds_location +
//                                    ((p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
//                                      g_nv_loc_len.ds1_len : g_nv_loc_len.ds2_len);

    p_gdata->bytes_erased = 0;

    return 1;
}

wiced_bool_t wiced_firmware_upgrade_erase_nv(uint32_t start, uint32_t size)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t sector_size = sfi_sectorErase256K ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
    uint32_t offset;

    if (start % sector_size)
        return WICED_FALSE;

    for (offset = 0; offset < size; offset += sector_size)
        fw_upgrade_erase_sector(start + offset + g_fw_upgrade.upgrade_ds_location);

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
    if (g_nvram_intf == NVRAM_INTF_SERIAL_FLASH)
    {
        uint32_t sector_size = sfi_sectorErase256K ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
        uint8_t  skip_len = 0;

        // if this is a beginning of a new sector erase first.
        if ((offset % sector_size) == 0 && !fw_upgrade_is_sector_erased(offset))
        {
            fw_upgrade_erase_sector(offset + g_fw_upgrade.upgrade_ds_location);
        }
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        if (offset == 0)
        {
            memcpy(&g_fw_upgrade.upgrade_ds_signature, data, 4);
            offset += 4;
            data += 4;
            len -= 4;
            skip_len = 4;
        }

        // The real offset into the NV is the current offset + the upgrade DS location.
        offset += g_fw_upgrade.upgrade_ds_location;
        return (skip_len + fw_upgrade_write_mem(offset, data, len));
    }
    return 0;
}

// Retrieve chunk of data from the physical NV storage medium. if success returns len, else return 0
uint32_t wiced_firmware_upgrade_retrieve_from_nv(uint32_t offset, uint8_t *data, uint32_t len)
{
    if (g_nvram_intf == NVRAM_INTF_SERIAL_FLASH)
    {
        uint8_t  skip_len = 0;
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        if (offset == 0)
        {
            memcpy(data, &g_fw_upgrade.upgrade_ds_signature, 4);
            offset += 4;
            data += 4;
            len -= 4;
            skip_len = 4;
        }

        // The real offset into the NV is the current offset + the upgrade DS location.
        offset += g_fw_upgrade.upgrade_ds_location;
        return ( skip_len + fw_upgrade_read_mem(offset, data, len) );
    }

    return 0;
}

// After download is completed and verified this function is
// called to switch active partitions with the one that has been
// receiving the new image.
void wiced_firmware_upgrade_finish(void)
{
    if (!firmware_upgrade_switch_active_ds())
    {
        WICED_BT_TRACE("No fail safe OTA patch. Cannot upgrade firmware\n");
        // just maybe print message but don't return, still fall through to reset
    }

    WICED_BT_TRACE("FW upgrade completed\n");

    wiced_hal_wdog_reset_system();
    // End of the world - will not return.
}

/* internal utility functions */
uint8_t firmware_upgrade_switch_active_ds(void)
{
    if (g_nvram_intf == NVRAM_INTF_SERIAL_FLASH)
    {
        return firmware_upgrade_switch_sflash_active_ds();
    }
    return 0;
}

uint8_t firmware_upgrade_switch_sflash_active_ds(void)
{
    uint32_t sector_size = sfi_sectorErase256K ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
    uint32_t signature = 0;

    // commit reserved first 4 bytes of download to complete
    // this is done last and after crc in case of power loss during download
    // boot rom checks this signature to validate DS, checking DS1 first, then DS2
    wiced_hal_sflash_write(g_fw_upgrade.upgrade_ds_location, 4, (uint8_t *)&g_fw_upgrade.upgrade_ds_signature);

    // check that the write completed
    wiced_hal_sflash_read(g_fw_upgrade.upgrade_ds_location, 4, (uint8_t *)&signature);
    if (signature != g_fw_upgrade.upgrade_ds_signature)
    {
        return 0;
    }

    // clear first active DS sector in sflash, so that on next boot, CRC check will fail and ROM code boots from upgraded DS
    wiced_hal_sflash_erase(g_config_Info.active_ds_base, sector_size);
    return 1;
}

//Erases the Serial Flash Sector
void fw_upgrade_erase_sector(uint32_t erase_addr)
{
    uint32_t sector_size = sfi_sectorErase256K ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;
    wiced_hal_sflash_erase(erase_addr, sector_size);
}

//Reads the given length of data from SF/EEPROM. If success returns len, else returns 0
uint32_t fw_upgrade_read_mem(uint32_t read_from, uint8_t *buf, uint32_t len)
{
    // WICED_BT_TRACE("sflash_read from:%x len:%d\n", read_from, len);
    if (wiced_hal_sflash_read(read_from, len, buf) != len)
    {
        WICED_BT_TRACE("sflash_read failed\n");
        return 0;
    }
    return len;
}

// Writes the given length of data to SF. If success returns len, else returns 0
uint32_t fw_upgrade_write_mem(uint32_t write_to, uint8_t *data, uint32_t len)
{
    WICED_BT_TRACE("sflash_write to:%x len:%d\n", write_to, len);
    return wiced_hal_sflash_write(write_to, len, data);
}

#define PARTITION_ACTIVE    0
#define PARTITION_UPGRADE   1

void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    if (partition == PARTITION_ACTIVE)
    {
        fw_upgrade_read_mem(offset + g_config_Info.active_ds_base, p_data, data_len);
    }
    else
    {
        wiced_firmware_upgrade_retrieve_from_nv(offset, p_data, data_len);
    }
}

uint32_t wiced_bt_get_nv_sector_size()
{
    uint32_t sector_size = sfi_sectorErase256K ? WICED_FW_UPGRADE_SF_SECTOR_SIZE_256K : WICED_FW_UPGRADE_SF_SECTOR_SIZE_4K;

    return sector_size;
}
#endif // CYW20735B1
