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

#if CYW20819A1 || CYW20820A1
#include "bt_types.h"

#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_hal_eflash.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#ifdef ENABLE_SFLASH_UPGRADE
#include "wiced_hal_sflash.h"
#endif
#include "ota_fw_upgrade.h"
#include "ofu_ds2.h"
#include "clock_timer.h"

/******************************************************
 *                      Constants
 ******************************************************/
#ifdef ENABLE_SFLASH_UPGRADE
#define WICED_FW_UPGRADE_SF_SECTOR_SIZE   OTA_SFLASH_SECTOR_SIZE //Serial Flash Sector size
#define WICED_FW_UPGRADE_SF_START       0
#endif

#define EF_BASE_ADDR  FLASH_BASE_ADDRESS
#define EF_PAGE_SIZE  FLASH_SECTOR_SIZE

/******************************************************
 *                     Structures
 ******************************************************/

typedef enum {
    FW_UPDATE_INTF_EFLASH,
    FW_UPDATE_INTF_SFLASH
} FW_UPDATE_INTF_t;

//ws_upgrade global data
typedef struct
{
    uint32_t active_ds_location;
    uint32_t upgrade_ds_location;
    uint32_t upgrade_ds_length;
    uint32_t upgrade_type;
    uint32_t upgrade_ds_signature;
    uint32_t upgrade_bytes_written;

    uint32_t erase_start;
    uint32_t bytes_erased;
} wiced_fw_upgrade_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
/********************************************************************************************************
* Recommended firmware upgrade 1 M byte eflash offsets
 * -------------------------------------------------------------------------------------------------------------------
 * |  SS (4K @ 0)  |  VS1 (4K @ 0x1000)  | VS2 (4K @ 0x2000)  | DS1 (506K @ 0x3000)  | DS2 (506K @ 0x81800)
 *  -------------------------------------------------------------------------------------------------------------------
 *******************************************************************************************************/
wiced_fw_upgrade_nv_loc_len_t   g_nv_loc_len;

#ifdef ENABLE_SFLASH_UPGRADE
uint32_t                        g_wiced_sflash_size;
#endif

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

#ifdef ENABLE_SFLASH_UPGRADE
extern uint32_t             Config_DS_End_Location;
#endif

/* internal functions */
uint8_t     firmware_upgrade_switch_active_ds(void);
uint8_t     firmware_upgrade_switch_eflash_active_ds(void);

#ifdef ENABLE_SFLASH_UPGRADE
uint8_t     firmware_upgrade_switch_sflash_active_ds(void);
uint32_t    fw_upgrade_write_mem(uint32_t write_to, uint8_t *data, uint32_t len);
uint32_t    fw_upgrade_read_mem(uint32_t read_from, uint8_t *buf, uint32_t len);
void fw_upgrade_erase_sector(uint32_t erase_addr);
#endif


#ifdef ENABLE_WICED_FW_DEBUG
UINT8 first_256_byte_dump[256];
#endif

#ifdef OTA_ENCRYPT_SFLASH_DATA
    void *secure = NULL;
    uint32_t ofu_secure_size = 0;
    uint32_t next_aes_ctr_block_storage = 0;
    //#define OFU_CRYPT_TYPE OFU_CRYPT_TYPE_AES_CFB128
    #define OFU_CRYPT_TYPE OFU_CRYPT_TYPE_AES_CTR
    #if (OFU_CRYPT_TYPE == OFU_CRYPT_TYPE_AES_CTR)
        // use fixed block lengths
        // encrypt sequence number == block offset
    #endif
#endif


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

    WICED_BT_TRACE("ds2:%x\n", g_nv_loc_len.ds2_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds2_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

#ifdef ENABLE_SFLASH_UPGRADE
    g_wiced_sflash_size = sflash_size;
#ifdef OTA_FW_UPGRADE_SFLASH_COPY
    wiced_ofu_sflash_init(WICED_OFU_DEFAULT_SPI_CLK);
#endif
#endif

#ifdef OTA_ENCRYPT_SFLASH_DATA
    if(NULL == secure)
    {
        ofu_secure_size = wiced_ofu_get_external_storage_context_size();
        secure = (void *)  wiced_bt_get_buffer(ofu_secure_size);
        if(NULL == secure)
        {
            WICED_BT_TRACE("could not allocate secure context size %d\n", ofu_secure_size);
            return WICED_FALSE;
        }
    }
#endif

#ifndef OTA_FW_UPGRADE_SFLASH_COPY
    if ((g_config_Info.active_ds_base != p_sflash_nv_loc_len->ds1_loc) &&
        (g_config_Info.active_ds_base != p_sflash_nv_loc_len->ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("WARNING: Upgrade will fail - active DS is not one of the expected locations\n");
        WICED_BT_TRACE("WARNING: Are ConfigDSLocation and DLConfigVSLocation in the .btp set up as in nv_loc_len[]?\n");
        return WICED_FALSE;
    }
#endif
    return WICED_TRUE;
}


// setup NVRAM locations to be used during upgrade. if success returns 1, else fails return 0
uint32_t wiced_firmware_upgrade_init_nv_locations(void)
{
    wiced_fw_upgrade_t  *p_gdata = &g_fw_upgrade;

#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("g_nv_ss:%x\n",  g_nv_loc_len.ss_loc);
    WICED_BT_TRACE("g_nv_vs:%x\n",  g_nv_loc_len.vs1_loc);
    WICED_BT_TRACE("g_nv_ds1:%x\n", g_nv_loc_len.ds1_loc);
    WICED_BT_TRACE("g_nv_ds2:%x\n", g_nv_loc_len.ds2_loc);

    WICED_BT_TRACE("g_cfg_ss:%x\n",  g_config_Info.ss_base);
    WICED_BT_TRACE("g_cfg_ds1:%x\n", g_config_Info.active_ds_base);

    WICED_BT_TRACE("g_cfg_vs1:                  %x\n",  g_config_Info.layout.vs_copy1_base);
    WICED_BT_TRACE("g_cfg_failsafe_ds_base      %x\n",  g_config_Info.layout.failsafe_ds_base  );
    WICED_BT_TRACE("g_cfg_upgradable_ds_base    %x\n",  g_config_Info.layout.upgradable_ds_base);
    WICED_BT_TRACE("g_cfg_vs_copy1_base         %x\n",  g_config_Info.layout.vs_copy1_base     );
    WICED_BT_TRACE("g_cfg_vs_copy2_base         %x\n",  g_config_Info.layout.vs_copy2_base     );
    WICED_BT_TRACE("g_cfg_vs_length_per_copy    %x\n",  g_config_Info.layout.vs_length_per_copy);
    WICED_BT_TRACE("g_cfg_vs_block_size         %x\n",  g_config_Info.layout.vs_block_size     );
    WICED_BT_TRACE("g_cfg_media_page_size       %x\n",  g_config_Info.layout.media_page_size   );
#endif

    if ((g_config_Info.active_ds_base != g_nv_loc_len.ds1_loc) &&
        (g_config_Info.active_ds_base != g_nv_loc_len.ds2_loc))
    {
        // If active DS is neither DS1 nor DS2 we expect to see, fail the download.
        WICED_BT_TRACE("Active DS %x is not DS1:%x and not DS2:%x. Cannot upgrade.\n", g_config_Info.active_ds_base, g_nv_loc_len.ds1_loc, g_nv_loc_len.ds2_loc);
        return 0;
    }
#ifdef OTA_FW_UPGRADE_SFLASH_COPY
    g_fw_upgrade.upgrade_type = FW_UPDATE_INTF_SFLASH;
    p_gdata->active_ds_location = g_nv_loc_len.ds1_loc;
    p_gdata->upgrade_ds_location = WICED_FW_UPGRADE_SF_START;
    p_gdata->upgrade_ds_length   = g_nv_loc_len.ds1_len;
#else
    p_gdata->active_ds_location = g_config_Info.active_ds_base;
#ifdef OTA_FW_UPGRADE_EFLASH_COPY
    p_gdata->upgrade_ds_location = p_gdata->active_ds_location + (g_nv_loc_len.ds1_len / 2);
    p_gdata->upgrade_ds_length   = g_nv_loc_len.ds1_len / 2;
#else
    p_gdata->upgrade_ds_location = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_loc : g_nv_loc_len.ds1_loc;
    p_gdata->upgrade_ds_length   = (p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                    g_nv_loc_len.ds2_len : g_nv_loc_len.ds1_len;
#endif
    WICED_BT_TRACE("Active: 0x%08X, Upgrade: 0x%08X, UG length: 0x%08X", p_gdata->active_ds_location, p_gdata->upgrade_ds_location, p_gdata->upgrade_ds_length);

#ifdef ENABLE_SFLASH_UPGRADE
    g_fw_upgrade.upgrade_type = FW_UPDATE_INTF_SFLASH;
    Config_DS_End_Location = p_gdata->active_ds_location +
                                    ((p_gdata->active_ds_location == g_nv_loc_len.ds1_loc) ?
                                      g_nv_loc_len.ds1_len : g_nv_loc_len.ds2_len);
#else
    g_fw_upgrade.upgrade_type = FW_UPDATE_INTF_EFLASH;
#endif
#endif
    p_gdata->upgrade_ds_signature = 0;
    p_gdata->upgrade_bytes_written = 0;
    p_gdata->bytes_erased = 0;

    return 1;
}

wiced_bool_t wiced_firmware_upgrade_erase_nv(uint32_t start, uint32_t size)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t offset = 0;

    if (start % EF_PAGE_SIZE)
        return WICED_FALSE;

    if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
    {
        wiced_ofu_enter_eflash_write_or_erase();
        for (offset = 0; offset < size; offset += EF_PAGE_SIZE)
        {
            wiced_hal_eflash_erase(start + offset + g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
        }
        wiced_ofu_leave_eflash_write_or_erase();
    }
#ifdef ENABLE_SFLASH_UPGRADE
    else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
    {
        for (offset = 0; offset < size; offset += WICED_FW_UPGRADE_SF_SECTOR_SIZE)
        {
            fw_upgrade_erase_sector(start + offset);
        }
    }
#endif

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
    if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
    {
        // if this is a beginning of a new sector erase first.
        if ((offset % EF_PAGE_SIZE) == 0 && !fw_upgrade_is_sector_erased(offset))
        {
            wiced_ofu_enter_eflash_write_or_erase();
            wiced_hal_eflash_erase(offset + g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
            wiced_ofu_leave_eflash_write_or_erase();
        }
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        // when storing image in DS1b (eflash copy), go ahead and write image signature
#ifndef OTA_FW_UPGRADE_EFLASH_COPY
        if(offset == 0)
        {
            memcpy(&g_fw_upgrade.upgrade_ds_signature, data, 4);
            /* 819 ocf allows only page writes, so invalidate first 4 bytes before commit */
            *(data + 0) = 0xff;
            *(data + 1) = 0xff;
            *(data + 2) = 0xff;
            *(data + 3) = 0xff;
        }
#endif
        offset += (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR);

        //WICED_BT_TRACE("write: offset:%x len:%d\n", offset, len);
        wiced_ofu_enter_eflash_write_or_erase();
        if (wiced_hal_eflash_write(offset, data, len) == WICED_SUCCESS)
        {
            g_fw_upgrade.upgrade_bytes_written += len;
        }
        else
        {
            WICED_BT_TRACE("write: failed offset:%x len:%d\n", offset, len);
            len = 0;
        }
        wiced_ofu_leave_eflash_write_or_erase();
        return len;
    }

#ifdef ENABLE_SFLASH_UPGRADE
    else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
    {
        uint32_t written;
        // The real offset into the NV is the current offset + the upgrade DS location.
        offset += g_fw_upgrade.upgrade_ds_location;

#ifdef OTA_ENCRYPT_SFLASH_DATA
        // we could be restarting a new write after abort, so set up new key here
        // starting a write at upgrade_ds_location indicates a new key is needed
        if(offset == g_fw_upgrade.upgrade_ds_location)
        {
            wiced_ofu_new_external_storage_key(WICED_TRUE, OFU_CRYPT_TYPE, secure);
            wiced_ofu_store_external_storage_key(secure);
        }
        #if OFU_CRYPT_TYPE == OFU_CRYPT_TYPE_AES_CTR
        if(offset % OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT)
        {
            WICED_BT_TRACE("!!! invalid write offset for OFU_CRYPT_TYPE_AES_CTR\n");
            return 0;
        }
        if(len > OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT)
        {
            WICED_BT_TRACE("!!! invalid write length for OFU_CRYPT_TYPE_AES_CTR\n");
            return 0;
        }
        #endif
        if(!wiced_ofu_crypt(WICED_TRUE, offset, len, data, data, secure))
        {
            WICED_BT_TRACE("!!! bad encryption\n");
            return 0;
        }
#endif
        // if this is a beginning of a new sector erase first.
        if ((offset % WICED_FW_UPGRADE_SF_SECTOR_SIZE) == 0 && !fw_upgrade_is_sector_erased(offset))
        {
            fw_upgrade_erase_sector(offset);
        }
        written = fw_upgrade_write_mem(offset, data, len);
        g_fw_upgrade.upgrade_bytes_written += written;
        return written;
    }
#endif

    return 0;
}

// Retrieve chunk of data from the physical NV storage medium. if success returns len, else return 0
uint32_t wiced_firmware_upgrade_retrieve_from_nv(uint32_t offset, uint8_t *data, uint32_t len)
{
    if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
    {
        // reserve first 4 bytes of download to commit when complete, in case of unexpected power loss
        // boot rom checks this signature to validate DS
        offset += (g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR);

        if (wiced_hal_eflash_read(offset, data, len) == WICED_SUCCESS)
        {
            // when storing image in DS1b (eflash copy), image signature is in flash
#ifndef OTA_FW_UPGRADE_EFLASH_COPY
            if( offset == ( g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR ) )
            {
                memcpy(data, &g_fw_upgrade.upgrade_ds_signature, 4);
            }
#endif
            return len;
        }
        else
        {
            WICED_BT_TRACE("failed to read offset:%x\n", offset);
            return 0;
        }
    }

#ifdef ENABLE_SFLASH_UPGRADE
    else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
    {
        uint32_t read = 0;
        // The real offset into the NV is the current offset + the upgrade DS location.
        offset += g_fw_upgrade.upgrade_ds_location;
#ifdef OTA_ENCRYPT_SFLASH_DATA
        // starting a read at upgrade_ds_location indicates we need to fetch previous key
        if(offset == g_fw_upgrade.upgrade_ds_location)
        {
            wiced_ofu_restore_external_storage_key(secure);
        }
        #if OFU_CRYPT_TYPE == OFU_CRYPT_TYPE_AES_CTR
        if(offset % OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT)
        {
            WICED_BT_TRACE("!!! invalid read offset for OFU_CRYPT_TYPE_AES_CTR\n");
            return 0;
        }
        if(len > OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT)
        {
            WICED_BT_TRACE("!!! invalid read length for OFU_CRYPT_TYPE_AES_CTR\n");
            return 0;
        }
        #endif
        read = fw_upgrade_read_mem(offset, data, len);
        if(read == len)
        {
            wiced_ofu_crypt(WICED_FALSE, offset, len, data, data, secure);
        }
#else
        read = fw_upgrade_read_mem(offset, data, len);
#endif
        return read;
    }
#endif

    return 0;
}

// After download is completed and verified this function is
// called to switch active partitions with the one that has been
// receiving the new image.
void wiced_firmware_upgrade_finish(void)
{
#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("before switch active ds:%x\n", g_fw_upgrade.active_ds_location);
    WICED_BT_TRACE("ss:\n");
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(0, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", g_nv_loc_len.ds1_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds1_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds2:%x\n", g_nv_loc_len.ds2_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds2_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

    if (!firmware_upgrade_switch_active_ds())
    {
#ifdef ENABLE_SFLASH_UPGRADE
        WICED_BT_TRACE("No fail safe OTA patch.\n");
#endif
        WICED_BT_TRACE("FAIL: cannot upgrade firmware\n");
        // just maybe print message but don't return, still fall through to reset
    }

    WICED_BT_TRACE("FW upgrade completed\n");

#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("after switch\n");
    WICED_BT_TRACE("ss:\n");
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(0, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds1:%x\n", g_nv_loc_len.ds1_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds1_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);

    WICED_BT_TRACE("ds2:%x\n", g_nv_loc_len.ds2_loc);
    memset(first_256_byte_dump, 0, 256);
    wiced_hal_eflash_read(g_nv_loc_len.ds2_loc - EF_BASE_ADDR, first_256_byte_dump, 256);
    dump_hex(first_256_byte_dump, 256);
#endif

#if defined(OTA_FW_UPGRADE_SFLASH_COPY) || defined(OTA_FW_UPGRADE_EFLASH_COPY)
    WICED_BT_TRACE("rebooting to DS2\n");
#endif
    wiced_ofu_reset_device();
    // End of the world - will not return.
}

/* internal utility functions */
uint8_t firmware_upgrade_switch_active_ds(void)
{
    if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
    {
        return firmware_upgrade_switch_eflash_active_ds();
    }
#ifdef ENABLE_SFLASH_UPGRADE
    else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
    {
        return firmware_upgrade_switch_sflash_active_ds();
    }
#endif
    return 0;
}

uint8_t firmware_upgrade_switch_eflash_active_ds(void)
{
#ifdef OTA_FW_UPGRADE_EFLASH_COPY
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t offset;
    uint8_t *ptr;

    // invalidate DS1 so reboot runs DS2 sflash copy to DS1
    WICED_BT_TRACE("firmware_upgrade_switch_eflash_active_ds\n");
    ptr =  wiced_bt_get_buffer(EF_PAGE_SIZE);
    if (ptr == NULL)
    {
#if ENABLE_WICED_FW_DEBUG
        WICED_BT_TRACE(" fw upgrade fail!! No resources to switch to upgraded firmware \n");
#endif
        return 0;
    }
    // 819 onchip flash allows only page writes, so read the page and update first 4 bytes with signature.
    offset = g_fw_upgrade.active_ds_location - EF_BASE_ADDR;
    WICED_BT_TRACE("reading active DS1 signature from %06x\n", offset);
    wiced_hal_eflash_read(offset, ptr, EF_PAGE_SIZE);

    // store actual number of bytes written for ds2 copy app
    memcpy(ptr, (uint8_t *)&g_fw_upgrade.upgrade_bytes_written, sizeof(g_fw_upgrade.upgrade_bytes_written));

    // write the whole page now with invalid signature; this will cause reboot to DS2
    WICED_BT_TRACE("writing to invalidate DS1 signature, upgrade image length %06x\n", offset);
    wiced_ofu_enter_eflash_write_or_erase();
    wiced_hal_eflash_erase(offset, EF_PAGE_SIZE);
    wiced_hal_eflash_write(offset, ptr, EF_PAGE_SIZE);
    wiced_ofu_leave_eflash_write_or_erase();
#else
    wiced_result_t result;
    uint32_t signature = 0;
    uint8_t *ptr;

    // commit reserved first 4 bytes of download to complete
    // this is done last and after crc in case of power loss during download
    // boot rom checks this signature to validate DS, checking DS1 first, then DS2

    ptr =  wiced_bt_get_buffer(EF_PAGE_SIZE);
    if (ptr == NULL)
    {
#if ENABLE_WICED_FW_DEBUG
        WICED_BT_TRACE(" fw upgrade fail!! No resources to switch to upgraded firmware \n");
#endif
        return 0;
    }

    // 819 onchip flash allows only page writes, so read the page and update first 4 bytes with signature.
#if ENABLE_WICED_FW_DEBUG
    WICED_BT_TRACE("Update signature at %06x with %08x\n", g_fw_upgrade.upgrade_ds_location, g_fw_upgrade.upgrade_ds_signature);
#endif
    wiced_hal_eflash_read(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, ptr, EF_PAGE_SIZE);
    memcpy(ptr, (uint8_t *)&g_fw_upgrade.upgrade_ds_signature, 4);
    signature = g_fw_upgrade.upgrade_ds_signature;

    // write the whole page now with signature
    wiced_ofu_enter_eflash_write_or_erase();
    wiced_hal_eflash_erase(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
    wiced_hal_eflash_write(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, ptr, EF_PAGE_SIZE);
    wiced_ofu_leave_eflash_write_or_erase();

    // check that the write completed
    memset(ptr, 0, EF_PAGE_SIZE);
    wiced_hal_eflash_read(g_fw_upgrade.upgrade_ds_location - EF_BASE_ADDR, ptr, EF_PAGE_SIZE);
    if(signature != *(uint32_t *)ptr)
    {
        wiced_bt_free_buffer(ptr);
#if ENABLE_WICED_FW_DEBUG
        WICED_BT_TRACE("Read back of DS signature failed\n");
#endif
        return 0;
    }

    wiced_bt_free_buffer(ptr);

    // clear first active DS sector in eflash, so that on next boot, CRC check will fail and ROM code boots from upgraded DS
    wiced_ofu_enter_eflash_write_or_erase();
    result = wiced_hal_eflash_erase(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
    wiced_ofu_leave_eflash_write_or_erase();
    //WICED_BT_TRACE("active DS first sector erase result:%d active_ds:%x\n", result, g_config_Info.active_ds_base - EF_BASE_ADDR);
    UNUSED_VARIABLE(result);
#endif
    return 1;
}



#ifdef ENABLE_SFLASH_UPGRADE

uint8_t firmware_upgrade_switch_sflash_active_ds(void)
{
    wiced_fw_upgrade_t *p_gdata = &g_fw_upgrade;
    uint32_t offset;
    BOOL32 SS_updated = FALSE;
    uint8_t *ptr;

    // invalidate DS1 so reboot runs DS2 sflash copy to ocf
    WICED_BT_TRACE("firmware_upgrade_switch_sflash_active_ds\n");
    ptr =  wiced_bt_get_buffer(EF_PAGE_SIZE);
    if (ptr == NULL)
    {
#if ENABLE_WICED_FW_DEBUG
        WICED_BT_TRACE(" fw upgrade fail!! No resources to switch to upgraded firmware \n");
#endif
        return 0;
    }
   // 819 onchip flash allows only page writes, so read the page and update first 4 bytes with signature.
    WICED_BT_TRACE("reading active DS1 signature\n");
    wiced_hal_eflash_read(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, ptr, EF_PAGE_SIZE);
    // store actual number of bytes written for ds2 copy app
    memcpy(ptr, (uint8_t *)&g_fw_upgrade.upgrade_bytes_written, sizeof(g_fw_upgrade.upgrade_bytes_written));

    // write the whole page now with bad signature; this will cause reboot to DS2
    WICED_BT_TRACE("writing to invalidate DS1 signature\n");
    wiced_ofu_enter_eflash_write_or_erase();
    wiced_hal_eflash_erase(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, EF_PAGE_SIZE);
    wiced_hal_eflash_write(g_fw_upgrade.active_ds_location - EF_BASE_ADDR, ptr, EF_PAGE_SIZE);
    wiced_ofu_leave_eflash_write_or_erase();
    return 1;
}

//Erases the Serial Flash Sector
void fw_upgrade_erase_sector(uint32_t erase_addr)
{
    WICED_BT_TRACE("fw_upgrade_erase_sector:%x len:%d ts 0x%x\n",
                    erase_addr, WICED_FW_UPGRADE_SF_SECTOR_SIZE, clock_SystemTimeMicroseconds32_nolock());
    wiced_hal_sflash_erase(erase_addr,  WICED_FW_UPGRADE_SF_SECTOR_SIZE);
    WICED_BT_TRACE("   complete\n");

}

//Reads the given length of data from SF/EEPROM. If success returns len, else returns 0
uint32_t fw_upgrade_read_mem(uint32_t read_from, uint8_t *buf, uint32_t len)
{
    WICED_BT_TRACE("sflash_read from:%x len:%d\n", read_from, len);
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
    uint32_t rc;
    WICED_BT_TRACE("sflash_write to:%x len:%d\n", write_to, len);
    rc = wiced_hal_sflash_write(write_to, len, data);
    WICED_BT_TRACE("   complete\n");
    return rc;
}

#endif

#define PARTITION_ACTIVE    0
#define PARTITION_UPGRADE   1

void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    if (partition == PARTITION_ACTIVE)
    {
        if(g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
        {
            wiced_hal_eflash_read(g_config_Info.active_ds_base + offset - EF_BASE_ADDR, p_data, data_len);
        }
#ifdef ENABLE_SFLASH_UPGRADE
        else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
        {
            fw_upgrade_read_mem(g_config_Info.active_ds_base + offset, p_data, data_len);
        }
#endif
    }
    else if (partition == PARTITION_UPGRADE)
    {
#if OFU_CRYPT_TYPE == OFU_CRYPT_TYPE_AES_CTR
        uint8_t buf[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];
        uint32_t chunk_offset = offset % OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT;

        while (data_len)
        {
            uint16_t copy_len;

            copy_len = OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT - chunk_offset;
            if (copy_len > data_len)
                copy_len = data_len;

            wiced_firmware_upgrade_retrieve_from_nv(offset - chunk_offset, buf, OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT);
            memcpy(p_data, buf + chunk_offset, copy_len);
            p_data += copy_len;
            offset += OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT - chunk_offset;
            chunk_offset = 0;
            data_len -= copy_len;
        }
#else
        wiced_firmware_upgrade_retrieve_from_nv(offset, p_data, (data_len + 3) & 0xFFFFFFFC);
#endif
    }
}

uint32_t wiced_bt_get_nv_sector_size()
{
    uint32_t sector_size = 0;

    if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_EFLASH)
    {
        sector_size = EF_PAGE_SIZE;
    }
#ifdef ENABLE_SFLASH_UPGRADE
    else if (g_fw_upgrade.upgrade_type == FW_UPDATE_INTF_SFLASH)
    {
        sector_size = WICED_FW_UPGRADE_SF_SECTOR_SIZE;
    }
#endif

    return sector_size;
}
#endif // CYW20819 || CYW20820
