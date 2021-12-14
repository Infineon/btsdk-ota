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

#include "bt_types.h"
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_hal_eflash.h"
#ifdef OTA_FW_UPGRADE_SFLASH_COPY
#include "wiced_hal_sflash.h"
#endif
#include "wiced_platform.h"
#include "spar_utils.h"
#include "ofu_ds2.h"
#include "wiced_hal_puart.h"

extern GIVES void* dynamic_memory_AllocatePermanent(    UINT32 size_bytes,
                                                        BOOL32 allow_use_by_minidriver );

void ds2_app_entry(void);
UINT32 wiced_firmware_update_copy_sflash_start(int stage);
void wiced_firmware_update_copy_sflash(void);
wiced_result_t read_ofu_data(uint32_t offset, uint8_t *buffer, uint32_t dest, uint32_t len, void *handle);
void print_hex_buffer(uint8_t *buf, uint32_t len);

// option to leave room at start of sflash for VS or app specific data
#if !defined(OTA_FW_UPGRADE_EFLASH_COPY) && !defined(OFU_UPGRADE_IMAGE_SFLASH_OFFSET)
#define OFU_UPGRADE_IMAGE_SFLASH_OFFSET 0
#endif

#ifdef OTA_ENCRYPT_SFLASH_DATA
  //#define OFU_CRYPT_TYPE OFU_CRYPT_TYPE_AES_CFB128
  #define OFU_CRYPT_TYPE OFU_CRYPT_TYPE_AES_CTR
#else
  #define OFU_CRYPT_TYPE OFU_CRYPT_TYPE_NONE
#endif

// define GPIO assignments for puart and spi2
#if defined(CYBT_213043_MESH)
    #define GPIO_DS2_PUART_TXD WICED_P00
    #define GPIO_DS2_PUART_RXD WICED_P28
    #define GPIO_DS2_SPI2_MOSI WICED_P10
    #define GPIO_DS2_SPI2_CLK  WICED_P02
    #define GPIO_DS2_SPI2_CS   WICED_P15
    #define GPIO_DS2_SPI2_MISO WICED_P08
#elif defined(CYBT_213043_EVAL)
    #define GPIO_DS2_PUART_TXD WICED_P32
    #define GPIO_DS2_PUART_RXD WICED_P37
#elif defined(CYBT_223058_EVAL)
    #define GPIO_DS2_PUART_TXD WICED_P32
    #define GPIO_DS2_PUART_RXD WICED_P37
#elif defined (CYBT_263065_EVAL) || defined (CYBT_273063_EVAL)
	#define GPIO_DS2_PUART_TXD WICED_P03
    #define GPIO_DS2_PUART_RXD WICED_P37
#elif defined(CYBT_243053_EVAL)
    #define GPIO_DS2_PUART_TXD WICED_P32
    #define GPIO_DS2_PUART_RXD WICED_P37
#elif defined(CYBT_253059_EVAL)
    #define GPIO_DS2_PUART_TXD WICED_P32
    #define GPIO_DS2_PUART_RXD WICED_P37
#elif defined(CYW920819EVB_02) || defined(CYW920820EVB_02) || defined(CYW989820EVB_01)
    #define GPIO_DS2_PUART_TXD WICED_P32
    #define GPIO_DS2_PUART_RXD WICED_P37
    #define GPIO_DS2_SPI2_MOSI WICED_P06
    #define GPIO_DS2_SPI2_CLK  WICED_P09
    #define GPIO_DS2_SPI2_CS   WICED_P11
    #define GPIO_DS2_SPI2_MISO WICED_P17
#elif defined(CYW920819REF_KB_01) || defined(CYW920819REF_MS_01) || defined(CYW920819REF_RM_01)
    #define GPIO_DS2_PUART_TXD WICED_P31
    #define GPIO_DS2_PUART_RXD 0
#else
    #error Unknown PLATFORM_xxx
#endif

#ifndef UART_RX
#define UART_RX GPIO_DS2_PUART_RXD
#endif
#ifndef UART_TX
#define UART_TX GPIO_DS2_PUART_TXD
#endif
#ifndef SPI2_MOSI
#define SPI2_MOSI GPIO_DS2_SPI2_MOSI
#endif
#ifndef SPI2_CLK
#define SPI2_CLK GPIO_DS2_SPI2_CLK
#endif
#ifndef SPI2_CS
#define SPI2_CS GPIO_DS2_SPI2_CS
#endif
#ifndef SPI2_MISO
#define SPI2_MISO GPIO_DS2_SPI2_MISO
#endif

#define DATA_AT_DS2 __attribute__((section (".rodata.ds2"),used,nocommon))
#define CODE_AT_DS2 __attribute__((section (".text.ds2"),used))

/* create a DS2 header, it just calls into the entry function (no patch code) */
typedef struct __attribute__((__packed__)) tag_ds_struct
{
    char signature[8];
    uint32_t fill;
    uint32_t size;
    uint8_t call[3];
    void (*entry_function)(void);
    uint8_t terminator[3];
} ds_struct_t;

DATA_AT_DS2 const static ds_struct_t ds2 =
{
    .signature = "BRCMcfgD",
    .fill = 0,
    .size = 10,
    .call = {0x06,0x01,0x04},
    .entry_function = ds2_app_entry,
    .terminator = {0xfe, 0x00, 0x00},
};

extern UINT32 (*boot_init_PostConfigReplacement)(int stage);
void ds2_app_entry(void)
{
    wiced_ofu_set_post_config_callback(wiced_firmware_update_copy_sflash_start);
}

__attribute__((naked)) CODE_AT_DS2 void switch_stack (uint8_t *pStackBottom)
{
    /* Set SP to the passed (in R0) address of the end of an array buffer and return. */
    __asm__("MOV     SP, R0");
    __asm__("MOV     PC, LR");
}

#define LOCAL_STACK_SIZE 0xC00
UINT32 wiced_firmware_update_copy_sflash_start(int stage)
{
    if(stage == 4)
    {
        uint8_t *stack = (uint8_t *)dynamic_memory_AllocatePermanent(LOCAL_STACK_SIZE, 0);
        if(stack != 0)
        {
            // we aren't coming back...
            switch_stack(&stack[LOCAL_STACK_SIZE - 1]);
            wiced_firmware_update_copy_sflash();
        }
    }
    return 0;
}

void wiced_firmware_update_copy_sflash()
{
    uint8_t *buffer, *first_buffer;
    uint32_t ofu_size = 0;
    int32_t offset = 0;
    uint32_t write_base = wiced_ofu_get_ds1_offset();
#if defined(OTA_FW_UPGRADE_EFLASH_COPY)
    uint32_t read_base = write_base + ((DS2_LOCATION - DS_LOCATION)/2);
#else
    uint32_t read_base = OFU_UPGRADE_IMAGE_SFLASH_OFFSET;
#endif
    void *secure = NULL;
    // set up uart for debug print
    wiced_hal_puart_init();

    wiced_hal_puart_select_uart_pads(UART_RX, UART_TX, 0, 0);
#if defined(OTA_FW_UPGRADE_SFLASH_COPY)
    wiced_hal_gpio_select_function(SPI2_MOSI, WICED_SPI_2_MOSI);
    wiced_hal_gpio_select_function(SPI2_CLK,  WICED_SPI_2_CLK);
    wiced_hal_gpio_select_function(SPI2_CS,   WICED_SPI_2_CS);
    wiced_hal_gpio_select_function(SPI2_MISO, WICED_SPI_2_MISO);

    wiced_hal_puart_print("!!!!! wiced_firmware_update_copy_flash\n");
#endif
    wiced_ofu_pet_watchdog();

#if !defined(OTA_FW_UPGRADE_EFLASH_COPY)
    // set up sflash via spi2
    wiced_ofu_sflash_init(WICED_OFU_DEFAULT_SPI_CLK);
#endif

    do
    {
        buffer = dynamic_memory_AllocatePermanent(FLASH_SECTOR_SIZE, 0);
        if(NULL == buffer)
            break;

        first_buffer = dynamic_memory_AllocatePermanent(FLASH_SECTOR_SIZE, 0);
        if(NULL == first_buffer)
            break;

#ifdef OTA_ENCRYPT_SFLASH_DATA
        // set up decryption
        ofu_size = wiced_ofu_get_external_storage_context_size();
        secure = dynamic_memory_AllocatePermanent(ofu_size,0);
        if(NULL == secure)
        {
            wiced_hal_puart_print("!!!!! bad alloc for secure context\n");
            break;
        }
        if(!wiced_ofu_restore_external_storage_key(secure))
        {
            wiced_hal_puart_print("!!!!! could not restore secure context\n");
            break;
        }
        wiced_hal_puart_print("!!!!! got external storage key\n");
#endif
        // when DS1 is invalidated, information needed by DS2 app is used to overwrite signature
        // read in first block from eflash and get total image size
        wiced_hal_puart_print("!!!!! read eflash\n");
        if(WICED_SUCCESS != wiced_hal_eflash_read(write_base, buffer, FLASH_SECTOR_SIZE))
        {
            wiced_hal_puart_print("!!!!! bad eflash read\n");
            break;
        }
        memcpy((uint8_t *)&ofu_size, buffer, sizeof(ofu_size));
        if((ofu_size < 0x1a) || (ofu_size > FLASH_SIZE))
        {
            wiced_hal_puart_print("!!!!! bad update length read from eflash\n");
            break;
        }

        wiced_hal_puart_print("!!!!! image length\n");

        // start at beginning, but don't commit until after the rest is copied
        if(WICED_SUCCESS != read_ofu_data(offset + read_base, first_buffer, 0, FLASH_SECTOR_SIZE, secure))
        {
            wiced_hal_puart_print("!!!!! bad source flash read, 1st block\n");
            break;
        }
        offset += FLASH_SECTOR_SIZE;

        while(offset < ofu_size)
        {
            uint32_t len = ofu_size - offset;
            if(len > FLASH_SECTOR_SIZE)
            {
                len = FLASH_SECTOR_SIZE;
            }
            if(WICED_SUCCESS != read_ofu_data(offset + read_base, buffer, 0, len, secure))
            {
                wiced_hal_puart_print("!!!!! bad source flash read\n");
                break;
            }
            if(WICED_SUCCESS != wiced_hal_eflash_erase(write_base+offset, FLASH_SECTOR_SIZE))
            {
                wiced_hal_puart_print("!!!!! bad erase\n");
                break;
            }
            if(WICED_SUCCESS != wiced_hal_eflash_write(write_base+offset, buffer, len))
            {
                wiced_hal_puart_print("!!!!! bad write\n");
                break;
            }
            offset += FLASH_SECTOR_SIZE;
           // wiced_hal_puart_print("!!!!! wrote DS1 chunk\n");
        }
        // did we make it to the end?
        if(offset < ofu_size)
        {
            wiced_hal_puart_print("!!!!! failed to copy image data to end\n");
            break;
        }
        // commit first block now
        if(WICED_SUCCESS != wiced_hal_eflash_erase(write_base, FLASH_SECTOR_SIZE))
        {
            wiced_hal_puart_print("!!!!! bad erase\n");
            break;
        }
        if(WICED_SUCCESS != wiced_hal_eflash_write(write_base, first_buffer, FLASH_SECTOR_SIZE))
        {
            wiced_hal_puart_print("!!!!! bad write\n");
            break;
        }

        // the data is committed to eflash a page at a time, cannot just commit signature at end
        wiced_hal_puart_print("!!!!! all data transferred to eflash\n");

        wiced_ofu_delete_external_storage_key(secure);
        wiced_ofu_reset_device();
    } while(0);

    wiced_hal_puart_print("!!!!! ERROR exiting wiced_firmware_update_copy_sflash\n");
}

wiced_result_t read_ofu_data(uint32_t offset, uint8_t *buffer, uint32_t dest, uint32_t len, void *handle)
{
    wiced_result_t rc = WICED_SUCCESS;
    wiced_ofu_pet_watchdog();
    // copy to eflash block by block and continue with remaining
#if defined(OTA_FW_UPGRADE_EFLASH_COPY)
    if(WICED_SUCCESS != wiced_hal_eflash_read(offset, buffer, len))
#else
    if(len != wiced_hal_sflash_read(offset, len, buffer))
#endif
    {
        wiced_hal_puart_print("!!!!! bad source flash read\n");
        rc = WICED_ERROR;
    }
#ifdef OTA_ENCRYPT_SFLASH_DATA
    else
    {
        // decrypt block
        if(!wiced_ofu_crypt(WICED_FALSE, offset, len, buffer, buffer, handle))
        {
            wiced_hal_puart_print("!!!!! bad decrypt\n");
            rc = WICED_ERROR;
        }
    }
#endif
    return rc;
}


#if 1
void print_hex_buffer(uint8_t *buf, uint32_t len)
{
    uint8_t hex_char;
    uint32_t i;
    for(i = 0; i < len; i++)
    {
        hex_char = buf[i] >> 4;
        if(hex_char <= 9)
            hex_char += '0';
        else
            hex_char = 'a' + hex_char - 10;
        wiced_hal_puart_write(hex_char);
        hex_char = buf[i] & 0xf;
        if(hex_char <= 9)
            hex_char += '0';
        else
            hex_char = 'a' + hex_char - 10;
        wiced_hal_puart_write(hex_char);
        wiced_hal_puart_write(0x20);
    }
}
#endif
