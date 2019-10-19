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
 */
#if (OTA_FW_UPGRADE_TYPE == OTS_OTA_FW_UPGRADE)
#include "bt_types.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_timer.h"
#include "ota_fw_upgrade.h"

extern ota_fw_upgrade_state_t  ota_fw_upgrade_common_state;

/******************************************************
 *                      Constants
 ******************************************************/

wiced_bool_t ota_fw_upgrade_initialized = WICED_FALSE;

/******************************************************
 *                      Function Definitions
 ******************************************************/
/*
 * Initializes global data
 */
void ota_fw_upgrade_init_data()
{
    ota_fw_upgrade_common_state.state = OTA_STATE_IDLE;
}

/*
 * Send the OTA firmware upgrade status to the peer
 */
wiced_result_t  wiced_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status )
{
    //Sending the status to peer is taken care within the OTS library
    return WICED_SUCCESS;
}

/*
 * To check if indication support is enabled
 */
wiced_bool_t wiced_ota_indication_enabled( void )
{
    return 0;
}
#endif
