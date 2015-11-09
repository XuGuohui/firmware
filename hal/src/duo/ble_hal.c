/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * BLE Proximity Reporter Sample Application
 *
 * Features demonstrated
 *  - WICED BT GATT server APIs
 *
 * On startup this demo:
 *  - Initializes the GATT subsystem
 *  - Begins advertising
 *  - Waits for GATT clients to connect
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 */

#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wlan_hal.h"
#include "ble_hal.h"


/******************************************************
 *                      Macros
 ******************************************************/


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_bt_dev_status_t ble_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

extern bool fetch_or_generate_setup_ssid(wiced_ssid_t* SSID);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_ssid_t device_name;
static bool bt_statck_init_done = false;

/* Stack and buffer pool configuration tables */
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************
 *               Function Definitions
 ******************************************************/
void bt_stack_init(void)
{
	if(!bt_statck_init_done)
	{
		fetch_or_generate_setup_ssid(&device_name);
		device_name.value[device_name.length] = '\0';

		wiced_bt_cfg_settings.device_name = device_name.value;

		/* Initialize Bluetooth controller and host stack */
		wiced_bt_stack_init( ble_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );

		while(!bt_statck_init_done)
		{
			ble_idle_event();
		}
	}
}

void ble_idle_event(void)
{
	wiced_rtos_delay_milliseconds( 10 );
}

/******************************************************
 *             Callback Function Definitions
 ******************************************************/
/* Bluetooth management event handler */
static wiced_bt_dev_status_t ble_management_cback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth controller and host stack enabled */
        if (p_event_data->enabled.status == WICED_BT_SUCCESS)
        {
            /* BT stack initialized */
        	bt_statck_init_done = true;
        }
        break;

    default:
        break;
    }

    return (status);
}
