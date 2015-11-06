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
#include "ble_provision_gatt_db.h"
#include "ble_provision.h"
#include "wlan_hal.h"
#include "rgbled.h"
#include "dct.h"
#include "deviceid_hal.h"
#include "flash_access.h"
#include "spark_macros.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define SCAN_RECORD_MAX_NUM		15

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum
{
	GapStatus_Connected,
	GapStatus_Disconnect,
}GapStatus;

typedef enum
{
	BLE_PROVISION_COMMAND_SCAN_REQUEST = 0xA0,
	BLE_PROVISION_COMMAND_SET_SECUR_KEY,
	BLE_PROVISION_COMMAND_SYSTEM_INFO,
}BLEProvisionCmd_t;

typedef enum
{
	BLE_PROVISION_STATUS_IDLE = 0xB0,
	BLE_PROVISION_STATUS_SCANNING,
	BLE_PROVISION_STATUS_SCAN_COMPLETE,
	BLE_PROVISION_STATUS_CONNECTING,
	BLE_PROVISION_STATUS_CONNECTED,
}BLEProvisionSta_t;

typedef enum
{
	BLE_PROVISION_PKT_LAST = 0xC0,
	BLE_PROVISION_PKT_CONTINUOUS,
}BLEProvisionPkt_t;

typedef enum
{
	BLE_PROVISION_AP_CONFIGURED = 0xD0,
	BLE_PROVISION_AP_SCANNED,
}BLEProvisionAPState_t;

typedef struct
{
	wiced_bool_t             device_configured;
	wiced_config_ap_entry_t  ap_entry;
} temp_config_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void ble_provision_device_init(void);
static void ble_provision_notify( uint16_t attr_handle, uint8_t len, uint8_t *pbuf);
static void ble_provision_send_ap_details(uint8_t idx);
static void ble_provision_send_ip_config(void);
static void ble_provision_send_sys_info(void);

static wiced_bt_dev_status_t ble_provision_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request);
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request);
static wiced_result_t ble_provision_scan_result_handler(wiced_scan_handler_result_t* malloced_scan_result);

extern bool fetch_or_generate_setup_ssid(wiced_ssid_t* SSID);


/******************************************************
 *               Variable Definitions
 ******************************************************/

/* BLE connect variable */
static uint16_t 	connect_id;
static GapStatus  	connect_status = GapStatus_Disconnect;

/* GATT attrIbute values */
static uint32_t    	peripheral_gatt_attribute_service_changed = 0;
static uint16_t    	peripheral_gatt_generic_access_appearance = 0;

static uint8_t		command_value[20];
static uint8_t		command_value_len = 0;
static uint16_t    	command_notify_flag = 0x0000;
static uint8_t		status_value[20];
static uint8_t 		status_value_len;
static uint16_t		status_notify_flag = 0x0000;

static temp_config_t temp_config;

static wiced_scan_result_t scan_record[SCAN_RECORD_MAX_NUM];
static uint8_t scan_record_cnt = 0;
static uint8_t configured_ap_idx = 0xFF;
static uint8_t provision_status = BLE_PROVISION_STATUS_IDLE;

static wiced_ssid_t device_name;

/* Stack and buffer pool configuration tables */
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************
 *               Function Definitions
 ******************************************************/
void ble_provision_init(void)
{
	fetch_or_generate_setup_ssid(&device_name);
	device_name.value[device_name.length] = '\0';

	wiced_bt_cfg_settings.device_name = device_name.value;

	/* Initialize Bluetooth controller and host stack */
	wiced_bt_stack_init( ble_provision_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

void ble_provision_loop(void)
{
	wiced_rtos_delay_milliseconds( 10 );
}

void ble_provision_finalize(void)
{
	provision_status = BLE_PROVISION_STATUS_CONNECTED;

	ble_provision_send_ip_config();

	wiced_bt_gatt_disconnect(connect_id);
}

/* Initialize peripheral */
static void ble_provision_device_init(void)
{
	wiced_bt_ble_advert_data_t adv_data;
	wiced_bt_ble_128service_t  service;

	uint8_t buf[16] = { UUID_SERVCLASS_BLE_PROVISION };

    /* Set advertising data: device name and discoverable flag */
	service.list_cmpl = WICED_TRUE;
	memcpy(service.uuid128, buf, MAX_UUID_SIZE);
    adv_data.flag = 0x06;
    adv_data.p_services_128b = &service;

    wiced_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_FLAGS|BTM_BLE_ADVERT_BIT_SERVICE_128, &adv_data);

    /* Register for gatt event notifications */
    wiced_bt_gatt_register(&ble_provision_gatt_cback);

    /* Initialize GATT database */
    wiced_bt_gatt_db_init ((uint8_t *)gatt_db, gatt_db_size);

    /* Enable Bluetooth LE advertising and connectability */

    /* start LE advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

static void ble_provision_notify( uint16_t attr_handle, uint8_t len, uint8_t *pbuf)
{
	if(connect_status == GapStatus_Connected && len<=20)
	{
		if((attr_handle==HDLC_BLE_PROVISION_COMMAND_VALUE) && (command_notify_flag == 0x0100))
		{
			memcpy(command_value, pbuf, len);
			command_value_len = len;
			wiced_bt_gatt_send_notification(connect_id, attr_handle, len, pbuf);
		}
		else if((attr_handle==HDLC_BLE_PROVISION_STATUS_VALUE) && (status_notify_flag == 0x0100))
		{
			memcpy(status_value, pbuf, len);
			status_value_len = len;
			wiced_bt_gatt_send_notification(connect_id, attr_handle, len, pbuf);
		}
	}
}

static void ble_provision_send_ap_details(uint8_t idx)
{
	uint8_t 	buf[20];
	uint16_t 	rssi;
	uint8_t 	remain_len, tx_len;
	uint8_t     ap_configured = BLE_PROVISION_AP_SCANNED;

	memset(buf, 0x00, 20);

	if(wlan_has_credentials() == 0)
	{
		platform_dct_wifi_config_t* wifi_config = NULL;
		wiced_result_t result = wiced_dct_read_lock( (void**) &wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(*wifi_config));
		if (result==WICED_SUCCESS)
		{
				if(!memcmp(wifi_config->stored_ap_list[0].details.BSSID.octet, scan_record[idx].BSSID.octet, 6))
				{
					ap_configured = BLE_PROVISION_AP_CONFIGURED;
					configured_ap_idx = idx;
				}
		}
		wiced_dct_read_unlock(wifi_config, WICED_FALSE);
	}

	buf[0] = idx;
	memcpy(&buf[2], scan_record[idx].BSSID.octet, 0x06);	//Mac_Address
	rssi = ~(scan_record[idx].signal_strength-1);
	buf[8] = (uint8_t)(rssi >> 8);							//Rssi
	buf[9] = (uint8_t)(rssi >> 0);
	buf[10] = (uint8_t)(scan_record[idx].security>>24);		//Security
	buf[11] = (uint8_t)(scan_record[idx].security>>16);
	buf[12] = (uint8_t)(scan_record[idx].security>>8);
	buf[13] = (uint8_t)(scan_record[idx].security>>0);
	buf[14] = ap_configured;
	buf[15] = scan_record[idx].SSID.length;

	remain_len = scan_record[idx].SSID.length;
	if(remain_len <= 4)
	{
		buf[1] = BLE_PROVISION_PKT_LAST;					// Last packet to be sent
		memcpy(&buf[16], scan_record[idx].SSID.value, remain_len);
		tx_len = 16 + remain_len;
		remain_len = 0;
	}
	else
	{
		buf[1] = BLE_PROVISION_PKT_CONTINUOUS;				// There has next packet to be sent
		memcpy(&buf[16], scan_record[idx].SSID.value, 4);
		tx_len = 20;
		remain_len = scan_record[idx].SSID.length - 4;
	}
	ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, tx_len, buf);

	if(remain_len > 0)
	{
		wiced_rtos_delay_milliseconds(20);

		memset(&buf[1], 0x00, 19);
		if(remain_len <= 18)
		{
			buf[1] = BLE_PROVISION_PKT_LAST;				// Last packet to be sent
			memcpy(&buf[2], &scan_record[idx].SSID.value[4], remain_len);
			tx_len = 2 + remain_len;
			remain_len = 0;
		}
		else
		{
			buf[1] = BLE_PROVISION_PKT_CONTINUOUS;			// There has next packet to be sent
			memcpy(&buf[2], &scan_record[idx].SSID.value[4], 18);
			tx_len = 20;
			remain_len = scan_record[idx].SSID.length - 22;
		}
		ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, tx_len, buf);
	}

	if(remain_len > 0 && remain_len <= 18)
	{
		wiced_rtos_delay_milliseconds(20);

		memset(&buf[1], 0x00, 19);
		buf[1] = BLE_PROVISION_PKT_LAST;
		memcpy(&buf[2], &scan_record[idx].SSID.value[22], remain_len);
		tx_len = 2 + remain_len;
		ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, tx_len, buf);
	}

	wiced_rtos_delay_milliseconds(20);
}

static void ble_provision_send_ip_config(void)
{
	wiced_ip_address_t gateway_ip, station_ip;
	wiced_mac_t        gateway_mac;
	wiced_bss_info_t   ap_info;
	wiced_security_t   sec;
	uint8_t 	       buf[20];
	uint8_t 	       remain_len, tx_len;

	wiced_ip_get_ipv4_address(WWD_STA_INTERFACE, &station_ip);
	wiced_ip_get_gateway_address(WWD_STA_INTERFACE, &gateway_ip);
	wwd_wifi_get_mac_address(&gateway_mac, WWD_AP_INTERFACE);
	wwd_wifi_get_ap_info( &ap_info, &sec );

	buf[0] = BLE_PROVISION_STATUS_CONNECTED;
	buf[2] = (uint8_t)(station_ip.ip.v4 >> 24);		// Station IP address
	buf[3] = (uint8_t)(station_ip.ip.v4 >> 16);
	buf[4] = (uint8_t)(station_ip.ip.v4 >> 8);
	buf[5] = (uint8_t)(station_ip.ip.v4);
	buf[6] = (uint8_t)(gateway_ip.ip.v4 >> 24);		// Gateway IP address
	buf[7] = (uint8_t)(gateway_ip.ip.v4 >> 16);
	buf[8] = (uint8_t)(gateway_ip.ip.v4 >> 8);
	buf[9] = (uint8_t)(gateway_ip.ip.v4);
	memcpy(&buf[10], gateway_mac.octet, 6);			// Gateway MAC address
	buf[16] = ap_info.SSID_len;

	remain_len = ap_info.SSID_len;
	if(remain_len <= 3)
	{
		buf[1] = BLE_PROVISION_PKT_LAST;					// Last packet to be sent
		memcpy(&buf[17], ap_info.SSID, remain_len);
		tx_len = 17 + remain_len;
		remain_len = 0;
	}
	else
	{
		buf[1] = BLE_PROVISION_PKT_CONTINUOUS;				// There has next packet to be sent
		memcpy(&buf[17], ap_info.SSID, 3);
		tx_len = 20;
		remain_len = ap_info.SSID_len - 3;
	}
	ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, tx_len, buf);

	if(remain_len > 0)
	{
		wiced_rtos_delay_milliseconds(20);

		memset(&buf[1], 0x00, 19);
		if(remain_len <= 18)
		{
			buf[1] = BLE_PROVISION_PKT_LAST;				// Last packet to be sent
			memcpy(&buf[2], &ap_info.SSID[3], remain_len);
			tx_len = 2 + remain_len;
			remain_len = 0;
		}
		else
		{
			buf[1] = BLE_PROVISION_PKT_CONTINUOUS;			// There has next packet to be sent
			memcpy(&buf[2], &ap_info.SSID[3], 18);
			tx_len = 20;
			remain_len = ap_info.SSID_len - 21;
		}
		ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, tx_len, buf);
	}

	if(remain_len > 0 && remain_len <= 18)
	{
		wiced_rtos_delay_milliseconds(20);

		memset(&buf[1], 0x00, 19);
		buf[1] = BLE_PROVISION_PKT_LAST;
		memcpy(&buf[2], &ap_info.SSID[21], remain_len);
		tx_len = 2 + remain_len;
		ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, tx_len, buf);
	}

	wiced_rtos_delay_milliseconds(100);
}

static void ble_provision_send_sys_info(void)
{
	uint8_t 	       buf[20];
	uint16_t           module_version;

	HAL_device_ID(&buf[0], 12);

	module_version = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08000000);
	buf[12] = (uint8_t)(module_version>>8);
	buf[13] = (uint8_t)(module_version);

	module_version = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08020000);
	buf[14] = (uint8_t)(module_version>>8);
	buf[15] = (uint8_t)(module_version);

	module_version = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08040000);
	buf[16] = (uint8_t)(module_version>>8);
	buf[17] = (uint8_t)(module_version);

	module_version = FLASH_ModuleVersion(FLASH_INTERNAL, 0x080C0000);
	buf[18] = (uint8_t)(module_version>>8);
	buf[19] = (uint8_t)(module_version);

	ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, 20, buf);
	wiced_rtos_delay_milliseconds(20);
	memset(buf, 0x00, 20);

	const uint8_t *release_str = stringify(SYSTEM_VERSION_STRING);
	buf[0] = strlen(release_str);
	memcpy(&buf[1], release_str, buf[0]);

	ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, buf[0]+1, buf);
	wiced_rtos_delay_milliseconds(20);
}

/******************************************************
 *             Callback Function Definitions
 ******************************************************/
/* Bluetooth management event handler */
static wiced_bt_dev_status_t ble_provision_management_cback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth controller and host stack enabled */
        if (p_event_data->enabled.status == WICED_BT_SUCCESS)
        {
            /* Enable peripheral */
        	ble_provision_device_init();
        }
        break;

    default:
        break;
    }

    return (status);
}

/* GATT event handler */
static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch (event)
	{
		case GATT_CONNECTION_STATUS_EVT:
			/* GATT connection status change */
			if (p_event_data->connection_status.connected)
			{
				connect_status = GapStatus_Connected;
				connect_id = p_event_data->connection_status.conn_id;

				/* Disable connectability. */
				wiced_bt_start_advertisements (BTM_BLE_ADVERT_OFF, 0, NULL);
			}
			else
			{
				connect_status = GapStatus_Disconnect;
				connect_id = 0x0000;
				command_notify_flag = 0x0000;
				status_notify_flag = 0x0000;

				/* Connection released. Re-enable BLE connectability. */
				if(temp_config.device_configured != WICED_TRUE)
				{
					wiced_bt_start_advertisements (BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
				}
			}
			break;

		case GATT_ATTRIBUTE_REQUEST_EVT:
			/* GATT attribute read/write request */
			if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE)
			{
				status = ble_provision_gatt_write_request(&p_event_data->attribute_request.data.write_req);
			}
			else if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_READ)
			{
				status = ble_provision_gatt_read_request(&p_event_data->attribute_request.data.read_req);
			}
			break;

		default:
			break;
    }

    return (status);
}

/* Handler for attrubute write requests */
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request)
{
	uint8_t new_value[20];
	uint8_t len = p_write_request->val_len;

	wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    memcpy(new_value, (uint8_t *)p_write_request->p_val, len);

    switch (p_write_request->handle)
    {
    	/* Client Characteristic Configuration Descriptor */
		case HDLC_BLE_PROVISION_COMMAND_CCCD:
			command_notify_flag = new_value[0];
			command_notify_flag = (command_notify_flag<<8) + new_value[1];
			break;

		case HDLC_BLE_PROVISION_STATUS_CCCD:
			status_notify_flag = new_value[0];
			status_notify_flag = (status_notify_flag<<8) + new_value[1];
			break;

		/* Server characteristic attribute value */
        case HDLC_BLE_PROVISION_COMMAND_VALUE:
        	if((new_value[0] == BLE_PROVISION_COMMAND_SCAN_REQUEST) && (provision_status != BLE_PROVISION_STATUS_SCANNING))
        	{
				scan_record_cnt = 0;
				memset(scan_record, 0x00, sizeof(scan_record));

				wiced_wifi_scan_networks(ble_provision_scan_result_handler, NULL );

				provision_status = BLE_PROVISION_STATUS_SCANNING;
				ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, 1, &provision_status);
        	}
        	else if((new_value[0] == BLE_PROVISION_COMMAND_SET_SECUR_KEY) && (scan_record_cnt != 0))
			{
        		uint8_t record_idx = new_value[1];
        		uint8_t key_len = new_value[2];

        		if(record_idx < scan_record_cnt)
        		{
        			if(configured_ap_idx != record_idx)
					{
						memset(temp_config.ap_entry.security_key, 0x00, 64);
						temp_config.ap_entry.security_key_length = 0x00;

						memcpy(temp_config.ap_entry.security_key, &new_value[3], key_len);
						temp_config.ap_entry.security_key_length = key_len;

						memcpy(&temp_config.ap_entry.details, &scan_record[record_idx], sizeof(wiced_ap_info_t));
						temp_config.device_configured = WICED_TRUE;

						wiced_dct_write( &temp_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(temp_config_t) );
					}
        			HAL_WLAN_notify_simple_config_done();

					provision_status = BLE_PROVISION_STATUS_CONNECTING;
					ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, 1, &provision_status);
        		}
			}
        	else if((new_value[0] == BLE_PROVISION_COMMAND_SYSTEM_INFO) && (provision_status == BLE_PROVISION_STATUS_IDLE))
        	{
        		ble_provision_send_sys_info();
        	}
        	break;

		default:
			status = WICED_BT_GATT_WRITE_NOT_PERMIT;
			break;
    }

    return (status);
}

/* Handler for attrubute read requests */
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    void *p_attribute_value_source;
    uint16_t attribute_value_length = 0;

    switch (p_read_request->handle)
    {
		case HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE:
			p_attribute_value_source    = &peripheral_gatt_attribute_service_changed;
			attribute_value_length      = sizeof(peripheral_gatt_attribute_service_changed);
			break;

		case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
			p_attribute_value_source    = (void *)wiced_bt_cfg_settings.device_name;
			attribute_value_length      = strlen((char *)wiced_bt_cfg_settings.device_name);
			break;

		case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
			p_attribute_value_source    = &peripheral_gatt_generic_access_appearance;
			attribute_value_length      = sizeof(peripheral_gatt_generic_access_appearance);
			break;

		case HDLC_BLE_PROVISION_COMMAND_CCCD:
			p_attribute_value_source    = &command_notify_flag;
			attribute_value_length      = sizeof(command_notify_flag);
			break;

		case HDLC_BLE_PROVISION_STATUS_CCCD:
			p_attribute_value_source    = &status_notify_flag;
			attribute_value_length      = sizeof(status_notify_flag);
			break;

		case HDLC_BLE_PROVISION_COMMAND_VALUE:
			p_attribute_value_source    = command_value;
			attribute_value_length      = command_value_len;
			break;

		case HDLC_BLE_PROVISION_STATUS_VALUE:
			p_attribute_value_source    = status_value;
			attribute_value_length      = status_value_len;
			break;

		default:
			status = WICED_BT_GATT_READ_NOT_PERMIT;
			break;
    }

    /* Validate destination buffer size */
    if (attribute_value_length > *p_read_request->p_val_len)
    {
        *p_read_request->p_val_len = attribute_value_length;
    }

    /* Copy the attribute value */
    if (attribute_value_length)
    {
        memcpy(p_read_request->p_val, p_attribute_value_source, attribute_value_length);
    }

    /* Indicate number of bytes copied */
    *p_read_request->p_val_len = attribute_value_length;

    return (status);
}

/* Callback function to handle scan results */
static wiced_result_t ble_provision_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    malloc_transfer_to_curr_thread( malloced_scan_result );

    if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
    {
        wiced_scan_result_t* record = &malloced_scan_result->ap_details;
        record->SSID.value[record->SSID.length] = 0; /* Ensure the SSID is null terminated */

        if(scan_record_cnt < SCAN_RECORD_MAX_NUM)
        {
        	memcpy(&scan_record[scan_record_cnt], record, sizeof(wiced_scan_result_t));
        	ble_provision_send_ap_details(scan_record_cnt);
        	scan_record_cnt++;
        }
    }
    else
    {
    	provision_status = BLE_PROVISION_STATUS_SCAN_COMPLETE;
    	ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, 1, &provision_status);
    }

    free( malloced_scan_result );

    return WICED_SUCCESS;
}
