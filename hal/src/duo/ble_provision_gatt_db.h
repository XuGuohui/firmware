/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * Proximity Server Sample Application (GATT Server database definitions)
 *
 */
#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

#define		UUID_SERVCLASS_BLE_PROVISION	0x1E,0x90,0x9E,0x66,0x85,0x7A,0xD9,0xA0,0xC3,0x49,0xCD,0x89,0x00,0x14,0xC6,0x3E
#define		GATT_UUID_BLE_PROVISION_COMMAND	0x1E,0x90,0x9E,0x66,0x85,0x7A,0xD9,0xA0,0xC3,0x49,0xCD,0x89,0x01,0x14,0xC6,0x3E
#define		GATT_UUID_BLE_PROVISION_STATUS	0x1E,0x90,0x9E,0x66,0x85,0x7A,0xD9,0xA0,0xC3,0x49,0xCD,0x89,0x02,0x14,0xC6,0x3E


enum {
    // ***** Primary service 'Generic Attribute'
    HDLS_GENERIC_ATTRIBUTE      =  0x0100,
    HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED,
    HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE,

    // ***** Primary service 'Generic Access'
    HDLS_GENERIC_ACCESS,
    HDLC_GENERIC_ACCESS_DEVICE_NAME,
    HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
    HDLC_GENERIC_ACCESS_APPEARANCE,
    HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,


    // ***** Primary service 'Provision'
    HDLS_BLE_PROVISION,
    HDLC_BLE_PROVISION_COMMAND,
	HDLC_BLE_PROVISION_COMMAND_VALUE,
	HDLC_BLE_PROVISION_COMMAND_CCCD,

	HDLC_BLE_PROVISION_STATUS,
	HDLC_BLE_PROVISION_STATUS_VALUE,
	HDLC_BLE_PROVISION_STATUS_CCCD,
};


extern const uint8_t  gatt_db[];
extern const uint16_t gatt_db_size;

#ifdef __cplusplus
}
#endif
