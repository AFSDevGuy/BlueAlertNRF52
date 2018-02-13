/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#include "ble_alert.h"
#include <string.h>
#include "ble_srv_common.h"
#include "alert_timer.h"


#define INVALID_ALERT_LEVEL 255

/* 128-bit UUIDs */
/**@brief 128-bit UUID base List. */
static ble_uuid128_t const m_alert_srv_uuid128 =
{
   {
       0xAB, 0x08, 0xFD, 0xF2, 0x6A, 0x88, 0x46, 0x11,
       0xBD, 0x85, 0xDA, 0x3E, 0xE2, 0xEA, 0x76, 0x3D
   }
};
static uint16_t const m_alert_srv_uuid16 = 0x3d76;

ble_uuid_t m_alert_srv_uuid;

static ble_uuid128_t const m_alert_char_uuid128 =
{
   {
       0x8C, 0xFE, 0xCE, 0x3C, 0xB6, 0x97, 0x45, 0xE9,
       0xAD, 0x63, 0x60, 0x55, 0x62, 0x66, 0xA3, 0xAE
   }
};
static uint16_t const m_alert_char_uuid16 = 0xaea3;


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_alert     Alert Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_alert_t * p_alert, ble_evt_t const * p_ble_evt)
{
    p_alert->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_alert     Alert Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_alert_t * p_alert, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_alert->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_alert     Alert Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_alert_t * p_alert, ble_evt_t const * p_ble_evt)
{

    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    //TODO: filter these events a little more effectively - right now we
    // "know" that *any* write should trigger an alert.
    alert_sound();
    if (    (p_evt_write->handle == p_alert->alert_level_handles.cccd_handle)
        &&  (p_evt_write->len == 2))
    {
        if (p_alert->evt_handler == NULL)
        {
            return;
        }

        ble_alert_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BLE_ALERT_EVT_NOTIFICATION_ENABLED;
        }
        else
        {
            evt.evt_type = BLE_ALERT_EVT_NOTIFICATION_DISABLED;
        }

        // CCCD written, call application event handler.
        p_alert->evt_handler(p_alert, &evt);
    }
}


void ble_alert_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_alert_t * p_alert = (ble_alert_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_alert, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_alert, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_alert, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Alert Level characteristic.
 *
 * @param[in]   p_alert      Alert Service structure.
 * @param[in]   p_alert_init Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t alert_level_char_add(ble_alert_t * p_alert, const ble_alert_init_t * p_alert_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_alert_level;
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;

    // Add Alert Level characteristic
    if (p_alert->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        cccd_md.read_perm = p_alert_init->alert_level_char_attr_md.read_perm;

        cccd_md.write_perm = p_alert_init->alert_level_char_attr_md.write_perm;

        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.char_props.read   = 0;
    char_md.char_props.notify = (p_alert->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_alert->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.uuid = m_alert_char_uuid16;

    err_code = sd_ble_uuid_vs_add(&m_alert_char_uuid128, &ble_uuid.type);
    APP_ERROR_CHECK(err_code);


    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_alert_init->alert_level_char_attr_md.read_perm;
    attr_md.write_perm = p_alert_init->alert_level_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_alert_level = p_alert_init->initial_alert_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &initial_alert_level;

    err_code = sd_ble_gatts_characteristic_add(p_alert->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_alert->alert_level_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#if 0
    if (p_alert_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_alert_init->alert_level_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;

        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_alert_init->p_report_ref);

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_alert->alert_level_handles.value_handle,
                                               &attr_char_value,
                                               &p_alert->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
#endif
    {
        p_alert->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


uint32_t ble_alert_init(ble_alert_t * p_alert, const ble_alert_init_t * p_alert_init)
{
    if (p_alert == NULL || p_alert_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;

    // Initialize service structure
    p_alert->evt_handler               = p_alert_init->evt_handler;
    p_alert->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_alert->is_notification_supported = p_alert_init->support_notification;
    p_alert->alert_level_last        = INVALID_ALERT_LEVEL;

    // Add service
    m_alert_srv_uuid.uuid = m_alert_srv_uuid16;

    err_code = sd_ble_uuid_vs_add(&m_alert_srv_uuid128, &m_alert_srv_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &m_alert_srv_uuid, &p_alert->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add alert level characteristic
    return alert_level_char_add(p_alert, p_alert_init);
}


uint32_t ble_alert_alert_level_update(ble_alert_t * p_alert, uint8_t alert_level)
{
    if (p_alert == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (alert_level != p_alert->alert_level_last)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &alert_level;

        // Update database.
        err_code = sd_ble_gatts_value_set(p_alert->conn_handle,
                                          p_alert->alert_level_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new akert value.
            p_alert->alert_level_last = alert_level;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_alert->conn_handle != BLE_CONN_HANDLE_INVALID) && p_alert->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_alert->alert_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_alert->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
