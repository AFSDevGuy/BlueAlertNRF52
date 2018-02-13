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
/** @file
 *
 * @defgroup ble_alert BlueAlert Service
 * @{
 * @ingroup ble_sdk_srv
 *
 * @details This module implements the BlueAlert service. It is a clone of the Battery
 *          Level service, to the degree that is re-usable.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_alert_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_ALERT_BLE_OBSERVER_PRIO,
 *                                   ble_alert_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */
#ifndef BLE_ALERT_H__
#define BLE_ALERT_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_ALERT_BLE_OBSERVER_PRIO 2 // not sure what this means exactly, so took the value from the batt service

/**@brief   Macro for defining a ble_alert instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ALERT_DEF(_name)                                                                          \
static ble_alert_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ALERT_BLE_OBSERVER_PRIO,                                                     \
                     ble_alert_on_ble_evt, &_name)

extern ble_uuid_t m_alert_srv_uuid;


/**@brief BlueAlert Service event type. */
typedef enum
{
    BLE_ALERT_EVT_NOTIFICATION_ENABLED,   /**< BlueAlert value notification enabled event. */
    BLE_ALERT_EVT_NOTIFICATION_DISABLED   /**< BlueAlert value notification disabled event. */
} ble_alert_evt_type_t;

/**@brief BlueAlert Service event. */
typedef struct
{
    ble_alert_evt_type_t evt_type;        /**< Type of event. */
} ble_alert_evt_t;

// Forward declaration of the ble_alert_t type.
typedef struct ble_alert_s ble_alert_t;

/**@brief BlueAlert Service event handler type. */
typedef void (*ble_alert_evt_handler_t) (ble_alert_t * p_alert, ble_alert_evt_t * p_evt);

/**@brief BlueAlert Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_alert_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the Alert Service. */
    bool                          support_notification;           /**< TRUE if notification of Alert Level is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Alert Level characteristic */
    uint8_t                       initial_alert_level;            /**< Initial alert level */
    ble_srv_cccd_security_mode_t  alert_level_char_attr_md;       /**< Initial security level for alert characteristics attribute */
    ble_gap_conn_sec_mode_t       alert_level_report_read_perm;   /**< Initial security level for alert report read attribute */
} ble_alert_init_t;

/**@brief Alert Service structure. This contains various status information for the service. */
struct ble_alert_s
{
    ble_alert_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the Alert Service. */
    uint16_t                      service_handle;                 /**< Handle of Alert Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      alert_level_handles;            /**< Handles related to the Alert Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       alert_level_last;               /**< Last Alert Level measurement passed to the Alert Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Alert Level is supported. */
};


/**@brief Function for initializing the BlueAlert Service.
 *
 * @param[out]  p_alert     BlueAlert Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_alert_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_alert_init(ble_alert_t * p_alert, const ble_alert_init_t * p_alert_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Alert Service.
 *
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Alert Service structure.
 */
void ble_alert_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the alert level.
 *
 * @details The application calls this function after having alerted. If
 *          notification has been enabled, the alert level characteristic is sent to the client.
 *
 * @note TBD if this is relevant for the Alert service
 *
 * @param[in]   p_alert          Alert Service structure.
 * @param[in]   alert_level  New Alert Level value
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_alert_alert_level_update(ble_alert_t * p_alert, uint8_t alert_level);


#ifdef __cplusplus
}
#endif

#endif // BLE_ALERT_H__

/** @} */
