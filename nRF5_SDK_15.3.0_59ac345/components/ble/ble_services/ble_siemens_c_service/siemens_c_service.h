/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
/**@file
 *
 * @defgroup ble_bas_c Battery Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Battery Service Client module.
 *
 * @details  This module contains APIs to read and interact with the Battery Service of a remote
 *           device.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_bas_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_BAS_C_BLE_OBSERVER_PRIO,
 *                                   ble_bas_c_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_SIEMENS_C_H__
#define BLE_SIEMENS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"
#include "app_util_bds.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_bas_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_SIEMENS_C_DEF(_name)                                                                        \
static ble_siemens_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_SIEMENS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_siemens_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_bas_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_SIEMENS_C_ARRAY_DEF(_name, _cnt)                 \
static ble_siemens_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_SIEMENS_C_BLE_OBSERVER_PRIO,       \
                      ble_siemens_c_on_ble_evt, &_name, _cnt)


#define SIEMENS_BASE_UUID  {{0x4E, 0xDE, 0x51, 0x86, 0xE8, 0xD2, 0xE4, 0xA2, 0x27, 0x4C, 0x6D, 0x11, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */
#define BLE_UUID_SIEMENS_SERVICE 0x0001 /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_SIEMENS_COMMAND_CHARACTERISTIC       0x0002                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_SIEMENS_MIN_CHARACTERISTIC           0x0005                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_SIEMENS_MAX_CHARACTERISTIC           0x0004                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_SIEMENS_ZERO_CROSSING_CHARACTERISTIC 0x0006                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_SIEMENS_RMS_CHARACTERISTIC           0x0007                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_SIEMENS_X_VALUE_CHARACTERISTIC       0x0008
#define BLE_UUID_SIEMENS_Y_VALUE_CHARACTERISTIC       0x0009
#define BLE_UUID_SIEMENS_Z_VALUE_CHARACTERISTIC       0x0010
#define BLE_UUID_SIEMENS_MEAN_VALUE_CHARACTERISTIC    0x000B

#define COMMAND_INDEX 1
#define X_INDEX 2
#define Y_INDEX 3
#define Z_INDEX 4

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

/**
 * @defgroup bas_c_enums Enumerations
 * @{
 */

/**@brief Battery Service Client event type. */
typedef enum
{
    BLE_SIEMENS_C_EVT_DISCOVERY_COMPLETE,  /**< Event indicating that the Battery Service has been discovered at the peer. */
    BLE_SIEMENS_C_EVT_MIN_NOTIFICATION,   /**< Event indicating that a notification of the Battery Level characteristic has been received from the peer. */
    BLE_SIEMENS_C_EVT_MIN_READ_RESP,      /**< Event indicating that a read response on Battery Level characteristic has been received from peer. */
    BLE_SIEMENS_C_EVT_MAX_NOTIFICATION,
    BLE_SIEMENS_C_EVT_MAX_READ_RESP,
    BLE_SIEMENS_C_EVT_RMS_NOTIFICATION,
    BLE_SIEMENS_C_EVT_RMS_READ_RESP,
    BLE_SIEMENS_C_EVT_ZERO_CROSSING_NOTIFICATION,
    BLE_SIEMENS_C_EVT_ZERO_CROSSING_READ_RESP,
    BLE_SIEMENS_C_EVT_MEAN_NOTIFICATION,
    BLE_SIEMENS_C_EVT_MEAN_READ_RESP,
    BLE_SIEMENS_C_EVT_DISCONNECTED
} ble_siemens_c_evt_type_t;

/** @} */

/**
 * @defgroup bas_c_structs Structures
 * @{
 */

/**@brief Structure containing the handles related to the Battery Service found on the peer. */
typedef struct
{
    uint16_t                min_cccd_handle;  /**< Handle of the CCCD of the Battery Level characteristic. */
    uint16_t                min_handle;       /**< Handle of the Battery Level characteristic as provided by the SoftDevice. */
    uint16_t                max_cccd_handle;
    uint16_t                max_handle;
    uint16_t                rms_cccd_handle;
    uint16_t                rms_handle;
    uint16_t                zero_crossing_cccd_handle;
    uint16_t                zero_crossing_handle;
    uint16_t                mean_cccd_handle;
    uint16_t                mean_handle;
    uint16_t                command_handle;
    uint16_t                x_value_handle;
    uint16_t                y_value_handle;
    uint16_t                z_value_handle;
} ble_siemens_c_db_t;

/**@brief Battery Service Client Event structure. */
typedef struct
{
    ble_siemens_c_evt_type_t evt_type;  /**< Event Type. */
    uint16_t conn_handle;           /**< Connection handle relevent to this event.*/
    union
    {
        ble_siemens_c_db_t siemens_db;         /**< Battery Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_BAS_C_EVT_DISCOVERY_COMPLETE.*/
        uint8_t        min_value[8];  /**< Battery level received from peer. This field will be used for the events @ref BLE_BAS_C_EVT_BATT_NOTIFICATION and @ref BLE_BAS_C_EVT_BATT_READ_RESP.*/
        uint8_t       max_value[8];
        uint8_t       rms_value[8];
        uint8_t       zero_crossing[8];
        uint8_t       mean_value[8];
    } params;
} ble_siemens_c_evt_t;

/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, meaning 4 bits for exponent (base 10) and 12 bits mantissa) */
typedef struct
{
  uint16_t exponent: 16;                                             /**< Base 10 exponent, should be using only 4 bits */
  uint64_t mantissa;                                            /**< Mantissa, should be using only 12 bits */
} csfloat_t;

/** @} */

/**
 * @defgroup bas_c_types Types
 * @{
 */

// Forward declaration of the ble_bas_t type.
typedef struct ble_siemens_c_s ble_siemens_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_siemens_c_evt_handler_t) (ble_siemens_c_t * p_siemens_c, ble_siemens_c_evt_t * p_evt);

/** @} */

/**
 * @addtogroup bas_c_structs
 * @{
 */

/**@brief   Battery Service Client structure. */
struct ble_siemens_c_s
{
    uint16_t                conn_handle;     /**< Connection handle as provided by the SoftDevice. */
    ble_siemens_c_db_t          peer_siemens_db;     /**< Handles related to BAS on the peer*/
    ble_siemens_c_evt_handler_t evt_handler;     /**< Application event handler to be called when there is an event related to the Battery service. */
};

typedef  struct {
  //float     f;
  signed short mantisa : 12;
  signed char exponent : 4;
  //struct 
      //{
        //signed short mantisa : 12;
        //signed char exponent : 4;
      //} parts;
} float_cast;

/**@brief   Battery Service Client initialization structure. */
typedef struct
{
    ble_siemens_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Battery Service Client module whenever there is an event related to the Battery Service. */
} ble_siemens_c_init_t;


/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, meaning 4 bits for exponent (base 10) and 12 bits mantissa) */
typedef struct
{
  uint64_t exponent: 32;                                             /**< Base 10 exponent, should be using only 4 bits */
  uint8_t mantissa;                                            /**< Mantissa, should be using only 12 bits */
} zero_float_t;

/**
 * @defgroup bas_c_functions Functions
 * @{
 */

/**@brief      Function for initializing the Battery Service Client module.
 *
 * @details    This function will initialize the module and set up Database Discovery to discover
 *             the Battery Service. After calling this function, call @ref ble_db_discovery_start
 *             to start discovery once a link with a peer has been established.
 *
 * @param[out] p_ble_bas_c      Pointer to the Battery Service client structure.
 * @param[in]  p_ble_bas_c_init Pointer to the Battery Service initialization structure containing
 *                              the initialization information.
 *
 * @retval     NRF_SUCCESS      Operation success.
 * @retval     NRF_ERROR_NULL   A parameter is NULL.
 *                              Otherwise, an error code returned by @ref ble_db_discovery_evt_register.
 */
uint32_t ble_siemens_c_init(ble_siemens_c_t * p_ble_siemens_c, ble_siemens_c_init_t * p_ble_siemens_c_init);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If the BLE
 *            event is relevant for the Battery Service Client module, then it is used to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @note      This function must be called by the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the Battery Service client structure.
 */
void ble_siemens_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for enabling notifications on the Battery Level characteristic.
 *
 * @details This function will enable to notification of the Battery Level characteristic at the
 *          peer by writing to the CCCD of the Battery Level Characteristic.
 *
 * @param   p_ble_bas_c Pointer to the Battery Service client structure.
 *
 * @retval  NRF_SUCCESS     If the SoftDevice has been requested to write to the CCCD of the peer.
 *          NRF_ERROR_NULL  Parameter is NULL.
 *                          Otherwise, an error code returned by the SoftDevice API @ref
 *                          sd_ble_gattc_write.
 */
uint32_t ble_siemens_c_min_notif_enable(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_max_notif_enable(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_zero_crossing_notif_enable(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_rms_notif_enable(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_mean_notif_enable(ble_siemens_c_t * p_ble_siemens_c);


/**@brief   Function for reading the Battery Level characteristic.
 *
 * @param   p_ble_bas_c Pointer to the Battery Service client structure.
 *
 * @retval  NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
uint32_t ble_siemens_c_min_read(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_max_read(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_rms_read(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_mean_read(ble_siemens_c_t * p_ble_siemens_c);
uint32_t ble_siemens_c_zero_crossing_read(ble_siemens_c_t * p_ble_siemens_c);


/**@brief     Function for handling events from the database discovery module.
 *
 * @details   Call this function when getting a callback event from the DB discovery modue.
 *            This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of Battery service at the peer. If so, it will
 *            call the application's event handler indicating that the Battery service has been
 *            discovered at the peer. It also populates the event with the service related
 *            information before providing it to the application.
 *
 * @param     p_ble_bas_c Pointer to the Battery Service client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 *
 */
void ble_siemens_on_db_disc_evt(ble_siemens_c_t * p_ble_siemens_c, ble_db_discovery_evt_t const * p_evt);


/**@brief     Function for assigning handles to a this instance of bas_c.
 *
 * @details   Call this function when a link has been established with a peer to
 *            associate this link to this instance of the module. This makes it
 *            possible to handle several link and associate each link to a particular
 *            instance of this module. The connection handle and attribute handles will be
 *            provided from the discovery event @ref BLE_BAS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_bas_c    Pointer to the Battery client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associated with the given Battery Client Instance.
 * @param[in] p_peer_handles Attribute handles on the BAS server you want this BAS client to
 *                           interact with.
 */
uint32_t ble_siemens_c_handles_assign(ble_siemens_c_t *    p_ble_siemens_c,
                                  uint16_t         conn_handle,
                                  ble_siemens_c_db_t * p_peer_handles);



uint32_t ble_siemens_c_write(ble_siemens_c_t * p_ble_siemens_c, uint8_t * p_string, uint16_t length, uint16_t xteristic);

static __INLINE uint8_t bds_sfloat_encode(const sfloat_t * p_value, uint8_t * p_encoded_data );

sfloat_t encode_float(float);
float decode_float8_values(const uint8_t *p_encoded_value, const uint16_t len);


float decode_float5_values(const uint8_t *p_encoded_value, const uint16_t len);

static __INLINE uint8_t bds_float5_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          zero_float_t  * p_decoded_val)
{

    p_decoded_val->exponent = 0;
    bds_uint40_decode(len, p_encoded_data, (uint64_t*)&p_decoded_val->mantissa);
    p_decoded_val->exponent = (uint8_t)((p_decoded_val->mantissa & 0xFF00000000000000) >> 8);
    p_decoded_val->mantissa &= 0x00FFFFFFFFFFFFFF;
    return len;
}

/**@brief Function for decoding a uint40 value.
 *
 * @param[in]   len              length of the field to be decoded.
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @param[in]   p_decoded_val    pointer to the decoded value
 *
 * @return      length of the decoded field.
 */
static __INLINE uint8_t bds_uint64_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          uint64_t      * p_decoded_val)
{
    UNUSED_VARIABLE(len);
    *p_decoded_val = (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
                     (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
                     (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
                     (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24 )|
                     (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32 )|
                     (((uint64_t)((uint8_t *)p_encoded_data)[5]) << 40 )|
                     (((uint64_t)((uint8_t *)p_encoded_data)[6]) << 48 )|
                     (((uint64_t)((uint8_t *)p_encoded_data)[7]) << 56 );
    return (56);
}

static __INLINE uint8_t bds_float8_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          csfloat_t      * p_decoded_val)
{

    p_decoded_val->exponent = 0;
    bds_uint64_decode(len, p_encoded_data, (uint64_t*)&p_decoded_val->mantissa);
    p_decoded_val->exponent = (uint8_t)((p_decoded_val->mantissa & 0xFFFFFFFFFFFF0000) >> 48);
    p_decoded_val->mantissa &= 0x000000000000FFFF;
    return len;
}

/** @} */ // End tag for Function group.

#ifdef __cplusplus
}
#endif

#endif // BLE_BAS_C_H__

/** @} */ // End tag for the file.
