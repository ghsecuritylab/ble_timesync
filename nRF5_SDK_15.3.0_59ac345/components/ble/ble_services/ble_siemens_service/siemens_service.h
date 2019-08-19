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
/** @file
 *
 * @defgroup ble_dis Device Information Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Device Information Service  module.
 *
 * @details This module implements the Device Information Service.
 *          During initialization it adds the Device Information Service to the BLE stack database.
 *          It then encodes the supplied information, and adds the corresponding characteristics.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef SIEMENS_SERVICE_H__
#define SIEMENS_SERVICE_H__

#include <stdint.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "app_util_bds.h"


#define BLE_UUID_SIEMENS_SERVICE 0x0001 /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_ENV_ADC_VALUE_CHARACTERISTIC 0x0003                                                                          /**< The UUID of the TX Characteristic. */
#define BLE_UUID_ENV_COMMAND_CHARACTERISTIC 0x0002                                                                            /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ENV_MINIMUM_CHARACTERISTIC 0x0005                                                                            /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ENV_MAXIMUM_CHARACTERISTIC 0x0004                                                                            /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ENV_ZERO_CROSSING_CHARACTERISTIC 0x0006                                                                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ENV_RMS_CHARACTERISTIC 0x0007                                                                                /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ENV_X_VALUE_CHARACTERISTIC 0x0008
#define BLE_UUID_ENV_Y_VALUE_CHARACTERISTIC 0x0009
#define BLE_UUID_ENV_Z_VALUE_CHARACTERISTIC 0x0010
#define BLE_UUID_ENV_MEAN_VALUE_CHARACTERISTIC 0x000B

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_SIEMENS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_SIEMENS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup DIS_VENDOR_ID_SRC_VALUES Vendor ID Source values
 * @{
 */
#define BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG   1                 /**< Vendor ID assigned by Bluetooth SIG. */
#define BLE_DIS_VENDOR_ID_SRC_USB_IMPL_FORUM  2                 /**< Vendor ID assigned by USB Implementer's Forum. */
#define NUMBER_OF_SIEMENS_XTERISTICS          13                 /**< Number of xteristics in the siemens service. */
#define COMMAND_INDEX                         0
#define MIN_INDEX                             1
#define MAX_INDEX                             2
#define RMS_INDEX                             3
#define ZERO_INDEX                            4
#define ADC_INDEX                             5
#define REPORT_INDEX                          6
#define INCASE_INDEX                          7
#define PNP_INDEX                             8
#define MEAN_INDEX                            9
#define X_VALUE_INDEX                         10
#define Y_VALUE_INDEX                         11
#define Z_VALUE_INDEX                         12
/** @} */

/**@brief System ID parameters */
typedef struct
{
    uint64_t manufacturer_id;                                   /**< Manufacturer ID. Only 5 LSOs shall be used. */
    uint32_t organizationally_unique_id;                        /**< Organizationally unique ID. Only 3 LSOs shall be used. */
} ble_siemens_sys_id_t;

/**@brief IEEE 11073-20601 Regulatory Certification Data List Structure */
typedef struct
{
    uint8_t *  p_list;                                          /**< Pointer the byte array containing the encoded opaque structure based on IEEE 11073-20601 specification. */
    uint8_t    list_len;                                        /**< Length of the byte array. */
} ble_siemens_reg_cert_data_list_t;

/**@brief PnP ID parameters */
typedef struct
{
    uint8_t  vendor_id_source;                                  /**< Vendor ID Source. see @ref DIS_VENDOR_ID_SRC_VALUES. */
    uint16_t vendor_id;                                         /**< Vendor ID. */
    uint16_t product_id;                                        /**< Product ID. */
    uint16_t product_version;                                   /**< Product Version. */
} ble_siemens_pnp_id_t;

/**@brief Battery Service event type. */
typedef enum
{
    BLE_SIEMENS_EVT_NOTIFICATION_ENABLED,  /**< Battery value notification enabled event. */
    BLE_SIEMENS_EVT_NOTIFICATION_DISABLED, /**< Battery value notification disabled event. */
    BLE_SIEMENS_EVT_ON_WRITE
} ble_siemens_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    ble_siemens_evt_type_t   evt_type;    /**< Type of event. */
    uint16_t                 conn_handle; /**< Connection handle. */
    uint8_t                  const * p_data;            /**< A pointer to the buffer with received data. */
    uint16_t                 length;                    /**< Length of received data. */
} ble_siemens_evt_t;

// Forward declaration of the ble_siemens_t type.
typedef struct ble_siemens_s ble_siemens_t;

/**@brief Battery Service event handler type. */
typedef void (* ble_siemens_evt_handler_t) (ble_siemens_t * p_siemens, ble_siemens_evt_t * p_evt);

typedef union {
    uint16_t                        all_uint8_t;
    float                          all_sfloat_t;
    uint64_t                       zero_incase;
} all_xteristics;

/**@brief Battery Service structure. This contains various status information for the service. */
struct ble_siemens_s
{
    ble_siemens_evt_handler_t evt_handler;              /**< Event handler to be called for handling events in the Battery Service. */
    ble_gatts_char_handles_t battery_level_handles;     /**< Handles related to the Battery Level characteristic. */
    uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
    all_xteristics           battery_level_last;
    bool                     is_notification_supported; /**< TRUE if notification of Battery Level is supported. */
};



/**@brief Device Information Service init structure. This contains all possible characteristics
 *        needed for initialization of the service.
 */
typedef struct
{
    all_xteristics *               characteristics;
    ble_srv_report_ref_t *         p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    security_req_t                 dis_char_rd_sec;                /**< Security requirement for reading any siemens characteristic value. */
    security_req_t                 siemens_cccd_wr_sec;            /**< Security requirement for writing the siemens characteristic CCCD. */
    security_req_t                 siemens_report_rd_sec;          /**< Security requirement for reading the BL characteristic descriptor. */
    ble_siemens_evt_handler_t      evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    bool                           support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_gatts_char_handles_t       siemens_xteristics_handles;
} ble_siemens_init_t;



typedef  union {
  float     f;
  struct 
      {
        signed short mantisa : 12;
        signed char exponent : 4;
      } parts;
} float_cast;



ret_code_t add_siemens_service(ble_siemens_t * p_siemens, ble_siemens_init_t const * p_siemens_init);


ret_code_t char_add(uint16_t                      uuid,
                    ble_siemens_t                 * p_siemens,
                    const ble_siemens_init_t      * p_siemens_init,
                    uint8_t                         xteristics_index);



/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, meaning 4 bits for exponent (base 10) and 12 bits mantissa) */
typedef struct
{
  uint16_t exponent: 16;                                             /**< Base 10 exponent, should be using only 4 bits */
  uint64_t mantissa;                                            /**< Mantissa, should be using only 12 bits */
} csfloat_t;

/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, meaning 4 bits for exponent (base 10) and 12 bits mantissa) */
typedef struct
{
  uint64_t exponent: 32;                                             /**< Base 10 exponent, should be using only 4 bits */
  uint8_t mantissa;                                            /**< Mantissa, should be using only 12 bits */
} zero_float_t;

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_bas_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Battery Service structure.
 */
void ble_siemens_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement.
 *          The battery level characteristic will only be sent to the clients which have
 *          enabled notifications. \ref BLE_CONN_HANDLE_ALL can be used as a connection handle
 *          to send notifications to all connected devices.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   battery_level  New battery measurement value (in percent of full capacity).
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_siemens_service_update(ble_siemens_t    * p_siemens,
                                      uint8_t          * value_buffer,
                                      uint16_t         * p_length,
                                      uint16_t         conn_handle,
                                      float            unencoded_value);


/**@brief Function for sending the last battery level when bonded client reconnects.
 *
 * @details The application calls this function, in the case of a reconnection of
 *          a bonded client if the value of the battery has changed since
 *          its disconnection.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the battery level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success,
 *              NRF_ERROR_INVALID_STATE when notification is not supported,
 *              otherwise an error code returned by @ref sd_ble_gatts_hvx.
 */
ret_code_t ble_siemens_service_on_reconnection_update(ble_siemens_t * p_siemens,
                                                      uint8_t     xteristics_index,
                                                      uint16_t    conn_handle);




static __INLINE uint8_t bds_sfloat_encode(const sfloat_t * p_value, uint8_t * p_encoded_data );

static __INLINE uint8_t bds_uint16_encode(const uint16_t * p_value, uint8_t * p_encoded_data);

static __INLINE uint8_t bds_sfloat_decode(const uint8_t, const uint8_t *, sfloat_t  *);

float decode_float_values(const uint8_t *, const uint16_t);

void on_write(ble_siemens_t * p_siemens, ble_evt_t const * p_ble_evt);

csfloat_t encode_float(float);
zero_float_t zero_encode_float(float);



/**@brief Function for encoding a uint40 value.
 *
 * @param[in]   p_value          Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t c_bds_uint64_encode(const uint64_t * p_value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((*p_value & 0x00000000000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((*p_value & 0x000000000000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((*p_value & 0x0000000000FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((*p_value & 0x00000000FF000000) >> 24);
    p_encoded_data[4] = (uint8_t) ((*p_value & 0x000000FF00000000) >> 32);
    p_encoded_data[5] = (uint8_t) ((*p_value & 0x0000FF0000000000) >> 40);
    p_encoded_data[6] = (uint8_t) ((*p_value & 0x00FF000000000000) >> 48);
    p_encoded_data[7] = (uint8_t) ((*p_value & 0xFF00000000000000) >> 56);
    return 8;
}

/**@brief Function for encoding a sfloat value.
 *
 * @param[in]   p_value          Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t c_bds_sfloat_encode(const csfloat_t * p_value, uint8_t * p_encoded_data)
{
    uint64_t encoded_val;

    encoded_val = (((uint64_t)p_value->exponent) << 48 & 0xFFFFFFFFFFFF0000) |
                            ((p_value->mantissa <<  0) & 0x000000000000FFFF);

    return(c_bds_uint64_encode(&encoded_val, p_encoded_data));
}

static __INLINE uint8_t zero_bds_sfloat_encode(const zero_float_t * p_value, uint8_t * p_encoded_data)
{
    uint64_t encoded_val;

    encoded_val = (((uint64_t)p_value->exponent) << 8 & 0xFF00000000000000) |
                            ((p_value->mantissa <<  0) & 0x00FFFFFFFFFFFFFF);

    return(bds_uint40_encode(&encoded_val, p_encoded_data));
}



/**@brief Function for decoding a sfloat value.
 *
 * @param[in]   len              length of the field to be decoded.
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @param[in]   p_decoded_val    pointer to the decoded value
 *
 * @return      length of the decoded field.

 */
/*static __INLINE uint8_t c_bds_sfloat_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          csfloat_t      * p_decoded_val)
{

    p_decoded_val->exponent = 0;
    c_bds_uint64_decode(len, p_encoded_data, (uint64_t*)&p_decoded_val->mantissa);
    p_decoded_val->exponent = (uint8_t)((p_decoded_val->mantissa & 0xFFFFFFFFFFFF0000) >> 48);
    p_decoded_val->mantissa &= 0x000000000000FFFF;
    return len;
}*/


#ifdef __cplusplus
}
#endif

#endif // BLE_DIS_H__

/** @} */
