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
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(SIEMENS_SERVICE)
#include "siemens_service.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "ble_conn_state.h"

//#define NUS_BASE_UUID      //{{0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */

#define SIEMENS_BASE_UUID  {{0x4E, 0xDE, 0x51, 0x86, 0xE8, 0xD2, 0xE4, 0xA2, 0x27, 0x4C, 0x6D, 0x11, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */


#define BLE_SIEMENS_MAX_RX_CHAR_LEN  BLE_SIEMENS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_DIS_SYS_ID_LEN 8                                                                                                  /**< Length of System ID Characteristic Value. */
#define BLE_DIS_PNP_ID_LEN 7

static uint16_t    service_handle;
uint8_t init_value[1] = {NULL};


/**@brief Function for encoding a System ID.
 *
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 * @param[in]   p_sys_id           System ID to be encoded.
 */
static void sys_id_encode(uint8_t * p_encoded_buffer, ble_siemens_sys_id_t const * p_sys_id)
{
    APP_ERROR_CHECK_BOOL(p_sys_id != NULL);
    APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);

    p_encoded_buffer[0] = (p_sys_id->manufacturer_id & 0x00000000FF);
    p_encoded_buffer[1] = (p_sys_id->manufacturer_id & 0x000000FF00) >> 8;
    p_encoded_buffer[2] = (p_sys_id->manufacturer_id & 0x0000FF0000) >> 16;
    p_encoded_buffer[3] = (p_sys_id->manufacturer_id & 0x00FF000000) >> 24;
    p_encoded_buffer[4] = (p_sys_id->manufacturer_id & 0xFF00000000) >> 32;

    p_encoded_buffer[5] = (p_sys_id->organizationally_unique_id & 0x0000FF);
    p_encoded_buffer[6] = (p_sys_id->organizationally_unique_id & 0x00FF00) >> 8;
    p_encoded_buffer[7] = (p_sys_id->organizationally_unique_id & 0xFF0000) >> 16;
}


/**@brief Function for encoding a PnP ID.
 *
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 * @param[in]   p_pnp_id           PnP ID to be encoded.
 */
static void pnp_id_encode(uint8_t * p_encoded_buffer, ble_siemens_pnp_id_t const * p_pnp_id)
{
    uint8_t len = 0;

    APP_ERROR_CHECK_BOOL(p_pnp_id != NULL);
    APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);

    p_encoded_buffer[len++] = p_pnp_id->vendor_id_source;

    len += uint16_encode(p_pnp_id->vendor_id, &p_encoded_buffer[len]);
    len += uint16_encode(p_pnp_id->product_id, &p_encoded_buffer[len]);
    len += uint16_encode(p_pnp_id->product_version, &p_encoded_buffer[len]);

    APP_ERROR_CHECK_BOOL(len == BLE_DIS_PNP_ID_LEN);
}


/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   rd_sec         Security requirement for reading characteristic value.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t char_add(uint16_t                      uuid,
                    ble_siemens_t                 * p_siemens,
                    const ble_siemens_init_t      * p_siemens_init,
                    uint8_t                         xteristics_index)
{
    ble_add_char_params_t           add_char_params;
    uint8_t                         init_len;
    ret_code_t                      err_code;
    uint8_t                         encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    ble_add_descr_params_t          add_descr_params;
    float_cast                      p_data;
    csfloat_t                        encode_p_data;
    uint8_t                         sizeof_encoded;
    

    switch (xteristics_index)
    {
        case MIN_INDEX:
        case MAX_INDEX:
        case ADC_INDEX:
            memset(&add_char_params, 0, sizeof(add_char_params));


            p_siemens->evt_handler = p_siemens_init->evt_handler;
            p_siemens->is_notification_supported = p_siemens_init->support_notification;
            p_siemens->battery_level_last.all_uint8_t = NULL;
            add_char_params.uuid              = uuid;
            add_char_params.max_len           = sizeof(uint16_t);
            add_char_params.init_len          = sizeof(uint16_t);
            add_char_params.p_init_value      = init_value;
            add_char_params.char_props.notify = p_siemens->is_notification_supported;
            add_char_params.char_props.read   = 1;
            add_char_params.cccd_write_access = p_siemens_init->siemens_cccd_wr_sec;
            add_char_params.read_access       = p_siemens_init->dis_char_rd_sec;

            err_code = characteristic_add(service_handle,
                                          &add_char_params,
                                          &(p_siemens->battery_level_handles));
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }

            if (p_siemens_init->p_report_ref != NULL)
            {
                // Add Report Reference descriptor
                init_len = ble_srv_report_ref_encode(encoded_report_ref, p_siemens_init->p_report_ref);

                memset(&add_descr_params, 0, sizeof(add_descr_params));
                add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
                add_descr_params.read_access = p_siemens_init->siemens_report_rd_sec;
                add_descr_params.init_len    = init_len;
                add_descr_params.max_len     = add_descr_params.init_len;
                add_descr_params.p_value     = encoded_report_ref;

                err_code = descriptor_add(p_siemens->battery_level_handles.value_handle,
                                          &add_descr_params,
                                          &p_siemens->report_ref_handle);
                return err_code;
            }
            else
            {
                p_siemens->report_ref_handle = BLE_GATT_HANDLE_INVALID;
            }

            return NRF_SUCCESS;
            break;

        case MEAN_INDEX:
        case RMS_INDEX:

            memset(&add_char_params, 0, sizeof(add_char_params));

            p_siemens->evt_handler = p_siemens_init->evt_handler;
            p_siemens->is_notification_supported = p_siemens_init->support_notification;
            p_data.f = p_siemens_init->characteristics->all_sfloat_t;
            encode_p_data.exponent = p_data.parts.exponent;
            encode_p_data.mantissa = p_data.parts.mantisa;
            p_siemens->battery_level_last.all_sfloat_t = NULL;

            add_char_params.uuid              = uuid;
            add_char_params.max_len           = sizeof(float);
            add_char_params.init_len          = sizeof(float);
            add_char_params.p_init_value      = init_value;
            add_char_params.char_props.notify = p_siemens->is_notification_supported;
            add_char_params.char_props.read   = 1;
            add_char_params.cccd_write_access = p_siemens_init->siemens_cccd_wr_sec;
            add_char_params.read_access       = p_siemens_init->dis_char_rd_sec;

            err_code = characteristic_add(service_handle,
                                          &add_char_params,
                                          &(p_siemens->battery_level_handles));
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }

            if (p_siemens_init->p_report_ref != NULL)
            {
                // Add Report Reference descriptor
                init_len = ble_srv_report_ref_encode(encoded_report_ref, p_siemens_init->p_report_ref);

                memset(&add_descr_params, 0, sizeof(add_descr_params));
                add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
                add_descr_params.read_access = p_siemens_init->siemens_report_rd_sec;
                add_descr_params.init_len    = init_len;
                add_descr_params.max_len     = add_descr_params.init_len;
                add_descr_params.p_value     = encoded_report_ref;

                err_code = descriptor_add(p_siemens->battery_level_handles.value_handle,
                                          &add_descr_params,
                                          &p_siemens->report_ref_handle);
                return err_code;
            }
            else
            {
                p_siemens->report_ref_handle = BLE_GATT_HANDLE_INVALID;
            }

            return NRF_SUCCESS;
            break;


        case COMMAND_INDEX:
        case ZERO_INDEX:
        case X_VALUE_INDEX:
        case Y_VALUE_INDEX:
        case Z_VALUE_INDEX:
        
            memset(&add_char_params, 0, sizeof(add_char_params));
            p_siemens->evt_handler                   = p_siemens_init->evt_handler;
            add_char_params.uuid                     = uuid;
            add_char_params.max_len                  = BLE_SIEMENS_MAX_RX_CHAR_LEN;
            add_char_params.init_len                 = BLE_SIEMENS_MAX_RX_CHAR_LEN;
            add_char_params.is_var_len               = true;
            add_char_params.char_props.write         = 1;
            add_char_params.char_props.write_wo_resp = 1;
            add_char_params.read_access              = SEC_OPEN;
            add_char_params.write_access             = SEC_OPEN;

            return characteristic_add(service_handle, &add_char_params, &p_siemens->battery_level_handles);
            break;
        
        case INCASE_INDEX:
            memset(&add_char_params, 0, sizeof(add_char_params));

            p_siemens->evt_handler = p_siemens_init->evt_handler;
            p_siemens->is_notification_supported = p_siemens_init->support_notification;
            p_siemens->battery_level_last.zero_incase = NULL;

            add_char_params.uuid              = uuid;
            add_char_params.max_len           = sizeof(uint64_t);
            add_char_params.init_len          = sizeof(uint64_t);
            add_char_params.p_init_value      = init_value;
            add_char_params.char_props.notify = p_siemens->is_notification_supported;
            add_char_params.char_props.read   = 1;
            add_char_params.cccd_write_access = p_siemens_init->siemens_cccd_wr_sec;
            add_char_params.read_access       = p_siemens_init->dis_char_rd_sec;

            err_code = characteristic_add(service_handle,
                                          &add_char_params,
                                          &(p_siemens->battery_level_handles));
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }

            if (p_siemens_init->p_report_ref != NULL)
            {
                // Add Report Reference descriptor
                init_len = ble_srv_report_ref_encode(encoded_report_ref, p_siemens_init->p_report_ref);

                memset(&add_descr_params, 0, sizeof(add_descr_params));
                add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
                add_descr_params.read_access = p_siemens_init->siemens_report_rd_sec;
                add_descr_params.init_len    = init_len;
                add_descr_params.max_len     = add_descr_params.init_len;
                add_descr_params.p_value     = encoded_report_ref;

                err_code = descriptor_add(p_siemens->battery_level_handles.value_handle,
                                          &add_descr_params,
                                          &p_siemens->report_ref_handle);
                return err_code;
            }
            else
            {
                p_siemens->report_ref_handle = BLE_GATT_HANDLE_INVALID;
            }

            return NRF_SUCCESS;
            break;


        default:
            //can't think of anything for now
            ;

    }
}

ret_code_t add_siemens_service(ble_siemens_t * p_siemens, ble_siemens_init_t const * p_siemens_init)
{
    ret_code_t   err_code;
    ble_uuid_t ble_uuid;

    ble_uuid128_t siemens_base_uuid = SIEMENS_BASE_UUID;
    uint8_t uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;
    
    for(int i = 0; i < NUMBER_OF_SIEMENS_XTERISTICS; i++)
    {
        if (&p_siemens[i] == NULL || p_siemens_init == NULL)
        {
            err_code = NRF_ERROR_NULL;
            APP_ERROR_CHECK(err_code);
        }
    }

    // Add service

    err_code = sd_ble_uuid_vs_add(&siemens_base_uuid, &uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN ;
    ble_uuid.uuid = BLE_UUID_SIEMENS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &service_handle);
     //printf("%d",err_code);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);
    
    
    
    
    //BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE);

    //err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    //VERIFY_SUCCESS(err_code);

    return err_code;
}


void on_write(ble_siemens_t * p_siemens, ble_evt_t const * p_ble_evt)
{
    for (uint8_t i = 0; i < NUMBER_OF_SIEMENS_XTERISTICS; i++)
    {
        //if (!p_siemens[i].is_notification_supported)
        //{
             //printf("\rpleasoo\n");
            //continue;
        //}

        ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        //printf("\rThe evt handle is %x and the cccd_handle is %x\n",p_evt_write->handle, p_siemens[i].battery_level_handles.cccd_handle);

        if (    (p_evt_write->handle == p_siemens[i].battery_level_handles.cccd_handle)
            &&  (p_evt_write->len == 2))
        {

            if (p_siemens[i].evt_handler == NULL)
            {
                continue;
            }

            ble_siemens_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SIEMENS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SIEMENS_EVT_NOTIFICATION_DISABLED;
            }
            evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

            // CCCD written, call application event handler.
            p_siemens[i].evt_handler(&p_siemens[i], &evt);
        } else if ((p_evt_write->handle == p_siemens[i].battery_level_handles.value_handle)
            && (p_evt_write->len > 0)
            && (p_siemens[i].evt_handler != NULL))
        {
            ble_siemens_evt_t evt;
            evt.evt_type              = BLE_SIEMENS_EVT_ON_WRITE;
            evt.p_data                = p_evt_write->data;
            evt.length                = p_evt_write->len;
            evt.conn_handle           = p_ble_evt -> evt.gatts_evt.conn_handle;
            p_siemens[i].evt_handler(&p_siemens[i], &evt);
        }

        else
        {
            // Do Nothing. This event is not relevant for this service.
            //printf("\r7695trfrj\n");
        }
    }
    
}


void ble_siemens_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }
    ble_siemens_t *p_siemens = (ble_siemens_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_siemens, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for sending notifications with the Battery Level characteristic.
 *
 * @param[in]   p_hvx_params Pointer to structure with notification data.
 * @param[in]   conn_handle  Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t siemens_notification_send(ble_gatts_hvx_params_t * const p_hvx_params,
                                            uint16_t                       conn_handle)
{
    ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
    if (err_code == NRF_SUCCESS)
    {
        //NRF_LOG_INFO("Battery notification has been sent using conn_handle: 0x%04X", conn_handle);
    }
    else
    {
        //NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                      //err_code,
                      //conn_handle);
    }
    return err_code;
}


ret_code_t ble_siemens_service_update(ble_siemens_t    * p_siemens,
                                      uint8_t          * value_buffer,
                                      uint16_t         * p_length,
                                      uint16_t         conn_handle,
                                      float            unencoded_value)
{

    if (p_siemens == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t               err_code = NRF_SUCCESS;
    ble_gatts_value_t        gatts_value;

    //printf("\r%d\n",value_buffer[0]);
    //printf("%d\n",&value_buffer[1]);


    if (unencoded_value  - p_siemens->battery_level_last.all_sfloat_t > 0 || unencoded_value  - p_siemens->battery_level_last.all_sfloat_t < 0)
    {
         p_siemens->battery_level_last.all_sfloat_t = unencoded_value;
        if (p_siemens->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_siemens->battery_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = p_length;
            hvx_params.p_data = value_buffer;



            if (conn_handle == BLE_CONN_HANDLE_ALL)
            {
                ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

                // Try sending notifications to all valid connection handles.
                for (uint32_t i = 0; i < conn_handles.len; i++)
                {
                    if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
                    {
                        if (err_code == NRF_SUCCESS)
                        {
                            err_code = siemens_notification_send(&hvx_params,
                                                                 conn_handles.conn_handles[i]);
                        }
                        else
                        {
                            // Preserve the first non-zero error code
                            UNUSED_RETURN_VALUE(siemens_notification_send(&hvx_params,
                                                                          conn_handles.conn_handles[i]));
                        }
                    }
                }
            }
            else
            {
                err_code = siemens_notification_send(&hvx_params, conn_handle);
            }
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }
}


ret_code_t ble_siemens_services_on_reconnection_update(ble_siemens_t * p_siemens,
                                                      uint8_t     xteristics_index,
                                                      uint16_t    conn_handle)
{
    if (&p_siemens[xteristics_index] == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err_code;

    if (p_siemens[xteristics_index].is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint16_t               len = sizeof(uint8_t);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_siemens[xteristics_index].battery_level_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = &p_siemens[xteristics_index].battery_level_last;

        err_code = siemens_notification_send(&hvx_params, conn_handle);

        return err_code;
    }

    return NRF_ERROR_INVALID_STATE;
}

float decode_float_values(const uint8_t *p_encoded_value, const uint16_t len)
{
    //start
    sfloat_t p_decoded_value;
    float mantissa;
    bds_sfloat_decode(len, p_encoded_value, &p_decoded_value);
    //printf("\rThe Mantissa is %d and the exponent is %d\n", p_decoded_value.mantissa,p_decoded_value.exponent);

    mantissa = p_decoded_value.mantissa;

    while (mantissa >= 1)
    {
        mantissa /= 2;
    }

    return ldexp(mantissa, p_decoded_value.exponent);
}

csfloat_t encode_float(float num)
{
    float mantissa; 
    int exponent, bit_count;
    uint64_t int_mantissa;
    csfloat_t encode;
    mantissa = frexp(num, &exponent);
    encode.exponent = (int8_t) exponent;

    bit_count = 0;
    int_mantissa = (int)mantissa;

    while (abs(int_mantissa) < 281474976710655UL && bit_count < 48)
    {
        mantissa *= 2;
        int_mantissa = (int)mantissa;
        bit_count += 1;
    }

    encode.mantissa = (unsigned short) int_mantissa;

    return encode; 

}

zero_float_t zero_encode_float(float num)
{
    float mantissa; 
    int exponent, bit_count;
    uint64_t int_mantissa;
    zero_float_t encode;
    mantissa = frexp(num, &exponent);
    encode.exponent = (int8_t) exponent;

    bit_count = 0;
    int_mantissa = (int)mantissa;

    while (abs(int_mantissa) < 255 && bit_count < 8)
    {
        mantissa *= 2;
        int_mantissa = (int)mantissa;
        bit_count += 1;
    }

    encode.mantissa = (unsigned short) int_mantissa;

    return encode; 

}

#endif // NRF_MODULE_ENABLED(BLE_DIS)