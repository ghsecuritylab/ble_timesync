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

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(SIEMENS_C_SERVICE)
#include <stdlib.h>
#include <math.h>

#include "ble.h"
#include "siemens_c_service.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "ble_db_discovery.h"
#include "ble_types.h"

#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_siemens_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK       0x0F                  /**< TX Buffer mask, must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE       (TX_BUFFER_MASK + 1)  /**< Size of the send buffer, which is 1 higher than the mask. */
#define WRITE_MESSAGE_LENGTH BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT -  OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;


static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */


/**@brief Function for passing any pending request from the buffer to the stack.
 */
/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("SD Read/Write API returns Success..");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                "attempted again..");
        }
    }
}


uint32_t ble_siemens_c_init(ble_siemens_c_t * p_ble_siemens_c, ble_siemens_c_init_t * p_ble_siemens_c_init)
{
    uint32_t      err_code;
    uint8_t uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;
    ble_uuid_t    siemens_uuid;
    ble_uuid128_t siemens_base_uuid = SIEMENS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c_init);

    err_code = sd_ble_uuid_vs_add(&siemens_base_uuid, &uuid_type);
    VERIFY_SUCCESS(err_code);

    siemens_uuid.type = uuid_type;
    siemens_uuid.uuid = BLE_UUID_SIEMENS_SERVICE;

    p_ble_siemens_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_siemens_c->evt_handler           = p_ble_siemens_c_init->evt_handler;


    // services 
    p_ble_siemens_c->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.max_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.min_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.rms_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.mean_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.command_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.zero_crossing_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.x_value_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.y_value_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_siemens_c->peer_siemens_db.z_value_handle = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&siemens_uuid);
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
        handle_cccd,conn_handle);
        printf("Configuring CCCD. CCCD Handle = %x, Connection Handle = %x",
        handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

/*static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };

    return sd_ble_gattc_write(conn_handle, &write_params);
}*/


uint32_t ble_siemens_c_min_notif_enable(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if ( (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_siemens_c->peer_siemens_db.min_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_siemens_c->conn_handle,p_ble_siemens_c->peer_siemens_db.min_cccd_handle, true);
}

uint32_t ble_siemens_c_max_notif_enable(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if ( (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_siemens_c->peer_siemens_db.max_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_siemens_c->conn_handle,p_ble_siemens_c->peer_siemens_db.max_cccd_handle, true);
}


uint32_t ble_siemens_c_zero_crossing_notif_enable(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if ( (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_siemens_c->peer_siemens_db.zero_crossing_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_siemens_c->conn_handle,p_ble_siemens_c->peer_siemens_db.zero_crossing_cccd_handle, true);
}


uint32_t ble_siemens_c_rms_notif_enable(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if ( (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_siemens_c->peer_siemens_db.rms_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_siemens_c->conn_handle,p_ble_siemens_c->peer_siemens_db.rms_cccd_handle, true);
}

uint32_t ble_siemens_c_mean_notif_enable(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if ( (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_siemens_c->peer_siemens_db.mean_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_siemens_c->conn_handle,p_ble_siemens_c->peer_siemens_db.mean_cccd_handle, true);
}


uint32_t ble_siemens_c_min_read(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    tx_message_t * msg;

    msg                  = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    msg->req.read_handle = p_ble_siemens_c->peer_siemens_db.min_handle;
    msg->conn_handle     = p_ble_siemens_c->conn_handle;
    msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_siemens_c_max_read(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    tx_message_t * msg;

    msg                  = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    msg->req.read_handle = p_ble_siemens_c->peer_siemens_db.max_handle;
    msg->conn_handle     = p_ble_siemens_c->conn_handle;
    msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_siemens_c_rms_read(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    tx_message_t * msg;

    msg                  = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    msg->req.read_handle = p_ble_siemens_c->peer_siemens_db.rms_handle;
    msg->conn_handle     = p_ble_siemens_c->conn_handle;
    msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_siemens_c_mean_read(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    tx_message_t * msg;

    msg                  = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    msg->req.read_handle = p_ble_siemens_c->peer_siemens_db.mean_handle;
    msg->conn_handle     = p_ble_siemens_c->conn_handle;
    msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_siemens_c_zero_crossing_read(ble_siemens_c_t * p_ble_siemens_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    tx_message_t * msg;

    msg                  = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    msg->req.read_handle = p_ble_siemens_c->peer_siemens_db.zero_crossing_handle;
    msg->conn_handle     = p_ble_siemens_c->conn_handle;
    msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}



/**@brief Function for handling write response events.
 *
 * @param[in] p_bas_c   Pointer to the Battery Service Client Structure.
 * @param[in] p_ble_evt Pointer to the SoftDevice event.
 */
static void on_write_rsp(ble_siemens_c_t * p_siemens_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_siemens_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


/**@brief     Function for handling read response events.
 *
 * @details   This function will validate the read response and raise the appropriate
 *            event to the application.
 *
 * @param[in] p_bas_c   Pointer to the Battery Service Client Structure.
 * @param[in] p_ble_evt Pointer to the SoftDevice event.
 */
static void on_read_rsp(ble_siemens_c_t * p_siemens_c, ble_evt_t const * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;
    // Check if the event if on the link for this instance
    if (p_siemens_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

    if (p_response->handle == p_siemens_c->peer_siemens_db.min_handle)
    {
        ble_siemens_c_evt_t evt;

        evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        evt.evt_type = BLE_SIEMENS_C_EVT_MIN_READ_RESP;

        evt.params.min_value[0] = p_response->data[0];
        evt.params.min_value[1] = p_response->data[1];
        evt.params.min_value[2] = p_response->data[2];
        evt.params.min_value[3] = p_response->data[3];
        evt.params.min_value[4] = p_response->data[4];
        evt.params.min_value[5] = p_response->data[5];
        evt.params.min_value[6] = p_response->data[6];
        evt.params.min_value[7] = p_response->data[7];

        p_siemens_c->evt_handler(p_siemens_c, &evt);
    }

    if (p_response->handle == p_siemens_c->peer_siemens_db.max_handle)
    {
        ble_siemens_c_evt_t evt;

        evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        evt.evt_type = BLE_SIEMENS_C_EVT_MAX_READ_RESP;

        evt.params.max_value[0] = p_response->data[0];
        evt.params.max_value[1] = p_response->data[1];
        evt.params.max_value[2] = p_response->data[2];
        evt.params.max_value[3] = p_response->data[3];
        evt.params.max_value[4] = p_response->data[4];
        evt.params.max_value[5] = p_response->data[5];
        evt.params.max_value[6] = p_response->data[6];
        evt.params.max_value[7] = p_response->data[7];

        p_siemens_c->evt_handler(p_siemens_c, &evt);
    }


    //printf("\r%x  %x\n",p_response->handle, p_siemens_c[i].peer_siemens_db.rms_handle);
    if (p_response->handle == p_siemens_c->peer_siemens_db.rms_handle)
    {
        ble_siemens_c_evt_t evt;

        evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        evt.evt_type = BLE_SIEMENS_C_EVT_RMS_READ_RESP;

        //evt.params.rms_value[0] = p_response->data[0];
        //evt.params.rms_value[1] = p_response->data[1];
        evt.params.rms_value[0] = p_response->data[0];
        evt.params.rms_value[1] = p_response->data[1];
        evt.params.rms_value[2] = p_response->data[2];
        evt.params.rms_value[3] = p_response->data[3];
        evt.params.rms_value[4] = p_response->data[4];
        evt.params.rms_value[5] = p_response->data[5];
        evt.params.rms_value[6] = p_response->data[6];
        evt.params.rms_value[7] = p_response->data[7];

        p_siemens_c->evt_handler(p_siemens_c, &evt);
    }


    //printf("\r%x  %x\n",p_response->handle, p_siemens_c[i].peer_siemens_db.zero_crossing_handle);
    if (p_response->handle == p_siemens_c->peer_siemens_db.zero_crossing_handle)
    {
        ble_siemens_c_evt_t evt;

        evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        evt.evt_type = BLE_SIEMENS_C_EVT_ZERO_CROSSING_READ_RESP;

        evt.params.zero_crossing[0] = p_response->data[0];
        evt.params.zero_crossing[1] = p_response->data[1];
        evt.params.zero_crossing[2] = p_response->data[2];
        evt.params.zero_crossing[3] = p_response->data[3];
        evt.params.zero_crossing[4] = p_response->data[4];

        if(p_response -> len > 5)
        {
            evt.params.zero_crossing[5] = p_response->data[5];
            evt.params.zero_crossing[6] = p_response->data[6];
            evt.params.zero_crossing[7] = p_response->data[7];
        }

        p_siemens_c->evt_handler(p_siemens_c, &evt);
    }

    if (p_response->handle == p_siemens_c->peer_siemens_db.mean_handle)
    {
        ble_siemens_c_evt_t evt;

        evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        evt.evt_type = BLE_SIEMENS_C_EVT_MEAN_READ_RESP;

        evt.params.mean_value[0] = p_response->data[0];
        evt.params.mean_value[1] = p_response->data[1];
        evt.params.mean_value[2] = p_response->data[2];
        evt.params.mean_value[3] = p_response->data[3];
        evt.params.mean_value[4] = p_response->data[4];
        evt.params.mean_value[5] = p_response->data[5];
        evt.params.mean_value[6] = p_response->data[6];
        evt.params.mean_value[7] = p_response->data[7];
        //evt.params.mean_value[1] = p_response->data[1];

        p_siemens_c->evt_handler(p_siemens_c, &evt);
    }
    // Check if there is any buffered transmissions and send them.
    tx_buffer_process();

}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will handle the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the Battery Level measurement from the peer. If
 *            so, this function will decode the battery level measurement and send it to the
 *            application.
 *
 * @param[in] p_ble_bas_c Pointer to the Battery Service Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_siemens_c_t * p_ble_siemens_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_siemens_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if this notification is a min_value notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_siemens_c->peer_siemens_db.min_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len >= 1)
        {
            ble_siemens_c_evt_t ble_siemens_c_evt;
            ble_siemens_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
            ble_siemens_c_evt.evt_type    = BLE_SIEMENS_C_EVT_MIN_NOTIFICATION;

            ble_siemens_c_evt.params.min_value[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_siemens_c_evt.params.min_value[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_siemens_c_evt.params.min_value[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_siemens_c_evt.params.min_value[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_siemens_c_evt.params.min_value[4] = p_ble_evt->evt.gattc_evt.params.hvx.data[4];
            ble_siemens_c_evt.params.min_value[5] = p_ble_evt->evt.gattc_evt.params.hvx.data[5];
            ble_siemens_c_evt.params.min_value[6] = p_ble_evt->evt.gattc_evt.params.hvx.data[6];
            ble_siemens_c_evt.params.min_value[7] = p_ble_evt->evt.gattc_evt.params.hvx.data[7];

            p_ble_siemens_c->evt_handler(p_ble_siemens_c, &ble_siemens_c_evt);
        }
    }

    // Check if this notification is a max_value notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_siemens_c->peer_siemens_db.max_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len >= 1)
        {
            ble_siemens_c_evt_t ble_siemens_c_evt;
            ble_siemens_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
            ble_siemens_c_evt.evt_type    = BLE_SIEMENS_C_EVT_MAX_NOTIFICATION;

            ble_siemens_c_evt.params.max_value[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_siemens_c_evt.params.max_value[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_siemens_c_evt.params.max_value[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_siemens_c_evt.params.max_value[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_siemens_c_evt.params.max_value[4] = p_ble_evt->evt.gattc_evt.params.hvx.data[4];
            ble_siemens_c_evt.params.max_value[5] = p_ble_evt->evt.gattc_evt.params.hvx.data[5];
            ble_siemens_c_evt.params.max_value[6] = p_ble_evt->evt.gattc_evt.params.hvx.data[6];
            ble_siemens_c_evt.params.max_value[7] = p_ble_evt->evt.gattc_evt.params.hvx.data[7];

            p_ble_siemens_c->evt_handler(p_ble_siemens_c, &ble_siemens_c_evt);
        }
    }

    // Check if this notification is a battery level notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_siemens_c->peer_siemens_db.rms_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len >= 1)
        {
            ble_siemens_c_evt_t ble_siemens_c_evt;
            ble_siemens_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
            ble_siemens_c_evt.evt_type    = BLE_SIEMENS_C_EVT_RMS_NOTIFICATION;

            ble_siemens_c_evt.params.rms_value[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_siemens_c_evt.params.rms_value[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_siemens_c_evt.params.rms_value[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_siemens_c_evt.params.rms_value[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_siemens_c_evt.params.rms_value[4] = p_ble_evt->evt.gattc_evt.params.hvx.data[4];
            ble_siemens_c_evt.params.rms_value[5] = p_ble_evt->evt.gattc_evt.params.hvx.data[5];
            ble_siemens_c_evt.params.rms_value[6] = p_ble_evt->evt.gattc_evt.params.hvx.data[6];
            ble_siemens_c_evt.params.rms_value[7] = p_ble_evt->evt.gattc_evt.params.hvx.data[7];

            p_ble_siemens_c->evt_handler(p_ble_siemens_c, &ble_siemens_c_evt);
        }
    }

    // Check if this notification is a battery level notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_siemens_c->peer_siemens_db.zero_crossing_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len >= 1)
        {
            ble_siemens_c_evt_t ble_siemens_c_evt;
            ble_siemens_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
            ble_siemens_c_evt.evt_type    = BLE_SIEMENS_C_EVT_ZERO_CROSSING_NOTIFICATION;
            ble_siemens_c_evt.params.zero_crossing[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_siemens_c_evt.params.zero_crossing[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_siemens_c_evt.params.zero_crossing[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_siemens_c_evt.params.zero_crossing[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_siemens_c_evt.params.zero_crossing[4] = p_ble_evt->evt.gattc_evt.params.hvx.data[4];

            if(p_ble_evt->evt.gattc_evt.params.hvx.len > 5)
            {
                ble_siemens_c_evt.params.zero_crossing[5] = p_ble_evt->evt.gattc_evt.params.hvx.data[5];
                ble_siemens_c_evt.params.zero_crossing[6] = p_ble_evt->evt.gattc_evt.params.hvx.data[6];
                ble_siemens_c_evt.params.zero_crossing[7] = p_ble_evt->evt.gattc_evt.params.hvx.data[7];
            }

            p_ble_siemens_c->evt_handler(p_ble_siemens_c, &ble_siemens_c_evt);
        }
    }


    // Check if this notification is a battery level notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_siemens_c->peer_siemens_db.mean_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len >= 1)
        {
            ble_siemens_c_evt_t ble_siemens_c_evt;
            ble_siemens_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
            ble_siemens_c_evt.evt_type    = BLE_SIEMENS_C_EVT_MEAN_NOTIFICATION;

            ble_siemens_c_evt.params.mean_value[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_siemens_c_evt.params.mean_value[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_siemens_c_evt.params.mean_value[2] = p_ble_evt->evt.gattc_evt.params.hvx.data[2];
            ble_siemens_c_evt.params.mean_value[3] = p_ble_evt->evt.gattc_evt.params.hvx.data[3];
            ble_siemens_c_evt.params.mean_value[4] = p_ble_evt->evt.gattc_evt.params.hvx.data[4];
            ble_siemens_c_evt.params.mean_value[5] = p_ble_evt->evt.gattc_evt.params.hvx.data[5];
            ble_siemens_c_evt.params.mean_value[6] = p_ble_evt->evt.gattc_evt.params.hvx.data[6];
            ble_siemens_c_evt.params.mean_value[7] = p_ble_evt->evt.gattc_evt.params.hvx.data[7];

            p_ble_siemens_c->evt_handler(p_ble_siemens_c, &ble_siemens_c_evt);
        }
    }
}



void ble_siemens_on_db_disc_evt(ble_siemens_c_t * p_ble_siemens_c, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the Battery Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE
        &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_SIEMENS_SERVICE
        &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
    {
        // Find the CCCD Handle of the Battery Level characteristic.
        uint8_t i;

        ble_siemens_c_evt_t evt;
        evt.evt_type    = BLE_SIEMENS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;
        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_MIN_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.min_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.siemens_db.min_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_MAX_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.max_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.siemens_db.max_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_RMS_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.rms_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.siemens_db.rms_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_ZERO_CROSSING_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.zero_crossing_cccd_handle = 
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.siemens_db.zero_crossing_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_MEAN_VALUE_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.mean_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.siemens_db.mean_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_COMMAND_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.command_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_X_VALUE_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.x_value_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_Y_VALUE_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.y_value_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_SIEMENS_Z_VALUE_CHARACTERISTIC)
            {
                // Found Battery Level characteristic. Store CCCD handle and break.
                evt.params.siemens_db.z_value_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            else{}
        }

        NRF_LOG_DEBUG("Battery Service discovered at peer.");

        //If the instance has been assigned prior to db_discovery, assign the db_handles
        if (p_ble_siemens_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_siemens_c->peer_siemens_db.min_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_siemens_c->peer_siemens_db.min_handle      == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_siemens_c->peer_siemens_db = evt.params.siemens_db;
            }
            
            if ((p_ble_siemens_c->peer_siemens_db.max_cccd_handle   == BLE_GATT_HANDLE_INVALID)&&
                       (p_ble_siemens_c->peer_siemens_db.max_handle == BLE_GATT_HANDLE_INVALID))

            {
                p_ble_siemens_c->peer_siemens_db = evt.params.siemens_db;

            } 
            
            
            if ((p_ble_siemens_c->peer_siemens_db.rms_cccd_handle   == BLE_GATT_HANDLE_INVALID)&&
                       (p_ble_siemens_c->peer_siemens_db.rms_handle == BLE_GATT_HANDLE_INVALID))

            {
                p_ble_siemens_c->peer_siemens_db = evt.params.siemens_db;

            }
 
            
            
            if ((p_ble_siemens_c->peer_siemens_db.zero_crossing_cccd_handle    == BLE_GATT_HANDLE_INVALID)&&
                       (p_ble_siemens_c->peer_siemens_db.zero_crossing_handle  == BLE_GATT_HANDLE_INVALID))

            {
                p_ble_siemens_c->peer_siemens_db = evt.params.siemens_db;

            }

            
            if ((p_ble_siemens_c->peer_siemens_db.mean_cccd_handle    == BLE_GATT_HANDLE_INVALID)&&
                       (p_ble_siemens_c->peer_siemens_db.mean_handle  == BLE_GATT_HANDLE_INVALID))

            {
                p_ble_siemens_c->peer_siemens_db = evt.params.siemens_db;

            }
        }

        p_ble_siemens_c->evt_handler(p_ble_siemens_c, &evt);
    }
    else
    {
        NRF_LOG_DEBUG("Battery Service discovery failure at peer. ");
    }
}

/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check if the disconnect event is happening on the link
 *            associated with the current instance of the module, if so it will set its
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_bas_c Pointer to the Battery Service Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_siemens_c_t * p_ble_siemens_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_siemens_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        ble_siemens_c_evt_t siemens_c_evt;
        siemens_c_evt.evt_type = BLE_SIEMENS_C_EVT_DISCONNECTED;
        p_ble_siemens_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.min_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.min_handle      = BLE_GATT_HANDLE_INVALID;

        p_ble_siemens_c->peer_siemens_db.max_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.max_handle      = BLE_GATT_HANDLE_INVALID;

        p_ble_siemens_c->peer_siemens_db.rms_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.rms_handle      = BLE_GATT_HANDLE_INVALID;

        p_ble_siemens_c->peer_siemens_db.zero_crossing_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.zero_crossing_handle      = BLE_GATT_HANDLE_INVALID;

        p_ble_siemens_c->peer_siemens_db.mean_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_siemens_c->peer_siemens_db.mean_handle      = BLE_GATT_HANDLE_INVALID;
    }
}

void ble_siemens_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_ble_evt == NULL) || (p_context == NULL))
    {
        return;
    }

    ble_siemens_c_t * p_ble_siemens_c = (ble_siemens_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_siemens_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_siemens_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(p_ble_siemens_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_siemens_c, p_ble_evt);
            break;

        default:
            break;
    }
}


uint32_t ble_siemens_c_handles_assign(ble_siemens_c_t *    p_ble_siemens_c,
                                  uint16_t         conn_handle,
                                  ble_siemens_c_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    p_ble_siemens_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_siemens_c->peer_siemens_db = *p_peer_handles;
    }
    return NRF_SUCCESS;
}

uint32_t ble_siemens_c_write(ble_siemens_c_t * p_ble_siemens_c, uint8_t * p_string, uint16_t length, uint16_t xteristic)
{
    VERIFY_PARAM_NOT_NULL(p_ble_siemens_c);

    if (length > m_ble_nus_max_data_len)
    {
        NRF_LOG_WARNING("Content too long.");
        //printf("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_siemens_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t write_params;
    switch(xteristic)
    {
        
        case COMMAND_INDEX:
            write_params.handle = p_ble_siemens_c -> peer_siemens_db.command_handle;
            break;
        case X_INDEX:
            write_params.handle = p_ble_siemens_c -> peer_siemens_db.x_value_handle;
            break;
        case Y_INDEX:
            write_params.handle = p_ble_siemens_c -> peer_siemens_db.y_value_handle;
            break;
        case Z_INDEX:
            write_params.handle = p_ble_siemens_c -> peer_siemens_db.z_value_handle;
            break;

        default:
            ;
    }
    write_params.write_op = BLE_GATT_OP_WRITE_CMD;
    write_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.offset = 0;
    write_params.len = length;
    write_params.p_value = p_string;
    //ble_gattc_write_params_t const write_params =
    //{
        //.write_op = BLE_GATT_OP_WRITE_CMD,
        //.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        //.handle   = p_ble_siemens_c->peer_siemens_db.command_handle,
        //.offset   = 0,
        //.len      = length,
        //.p_value  = p_string
    //};

    return sd_ble_gattc_write(p_ble_siemens_c->conn_handle, &write_params);
}

sfloat_t encode_float(float num)
{
    float mantissa; 
    int exponent, bit_count, int_mantissa;
    sfloat_t encode;
    mantissa = frexp(num, &exponent);
    encode.exponent = (int8_t) exponent;

    bit_count = 0;
    int_mantissa = (int)mantissa;

    while (abs(int_mantissa) < 4095 && bit_count < 12)
    {
        mantissa *= 2;
        int_mantissa = (int)mantissa;
        bit_count += 1;
    }

    encode.mantissa = (unsigned short) int_mantissa;

    return encode; 

}


/**@brief Function for decoding a uint40 value.
 *
 * @param[in]   len              length of the field to be decoded.
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @param[in]   p_decoded_val    pointer to the decoded value
 *
 * @return      length of the decoded field.
 */
static __INLINE uint8_t c_bds_uint40_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          uint64_t      * p_decoded_val)
{
    UNUSED_VARIABLE(len);
    *p_decoded_val = (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
                     (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
                     (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
                     (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24 )|
                     (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32 );
    return (40);
}

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



/**@brief Function for decoding a sfloat value.
 *
 * @param[in]   len              length of the field to be decoded.
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @param[in]   p_decoded_val    pointer to the decoded value
 *
 * @return      length of the decoded field.

 */
static __INLINE uint8_t c_bds_sfloat_decode(const uint8_t len,
                                          const uint8_t * p_encoded_data,
                                          csfloat_t      * p_decoded_val)
{

    p_decoded_val->exponent = 0;
    bds_uint40_decode(len, p_encoded_data, (uint64_t*)&p_decoded_val->mantissa);
    p_decoded_val->exponent = (uint8_t)((p_decoded_val->mantissa & 0xFFFFFFFFFFFFFFF0) >> 60);
    p_decoded_val->mantissa &= 0x000000000000000F;
    return len;
}

float decode_float5_values(const uint8_t *p_encoded_value, const uint16_t len)
{
    //start
    zero_float_t p_decoded_value;
    float mantissa;
    bds_float5_decode(len, p_encoded_value, &p_decoded_value);
    mantissa = p_decoded_value.mantissa;

    while (mantissa >= 1)
    {
        mantissa /= 2;
    }

    return ldexp(mantissa, p_decoded_value.exponent);
}

float decode_float8_values(const uint8_t *p_encoded_value, const uint16_t len)
{
    //start
    csfloat_t p_decoded_value;
    float mantissa;
    bds_float8_decode(len, p_encoded_value, &p_decoded_value);
    mantissa = p_decoded_value.mantissa;

    while (mantissa >= 1)
    {
        mantissa /= 2;
    }

    return ldexp(mantissa, p_decoded_value.exponent);
}


#endif // NRF_MODULE_ENABLED(BLE_SIEMENS_C)
