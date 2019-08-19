/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
//#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_drv_ppi.h"
#include "nrf_timer.h"
#include "time_sync.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "siemens_c_service.h"
#include "nrf_drv_timer.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define SIEMENS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */



#define SCAN_INTERVAL             0x00A0                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)      /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)       /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                     /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(600, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE               2                                     /**< Size of a UUID, in bytes. */

#define SAMPLES_IN_BUFFER 1
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

static char const m_target_periph_name[] = "Nordic_Blinky";         /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */
char m_target_periph_name_array[3][32];
volatile uint8_t state = 1;
static nrf_saadc_value_t  m_buffer[SAMPLES_IN_BUFFER];
static uint8_t saadc_monitor = 0; // to ensure that computation of sample mean runs once
static float sample_mean = 0;
uint8_t uart_tracker = 0;
static int max_command_length = 70;
char address[30];
char command[50];
uint8_t temp_address_array[BLE_GAP_ADDR_LEN];

char get_address_fragments[10];
static uint8_t match_index = 0;
uint8_t data_to_send[50];
char arr[50];
const bool BLE_DATA_READY = false;
static bool processed = false;
static uint8_t slave_list_index_where_match_occured = 4;
bool measurement_begin = false;
float store_analogValues[20] = {[0 ... 19] = 0};
uint32_t get_timestamp[20] = {[0 ... 19] = 0};
uint16_t sampling_size = 20;
uint16_t copy_sampling_size = 20;
uint16_t sample_max = 0;
uint16_t sample_min = 0;
float    root_mean_square = 0.0;
float zero_crossing[20];
uint8_t notification_track = 0;
uint8_t command_value[1] = {1};
bool data_ready = false;

//DEBUG
bool debug = false;

//UART CONSTANTS
const char *CHECK_UART = "AT";
const char *ADD_NODE   = "AT+ADD+";//N=SENSORNAME+M=123456789000+V=I+X=1234+Y=1234+Z=1234";
const char *ADD_NODE_N = "N";
const char *ADD_NODE_M = "M";
const char *ADD_NODE_V = "V";
const char *ADD_NODE_X = "X";
const char *ADD_NODE_Y = "Y";
const char *ADD_NODE_Z = "Z";
const char *REMOVE_NODE    = "AT+RMM+";//N=SENSORNAME+M=123456789000
const char *START_CONNECTION = "AT+CONN=1";
const char *SHOW_CONNECTED_DEVICES = "AT+CONN=?";
const char *START_MEASURING = "AT+MEAS";
const char *RESET = "AT+Z0";
const char *SUBSCRIBE = "AT+SUB";
const char *DEBUG     = "SWITCHDEBUG";


const int UART_VALID_INPUT = 1;
const int UART_CARRIGE_RETURN = 2;
const int UNART_UNKNOWN_CHAR = -1;

const uint8_t UART_CR_CODE = 13;
const uint8_t UART_ALPHABETH_MIN = 32;
const uint8_t UART_ALPHABETH_MAX = 126;

const int UART_COMMAND_RESET = 0;
const int UART_COMMAND_CHECK = 1;
const int UART_COMMAND_MEASURING = 2;
const int UART_COMMAND_START_CONNECTION = 3;
const int UART_SHOW_CONNECTED_DEVICES = 4;
const int UART_ADD_ADDRESS_TO_WHITELIST = 5;
const int UART_TOGGLE_DEBUG = 6;
const int UART_REMOVE_SENSOR = 7;
const int UART_COMMAND_UNKNOWN = -1;

const char* UART_RESPONSE_OK       = "\r\nOK.\r\n";
const char* UART_RESPONSE_OK1      = "\r\nOK1.\r\n";
const char* UART_RESPONSE_OK2      = "\r\nOK2.\r\n";
const char* UART_RESPONSE_OK3      = "\r\nOK3.\r\n";
const char* UART_RESPONSE_OK4      = "\r\nOK4.\r\n";
const char* UART_RESPONSE_OK5      = "\r\nOK5.\r\n";
const char* UART_RESPONSE_OK_DEBUG = "\r\nOK_DB.\r\n";
const char* UART_RESPONSE_OK6      = "\r\nOK6.\r\n";
const char* UART_RESPONSE_ERR      = "\r\nERR.\r\n";
const char* UART_RESPONSE_ERR_ITL  = "\r\nERR-INPUTTOLONG\r\n";
const char* UART_RESPONSE_ERR_INK  = "\r\nERR2-COMMANDUNKNOWN.\r\n";
const char* UART_RESPONSE_ERR_FUL  = "\r\nERR3-FULL.\r\n";
const char* UART_RESPONSE_ERR_404  = "\r\nERR3-404.\r\n";
const char* UART_RESPONSE_ERR_ADD  = "\r\nERRADD\r\n";

//definition of the sensor values should come via uart later
//const float sensor_1_x = 43.0f;
//const float sensor_1_y = 3.24f;
//const float sensor_1_z = 124.0f;

//const float sensor_2_x = 43.0f;
//const float sensor_2_y = 3.24f;
//const float sensor_2_z = 124.0f;

//const float sensor_3_x = 43.0f;
//const float sensor_3_y = 3.24f;
//const float sensor_3_z = 124.0f;

const float sensor_x = 43.0f;
const float sensor_y = 3.24f;
const float sensor_z = 124.0f;

//const float sensor_constants[3] = {43.0, 3.24, 124.0};


/**
 * @brief This structure contains various information on the nodes that should connect to the master 
 */
typedef struct
{
    char name[32];     // Name of the Sensor
    char mac[32];      // MAC Address
    char value[10];           // Value to be measured 
    uint8_t x[2];               // x value 
    uint8_t y[2];               // y value
    uint8_t z[2];               // z value

} slave_information;
const int max_nr_of_slaves = 3;
int current_nr_of_slaves = 0;
slave_information slave_list[3];

bool currently_scanning = false;



//BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);        /**< BLE Nordic UART Service (NUS) client instance. */
BLE_SIEMENS_C_ARRAY_DEF(m_siemens_c,NRF_SDH_BLE_CENTRAL_LINK_COUNT);
//BLE_SIEMENS_C_DEF(m_siemens_c);                                             /**< Structure used to identify the Battery Service client module. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
//BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc,NRF_SDH_BLE_CENTRAL_LINK_COUNT);
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT -  OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid[] =
{
    {BLE_UUID_SIEMENS_SERVICE, SIEMENS_SERVICE_UUID_TYPE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};





static ble_gap_addr_t connected_peer_addr[NRF_SDH_BLE_CENTRAL_LINK_COUNT] = {0x00};


/**@brief - Encode x,y,z values for the sensors. */
void encode_sensor_constants()
{
    sfloat_t split_float;
    //encode sensor values and assign to corresponding variables
    for (uint8_t i = 0; i < sizeof(slave_list)/sizeof(slave_list[0]); i++)
    {
          //split_float = encode_float(sensor_constants[i]);
          split_float = encode_float(sensor_x);
          bds_sfloat_encode(&split_float, slave_list[i].x);
          split_float = encode_float(sensor_y);
          bds_sfloat_encode(&split_float, slave_list[i].y);
          split_float = encode_float(sensor_z);
          bds_sfloat_encode(&split_float, slave_list[i].z);
    }
}



/*
 * Checks the validity of an incoming uart char 
 */
bool check_uart_input(uint8_t *uart_input_string)
{
    bool valid_input = false;
    uint8_t str_size = strlen(uart_input_string);
    //printf("%d\n", strlen(uart_input_string));
    //printf("\r%d\n", str_size);
    for(int i = 0; i < str_size; i++)
    {
        if(i != (str_size - 1)){
          if(uart_input_string[str_size - 1] == UART_CR_CODE || (uart_input_string[i] >= UART_ALPHABETH_MIN && uart_input_string[i] <= UART_ALPHABETH_MAX))
          {
            //printf("%d\n", uart_input_string[i] );
            //printf("char false\n");
            valid_input =  true;
          } else{
              valid_input = false;
          }
    }
    }
    //app_uart_flush();
    return valid_input ? UART_VALID_INPUT : UNART_UNKNOWN_CHAR;
}

/*
 * Reset and dump the command char*
 */


/**@computing zero crossing and prepare the string to be sent.
 *
 * @subject to change depending on the performance of the algorithm
 */


 void compute_sample_mean(void)
 {
    csfloat_t encode;
    for(int i = 0; i < copy_sampling_size; i++)
    {
        sample_mean += store_analogValues[i];
        sample_max = sample_max > store_analogValues[i] ? sample_max : store_analogValues[i];
        sample_min = sample_min < store_analogValues[i] ? sample_min : store_analogValues[i];
    }

    printf("{\"Device\":\"Master\",\"SensorType\":\"Voltage_Mean\",\"values\":%f}",sample_mean/copy_sampling_size);
    printf("{\"Device\":\"Master\",\"SensorType\":\"Voltage_Min\",\"values\":%d}",sample_min);
    printf("{\"Device\":\"Master\",\"SensorType\":\"Voltage_Max\",\"values\":%d}",sample_max);


 }

 /**@brief Function for computing rms.
 */

 void compute_rms(void)
 {
    csfloat_t encode;
    for(int i = 0; i < copy_sampling_size; i++)
    {
        root_mean_square += pow(store_analogValues[i],2);
        
    }
    root_mean_square /= sample_mean;
    root_mean_square = sqrt(root_mean_square);

    printf("{\"Device\":\"Master\",\"SensorType\":\"Voltage_RMS\",\"values\":%d}",root_mean_square);

 }

 /**@brief Function for computing sample offset.
 */
void compute_sample_offset(void)
 {
    for(int i = 0; i < copy_sampling_size; i++)
    {
        store_analogValues[i] -= sample_mean;
    }
 }

/**@brief Function for computing zero crossings.
 */
void compute_zero_crossing(void)
{
    compute_sample_offset();
    
    for(int i = 0; i + 1 < copy_sampling_size; i++)
    {
        if(i != copy_sampling_size - 1)
        {
            store_analogValues[i] = (store_analogValues[i] * 1 < 0 && store_analogValues[i + 1] * 1 > 0) || 
                                    (store_analogValues[i] * 1 > 0 && store_analogValues[i + 1] * 1 < 0) ? 
                                    store_analogValues[i] : 0;
        }
    }

    for(int i = 0; i + 1 < copy_sampling_size; i++)
    {
        if (store_analogValues[i] != NULL)
        {
            zero_crossing[i] = (store_analogValues[i] + store_analogValues[i + 1]) / 2;
            zero_crossing[i + 1] = (get_timestamp[i] + get_timestamp[i + 1]) / 2;
            i += 1;
        }
    }

    for(int i = 0; sizeof(zero_crossing)/sizeof(zero_crossing[0]); i++)
    {
        //stanle lsdkfjld
        if (i % 2 == 0)
        {
            printf("{\"Device\":\"Master\",\"SensorType\":\"Z_ADC\",\"values\":%f}",zero_crossing[i]);
        } else
        {
            printf("{\"Device\":\"Master\",\"SensorType\":\"Z_TIME\",\"values\":%f}",zero_crossing[i]);

        }
    }

    



}




void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    int sample_sum = 0;
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        if(measurement_begin == true && sampling_size > 0)
        {

            get_timestamp[copy_sampling_size - sampling_size] = ts_timestamp_get_ticks_u32(NRF_PPI_CHANNEL10);
            store_analogValues[copy_sampling_size - sampling_size] = p_event->data.done.p_buffer[0];

            sampling_size -= 1;

        } else if (sampling_size == 0)
        {
            data_ready = true;
        } else
        {
            //do nothing
        }
    }

}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}




/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}




/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        //.use_whitelist  = 0,
        //.adv_dir_report = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

bool address_exists_in_whitelist(ble_gap_addr_t *peer_addr){
    char adr_string[18];
    bool peer_in_whitelist=false;
    snprintf(adr_string, 18, "%02X:%02X:%02X:%02X:%02X:%02X",  
      peer_addr->addr[0],
      peer_addr->addr[1],
      peer_addr->addr[2],
      peer_addr->addr[3],
      peer_addr->addr[4],
      peer_addr->addr[5]
    );
    for(int i=0; i < current_nr_of_slaves; i++){
      if(debug){
        printf("\n");
        printf("lenght %d\n", strlen(slave_list[i].mac));
        printf("in %s x\n", slave_list[i].mac);
        printf("lenght %d\n", strlen(adr_string));
        printf("in %s x\n", adr_string);
        printf("--------------\n");
      }
      if(strcmp(slave_list[i].mac,adr_string) == 0){
        peer_in_whitelist=true;
        slave_list_index_where_match_occured = i;
        break;
      }
    }
    free(adr_string);
    return peer_in_whitelist;
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(ble_evt_t const * p_ble_evt)
{
    uint32_t      err_code;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    bool          do_connect = false;

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Prepare advertisement report for parsing.
    adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data.p_data;
    adv_data.size  = p_gap_evt->params.adv_report.data.len;

    // Search for advertising names.
    bool found_name = false;
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                &adv_data,
                                &dev_name);
    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return;
        }
        else
        {
            found_name = true;
        }
    }
    else
    {
        found_name = true;
    }

    if (found_name)
    {
        int i;
        for(i=0; i< max_nr_of_slaves; i++){
          if(m_target_periph_name_array[i]!= NULL){
            if((memcmp(m_target_periph_name_array[i], dev_name.p_data, dev_name.size) == 0)&&(address_exists_in_whitelist(peer_addr)))
            {
                do_connect = true;
                for( int i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
                {
                    if(memcmp(&connected_peer_addr[i], peer_addr, sizeof(ble_gap_addr_t)) == 0)
                    {
                        do_connect = false;
                    }
                }
            }
          }
        }
    }
    if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param, APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
    }
}


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for starting scanning. */
/*static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;
    int i;
    for(i=0; i< max_nr_of_slaves; i++){
        if(m_target_periph_name_array[i]!= NULL){
          NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name_array[i]);
          if(debug){
            printf("Start scanning for device name %s.", m_target_periph_name_array[i]);
          }
        }
    }
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}









/*@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
              printf("\rThe error code is %d\n",err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             printf("\rScan timeout reached\n",err_code);
             scan_start();
         } break;


         default:
             break;
    }
}



/*static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}*/


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
//static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
//{
    //ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
    //ble_siemens_on_db_disc_evt(&m_siemens_c[p_evt->conn_handle], p_evt);
//}
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
    ble_siemens_on_db_disc_evt(&m_siemens_c[p_evt->conn_handle], p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            printf("%s",p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            //ret_val = ble_nus_c_string_send(&m_ble_nus_c[0], p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}




/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 *
 *@ BLE - BLE communication
 *
 */
/*static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    //NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
    //NOTE process data received from BLE
    
    
    
    if (BLE_DATA_READY) // toggle this when data is ready
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}*/


void reset_uart_command_array(){
  memset(&command[0], 0, sizeof(command));
  uart_tracker = 0;
}



/*
 * Returns an error and resets command char* if received character is
 * not interpretable
 */
void handle_error_uart_input(char* errormessage){
  reset_uart_command_array();
  printf(errormessage);
}



/*
 * Updates the device list whenever a new device was added via uart
 * if the device name differs from the device names already known it 
 * is added and the scan process is restarted (only if it is running)
 */
void  update_device_list(){
  uint32_t       err_code;
  char *device_name = slave_list[current_nr_of_slaves-1].name;
  int i;
  bool name_present = false;
  if(debug){
    printf("Updating device list \r\n");
    printf("NAME %s \n", device_name);
    printf("NR %d \n", current_nr_of_slaves);
  }
  for(i=0; i< current_nr_of_slaves-1; i++){
    if(strcmp(m_target_periph_name_array[i], device_name)==0){
      name_present = true;
      if(debug){
        printf("Found name \n");
        printf("o %s  n %s ", m_target_periph_name_array[i],  device_name);
      }
    }
  } 
  if(name_present==false){
    if(debug){
      printf("Adding device %s at %d ", device_name, (current_nr_of_slaves-1));
    }
    memcpy(m_target_periph_name_array[(current_nr_of_slaves-1)], device_name, strlen(device_name));
    if(currently_scanning){  
      err_code = sd_ble_gap_scan_stop();
      if(debug){
        printf("Stopped scanning %d", err_code);
      }
      scan_start();
    }
  } 
}

/*
 * Implements the reponse to the connected devices request
 */
void handle_show_connected_devices_request(){
 printf("\r\n {\"sensors\" : [");
 for (int i = 0; i < sizeof(connected_peer_addr)/sizeof(connected_peer_addr[0]); i++){
     if (memcmp(&connected_peer_addr[i], address, sizeof(ble_gap_addr_t)) != 0){
             printf("\"%02X:%02X:%02X:%02X:%02X:%02X\"",
                     connected_peer_addr[i].addr[0],
                     connected_peer_addr[i].addr[1],
                     connected_peer_addr[i].addr[2],
                     connected_peer_addr[i].addr[3],
                     connected_peer_addr[i].addr[4],
                     connected_peer_addr[i].addr[5]
                   );
              if(i < (sizeof(connected_peer_addr)/sizeof(connected_peer_addr[0])-1)){
                 printf(",");
              }
     }
 }
 printf("]}\r\n");
}
/*
 * Removes a device from the master and disconnects from it
 */
void handle_remove_device_to_whitelist_request(void){
  bool all_required_variables_found = false;
  char* name = NULL;
  char* mac  = NULL;
  bool name_next = false;
  bool mac_next  = false;
  char delimiter[] = "+=";
  char *splitcommand;
  splitcommand = strtok(command, delimiter);
  int counter = 0;
  while(splitcommand != NULL){
        if(strcmp(splitcommand, ADD_NODE_N)==0){
          name_next = true;
        } 
        else if(strcmp(splitcommand, ADD_NODE_M)==0){
          mac_next = true;
        }
        else{
          if(name_next){
            name = splitcommand;
            name_next = false;
          }
          if(mac_next){
            mac = splitcommand;
            mac_next = false;
          }
        }
      splitcommand = strtok(NULL, delimiter);
      counter++;
  }
  if((mac!= NULL) && (name!= NULL)){
    printf(UART_RESPONSE_OK4);
  }else{
    handle_error_uart_input(UART_RESPONSE_ERR_404);
  }
}


/*
 * Adds a device to a whitelist
 */
void handle_add_device_to_whitelist_request(){
  bool all_required_variables_found = false;
  char* name = NULL;
  char* mac  = NULL;
  char* value = NULL;
  bool name_next = false;
  bool mac_next  = false;
  bool val_next  = false;
  char delimiter[] = "+=";
  char *splitcommand;
  splitcommand = strtok(command, delimiter);
  int counter = 0;
  while(splitcommand != NULL){
        if(strcmp(splitcommand, ADD_NODE_N)==0){
          name_next = true;
        } 
        else if(strcmp(splitcommand, ADD_NODE_M)==0){
          mac_next = true;
        }
        else if(strcmp(splitcommand, ADD_NODE_V)==0){
          val_next = true;
        }else{
          if(name_next){
            name = splitcommand;
            name_next = false;
          }
          if(mac_next){
            mac = splitcommand;
            mac_next = false;
          }
          if(val_next){
            value = splitcommand;
            val_next = false;
          }
        }
      splitcommand = strtok(NULL, delimiter);
      counter++;
  }
  if((mac!= NULL) && (name!= NULL) && (value!= NULL)){
    slave_list[current_nr_of_slaves] = (slave_information) {.name='_', .mac='1', .value='_'};
    memcpy(slave_list[current_nr_of_slaves].name, name, strlen(name));
    memcpy(slave_list[current_nr_of_slaves].mac, mac, strlen(mac));
    memcpy(slave_list[current_nr_of_slaves].value, value, strlen(value));
    current_nr_of_slaves = current_nr_of_slaves+1;
      if(debug){
          printf("\n--------------\n");
          printf("M: %s \n", slave_list[current_nr_of_slaves-1].mac);
          printf("S: %s \n", slave_list[current_nr_of_slaves-1].name);
          printf("V: %s \n", slave_list[current_nr_of_slaves-1].value);
          printf("X: %f \n", slave_list[current_nr_of_slaves-1].x);
          printf("Y: %f \n", slave_list[current_nr_of_slaves-1].y);
          printf("Z: %f \n", slave_list[current_nr_of_slaves-1].z);
          printf("nr: %d \n", current_nr_of_slaves);
      }
      update_device_list();
      free(name);
      free(mac);
      free(value);
      free(splitcommand);
      free(delimiter);
      printf(UART_RESPONSE_OK4);
    }else{
      handle_error_uart_input(UART_RESPONSE_ERR_ADD);
    }
}

/*
 * Adds a device to a whitelist
 */
void handle_start_connection_request(){
  currently_scanning = true;
  scan_start();
  printf(UART_RESPONSE_OK3);
}


/*
 * Adds a device to a whitelist
 */
void handle_start_measuring_request(){
  printf(UART_RESPONSE_OK2);
}

/*
 * Adds input received via uart to command char*
 */
void add_input_to_command(uint8_t uart_input_character){
  if(uart_tracker < max_command_length){
    command[uart_tracker] = uart_input_character;
    uart_tracker += 1;
  }else{
      reset_uart_command_array();
      handle_error_uart_input(UART_RESPONSE_ERR_ITL);
  }
}



/*
 * @brief Distinguish between commands that were received 
 */
int type_of_command_received(){
  char *address_add_found = strstr(command, ADD_NODE);
  char *address_remove_found = strstr(command, REMOVE_NODE);
  if(strcmp(command,RESET) == 0){
    free(address_add_found);
    free(address_remove_found);
    return UART_COMMAND_RESET;
  }
  else if(strcmp(command, CHECK_UART)==0){
    free(address_add_found);
    free(address_remove_found);
    return UART_COMMAND_CHECK;
  }
  else if (strcmp(command, START_MEASURING)==0){
    free(address_add_found);
    free(address_remove_found);
    return UART_COMMAND_MEASURING;
  }
  else if (strcmp(command, START_CONNECTION)==0){
    free(address_add_found);
    free(address_remove_found);
    return UART_COMMAND_START_CONNECTION;
  }
  else if (strcmp(command, SHOW_CONNECTED_DEVICES) == 0){
    free(address_add_found);
    free(address_remove_found);
    return UART_SHOW_CONNECTED_DEVICES;
  }
  else if (strcmp(command, DEBUG) == 0){
    free(address_add_found);
    free(address_remove_found);
    return UART_TOGGLE_DEBUG;
  }
  else if ((address_add_found != NULL)){
    free(address_add_found);
    free(address_remove_found);
    return UART_ADD_ADDRESS_TO_WHITELIST;
  }
  else if((address_remove_found != NULL)){ 
    free(address_add_found);
    free(address_remove_found);
    return UART_REMOVE_SENSOR;
  }
  else{
    free(address_add_found);
    free(address_remove_found);
    return UART_COMMAND_UNKNOWN;
  }
}

/*
 * @brief processes uart command 
 */
void process_uart_command(){
    if(debug){
      printf("Received %s",command);
    }
    switch(type_of_command_received()){
    case UART_COMMAND_RESET:
      NVIC_SystemReset();
      printf(UART_RESPONSE_OK);
      reset_uart_command_array();
      break;
    case UART_COMMAND_CHECK:
      printf(UART_RESPONSE_OK1);
      reset_uart_command_array();
      break;
    case UART_COMMAND_MEASURING:
      handle_start_measuring_request();
      reset_uart_command_array();
      break;
    case UART_COMMAND_START_CONNECTION:
      handle_start_connection_request();
      reset_uart_command_array();
      break;
    case UART_SHOW_CONNECTED_DEVICES:
      handle_show_connected_devices_request();
      reset_uart_command_array();
      break;
    case UART_ADD_ADDRESS_TO_WHITELIST:
      if(current_nr_of_slaves < max_nr_of_slaves){
        handle_add_device_to_whitelist_request();
        int i = 0;
        if(debug){
          for(i=0;i<3;i++){
            printf("DEVICE ON POSITION %d %s \n", i, m_target_periph_name_array[i]);
          }
        }
      }else{
       handle_error_uart_input(UART_RESPONSE_ERR_FUL);
      }
      reset_uart_command_array();
      break;
    case UART_REMOVE_SENSOR:
      if(current_nr_of_slaves > 0){
        handle_remove_device_to_whitelist_request();
      }else{
       handle_error_uart_input(UART_RESPONSE_ERR_404);
      }
      break;
    case UART_TOGGLE_DEBUG:
      printf(UART_RESPONSE_OK_DEBUG);
      reset_uart_command_array();
      debug = !debug;
      break;
    default:
      reset_uart_command_array();
      handle_error_uart_input(UART_RESPONSE_ERR_INK);
      break;
    }
    
}





/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[50];
    //memset(&data_array[0], 0, sizeof(data_array));
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\r'))
            {
                if (index > 1)
                {
                  //NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                  //NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                  printf("length %d\n",index);
                  memcpy(command, data_array, index);
                  
                do
                {
                     printf("%s\n", command);
                     int decision = check_uart_input(command);
                     printf("\r%d\n",decision);
                     switch(decision){
                     case UART_VALID_INPUT:
                          command[index - 1] = 0;
                          process_uart_command();
                          break;
                     default:
                          //something unexpected happend -> error
                          handle_error_uart_input(UART_RESPONSE_ERR);
                          break;
                     }
             
                  if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                  {
                      APP_ERROR_CHECK(ret_val);
                  }
                } while (ret_val == NRF_ERROR_RESOURCES);
                
                }
                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}








/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 *
 *@ customizing this data to suit our needs we are doing a uart to uart and ble to ble

 */
/*void uart_event_handle(app_uart_evt_t * p_event, uint8_t * p_data, uint16_t data_len)
{
    static uint8_t data_array[70];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        /*case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if  (data_array[index - 1] == '\n')
            {
                //NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                //NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                //NOTE perform commands with data_array[index];
                index = 0;
            }

            //NOTE function to compute command here
            
            //NOTE use the function below for writing to serial
            /*p_data = 12; //
            data_len = 42;//
            for (uint32_t i = 0; i < data_len; i++)
            {
                do
                {
                    ret_val = app_uart_put(p_data[i]);
                    if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                    {
                        NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                        APP_ERROR_CHECK(ret_val);
                    }
                } while (ret_val == NRF_ERROR_BUSY);
            }
            if (p_data[data_len-1] == '\r')
            {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
            }
            break;

        /**@snippet [Handling data from UART] */
        /*case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}*/





/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
/*static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);
            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
                currently_scanning = true;
            }

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}*/



static void siemens_c_evt_handler(ble_siemens_c_t * p_siemens_c, ble_siemens_c_evt_t * p_siemens_c_evt)
{
    ret_code_t err_code;

    switch (p_siemens_c_evt->evt_type)
    {
        case BLE_SIEMENS_C_EVT_DISCOVERY_COMPLETE:
        {
            err_code = ble_siemens_c_handles_assign(p_siemens_c,
                                                p_siemens_c_evt->conn_handle,
                                                &p_siemens_c_evt->params.siemens_db);
            APP_ERROR_CHECK(err_code);

            encode_sensor_constants();

            // Battery service discovered. Enable notification of Battery Level.
            NRF_LOG_DEBUG("Siemens Service discovered. Reading values.");

            //err_code = ble_siemens_c_min_read(p_siemens_c);
            //APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Minimum Value Notification.");
            err_code = ble_siemens_c_min_notif_enable(p_siemens_c);
            printf("\rThe error code for min is %x\n", err_code);
            APP_ERROR_CHECK(err_code);

            err_code = ble_siemens_c_max_read(p_siemens_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Maximum Value Notification.");
            err_code = ble_siemens_c_max_notif_enable(p_siemens_c);
            printf("\rThe error code for max is %x\n", err_code);
            APP_ERROR_CHECK(err_code);

            err_code = ble_siemens_c_zero_crossing_read(p_siemens_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Zero Crossing Notification.");
            err_code = ble_siemens_c_zero_crossing_notif_enable(p_siemens_c);
            printf("\rThe error code for zero_crossing is %x\n", err_code);
            APP_ERROR_CHECK(err_code);

            err_code = ble_siemens_c_mean_read(p_siemens_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Mean value Notification.");
            err_code = ble_siemens_c_mean_notif_enable(p_siemens_c);
            printf("\rThe error code for mean is %x\n", err_code);
            APP_ERROR_CHECK(err_code);

            err_code = ble_siemens_c_rms_read(p_siemens_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling RMS value Notification.");
            err_code = ble_siemens_c_rms_notif_enable(p_siemens_c);
            printf("\rThe error code for rms is %x\n", err_code);
            APP_ERROR_CHECK(err_code);


            //Send X,Y.Z VALUES 
            //printf("\rThe total number of connections is %d\n",ble_conn_state_central_conn_count());

            //this function returns a bool value but we using it in an if clause is not necessary since
            //the test must have been passed to get to this stage
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            ble_siemens_c_write(p_siemens_c, slave_list[slave_list_index_where_match_occured].x, 2, X_INDEX);
            ble_siemens_c_write(p_siemens_c, slave_list[slave_list_index_where_match_occured].y, 2, Y_INDEX);
            ble_siemens_c_write(p_siemens_c, slave_list[slave_list_index_where_match_occured].z, 2, Z_INDEX);
            ble_siemens_c_write(p_siemens_c, command_value, 1, COMMAND_INDEX);
            measurement_begin = true;

        } break;

        case BLE_SIEMENS_C_EVT_MIN_NOTIFICATION:
            //identify the node
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            //decode values
            printf("\r{\"Device\":\"%s\",\"SensorType\":\"Voltage_MIN\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float8_values(p_siemens_c_evt->params.min_value, 8));
            NRF_LOG_INFO("Minimum Value Notification received %d %%.", p_siemens_c_evt->params.min_value[0]);
            notification_track +=1;
            break;

        //case BLE_SIEMENS_C_EVT_MIN_READ_RESP:
            //NRF_LOG_INFO("Minimum Value is %d %%.", p_siemens_c_evt->params.min_value);
            //break;

        case BLE_SIEMENS_C_EVT_ZERO_CROSSING_NOTIFICATION:
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            NRF_LOG_INFO("Zero Crossing Notification received %d %%.", p_siemens_c_evt->params.zero_crossing[0]);
            ble_evt_t const * p_ble_evt;
            if (p_ble_evt->evt.gattc_evt.params.hvx.len == 5)
            {
                //NON
                printf("\r{\"Device\":\"%s\",\"SensorType\":\"Z_ADC\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float5_values(p_siemens_c_evt->params.zero_crossing, 5));
            } else
            {
                printf("\r{\"Device\":\"%s\",\"SensorType\":\"Z_TIME\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float8_values(p_siemens_c_evt->params.zero_crossing, 8));
            }
            if(notification_track == ble_conn_state_central_conn_count())
            {
                ble_siemens_c_write(p_siemens_c, command_value, 1, COMMAND_INDEX);
                measurement_begin = true;
            }
            break;

        //case BLE_SIEMENS_C_EVT_ZERO_CROSSING_READ_RESP:
            //NRF_LOG_INFO("Zero Crossing Value is %d %%.", p_siemens_c_evt->params.zero_crossing);
            //break;

        case BLE_SIEMENS_C_EVT_MAX_NOTIFICATION:
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            NRF_LOG_INFO("Max Notification received %d %%.", p_siemens_c_evt->params.max_value[0]);
            printf("\r{\"Device\":\"%s\",\"SensorType\":\"Voltage_MAX\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float8_values(p_siemens_c_evt->params.max_value, 8));
            break;


        case BLE_SIEMENS_C_EVT_RMS_NOTIFICATION:
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            NRF_LOG_INFO("RMS Notification received %d %%.", p_siemens_c_evt->params.rms_value[0]);
            printf("\r{\"Device\":\"%s\",\"SensorType\":\"Voltage_RMS\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float8_values(p_siemens_c_evt->params.rms_value, 8));
            break;


        case BLE_SIEMENS_C_EVT_MEAN_NOTIFICATION:
            address_exists_in_whitelist(&connected_peer_addr[p_siemens_c->conn_handle]);
            NRF_LOG_INFO("Mean Notification received %d %%.", p_siemens_c_evt->params.rms_value[0]);
            printf("\r{\"Device\":\"%s\",\"SensorType\":\"Voltage_Mean\",\"values\":%f}\n",slave_list[slave_list_index_where_match_occured].name, decode_float8_values(p_siemens_c_evt->params.mean_value, 8));
            break;

        case BLE_SIEMENS_C_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_indication_set(BSP_INDICATE_SCANNING);
                scan_start();
            }
        }break;


        default:
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                         p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
            //err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            //err_code = ble_siemens_c_handles_assign(&m_siemens_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            //err_code = ble_siemens_c_handles_assign(&m_siemens_c, p_gap_evt->conn_handle, NULL);
            connected_peer_addr[p_gap_evt->conn_handle] = p_gap_evt->params.connected.peer_addr;
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            //err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_indication_set(BSP_INDICATE_SCANNING);
                scan_start();
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            memset(&connected_peer_addr[p_gap_evt->conn_handle], 0x00, sizeof(ble_gap_addr_t));
            bsp_indication_set(BSP_INDICATE_SCANNING);
            break;

        
        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
        //err_code = sd_ble_gap_disconnect(m_siemens_c.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
            for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
            {
                err_code = sd_ble_gap_disconnect(m_siemens_c[i].conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                    break;
                }
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
/*static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_nus_c_init(&m_siemens_c[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}*/

/**
 * @brief Battery level collector initialization.
 */
static void siemens_c_init(void)
{
    ble_siemens_c_init_t siemens_c_init_obj;
    ret_code_t err_code;

    siemens_c_init_obj.evt_handler = siemens_c_evt_handler;
    //err_code = ble_siemens_c_init(&m_siemens_c, &siemens_c_init_obj);
    //printf("\rThe error code is %d\n",err_code);

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_siemens_c_init(&m_siemens_c[i], &siemens_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void sync_timer_init(void)	
{
   uint32_t       err_code;
   uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
   static ts_params_t    ts_params;
   
   // Debug pin: Toggle P0.24 from sync timer to allow pin measurement
   nrf_gpiote_task_configure(3, 27, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
   nrf_gpiote_task_enable(3);
   //uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

   nrf_ppi_channel_endpoint_setup(
       NRF_PPI_CHANNEL0,
       (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
       nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
   nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

   nrf_ppi_channel_endpoint_setup(
       NRF_PPI_CHANNEL5,
       (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
       nrf_drv_saadc_sample_task_get());
   nrf_ppi_channel_enable(NRF_PPI_CHANNEL5);

   ts_params.high_freq_timer[0] = NRF_TIMER3;
   ts_params.high_freq_timer[1] = NRF_TIMER2;
   ts_params.rtc             = NRF_RTC1;
   ts_params.egu             = NRF_EGU3;
   ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
   ts_params.ppi_chg         = 0;
   ts_params.ppi_chns[0]     = 1;
   ts_params.ppi_chns[1]     = 2;
   ts_params.ppi_chns[2]     = 3;
   ts_params.ppi_chns[3]     = 4;
   ts_params.rf_chn          = 125; /* For testing purposes */	
   memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));

   err_code = ts_init(&ts_params);	
   APP_ERROR_CHECK(err_code);

   err_code = ts_enable();	
   APP_ERROR_CHECK(err_code);

   NRF_LOG_INFO("Started listening for beacons.\r\n");
   NRF_LOG_INFO("Press Button 1 to start sending sync beacons\r\n");
}


int getNr(char int_char){
  static int  digits_i[10] =  {0, 1, 2, 3, 4, 5, 6, 7, 8 ,9};
  static char digits_s[10] =  {'0', '1', '2', '3', '4', '5', '6', '7', '8' ,'9'};
  int i;
  for(i=0; i<9;i++){
    if(digits_s[i] == int_char){
      return digits_i[i];
    }
  }
}

int getPow(int potenz){
  if(potenz == 0){
   return 1;
  }
  if(potenz == 1){
    return 10;
  }
  return 100;
}







int main(void)
{
    // Initialize.

    uint8_t sending[2] = {1};



    log_init();
    timer_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    saadc_init();
    //nus_c_init();
    siemens_c_init();
    scan_init();
    //nrf_drv_saadc_sample();
    sync_timer_init();
    ts_tx_start(200);


    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    //scan_start();
    //sfloat_t testing = encode_float(3.33);

    // Enter main loop.
    for (;;)
    {
        if(data_ready == true)
        {
            //compute_sample_mean();
            //nrf_delay_ms(1000);
            //compute_rms();
            //nrf_delay_ms(1000);
            //compute_zero_crossing();
            //nrf_delay_ms(1000);

            sample_mean = 0.0;
            sample_max = 0;
            sample_min = 0;
            root_mean_square = 0.0;

            measurement_begin = false;
            data_ready = false;
            sampling_size = copy_sampling_size;
        }
        //for (int i = 0; i  < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        //{
            //sldfkjdl;
            //ble_siemens_c_write(&m_siemens_c[i], sending, 2, COMMAND_INDEX);
        //}
        //ble_siemens_c_write(&m_siemens_c[1], sending, 2, COMMAND_INDEX);
        //nrf_delay_ms(1000);
        idle_state_handle();
    }
}