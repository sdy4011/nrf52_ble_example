/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_types.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#include "bsp.h"
#include "nrf_delay.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
//#include "ble_lbs.h"
#include "notification_test.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#define DEVICE_NAME                     "My BLE Test"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

//#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_DURATION                0    //0 can adv forever                                  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define SAMPLE_IN_BUFFER 126												 /**<saadc buffer for 6 channel in which 6 bytes are allocated to 1 channel*/
//#define NRF_SDH_BLE_VS_UUID_COUNT 1

//APP_TIMER_DEF(our_test_id);  /**<my timer test.*/
BLE_LBS_DEF(m_lbs);          /**<lbs test. is equivalent to static ble_bls_t m_lbs*/
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

//temporary data storage;
//APP_TIMER_DEF(notify_timer);

////saadc timer
//APP_TIMER_DEF(saadc_timer);

typedef struct
{
	uint16_t conn_handle;
	ble_lbs_t * p_lbs;
	uint8_t a_data[126];
}tem_save;

static tem_save ts={0};
static uint16_t REPREAT_TIMES = 21; /*semg repeat times*/
static uint16_t CHANNLES			 =6  ;/*semg channels*/
static uint16_t INTERVAL_TIME  =21  ;/*intervale time*/
static uint8_t  a_data[126]={0};
static nrf_saadc_value_t m_buffer_pool[2][SAMPLE_IN_BUFFER];/*saadc buffer*/
static uint32_t m_adc_evt_counter = 0; /*sample number*/
static const nrf_drv_timer_t saadc_timer = NRF_DRV_TIMER_INSTANCE(1);//*saadc tiemr*/
static nrf_ppi_channel_t m_ppi_channel;
static uint8_t sampling_flag = 0;
uint64_t start_time = 0;

/*saadc callback*/
static void saadc_callback(nrfx_saadc_evt_t const*p_evt)
{
//	float val;
//	if(p_evt->type == NRFX_SAADC_EVT_DONE)//*buffer full evt
//	{
//		ret_code_t err_code;
//		err_code = nrfx_saadc_buffer_convert(p_evt->data.done.p_buffer,SAMPLE_IN_BUFFER);
//		APP_ERROR_CHECK(err_code);
//		memcpy(ts.a_data,p_evt->data.done.p_buffer,126);
////	  ble_lbs_on_button_change(ts.conn_handle,ts.p_lbs,ts.a_data);
//		
//	}
//	else
//	{
//	}
}


static void saadc_timeout(nrf_timer_event_t evt_type,void *p_context)
{
	if(UINT64_MAX !=start_time)
	{	
		start_time++;
	}
	else{
		start_time = 0;
	}
	switch (evt_type)
    {
        case NRF_TIMER_EVENT_COMPARE0://???0??
//					nrfx_saadc_sample();
//					ble_lbs_on_button_change(ts.conn_handle,ts.p_lbs,ts.a_data);
//					nrf_delay_ms(1);
            
            break;
        default:
            break;
    }
}

static void saadc_sampling_event_enable(void)
{
	
	
	ret_code_t err_code;
	err_code = nrfx_ppi_channel_enable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_disable(void)
{
	ret_code_t err_code;
	err_code = nrfx_ppi_channel_disable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);
}

static void saadc_timer_enable(void)
{
		nrf_drv_timer_enable(&saadc_timer);

}

static void saadc_timer_disable(void)
{
		nrf_drv_timer_disable(&saadc_timer);
}
	
static void saadc_sampling_event_init(void)
{
	ret_code_t err_code;
	//configure saadc_timer 
	nrf_drv_timer_config_t timercfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	err_code = nrf_drv_timer_init(&saadc_timer,&timercfg,saadc_timeout);
	APP_ERROR_CHECK(err_code);
	uint32_t ticks = nrf_drv_timer_ms_to_ticks(&saadc_timer,1);
	nrf_drv_timer_extended_compare(&saadc_timer,NRF_TIMER_CC_CHANNEL0,ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,true);
	
//	err_code = nrf_drv_ppi_init();
//	if(err_code == NRF_SUCCESS)
//	{
//		NRF_LOG_INFO("ppi init succeed");
//	}
//	else
//	{
//		NRF_LOG_INFO("ppi init failed");
//	}
//	APP_ERROR_CHECK(err_code);
//	//set task and event for ppi channel
//	uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&saadc_timer,NRF_TIMER_CC_CHANNEL0);
//	uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();
//	//get a PPI channel 
//	err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
//	APP_ERROR_CHECK(err_code);
//	//assign addrs of task and event for ppi channel
//	err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,timer_compare_event_addr,saadc_sample_event_addr);
//	APP_ERROR_CHECK(err_code);
	
}
	
	
static void saadc_init(void)
{
	ret_code_t err_code;
	nrf_saadc_channel_config_t a0 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	
	nrf_saadc_channel_config_t a1 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
	nrf_saadc_channel_config_t a2 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	nrf_saadc_channel_config_t a3 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
	nrf_saadc_channel_config_t a4 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
	nrf_saadc_channel_config_t a5 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	
	a0.acq_time = NRF_SAADC_ACQTIME_3US;
	a1.acq_time = NRF_SAADC_ACQTIME_3US;
	a2.acq_time = NRF_SAADC_ACQTIME_3US;
	a3.acq_time = NRF_SAADC_ACQTIME_3US;
	a4.acq_time = NRF_SAADC_ACQTIME_3US;
	a5.acq_time = NRF_SAADC_ACQTIME_3US;
//	a0.mode = NRF_SAADC_MODE_DIFFERENTIAL;
//	a1.mode = NRF_SAADC_MODE_DIFFERENTIAL;
//	a2.mode = NRF_SAADC_MODE_DIFFERENTIAL;
//	a3.mode = NRF_SAADC_MODE_DIFFERENTIAL;
//	a4.mode = NRF_SAADC_MODE_DIFFERENTIAL;
//	a5.mode = NRF_SAADC_MODE_DIFFERENTIAL;
	err_code = nrf_drv_saadc_init(NULL,saadc_callback);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(0,&a0);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(1,&a1);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(2,&a2);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(3,&a3);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(4,&a4);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_saadc_channel_init(5,&a5);
	APP_ERROR_CHECK(err_code);
//	err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0],SAMPLE_IN_BUFFER);
//	APP_ERROR_CHECK(err_code);
//	err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1],SAMPLE_IN_BUFFER);
//	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("saadc init succeed");
}
	
	


static void timeout(void *p)
{
	UNUSED_PARAMETER(p);
	ret_code_t err_code;
//	NRF_LOG_INFO("enter notify timer");
//		nrfx_saadc_sample(); // start a saadc sample
	uint8_t val_a0;
	uint8_t val_a1;
	uint8_t val_a2;
	uint8_t val_a3;
	uint8_t val_a4;
	uint8_t val_a5;
	//read ana value 
	if(m_adc_evt_counter<126)
	{
		uint32_t o0 = bsp_board_a_read(0);
		val_a0 = o0*255;//a0
//		NRF_LOG_INFO("original ao:%#x,after ao:%#x",o0,val_a0);
		val_a1 = bsp_board_a_read(1);//a1
		val_a2 = bsp_board_a_read(2);//a2
		val_a3 = bsp_board_a_read(3);//a3
		val_a4 = bsp_board_a_read(4);//a4
		val_a5 = bsp_board_a_read(5);//a5
		ts.a_data[m_adc_evt_counter++]=val_a0;
		ts.a_data[m_adc_evt_counter++]=val_a1;
		ts.a_data[m_adc_evt_counter++]=val_a2;
		ts.a_data[m_adc_evt_counter++]=val_a3;
		ts.a_data[m_adc_evt_counter++]=val_a4;
		ts.a_data[m_adc_evt_counter++]=val_a5;
		NRF_LOG_INFO("a0:%d,a1:%d,a2:%d,a3:%d,a4:%d,a5:%d",val_a0,val_a1,val_a2,val_a3,val_a4,val_a5);
	}
	else
	{
		m_adc_evt_counter = 0;
	}
	err_code = ble_lbs_on_button_change(ts.conn_handle,ts.p_lbs,ts.a_data);
	if(err_code != NRF_SUCCESS)
	{
		char a[10];
		int tem = (int)err_code;
		sprintf(a,"%#x",tem);
		
		NRF_LOG_INFO(a);
	}
}


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{		
		
//	  {LBS_UUID_SERVICE,BLE_UUID_TYPE_BLE},
//    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
	{LBS_UUID_SERVICE,BLE_UUID_TYPE_VENDOR_BEGIN}
};


static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

/**my timer timeout handle test*/
static void timeout_test(void *p)
{
	UNUSED_PARAMETER(p);
	start_time++;
	//deal
//	NRF_LOG_INFO("??");
}

//static void saadc_timeout_callback(void)
//{
//}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
//		err_code = app_timer_create(&our_test_id,APP_TIMER_MODE_REPEATED,timeout_test);
////		
//		APP_ERROR_CHECK(err_code);
//		err_code =  app_timer_create(&notify_timer,APP_TIMER_MODE_REPEATED,timeout);
//		APP_ERROR_CHECK(err_code);
//	  err_code = app_timer_create(&saadc_timer,APP_TIMER_MODE_REPEATED,saadc_timeout_callback);
    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */
		err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_WATCH_SPORTS_WATCH);														
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
		//opt event
//		ble_opt_t opt;
//		memset(&opt,0x00,sizeof(opt));
//		err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT,&opt);
//		APP_ERROR_CHECK(err_code);																		
																					
																					
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/*data acquisition*/
static uint8_t *data_acq(void)
{
}

/*notify timer init*/
//static void notify_timer_init(void)
//{
//		ret_code_t err_code;
//		err_code =  app_timer_create(&notify_timer,APP_TIMER_MODE_REPEATED,timeout);
//		APP_ERROR_CHECK(err_code);
//}

/*my ble_lbs write handler test */
static void led_write_handler(ble_lbs_t * p_lbs, ble_evt_t const * p_ble_evt)
{
		ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		if ((p_evt_write->handle == p_lbs->led_char_handles.value_handle)&&(p_evt_write->len == 1))
		{
				if(p_evt_write->data[0]==0x01)
			{
				
				uint16_t conn_handle = p_ble_evt->evt.common_evt.conn_handle;
				memset(&ts,0,sizeof(ts));
				ts.conn_handle = conn_handle;
				ts.p_lbs = p_lbs;
//				ts.button_state = 0x03;		
//				app_timer_start(notify_timer,APP_TIMER_TICKS(1000),NULL); //start timer
				saadc_timer_enable();
//				saadc_sampling_event_enable();	
				sampling_flag = 1;
//				bsp_board_led_on(1);
//				NRF_LOG_INFO("REV LED ON");
			}
			else if(p_evt_write->data[0]==0x02)
			{
				//other command
//				app_timer_stop(notify_timer);
//				saadc_sampling_event_disable();
				saadc_timer_disable();
//				saadc_sampling_event_disable();	
				sampling_flag = 0;
//				bsp_board_led_off(1);
//				NRF_LOG_INFO("REV LED OFF");
			}
			else
			{
//				app_timer_stop(notify_timer);
//				saadc_sampling_event_disable();
				saadc_timer_disable();
//				saadc_sampling_event_disable();	
				sampling_flag = 0;
//				bsp_board_led_off(1);
//				NRF_LOG_INFO("REV LED OFF");
			}
		}
}

/*my notify handler test*/
static void notify_handler(uint16_t conn_handle,ble_lbs_t *p_lbs,uint8_t led_state)
{
	ret_code_t err_code;
}

/*my disconnect handler*/
static void disconnect_handler(ble_lbs_t * p_lbs, ble_evt_t const * p_ble_evt)
{
//	app_timer_stop(notify_timer);
//	bsp_board_led_off(1);
		saadc_timer_disable();
//				saadc_sampling_event_enable();	
		sampling_flag = 1;
}
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
		 ble_lbs_init_t ble_init = {0};
		 ble_init.led_write_handler = led_write_handler;
		 ble_init.notify_handler = notify_handler;
		 ble_init.disconnect_handler = disconnect_handler;
		 err_code = ble_lbs_init(&m_lbs,&ble_init);
		 APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
//		ret_code_t err_code;
//		err_code = app_timer_start(our_test_id,APP_TIMER_TICKS(1),NULL);//200ms
//		APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();  //adv sleep when adv mode turns to idle
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
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
            // No implementation needed.
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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
//    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
		init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids)/sizeof(m_adv_uuids[0]);
		init.srdata.uuids_complete.p_uuids = m_adv_uuids;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION; //adv timeout time

    init.evt_handler = on_adv_evt;
	
		//**scan callback


    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
			/*ana pin init*,my define*/
//		bsp_board_a_all_init_input();
}


/**@brief Function for initializing the nrf log module.
 */
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


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
//				ble_advertising_conn_cfg_tag_set(&m_advertising,);
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}



void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    // Initialize.
    log_init();
		uart_init();
		printf("hello");
//		while(true){
//				printf("!!");
//					nrf_delay_ms(500);
//		}
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
		services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    // Start execution.
    NRF_LOG_INFO("Template example started.");
//    application_timers_start();

    advertising_start(erase_bonds);
//		ret_code_t err_code;
//		nrf_drv_timer_config_t timercfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//		err_code = nrf_drv_timer_init(&saadc_timer,&timercfg,saadc_timeout);
//		APP_ERROR_CHECK(err_code);
//		uint32_t ticks = nrf_drv_timer_ms_to_ticks(&saadc_timer,500);
//		nrf_drv_timer_extended_compare(&saadc_timer,NRF_TIMER_CC_CHANNEL0,ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,true);
//		nrf_drv_timer_enable(&saadc_timer);
//		
		saadc_sampling_event_init();
		saadc_init();
//		saadc_sampling_event_enable();

//		saadc_init();//saadc init 
//		nrf_gpio_cfg_input(2,NRF_GPIO_PIN_PULLUP);//
    // Enter main loop.
		uint64_t pre_t = 0;
		uint64_t cur_t = start_time;
		nrf_saadc_value_t a0;
		nrf_saadc_value_t a1;
		nrf_saadc_value_t a2;
		nrf_saadc_value_t a3;
		nrf_saadc_value_t a4;
		nrf_saadc_value_t a5;
		uint8_t rp = 0;
		ret_code_t err_code;
    for (;;)
    {
//					nrfx_saadc_sample();
//					nrf_delay_ms(1);
        idle_state_handle();
//			nrfx_saadc_sample();
			while(sampling_flag)
			{		
				idle_state_handle();
				if(rp!=SAMPLE_IN_BUFFER)
				{
					nrfx_saadc_sample_convert(0,&a0);
					nrfx_saadc_sample_convert(1,&a1);
					nrfx_saadc_sample_convert(2,&a2);
					nrfx_saadc_sample_convert(3,&a3);
					nrfx_saadc_sample_convert(4,&a4);
					nrfx_saadc_sample_convert(5,&a5);
					if(a0<0)a0=0;
					if(a1<0)a1=0;
					if(a2<0)a2=0;
					if(a3<0)a3=0;
					if(a4<0)a4=0;
					if(a5<0)a5=0;
					ts.a_data[rp++]=a0;
					ts.a_data[rp++]=a1;
					ts.a_data[rp++]=a2;
					ts.a_data[rp++]=a3;
					ts.a_data[rp++]=a4;
					ts.a_data[rp++]=a5;
//					NRF_LOG_INFO("a0:%d,a1:%d,a2:%d,a3:%d,a4:%d,a5:%d",a0,a1,a2,a3,a4,a5);
				}
				else
				{
//				nrfx_saadc_sample();
					cur_t = start_time;
					while(cur_t-pre_t<INTERVAL_TIME)
					{
						cur_t = start_time;
					}
					pre_t = start_time;
					err_code = ble_lbs_on_button_change(ts.conn_handle,ts.p_lbs,ts.a_data);
//						NRF_LOG_INFO("errcode :%d",err_code);
					rp = 0;
				}

			}
    }
}


/**
 * @}
 */
