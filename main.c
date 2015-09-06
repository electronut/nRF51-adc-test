/* 
   nrF51-ADC-test/main.c:

   This is a simple demonstration of ADC and NUS (Nordic UART Service) using 
   the nRF-DK board. ADC values are read in P0.01 where an LRD/resistor 
   divider is connected. The read in values are sent overt NUS. In addition, 
   LED2 is also flashed.

   The code combines Nordic examples ble_app_uart and adc_simple. It has 
   been cleaned up and simplified for clarity.
   
   Reference:

   https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/doc/8.1.0/s110/html/index.html

   Author: Mahesh Venkitachalam
   Website: electronut.in

 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_adc.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"


static ble_nus_t m_nus;                                  
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
volatile int32_t adc_sample;

// Function for assert macro callback.
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


// Function for the GAP initialization.
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    // for RedBearLab Nano, use different name
#ifdef BOARD_CUSTOM
    const char deviceName[] = "ADC-UART-Nano";
#else
    const char deviceName[] = "ADC-UART-nRF51-DK";
#endif

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) deviceName,
                                          strlen(deviceName));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(75, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

// Function for handling the data from the Nordic UART Service.
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, 
                             uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}

// Function for initializing services that will be used by the application.
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


// Function for handling an event from the Connection Parameters Module.
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

// Function for handling errors from the Connection Parameters module.
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


// Function for initializing the Connection Parameters module.
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000, 0);
    cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000, 0);
    cp_init.max_conn_params_update_count   = 3;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

// Function for handling advertising events.
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
             break;
        case BLE_ADV_EVT_IDLE:
             break;
        default:
            break;
    }
}


// Function for the Application's S110 SoftDevice event handler.
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = 
              sd_ble_gap_sec_params_reply(m_conn_handle, 
                                          BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, 
                                          NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


// Function for dispatching a S110 SoftDevice event to all modules 
// with a S110 SoftDevice event handler.
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);    
}

// Function for the S110 SoftDevice initialization.
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.service_changed = 0;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

// Function for handling app_uart events.
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || 
                (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

// Function for initializing the UART module.
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       256,
                       256,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


// Function for initializing the Advertising functionality.
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, 
                                 BLE_UUID_TYPE_VENDOR_BEGIN}};
    
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = 
      sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = 64;
    options.ble_adv_fast_timeout  = 180;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, 
                                    on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

// ADC initialization.
void adc_config(void)
{
    const nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    // for RedBearLab Nano, use P0.04 - analog ADC input 5
#ifdef BOARD_CUSTOM 
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_5);
#else 
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
#endif
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);
}

// ADC interrupt handler.
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();

    adc_sample = nrf_adc_result_get();

    // trigger next ADC conversion
    nrf_adc_start();
}

// Application main function.
int main(void)
{
    uint32_t err_code;

    // set up timer
    APP_TIMER_INIT(0, (2 + BSP_APP_TIMERS_NUMBER), 4, false);

    // initlialize BLE
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
   
    // intialize UART
    uart_init();

    // prints to serial port
    printf("starting...\n");

    // set up ADC
    adc_config();
    nrf_adc_start();


#ifdef BOARD_CUSTOM // custom board
    
    // ExploreEmbedded/Electronut breakout board
#ifdef BOARD_E3BO
    // set LED connected to P0.21 as output
    uint32_t pinNum = 21;
#else
    // for RedBearLab Nano, LED is on P0.19
    // set LED connected to P0.19 as output
    uint32_t pinNum = 19;
#endif

#else // nRF51-DK
    // set LED2 connected to P0.22 as output
    uint32_t pinNum = 22;
#endif

    nrf_gpio_pin_dir_set(pinNum, NRF_GPIO_PIN_DIR_OUTPUT);
    // Enter main loop.
    while(1) {

      // flash LED2 once
      nrf_gpio_pin_set(pinNum);
      nrf_delay_ms(500);
      nrf_gpio_pin_clear(pinNum);
      nrf_delay_ms(500);

      // send ADC value via NUS (Nordic UART service)
      uint8_t str[4];
      sprintf((char*)str, "%d", (int)adc_sample);
      ble_nus_string_send(&m_nus, str, strlen((char*)str));
    }
}
