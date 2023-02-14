#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "mpu6500_spi.h"
#include "bracelet_cfg.h"


NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) = {
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3e000,
    .end_addr   = 0x3ffff,
};

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          = {                                        /**< Universally unique service identifier. */
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/* bracelet main values */
nrf_saadc_value_t bracelet_signals[] = {0, 0, 0, 0, 0, 0}; // array for saving sensors signal values
volatile bool is_connected = false;
uint16_t signals_stage_1[] = {0, 0, 0, 0, 0, 0};
uint16_t signals_stage_2[] = {0, 0, 0, 0, 0, 0};
uint8_t sensors_idxs[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // indices of the signals that are used for gesture recognizing
uint16_t diffs[6];
float accelerations[3] = {0.0f, 0.0f, 0.0f};
float g_speed[3] = {0.0f, 0.0f, 0.0f};
volatile bool f_btn_is_pressed = false; // calibration/switch mode button flag
uint32_t f_btn_start_time = 0; // for keeping time the start from pressed button (calibration/switch mode)
volatile bool sw_off_is_pressed = false; // button flag, it is connected with switching off the bracelet
imu_data_s imu_data; // structure for transmitting mpu data
gesture_and_buttons_s switching_struct; // structure for transmitting info about gesture and buttons states
volatile bool is_gesture_defined = false;
charge_state_s charge = {DEVICE_ID | 0x50, 0}; // struct with message about battery charge
/* For filter */
uint8_t filter_shift = log2(MEAS_VALUES);
int16_t adc_prev[6][MEAS_VALUES];
uint32_t adc_sum[6];
volatile uint8_t count_adc = 0;
/* timer stuff */
volatile bool timer_0_flag = false;
const nrf_drv_timer_t timer_0 = NRF_DRV_TIMER_INSTANCE(2);
volatile bool timer_1_flag = false;
const nrf_drv_timer_t timer_1 = NRF_DRV_TIMER_INSTANCE(1);
volatile bool timer_2_flag = false;
const nrf_drv_timer_t timer_2 = NRF_DRV_TIMER_INSTANCE(3);

extern const nrf_drv_spi_t spi_inst;


int main(void) {
    ret_code_t rc;
    uint8_t ble_output_array[BLE_NUS_MAX_DATA_LEN]; // max 246 bytes

    digital_pins_init();
    /* fstorage stuff */
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    //(void) nrf5_flash_end_addr_get();

    // Initializing
    timers_init();
    //power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    saadc_init();
    advertising_start();
    gyroscope_accelerometer_init();
    read_bracelet_values();

    rc = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 4);
    APP_ERROR_CHECK(rc);

    switching_struct.idx = DEVICE_ID | 0x90;
#if defined(UAV_CONTROL) || defined (PLATFORM_CONTROL)
    imu_data.idx = DEVICE_ID | 0x30;
#else
    imu_data.idx = DEVICE_ID | 0x34;
#endif
    nrf_drv_timer_enable(&timer_0);
    nrf_drv_timer_enable(&timer_2); // timer for sending accelerations

    while (1) {
        f_btn_processing();
        check_switch_off_by_button();
        
        if (is_connected) {
            mpu_data_processing();
#ifndef UAV_CONTROL
            gesture_recognizing();
#endif
#if !defined(UAV_CONTROL) && !defined(PLATFORM_CONTROL)
            // inform_about_charge(); // TODO: it should be recommented for normal working!!
#endif
        }
#ifdef ANALOG_IN_GET
        for (uint8_t i = 0; i < 4; i++) {
            nrfx_saadc_sample_convert(i, &signals[i]);
        }

        sprintf((char *) ble_output_array, "%d %d %d %d", signals[0], signals[1], signals[2], signals[3]);
        uint16_t output_arr_len = strlen(ble_output_array);
        ble_nus_data_send(&m_nus, ble_output_array, &output_arr_len, m_conn_handle);
        nrf_delay_ms(500);
#endif
    }
}

/*************************************************************************************************************************/
/**@brief  Function for obtaining the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get(void) {
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt) {
    if (p_evt->result != NRF_SUCCESS) return;

    switch (p_evt->id) {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
            break;
        case NRF_FSTORAGE_EVT_ERASE_RESULT:
            break;
        default:
            break;
    }
}

static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage) {
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage)) {
        //power_manage();
    }
}

void saadc_callback_handler(nrf_drv_saadc_evt_t const *p_event) {}

static void saadc_init(void) {
    nrf_saadc_channel_config_t ch_cfg_0 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); // p0.02
    nrf_saadc_channel_config_t ch_cfg_1 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1); // p0.03
    nrf_saadc_channel_config_t ch_cfg_2 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2); // p0.04
    nrf_saadc_channel_config_t ch_cfg_3 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3); // p0.05
    nrf_saadc_channel_config_t battery_vltg_cfg = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3); // p0.28

    nrf_drv_saadc_init(NULL, saadc_callback_handler);
    nrfx_saadc_channel_init(0, &ch_cfg_0);
    nrfx_saadc_channel_init(1, &ch_cfg_1);
    nrfx_saadc_channel_init(2, &ch_cfg_2);
    nrfx_saadc_channel_init(3, &ch_cfg_3);
    // + channel 4
    // + channel 5
    nrfx_saadc_channel_init(6, &battery_vltg_cfg);
}

/**@brief Function for assert macro callback.
 * @details This function will be called in case of an assert in the SoftDevice.
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    /* configuring the timer 1 */
    uint32_t time_ms = SWITCH_OFF_CALIBRATING; // time in miliseconds
    uint32_t time_ticks;
    nrf_drv_timer_config_t timer_1_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer_1, &timer_1_cfg, timer_1_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&timer_1, time_ms);
    nrf_drv_timer_extended_compare(&timer_1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    /* configuring the timer 0 */
    time_ms = CHARGE_INFO_TIME; // time in miliseconds
    nrf_drv_timer_config_t timer_0_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer_0, &timer_0_cfg, timer_0_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&timer_0, time_ms);
    nrf_drv_timer_extended_compare(&timer_0, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    /* configuring the timer 2 */
    time_ms = MPU_INFO_TIME; // time in miliseconds
    nrf_drv_timer_config_t timer_2_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer_2, &timer_2_cfg, timer_2_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&timer_2, time_ms);
    nrf_drv_timer_extended_compare(&timer_2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

/**@brief Function for the GAP initialization
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance
 */
static void gap_params_init(void) {
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt) {
    if (p_evt->type == BLE_NUS_EVT_RX_DATA){}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application */
static void services_init(void) {
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module
 * @param[in] nrf_error  Error code containing information about what went wrong
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module */
static void conn_params_init(void) {
    uint32_t               err_code;
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

/**@brief Function for putting the chip into sleep mode
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events
 * @details This function will be called for advertising events which are passed to the application
 * @param[in] ble_adv_evt  Advertising event
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    uint32_t err_code;

    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for handling BLE events
 * @param[in]   p_ble_evt   Bluetooth stack event
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            is_connected = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 4);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            is_connected = false;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            {
                ble_gap_phys_t const phys = {
                    .rx_phys = BLE_GAP_PHY_AUTO,
                    .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for the SoftDevice initialization.
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
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
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}

/**@brief Function for initializing the GATT library */
static void gatt_init(void) {
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality */
static void advertising_init(void) {
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing power management */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising */
static void advertising_start(void) {
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialization the mpu module */
static void gyroscope_accelerometer_init(void) {
    nrf_delay_ms(500); // delay is necessary for mpu initialization
    if (mpu_init(29, 30, 31, 27)) {
        while (1) {
            nrf_gpio_pin_set(LED_1);
            nrf_delay_ms(200);
            nrf_gpio_pin_clear(LED_1);
            nrf_delay_ms(200);
        }
    }
}

/**@brief Initialization of digital outputs & inputs */
static void digital_pins_init(void) {
    nrf_gpio_cfg_output(OUT_MCP);
    nrf_gpio_pin_set(OUT_MCP);
 
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_cfg_output(V_S_S);
    nrf_gpio_cfg_output(AD5206_CS);
    nrf_gpio_pin_set(AD5206_CS);

    nrf_gpio_cfg_input(MC_DIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(F_BTN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(CHRG, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_pin_set(LED_2);
    /* necessary stuff, without that bracelet turnes off after shutdowning and launching!! */
    while (!nrf_gpio_pin_read(MC_DIN));
}

/**@brief Function for writitng resistance value to the certain AD5206 channel
 * @param[in]   address   address of the channel [0-5]
 * @param[in]   value     numeric value of the resistance [0-255]
 */
static void ad5206_write(uint8_t address, uint8_t value) {
    if ((value <= 255) && (value >= 0) && (address < 6)) {
        nrf_gpio_pin_clear(AD5206_CS);
        nrf_delay_ms(10); // TODO: there was 50 ms!!!!

        switch (address) {
            case 0: // 3
                address = 3;
                value = 255 - value;
                break;
            case 1: // 1
                break;
            case 2: // 4
                address = 4;
                value = 255 - value;
                break;
            case 3: // 5
                address = 5;
                break;
            case 4: // 0 - it doesn't work
                address = 0;
                break;
            case 5: // 2 - it doesn't work
                address = 2;
                value = 255 - value;
                break;
        }
        nrf_drv_spi_transfer(&spi_inst, &address, 1, NULL, 0);
        nrf_delay_ms(10);
        /*switch (address) {
            case 2:
            case 3:
            case 4:
                value = 255 - value;
                break;
        }*/
        nrf_drv_spi_transfer(&spi_inst, &value, 1, NULL, 0);
        nrf_delay_ms(10);
        nrf_gpio_pin_set(AD5206_CS);
    }
}

/**@brief Function for saving resistnace values with help of fstorage
 * @param[in]   obj_ptr   pointer to the array
 */
static void save_resistance_values(uint8_t *obj_ptr) {
    ret_code_t rc;
    uint32_t tmp = (uint32_t)obj_ptr[0] | ((uint32_t)obj_ptr[1] << 8) | ((uint32_t)obj_ptr[2] << 16) | ((uint32_t)obj_ptr[3] << 24);
    rc = nrf_fstorage_write(&fstorage, 0x3f004, &tmp, sizeof(tmp), NULL);
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    //nrf_delay_ms(50);

    tmp = 0;
    tmp = (uint32_t)obj_ptr[4] | ((uint32_t)obj_ptr[5] << 8);
    rc = nrf_fstorage_write(&fstorage, 0x3f008, &tmp, sizeof(tmp), NULL);
    APP_ERROR_CHECK(rc);
    //nrf_delay_ms(50);
    wait_for_flash_ready(&fstorage);

}

/**@brief Function for reading important values with help of fstorage */
static void read_bracelet_values(void) {
    ret_code_t rc;
    uint32_t tmp = 0;
    uint8_t idxs_sig_pos = 0;
    uint8_t tmp_count = 0;
    uint8_t resistance[] = {0, 0, 0, 0, 0, 0};
    uint8_t chosen_signals = 0;

     wait_for_flash_ready(&fstorage);

    /* get the bracelet mode value */
    //rc = nrf_fstorage_read(&fstorage, 0x3f000, &tmp, sizeof(tmp));
    //APP_ERROR_CHECK(rc);
    //wait_for_flash_ready(&fstorage);
    //bracelet_mode = tmp;

    /* get the chosen signals value */    
    tmp = 0;
    rc = nrf_fstorage_read(&fstorage, 0x3f024, &tmp, sizeof(tmp));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    chosen_signals = tmp;

#ifdef CALIBRATION_DEBUGGING
    nrf_delay_ms(3000);
    char ble_buf[50];
    sprintf(ble_buf, "%d\n", chosen_signals);
    uint16_t output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
#endif

    /* get the resistance values */
    tmp = 0;
    rc = nrf_fstorage_read(&fstorage, 0x3f004, &tmp, sizeof(tmp));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    resistance[0] = tmp;
    resistance[1] = tmp >> 8;
    resistance[2] = tmp >> 16;
    resistance[3] = tmp >> 24;
    tmp = 0;
    rc = nrf_fstorage_read(&fstorage, 0x3f008, &tmp, sizeof(tmp));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    resistance[4] = tmp;
    resistance[5] = tmp >> 8;

#ifdef CALIBRATION_DEBUGGING
    sprintf(ble_buf, "%d %d %d %d\n", resistance[0], resistance[1], resistance[2], resistance[3]);
    output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
#endif

    /* set the resistance values for each channel */
    for (uint8_t i = 0; i < 6; i++) {
        ad5206_write(i, resistance[i]);
        nrf_delay_ms(50);

        if (chosen_signals & 0x01) {
            sensors_idxs[idxs_sig_pos] = i;
            idxs_sig_pos++;
        }
        chosen_signals >>= 1;
    }

    /* get the calibration values */
    tmp = 0;
    uint32_t tmp_addr_stage_1 = 0x3f00C;
    uint32_t tmp_addr_stage_2 = 0x3f018;
    for (uint8_t i = 0; i < 6; i += 2) {
        nrf_fstorage_read(&fstorage, tmp_addr_stage_1, &tmp, sizeof(tmp));
        wait_for_flash_ready(&fstorage);
        tmp_addr_stage_1 += 4;
        signals_stage_1[i] = tmp;
        signals_stage_1[i + 1] = tmp >> 16;

        tmp = 0;
        nrf_fstorage_read(&fstorage, tmp_addr_stage_2, &tmp, sizeof(tmp));
        wait_for_flash_ready(&fstorage);
        tmp_addr_stage_2 += 4;
        signals_stage_2[i] = tmp;
        signals_stage_2[i + 1] = tmp >> 16;

        /* calculate difference for normalization */
        diffs[tmp_count] = abs(signals_stage_2[tmp_count] - signals_stage_1[tmp_count]);
        tmp_count++;
    }

#ifdef CALIBRATION_DEBUGGING
    sprintf(ble_buf, "%d %d %d %d\n", signals_stage_1[0], signals_stage_1[1], signals_stage_1[2], signals_stage_1[3]);
    output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
    sprintf(ble_buf, "%d %d %d %d\n", signals_stage_2[0], signals_stage_2[1], signals_stage_2[2], signals_stage_2[3]);
    output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
#endif
}

/**@brief Configuring the signals levels with help of digital potentiometer */
void configuring_signals_levels(void) {
    bool got_coeff[] = {false, false, false, false, false, false};
    uint8_t tmp_coeffs[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // there were zeroses!!
    bool led_tmp = true;

    /* indicating */
    nrf_gpio_pin_clear(LED_2);
    for (uint8_t i = 0; i < 6; i++) {
        nrf_delay_ms(300);
        nrf_gpio_pin_set(LED_1);
        nrf_delay_ms(300);
        nrf_gpio_pin_clear(LED_1);
    }
    nrf_delay_ms(1000);
    nrf_gpio_pin_set(LED_2);

    for (uint8_t tmp_val = 255; tmp_val > 0; tmp_val -= 5) {
        for (uint8_t j = 0; j < 6; j++) {
            ad5206_write(j, tmp_val);
            nrf_delay_ms(50);
        }

        /* waiting for filtering the data */
        for (uint8_t k = 0; k < MEAS_VALUES; k++) {
            get_signals();
            nrf_delay_ms(5);
        }

        /* checking the values */
        for (uint8_t j = 0; j < 6; j++) {
            if ((bracelet_signals[j] <= CALIBR_VALUE) && !got_coeff[j]) {
                got_coeff[j] = true;
                tmp_coeffs[j] = tmp_val;
            }
        }
        if (got_coeff[0] & got_coeff[1] & got_coeff[2] & got_coeff[3] /*& got_coeff[4] & got_coeff[5]*/) {
            nrf_gpio_pin_clear(LED_2);
            break;
        }

        led_tmp = (led_tmp) ? false : true;
        if (led_tmp) {
            nrf_gpio_pin_set(LED_2);
        } else {
            nrf_gpio_pin_clear(LED_2);
        }

    }
    nrf_gpio_pin_clear(LED_2);

#ifdef CALIBRATION_DEBUGGING
    char ble_buf[50];
    sprintf(ble_buf, "%d %d %d %d\n", tmp_coeffs[0], tmp_coeffs[1], tmp_coeffs[2], tmp_coeffs[3]);
    uint16_t output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
#endif

    save_resistance_values(tmp_coeffs);

    /* restoring found resistance values */
    for (uint8_t i = 0; i < 6; i++) {
        ad5206_write(i, tmp_coeffs[i]);
        nrf_delay_ms(50);
    }
}

/**@brief Function for calculating and saving calibration values
 * New signals values, calculating differencies,write obtained values with help of fstorage
 */
static void get_save_calibration_values(void) {
    uint8_t idxs_sig_pos = 0;
    uint8_t represent_signals = 0;
    ret_code_t rc;
    uint32_t tmp;
    uint32_t tmp_addr_stage_1 = 0x3f00C;
    uint32_t tmp_addr_stage_2 = 0x3f018;

    for (uint8_t i = 0; i < 6; i += 2) {
        sensors_idxs[i] = 0xFF;
        sensors_idxs[i + 1] = 0xFF;
        signals_stage_2[i] = bracelet_signals[i];
        signals_stage_2[i + 1] = bracelet_signals[i + 1];
        diffs[i] = abs(signals_stage_2[i] - signals_stage_1[i]);
        diffs[i + 1] = abs(signals_stage_2[i + 1] - signals_stage_1[i + 1]);

        if (diffs[i] >= MIN_DIFFERENCE) {
            sensors_idxs[idxs_sig_pos] = i;
            idxs_sig_pos++;

            switch (i) {
                case 0:
                    represent_signals |= 0x01;
                    break;
                case 2:
                    represent_signals |= 0x04;
                    break;
                case 4:
                    represent_signals |= 0x10;
                    break;
            }
        }

        if (diffs[i + 1] >= MIN_DIFFERENCE) {
            sensors_idxs[idxs_sig_pos] = i + 1;
            idxs_sig_pos++;

            switch (i + 1) {
                case 1:
                    represent_signals |= 0x02;
                    break;
                case 3:
                    represent_signals |= 0x08;
                    break;
                case 5:
                    represent_signals |= 0x20;
                    break;
            }
        }

        /* saving the obtained values */
        tmp = signals_stage_1[i] | ((uint32_t)signals_stage_1[i + 1] << 16);
        rc = nrf_fstorage_write(&fstorage, tmp_addr_stage_1, &tmp, sizeof(tmp), NULL);
        APP_ERROR_CHECK(rc);
        tmp_addr_stage_1 += 4;
        wait_for_flash_ready(&fstorage);
        //nrf_delay_ms(50);

        tmp = signals_stage_2[i] | ((uint32_t)signals_stage_2[i + 1] << 16);
        rc = nrf_fstorage_write(&fstorage, tmp_addr_stage_2, &tmp, sizeof(tmp), NULL);
        APP_ERROR_CHECK(rc);
        tmp_addr_stage_2 += 4;
        wait_for_flash_ready(&fstorage);
        //nrf_delay_ms(50);
    }
#ifdef CALIBRATION_DEBUGGING
    char ble_buf[50];
    sprintf(ble_buf, "%d %d %d %d\n", signals_stage_1[0], signals_stage_1[1], signals_stage_1[2], signals_stage_1[3]);
    uint16_t output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
    sprintf(ble_buf, "%d %d %d %d\n", signals_stage_2[0], signals_stage_2[1], signals_stage_2[2], signals_stage_2[3]);
    output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);

    sprintf(ble_buf, "%d\n", represent_signals);
    output_arr_len = sizeof(ble_buf);
    ble_nus_data_send(&m_nus, (uint8_t *) &ble_buf, &output_arr_len, m_conn_handle);
#endif

    /* saving the represent_signals value */
    tmp = represent_signals;
    rc = nrf_fstorage_write(&fstorage, 0x3f024, &tmp, sizeof(tmp), NULL);
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    //nrf_delay_ms(50);
}

/**@brief Calculating the normalization coefficients and writing them with fstorage */
void get_normalization_parameters(void) {
    /* waiting for filtering the data */
    for (uint8_t k = 0; k < MEAS_VALUES; k++) {
        get_signals();
        nrf_delay_ms(5);
    }

    /* save the values before the gesture was done */
    for (uint8_t cnt = 0; cnt < 6; cnt++) signals_stage_1[cnt] = bracelet_signals[cnt];

    /* indicating */
    for (uint8_t i = 0; i < 8; i++) {
        nrf_gpio_pin_set(LED_1);
        nrf_gpio_pin_clear(LED_2);
        nrf_delay_ms(100);
        nrf_gpio_pin_clear(LED_1);
        nrf_gpio_pin_set(LED_2);
        nrf_delay_ms(100);
    }
    nrf_gpio_pin_clear(LED_2);
    nrf_delay_ms(6000); // waiting for the gesture

    /* waiting for filtering the data */
    for (uint8_t k = 0; k < MEAS_VALUES; k++) {
        get_signals();
        nrf_delay_ms(5);
    }

    /* calculating and saving the main calibration results */
    get_save_calibration_values();

        /* indicating */
    for (uint8_t i = 0; i < 8; i++) {
        nrf_gpio_pin_set(LED_1);
        nrf_gpio_pin_clear(LED_2);
        nrf_delay_ms(100);
        nrf_gpio_pin_clear(LED_1);
        nrf_gpio_pin_set(LED_2);
        nrf_delay_ms(100);
    }
}

/**@brief Calibration process of the bracelet */
void bracelet_calibrating(void) {
    /* getting resistance values */
    configuring_signals_levels();
    /* getting data for signals normalization */
    get_normalization_parameters();
}

/**@brief Changing the mode of the bracelet with pressing the certain button */
void f_btn_processing(void) {
    if (!nrf_gpio_pin_read(F_BTN)) { // if button is pressed
        if (!f_btn_is_pressed) { // timer will start, if button is pressed
            f_btn_is_pressed = true;
            timer_1.p_reg -> TASKS_CLEAR = 1;
            nrf_drv_timer_enable(&timer_1);
        } else { // starting the calibration, if button is pressed during 3 seconds
            if (timer_1_flag) {
                timer_1_flag = false;
                f_btn_is_pressed = false;
                bracelet_calibrating();
            }
        }
    } else if (f_btn_is_pressed) {
        f_btn_is_pressed = false;
        nrf_drv_timer_disable(&timer_1);
        //if (++bracelet_mode == 3) bracelet_mode = 0;
        //nrf_delay_ms(400);

        ///* save new mode */
        //ret_code_t rc;
        //uint32_t tmp = bracelet_mode;
        //rc = nrf_fstorage_write(&fstorage, 0x3f000, &tmp, sizeof(tmp), NULL);
        //APP_ERROR_CHECK(rc);
        //wait_for_flash_ready(&fstorage);
        //nrf_delay_ms(50);

        switching_struct.gestures_buttons = 0x02;
        /*uint16_t output_arr_len = 2;
        ble_nus_data_send(&m_nus, (uint8_t *) &switching_struct, &output_arr_len, m_conn_handle);*/
    }
}

/**@brief Switching off the bracelet with help of button */
void check_switch_off_by_button(void) {
    if (!nrf_gpio_pin_read(MC_DIN)) { // if button is pressed
        if (!sw_off_is_pressed) { // timer will start, if button is pressed
            sw_off_is_pressed = true;
            timer_1.p_reg -> TASKS_CLEAR = 1;
            nrf_drv_timer_enable(&timer_1);
        } else { // turning off, if button is pressed during 3 seconds
            if (timer_1_flag) {
                timer_1_flag = false;
                nrf_gpio_pin_clear(LED_2);
                for (uint8_t k = 0; k < 8; k++) {
                    nrf_gpio_pin_toggle(LED_1);
                    nrf_gpio_pin_toggle(LED_2);
                    nrf_delay_ms(100);
                }

                /* sending zero data */
                imu_data.a_x = 0;
                imu_data.a_y = 0;
#if !defined(UAV_CONTROL) && !defined(PLATFORM_CONTROL)
                imu_data.a_z = constrain(accelerations[2] * 102, -1000, 1000);
                imu_data.w_x = g_speed[0] * 100; // TODO: 100 - is temporary variable
                imu_data.w_y = g_speed[1] * 100; // TODO: 100 - is temporary variable
                imu_data.w_y = g_speed[2] * 100; // TODO: 100 - is temporary variable
#endif
                uint16_t output_arr_len = sizeof(imu_data);
                ble_nus_data_send(&m_nus, (uint8_t *) &imu_data, &output_arr_len, m_conn_handle);
		nrf_delay_ms(200);

                nrf_gpio_pin_clear(OUT_MCP);
                sd_power_system_off();
            }
        }
    } else if (sw_off_is_pressed) {
        nrf_drv_timer_disable(&timer_1);
        sw_off_is_pressed = false;
        switching_struct.gestures_buttons = 0x01;
        uint16_t output_arr_len = 2;
        ble_nus_data_send(&m_nus, (uint8_t *) &switching_struct, &output_arr_len, m_conn_handle);
    }
}

/**@brief Processing data obtained with the MPU */
void mpu_data_processing(void) {
    if (timer_2_flag) {
        timer_2_flag = false;
        mpu_get_float_data(accelerations, g_speed);

        imu_data.a_x = constrain(accelerations[0] * 102, -1000, 1000);
        imu_data.a_y = constrain(accelerations[1] * 102, -1000, 1000);
    #if !defined(UAV_CONTROL) && !defined(PLATFORM_CONTROL)
        /*imu_data.a_z = constrain(accelerations[2] * 102, -1000, 1000);
        imu_data.w_x = g_speed[0] * 100; // TODO: 100 - is temporary variable
        imu_data.w_y = g_speed[1] * 100; // TODO: 100 - is temporary variable
        imu_data.w_y = g_speed[2] * 100; // TODO: 100 - is temporary variable*/
    #endif

        uint16_t output_arr_len = sizeof(imu_data);
        ble_nus_data_send(&m_nus, (uint8_t *) &imu_data, &output_arr_len, m_conn_handle);

        /* restarting the timer */
        timer_2.p_reg -> TASKS_CLEAR = 1;
        nrf_drv_timer_enable(&timer_2);
    }
}

/**@brief Get the signals from sensors */
void get_signals(void) {
    // with running avg filter
    if (++count_adc >= MEAS_VALUES) count_adc = 0;

    for (uint8_t i = 0; i < 6; i++) { /* i < 4 */
        nrfx_saadc_sample_convert(i, &bracelet_signals[i]);
        bracelet_signals[i] = constrain(bracelet_signals[i], 0, 0x3FFF);

        /* filtering */
        adc_sum[i] = adc_sum[i] - adc_prev[i][count_adc] + bracelet_signals[i];
        adc_sum[i] = constrain(adc_sum[i], 0, 0x1FFF8);
        adc_prev[i][count_adc] = bracelet_signals[i];
        bracelet_signals[i] = adc_sum[i] >> filter_shift;
    }
}

/**@brief Recognizing the one gesture */
void gesture_recognizing(void) {
    bool break_flag = false;
    get_signals();

    if ((accelerations[0] >= -3) && (accelerations[0] <= 3) && (accelerations[1] >= -3) && (accelerations[1] <= 3)) {
        for (uint8_t i = 0; i < 6; i++) {
            if (sensors_idxs[i] > 5) break;
            uint8_t local_idx = sensors_idxs[i];

            /* min-max saturation */
            uint16_t local_max = max(signals_stage_1[local_idx], signals_stage_2[local_idx]);
            uint16_t local_min = min(signals_stage_1[local_idx], signals_stage_2[local_idx]);

            bracelet_signals[local_idx] = constrain(bracelet_signals[local_idx], local_min, local_max);

            /* normalization */
            float norm_val = (float) abs(bracelet_signals[local_idx] - signals_stage_1[local_idx]) / diffs[local_idx];
            if (norm_val < THRESHOLD_VALUE) {
                break_flag = true;
                break;
            }
        }
    } else {
        break_flag = true;
    }

    if (!break_flag) {
        if (!is_gesture_defined) {
            is_gesture_defined = true;
            switching_struct.gestures_buttons = 0x04; // gesture was recognized
            uint16_t output_arr_len = 2;
            ble_nus_data_send(&m_nus, (uint8_t *) &switching_struct, &output_arr_len, m_conn_handle);
            nrf_gpio_pin_set(LED_1);
            nrf_delay_ms(200);
        }
    } else if (is_gesture_defined) {
        is_gesture_defined = false;
        switching_struct.gestures_buttons = 0x00; // there is no gesture
        uint16_t output_arr_len = 2;
        ble_nus_data_send(&m_nus, (uint8_t *) &switching_struct, &output_arr_len, m_conn_handle);
        nrf_gpio_pin_clear(LED_1);
        nrf_delay_ms(200);
    }
}

/**@brief Calculating percentage of the charge */
uint8_t get_charge(void) {
    nrf_saadc_value_t tmp_vltg;
    nrf_gpio_pin_set(V_S_S);
    nrf_delay_ms(5);
    nrfx_saadc_sample_convert(6, &tmp_vltg);
    uint8_t vtg = (constrain(tmp_vltg, 0, 0x3FFF) - 2052) / 4.26; // TODO: find coefficient!!
    nrf_gpio_pin_clear(V_S_S);
    if (vtg > 100) vtg = 100;
    return vtg; 
}

/**@brief Periodically sending data about charge status */
void inform_about_charge(void) {
    if (timer_0_flag) {
        timer_0_flag = false;
        charge.percents = get_charge();
        uint16_t output_arr_len = 2;
        ble_nus_data_send(&m_nus, (uint8_t *) &charge, &output_arr_len, m_conn_handle);
        nrf_drv_timer_enable(&timer_0);
    }
}

/**@brief Handler for timer_0 events */
void timer_0_event_handler(nrf_timer_event_t event_type, void* p_context) {
    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            timer_0_flag = true;
            nrf_drv_timer_disable(&timer_0);
            break;
    }
}

/**@brief Handler for timer_1 events */
void timer_1_event_handler(nrf_timer_event_t event_type, void* p_context) {
    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            timer_1_flag = true;
            nrf_drv_timer_disable(&timer_1);
            break;
    }
}

/**@brief Handler for timer_2 events */
void timer_2_event_handler(nrf_timer_event_t event_type, void* p_context) {
    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            timer_2_flag = true;
            nrf_drv_timer_disable(&timer_2);
            break;
    }
}
