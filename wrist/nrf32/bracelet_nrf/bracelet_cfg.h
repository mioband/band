typedef enum {
    BRACELET_MODE,
    RESISTANCE_VALUES,
    CALIBR_SENSORS_STAGES,
    CHOSEN_SIGNALS
} save_cases_e;

typedef enum {
    LEFT_BRACELET,
    RIGHT_BRACELET
} left_right_device_e;

typedef struct {
    uint8_t idx;
    int16_t a_x;
    int16_t a_y;
#if !defined(UAV_CONTROL) && !defined(PLATFORM_CONTROL)
    int16_t a_z;
    int16_t w_x;
    int16_t w_y;
    int16_t w_z;
#endif
} imu_data_s;

typedef struct {
    uint8_t idx;
    uint8_t percents;
} charge_state_s;

typedef struct {
    uint8_t idx;
    uint8_t gestures_buttons;
} gesture_and_buttons_s;

//#define UAV_CONTROL
#define PLATFORM_CONTROL
//#define CALIBRATION_DEBUGGING

#define DEVICE_ID                      LEFT_BRACELET
#define MEAS_VALUES                    8 // must be the power of 2!
#define CHARGE_INFO_TIME               10000
#define SWITCH_OFF_CALIBRATING         3000
#define CALIBR_VALUE                   1000 // TODO: this value should be replaced!!  1000
#define MIN_DIFFERENCE                 100 // TODO: this value should be replaced!!
#define THRESHOLD_VALUE                0.6
#define OVERFLOW_VAL                   ((uint32_t)(0xFFFFFFFF/16.384))
#define constrain(amt,low,high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define max(val_0, val_1) ((val_0) < (val_1) ? val_1 : val_0)
#define min(val_0, val_1) ((val_0) > (val_1) ? val_1 : val_0)


/* DIGITAL OUTPUTS */
#define LED_1                          7 // red
#define LED_2                          8 // green
#define OUT_MCP                        23
#define V_S_S                          24
#define AD5206_CS                      26

/* DIGITAL INPUTS */
#define MC_DIN                         22
#define F_BTN                          6
#define CHRG                           25

/* BLE configurations */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "LARS_Bracelet"                             /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(6000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static uint32_t nrf5_flash_end_addr_get(void);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);
void saadc_callback_handler(nrf_drv_saadc_evt_t const *p_event);
static void saadc_init(void);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void timers_init(void);
static void gap_params_init(void);
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void nus_data_handler(ble_nus_evt_t * p_evt);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void sleep_mode_enter(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_stack_init(void);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void gatt_init(void);
static void advertising_init(void);
static void power_management_init(void);
static void idle_state_handle(void);
static void advertising_start(void);
static void gyroscope_accelerometer_init(void);
static void digital_pins_init(void);
static void ad5206_write(uint8_t address, uint8_t value);
static void save_resistance_values(uint8_t *obj_ptr);
static void read_bracelet_values(void);
void f_btn_processing(void);
void check_switch_off_by_button(void);
void mpu_data_processing(void);
void gesture_recognizing(void);
void inform_about_charge(void);
void bracelet_calibrating(void);
void get_signals(void);
void configuring_signals_levels(void);
static void get_save_calibration_values(void);
void get_normalization_parameters(void);
void timer_0_event_handler(nrf_timer_event_t event_type, void* p_context);
void timer_1_event_handler(nrf_timer_event_t event_type, void* p_context);
