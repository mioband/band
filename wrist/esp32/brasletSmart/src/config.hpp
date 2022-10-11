#pragma once

#define DEVICE_ID           2 // 2 - металлический ремешок or 3 - резиновый ремешок

// #define MPU_TESTING
#define MPU_ON
// #define TEST_SIGNALS_READING
// #define ANALOG_SIGNALS_PRINTING
#define THRESHOLD_DEBUG
#define AUTOTHRESHOLDING

// #define OLD_VERSION


/* Digital pins */
#define F_BTN               GPIO_NUM_13 // falling
#define ON_VOLTAGE_MEAS     GPIO_NUM_27
#define ENABLE              GPIO_NUM_23
/* ADC */
#define IN_VOLTAGE          GPIO_NUM_34
#define TERM                GPIO_NUM_36 // SENSOR_VP
/* LED */
#ifdef OLD_VERSION
#define INT_MPU9250         GPIO_NUM_33
#define LED3                GPIO_NUM_18
#define LED4                GPIO_NUM_32
#define UART_RX_PIN         GPIO_NUM_17
#define UART_TX_PIN         GPIO_NUM_16
#else
#define UART_RX_PIN         GPIO_NUM_16
#define UART_TX_PIN         GPIO_NUM_17
#define INT_MPU9250         GPIO_NUM_19
#define RX_1_LED            GPIO_NUM_33 // 32K_XN
#define RX_3_LED            GPIO_NUM_14 // MTMS
#define RX_4_LED            GPIO_NUM_25
#define MC_DIN              GPIO_NUM_35 // VDET_2
#define BYTES_TO_READ       12
#endif
/* byte 0 - bracelet_mode, bytes 1-6 - coefficients, byte 7 - max_diff_num, bytes 8-9 - threshold value, byte 10 - condition flag */
#define EEPROM_SIZE         11

#if (DEVICE_ID == 2)
#define ACC_X_BIAS          0 // id2: -7
#define ACC_Y_BIAS          0 // id2: 0??
#elif (DEVICE_ID == 3)
#define ACC_X_BIAS          0 // id3: -8
#define ACC_Y_BIAS          0 // id3: -5, +4 in the code
#endif
#define ACC_X_SCALE         1 // 1.5
#define ACC_Y_SCALE         1 // 1.5

#define CHARGE_INFO_TIME    10000
#define SWITCH_OFF_TIME     3000
#define SWITCH_CALIBRATING  3000
#define NOMINAL_MODE        0xA0
#define VALUE_TO_ONE_CHAN   0xB0
#define GET_VALUES          0xC0
#define SET_DELAY           0xD0
#define SET_VAL_TO_ALL      0xE0
#define CALIBR_VALUE        1000

typedef struct {
    int id; // must be unique for each sender board
    int x;
    int y;
    int t;
    int q;
    int w; // жесты
} struct_message;

typedef struct {
    uint8_t header;
    uint8_t val;
} charge_state_msg;


void enable_supply(void);
void init_serials(void);
void init_base_pins(void);
void read_last_state(void);
void WiFi_enable(void);
void init_wireless_connection(void);
void calibrating(void);
void send_to_gd32(uint8_t command, uint8_t value);
void write_condition_values(uint8_t arr_number, uint16_t *thrshld_ptr, uint8_t cond_val);
void calibrating(void);
void change_mode_by_button(void);
void check_switch_off_by_button(void);
uint8_t get_charge(void);
void inform_about_charge(void);
void get_signals();
void send_data(uint8_t * addr_ptr);
void gesture_recognizing(void);
void state_machine(void);
void error_clbck(void);
#ifdef MPU_ON
void mpu_init(void);
void mpu_data_processing(void);
void mpu_get_all_data(void);
void mpu_get_print_accelerations(void);
void mpu_get_roll_pitch_angles(void);
#endif
