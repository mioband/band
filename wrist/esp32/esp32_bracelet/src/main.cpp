#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "MPU9250.h"
#include "config.hpp"


MPU9250 IMU(Wire, 0x68); /* imu object */
float ax, ay; // variables for keeping acclelerations
uint8_t serial_buf[BYTES_TO_READ] = {0,}; // array that saves data received from gd32
uint16_t sensors[6]; // array for saving sensors signal values
uint8_t sensors_idxs[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // indices of the signals that are used for gesture recognizing
volatile uint8_t bracelet_mode = 0; // it keeps bracelet mode (different receivers)
uint16_t maxs[6], mins[6], diffs[6]; // arrays for keeping data obtained from calibrations process
volatile bool uart_flag = false; // uart receiving flag
volatile bool sw_off_is_pressed = false; // button flag, it is connected with switching off the bracelet
unsigned long switch_off_start_time = 0; // for keeping the start moment from pressed button (switch off)
unsigned long ground_zeros = 0; // for time measurement and send information about battery charge
volatile bool f_btn_is_pressed = false; // calibration/switch mode button flag
unsigned long f_btn_start_time = 0; // for keeping time the start from pressed button (calibration/switch mode)
struct_message myData; // structure for transmitting with esp_now
charge_state_msg charge = {DEVICE_ID, 0}; // struct with message about battery charge
esp_now_peer_info_t peerInfo; // peer interface
uint8_t broadcastAddress_pc[] = {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4};
uint8_t broadcastAddress_car[] = {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xEC};
uint8_t broadcastAddress_tank[] = {0x30, 0xAE, 0xA4, 0x3B, 0x31, 0xD0};
/*1 версия приемника: {0x80, 0x7D, 0x3A, 0x99, 0x85, 0x14}
  2 версия приемника (с оригинальным stm): {0x30, 0xc6, 0xf7, 0xb0, 0xf6, 0xf0}
  2 версия приемника (в корпусе): {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4}
  2 версия приемника (без корпуса, малярный скотч на еспе): {0x30, 0xc6, 0xf7, 0xb0, 0xf6, 0xd0}
  машинка: {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xEC}
  Brown Tank: {0x30, 0xAE, 0xA4, 0x13, 0xD5, 0x58}
  Green Tank: {0x30, 0xAE, 0xA4, 0x3B, 0x31, 0xD0}
*/

void setup() {
    enable_supply();
    setCpuFrequencyMhz(80); // setting 80 MHz CPU clock for reducing the consumption
    init_serials();
    init_base_pins();
    read_last_state();
    init_wireless_connection();
    mpu_init();
    ground_zeros = millis();
}


void loop() {
    change_mode_by_button();
    check_switch_off_by_button();
#if !defined(TEST_SIGNALS_READING) && !defined(MPU_TESTING)
    /* MAIN */
    inform_about_charge();
    get_signals();
    state_machine();
#elif defined(MPU_TESTING)
    mpu_get_all_data();
    // mpu_get_print_accelerations();
    // mpu_get_roll_pitch_angles();
#elif defined(ANALOG_SIGNALS_PRINTING)
    Serial.print("Voltage: ");
    Serial.print(analogRead(IN_VOLTAGE));
    Serial.print("\tTemperature: ");
    Serial.println(analogRead(TERM));
    delay(500);
#else
    get_signals();
    Serial.print(sensors[0]);
    Serial.print('\t');
    Serial.print(sensors[1]);
    Serial.print('\t');
    Serial.print(sensors[2]);
    Serial.print('\t');
    Serial.print(sensors[3]);
    Serial.print('\t');
    Serial.print(sensors[4]);
    Serial.print('\t');
    Serial.println(sensors[5]);
#endif
}


/********************************************************************************/
/**
* @brief Enabling maintaining the power supply
* @retval None
*/
void enable_supply(void) {
    pinMode(26, OUTPUT);
    digitalWrite(26, 1);
}

/**
* @brief Initialization of serial connections
* @retval None
*/
void init_serials(void) {
    Serial.begin(115200);
    while (!Serial);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN, false);
    while (!Serial2);
}

/**
* @brief Initialization of GPIOs
* @retval None
*/
void init_base_pins(void) {
    pinMode(F_BTN, INPUT);
    pinMode(RX_1_LED, OUTPUT); // led
    pinMode(RX_3_LED, OUTPUT); // led
    pinMode(RX_4_LED, OUTPUT); // led

    pinMode(ENABLE, OUTPUT);
    pinMode(MC_DIN, INPUT);
    digitalWrite(ENABLE, 1);
    pinMode(ON_VOLTAGE_MEAS, OUTPUT);
    pinMode(IN_VOLTAGE, ANALOG);
    pinMode(TERM, ANALOG);
}

/**
* @brief Reading data from EEPROM
* @retval None
*/
void read_last_state(void) {
    EEPROM.begin(EEPROM_SIZE);
    bracelet_mode = EEPROM.read(0);
    switch (bracelet_mode) {
        default:
            digitalWrite(RX_3_LED, 1);
            digitalWrite(RX_4_LED, 0);
            break;
        case 1:
            digitalWrite(RX_3_LED, 0);
            digitalWrite(RX_4_LED, 1);
            break;
        case 2:
            digitalWrite(RX_3_LED, 1);
            digitalWrite(RX_4_LED, 1);
            break;
    }

    uint8_t represent_signals = EEPROM.read(31);
    uint8_t idxs_sig_pos = 0;
    /* set resistance for each channel */
    for (uint8_t i = 1; i < 7; i++) {
        uint8_t tmp_val = EEPROM.read(i);
        send_to_gd32(i-1, tmp_val);
        delay(200);

        if (represent_signals & 0x01) {
            sensors_idxs[idxs_sig_pos] = i - 1;
            idxs_sig_pos++;
        }
        represent_signals >>= 1;
    }

    uint8_t tmp_count = 0;
    for (uint8_t i = 7; i <= 17; i += 2) {
        EEPROM.readBytes(i, &mins[tmp_count], 2);
        EEPROM.readBytes(i + 12, &maxs[tmp_count], 2);
        diffs[tmp_count] = abs(maxs[tmp_count] - mins[tmp_count]);
        tmp_count++;
    }
    send_to_gd32(NOMINAL_MODE, 0);
}

/**
* @brief Ebabling the Wi-Fi module
* @retval None
*/
void init_wireless_connection(void) {
    myData.id = DEVICE_ID;
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) error_clbck();

    memcpy(peerInfo.peer_addr, broadcastAddress_pc, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) error_clbck();

    memcpy(peerInfo.peer_addr, broadcastAddress_car, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) error_clbck();

    memcpy(peerInfo.peer_addr, broadcastAddress_tank, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) error_clbck();

    if (WiFi.setTxPower(WIFI_POWER_2dBm) != 1) error_clbck();
    WiFi.disconnect();
    Serial.println("Device is ready");
    delay(100);
}

/**
* @brief Sending data to gd32
* @param command: byte
* @param value: value that depends on the command
* @retval None
*/
void send_to_gd32(uint8_t command, uint8_t value) {
    /*
    potentiometer addresses:
    AD5206 | number in the line
    ---------------------------
    addr 0 |    4 (4-old)
    addr 1 |    6 (5-old)
    addr 2 |    3 (3-old)
    addr 3 |    5 (6-old)
    addr 4 |    2 (2-old)
    addr 5 |    1 (1-old)
    */
    uint8_t tmp_buf[2] = {command, value};
    Serial2.write((char *) tmp_buf, 2);
}

/**
* @brief Writing the values to EEPROM
* @retval None
*/
void write_normalization_values(uint16_t min_v, uint16_t max_v, uint8_t cnt) {
    EEPROM.writeBytes(cnt, &min_v, 2);
    EEPROM.commit();
    EEPROM.writeBytes(cnt + 12, &max_v, 2);
    EEPROM.commit();
}

/**
* @brief Configuring the signals levels with help of digital potentiometer connected with gd32
* @retval None
*/
static void configuring_signals_levels(void) {
    bool got_coeff[6] = {false,};
    uint8_t tmp_coeffs[6] = {0,};
    unsigned long strt_tm;
    bool led_tmp = true;
#ifdef THRESHOLD_DEBUG
    Serial.println("Relax the hand");
#endif
    /* indicating */
    for (uint8_t i = 0; i < 6; i++) {
        delay(300);
        digitalWrite(RX_3_LED, 1);
        digitalWrite(RX_4_LED, 1);
        delay(300);
        digitalWrite(RX_3_LED, 0);
        digitalWrite(RX_4_LED, 0);   
    }
    digitalWrite(RX_1_LED, led_tmp);
    delay(1000);
    for (uint8_t tmp_val = 255; tmp_val > 0; tmp_val -= 5) {
        send_to_gd32(SET_VAL_TO_ALL, tmp_val);
        /* waiting for data from gd32 */
        strt_tm = millis();
        while (!uart_flag && (millis() - strt_tm <= 1000)) get_signals();
        uart_flag = false;
        /* checking the values */
        for (uint8_t j = 0; j < 6; j++) {
            if ((sensors[j] <= CALIBR_VALUE) && !got_coeff[j]) {
                got_coeff[j] = true;
                tmp_coeffs[j] = tmp_val;
            }
        }
        digitalWrite(RX_1_LED, led_tmp);
        led_tmp = (led_tmp) ? false : true;
    }
    digitalWrite(RX_1_LED, 0);

    /* restoring found resistance values */
    for (uint8_t i = 0; i < 6; i++) {
        send_to_gd32(i, tmp_coeffs[i]);
        EEPROM.write(i + 1, tmp_coeffs[i]);
        EEPROM.commit();
        delay(50);
    }
}

/**
* @brief Calculating the normalization coefficients and writing them to EEPROM
* @retval None
*/
static void get_normalization_parameters(void) {
    unsigned long strt_tm;
    delay(50);
    send_to_gd32(GET_VALUES, 0);
    // waiting for data from gd32 
    strt_tm = millis();
    while (!uart_flag && (millis() - strt_tm <= 1000)) get_signals();
    uart_flag = false;
    for (uint8_t cnt = 0; cnt < 6; cnt++) mins[cnt] = sensors[cnt]; // previous values
#ifdef THRESHOLD_DEBUG
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(sensors[i]);
        Serial.print('\t');
    }
    Serial.println("\nMake the gesture");
#endif
    /* indicating */
    for (uint8_t i = 0; i < 8; i++) {
        delay(100);
        digitalWrite(RX_3_LED, 1);
        digitalWrite(RX_4_LED, 0);
        delay(100);
        digitalWrite(RX_3_LED, 0);
        digitalWrite(RX_4_LED, 1);   
    }
    digitalWrite(RX_4_LED, 0);
    delay(6000);
    send_to_gd32(GET_VALUES, 0);
    /* waiting for data from gd32 */
    strt_tm = millis();
    while (!uart_flag && (millis() - strt_tm < 1000)) get_signals();
    uart_flag = false;
#ifdef THRESHOLD_DEBUG
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(sensors[i]);
        Serial.print('\t');
    }
#endif
    /* new signals values, calculating differencies, 
        write to eeprom obtained values
    */
    uint8_t tmp_cnt = 7;
    uint8_t idxs_sig_pos = 0;
    uint8_t represent_signals = 0;
    for (uint8_t cnt = 0; cnt < 6; cnt++) {
        sensors_idxs[cnt] = 0xFF;
        maxs[cnt] = sensors[cnt];
        diffs[cnt] = abs(maxs[cnt] - mins[cnt]);
        if (diffs[cnt] >= MIN_DIFFERENCE) {
            sensors_idxs[idxs_sig_pos] = cnt;
            idxs_sig_pos++;
            write_normalization_values(mins[cnt], maxs[cnt], tmp_cnt);
            switch (cnt) {
                case 0:
                    represent_signals |= 0x01;
                    break;
                case 1:
                    represent_signals |= 0x02;
                    break;
                case 2:
                    represent_signals |= 0x04;
                    break;
                case 3:
                    represent_signals |= 0x08;
                    break;
                case 4:
                    represent_signals |= 0x10;
                    break;
                case 5:
                    represent_signals |= 0x20;
                    break;
            }
        }
        tmp_cnt += 2;
    }
    EEPROM.write(31, represent_signals);
    EEPROM.commit();
#ifdef THRESHOLD_DEBUG
    Serial.println("\nminimums\tmaximums\tdiff");
    char tmp_str[16];
    for (uint8_t k = 0; k < 6; k++) {
        sprintf(tmp_str, "%i\t%i\t%i", mins[k], maxs[k], diffs[k]);
        Serial.println(tmp_str);
    }
#endif
}

/**
* @brief Calibration procees of the bracelet
* @retval None
*/
void calibrating(void) {
    /* resetting the gd32 and mpu */
    digitalWrite(RX_1_LED, 0);
    digitalWrite(ENABLE, 0);
    delay(10);
    digitalWrite(ENABLE, 1);
    delay(10);
    mpu_init();
    /* getting resistance values */
    configuring_signals_levels();
#ifdef THRESHOLD_DEBUG
    Serial.println("Waiting for new signals");
#endif
    /* getting data for signals normalization */
    get_normalization_parameters();
    delay(50);
    send_to_gd32(NOMINAL_MODE, 0);
#ifdef THRESHOLD_DEBUG
    Serial.println("Calibration was finished.");
#endif
}

/**
* @brief Changing the mode of the bracelet with pressing the certain button
* @retval None
*/
void change_mode_by_button(void) {
    if (!digitalRead(F_BTN)) {
        if (!f_btn_is_pressed) {
            f_btn_is_pressed = true;
            f_btn_start_time = millis();
        } else {
            unsigned long f_btn_timing = millis();
            if (f_btn_start_time > f_btn_timing) {
                f_btn_timing += (0xFFFFFFFF - f_btn_start_time);
                f_btn_start_time = 0;
            }

            if (f_btn_timing - f_btn_start_time >= SWITCH_CALIBRATING) {
                f_btn_is_pressed = false;
#ifdef AUTOTHRESHOLDING
                calibrating();
                switch (bracelet_mode) {
                    case 0:
                        digitalWrite(RX_3_LED, 1);
                        digitalWrite(RX_4_LED, 0);
                        break;
                    case 1:
                        digitalWrite(RX_3_LED, 0);
                        digitalWrite(RX_4_LED, 1);
                        break;
                    case 2:
                        digitalWrite(RX_3_LED, 1);
                        digitalWrite(RX_4_LED, 1);
                        break;
                }
#endif
            }
        }
    } else if (f_btn_is_pressed) {
        f_btn_is_pressed = false;
        if (++bracelet_mode == 3) bracelet_mode = 0;
        switch (bracelet_mode) {
            case 0:
                digitalWrite(RX_3_LED, 1);
                digitalWrite(RX_4_LED, 0);
                break;
            case 1:
                digitalWrite(RX_3_LED, 0);
                digitalWrite(RX_4_LED, 1);
                break;
            case 2:
                digitalWrite(RX_3_LED, 1);
                digitalWrite(RX_4_LED, 1);
                break;
    }
        delay(400);
        EEPROM.write(0, bracelet_mode);
        EEPROM.commit();
    }
}

/**
* @brief Switching off the bracelet with help of button
* @retval None
*/
void check_switch_off_by_button(void) {
    if (!digitalRead(MC_DIN)) {
        if (!sw_off_is_pressed) {
            sw_off_is_pressed = true;
            switch_off_start_time = millis();
        } else {
            unsigned long current_time = millis();
            if (switch_off_start_time > current_time) {
                current_time += (0xFFFFFFFF - switch_off_start_time);
                switch_off_start_time = 0;
            }
            
            if (current_time - switch_off_start_time >= SWITCH_OFF_TIME) {
                digitalWrite(RX_1_LED, 1);
                digitalWrite(RX_3_LED, 1);
                digitalWrite(RX_4_LED, 1);
                esp_deep_sleep_start();
            }
        }
    } else if (sw_off_is_pressed) {
        sw_off_is_pressed = false;
    }
}

/**
* @brief Calculating percentage of the charge
* @retval None
*/
uint8_t get_charge(void) {
    digitalWrite(ON_VOLTAGE_MEAS, 1);
    delay(5);
    uint8_t vtg = (analogRead(IN_VOLTAGE) - 2052) / 4.26; // 1981<->2052 - digital value that equal 3.3V !! in denominator: 5.14 <-> 4.26
    digitalWrite(ON_VOLTAGE_MEAS, 0);
    if (vtg > 100) vtg = 100;
    return vtg; 
}

/**
* @brief Periodically sending data about charge status
* @retval None
*/
void inform_about_charge(void) {
    unsigned long current_ticks = millis();
    if (ground_zeros > current_ticks) {
        current_ticks += (0xFFFFFFFF - ground_zeros);
        ground_zeros = 0;
    }

    if (current_ticks - ground_zeros >= CHARGE_INFO_TIME) {
        charge.val = get_charge();
        esp_now_send(broadcastAddress_pc, (uint8_t *)&charge, sizeof(charge));
        ground_zeros = millis();
    }
}

/**
* @brief Receiving data from gd32 about sensors signals
* @retval None
*/
void get_signals(void) {
    if (Serial2.read() == 'S') {
        uart_flag = true;
        Serial2.readBytes(serial_buf, BYTES_TO_READ);
        uint8_t tmp_count = 0;
        for (uint8_t i = 0; i < 6; i++) {
            sensors[i] = serial_buf[tmp_count] | (serial_buf[tmp_count+1] << 8);
            tmp_count += 2;
        }
    }
}

/**
* @brief Recognizing the one gesture
* @retval None
*/
void gesture_recognizing(void) {
    bool break_flag = false;
    if ((ax >= -3) && (ax <= 3) && (ay >= -3) && (ay <= 3)) {
        for (uint8_t i = 0; i < 6; i++) {
            if (sensors_idxs[i] > 5) break;
            uint8_t local_idx = sensors_idxs[i];

            /* min-max saturation */
            uint16_t local_max = max(mins[local_idx], maxs[local_idx]);
            uint16_t local_min = min(mins[local_idx], maxs[local_idx]);
            if (sensors[local_idx] > local_max) {
                sensors[local_idx] = local_max;
            } else if (sensors[local_idx] < local_min) {
                sensors[local_idx] = mins[local_idx];
            }
            /* normalization */
            float norm_val = (float) abs(sensors[local_idx] - mins[local_idx]) / diffs[local_idx];

#ifdef THRESHOLD_DEBUG
            Serial.print('n'); Serial.print(local_idx); Serial.print(' ');
            Serial.println(norm_val, 2);
#endif
            if (norm_val < THRESHOLD_VALUE) {
                break_flag = true;
                break;
            }
        }
    } else {
       break_flag = true; 
    }

#ifdef THRESHOLD_DEBUG
    if (!break_flag) {
        myData.w = 1; // есть жест
        digitalWrite(RX_1_LED, 1);
        Serial.println('1');
    } else {
        myData.w = 0; // расслабленная рука
        digitalWrite(RX_1_LED, 0);
        Serial.println('0');
    }
#else
    if (!break_flag) {
        if (myData.w == 0) {
            myData.w = 1; // gesture was recognized
            digitalWrite(RX_1_LED, 1);
        }
    } else if (myData.w) {
        myData.w = 0; // there is no gesture
        digitalWrite(RX_1_LED, 0);
    }
#endif
}

/**
* @brief Wireless sendizing the data with help of ESP-NOW
* @retval None
*/
void send_data(uint8_t *addr_ptr) {
    if (uart_flag) {
        uart_flag = false;
        mpu_data_processing();
// #if (DEVICE_ID == 3)
        gesture_recognizing();
// #endif
        esp_now_send(addr_ptr, (uint8_t *)&myData, sizeof(myData));
    }
}

/**
* @brief Switching between different wireless receivers
* @retval None
*/
void state_machine(void) {
    switch (bracelet_mode) {
        case 0:
            send_data(broadcastAddress_pc);
            break;
        case 1:
            send_data(broadcastAddress_car);
            break;
        case 2:
            send_data(broadcastAddress_tank);
            break;            
    }
}

/**
* @brief Error callback, switches off the bracelet if problem is found
* @retval None
*/
void error_clbck(void) {
    while (1) {
        digitalWrite(RX_1_LED, 0);
        digitalWrite(RX_3_LED, 0);
        digitalWrite(RX_4_LED, 0);
        delay(250);
        digitalWrite(RX_1_LED, 1);
        digitalWrite(RX_3_LED, 1);
        digitalWrite(RX_4_LED, 1);
        delay(250);
        if (!digitalRead(F_BTN)) esp_deep_sleep_start();
    }
}

/**
* @brief Initialization and configuration of the MPU 
* @retval None
*/
void mpu_init(void) {
    delay(500);
    // pinMode(INT_MPU9250, INPUT);
    int status = IMU.begin();

    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
    } else {
        Serial.print("MPU init status: ");
        Serial.println(status);
        // setting the accelerometer full scale range to +/-2G
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        // setting DLPF bandwidth to 20 Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
        IMU.setAccelCalX(ACC_X_BIAS, ACC_X_SCALE);
        IMU.setAccelCalY(ACC_X_BIAS, ACC_Y_SCALE);
    }
}

/**
* @brief Processing data obtained with the MPU 
* @retval None
*/
void mpu_data_processing(void) {
    IMU.readSensor();
    ax = IMU.getAccelX_mss();

#if (DEVICE_ID == 3)
    ay = IMU.getAccelY_mss() + 4;
#else
    ay = IMU.getAccelY_mss();
#endif

    if (ax > 9) {
        ax = 9;
    } else if (ax < -9) {
        ax = -9;
    }

    if (ay > 9) {
        ay = 9;
    } else if (ay < -9) {
        ay = -9;
    }

    if (ax < 0) {
        myData.y = -ax;
        myData.x = 1;
    } else {
        myData.y = ax;
        myData.x = 0;
    }

    if (ay < 0) {
        myData.q = -ay;
        myData.t = 1;
    } else {
        myData.q = ay;
        myData.t = 0;
    }
}

/**
* @brief Testing function, it can be used for obtaining all MPU data 
* @retval None
*/
void mpu_get_all_data(void) {
    IMU.readSensor();

    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);

    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);

    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);

    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);
    delay(250);
}

/**
* @brief Testing function, it can be used for obtaining MPU accelerations and angle speeds
* @retval None
*/
void mpu_get_print_accelerations(void) {
    IMU.readSensor();
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.println(IMU.getGyroZ_rads(), 6);
    delay(50);
}

/**
* @brief Testing function, it can be used for calculating roll and pitch angles with help of MPU data
* @retval None
*/
void mpu_get_roll_pitch_angles(void) {
    IMU.readSensor();
    float a_x = IMU.getAccelX_mss();
    float a_y = IMU.getAccelY_mss();
    float a_z = IMU.getAccelZ_mss();
    Serial.print(atan(a_y / a_z) * 57.3, 6);
    Serial.print("\t");
    Serial.println((atan(-a_x / pow(a_y*a_y + a_z*a_z, 1/2))) * 57.3, 6);
    delay(50);
}
