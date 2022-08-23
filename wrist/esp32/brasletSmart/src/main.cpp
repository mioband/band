#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "MPU9250.h"
#include "config.hpp"


void init_wireless_connection(void);
void init_base_pins(void);
void enable_supply(void);
void sleep_mode_stuff(void);
void init_serials(void);
void get_signals();
void send_data(void);
void gesture_recognizing(void);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void set_all_signal_coeff(uint8_t unit_val);
void set_signal_coefficient(uint8_t cell_addr, uint8_t value);
#ifdef MPU_ON
void mpu_init(void);
void mpu_data_processing(void);
void mpu_get_all_data(void);
void mpu_get_print_accelerations(void);
#endif

#ifdef MPU_ON
MPU9250 IMU(Wire, 0x68);
#endif
uint8_t serial_buf[BYTES_TO_READ] = {0,};
bool uart_flag = false;
int sensors[6]; // - array for saving sensors signal values

struct_message myData; // structure for using it with esp_now
esp_now_peer_info_t peerInfo; // creating peer interface
uint8_t broadcastAddress[] = {0x30, 0xc6, 0xf7, 0xb0, 0xf6, 0xf0}; // THE RECEIVER'S MAC-address
/*{0x80, 0x7D, 0x3A, 0x99, 0x85, 0x14};*/ /*{0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4};*/


void setup() {
#ifndef OLD_VERSION
    enable_supply();
#endif
    init_serials();
    init_wireless_connection();
    init_base_pins();
    sleep_mode_stuff();
#if defined(MPU_ON)
    mpu_init();
#endif
    set_all_signal_coeff(30);
    delay(500);
}


void loop() {
#ifdef DEEP_SLEEP_MODE_ON
    if (digitalRead(WKUP) == 0) {
        delay(300);
        Serial.println("Enter in the sleep mode");
        esp_deep_sleep_start();
    }
#endif

#if !defined(TEST_SIGNALS_READING) && !defined(MPU_TESTING)
    /* MAIN */
    get_signals();
    send_data();
    // Serial.print(analogRead(IN_VOLTAGE));
    // delay(400);
#elif defined(MPU_TESTING)
    mpu_get_all_data();
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
    Serial.print(sensors[5]);
    Serial.println();
#endif
}


/********************************************************************************/
void enable_supply(void) {
    pinMode(26, OUTPUT);
    digitalWrite(26, 1);
}


void init_serials(void) {
    Serial.begin(115200);
    while (!Serial);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN, false);
    while (!Serial2);
}


void sleep_mode_stuff(void) {
#if defined(SLEEP_MODE_TEST) || defined(DEEP_SLEEP_MODE_ON)
    switch (esp_sleep_enable_ext0_wakeup(WKUP, 0)) { // enable pin WKUP to exit ESP32 from sleep mode
        case ESP_OK:
            Serial.println("Sleep mode is configured successfully");
            break;
        case ESP_ERR_INVALID_ARG:
            Serial.println("Error, something is wrong.");
            break;
    }

#ifdef SLEEP_MODE_TEST
#ifdef OLD_VERSION
    digitalWrite(LED3, 0);
    digitalWrite(LED4, 0);
#else
    digitalWrite(RX_1_LED, 0);
    digitalWrite(RX_3_LED, 0);
    digitalWrite(RX_4_LED, 0);
#endif // OLD_VERSION
    Serial.println("Enter in the sleep mode");
    esp_deep_sleep_start();
#endif
#endif /* defined(SLEEP_MODE_TEST) || defined(DEEP_SLEEP_MODE_ON) */
}


void init_wireless_connection(void) {
    myData.id = DEVICE_ID;
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        while(1);
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    // esp_now_register_send_cb(OnDataSent); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        while(1);
    }

    Serial.println("Device is ready");
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    delay(100);
}


void set_signal_coefficient(uint8_t cell_addr, uint8_t value) {
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
    uint8_t tmp_buf[2] = {cell_addr, value};
    Serial2.write((char *) tmp_buf, 2);
}


void set_all_signal_coeff(uint8_t unit_val) {
    for (uint8_t i = 0; i < 6; i++) {
        set_signal_coefficient(i, unit_val);
        delay(250);
    }
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    _NOP();
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void get_signals(void) {
    if (Serial2.available()) {
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
}


void send_data(void) {
    if (uart_flag) {
    uart_flag = false;

    gesture_recognizing();
    mpu_data_processing();
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData)); // - отправка данных на ответное устройство
    }
}


void gesture_recognizing(void) {
    if ((1500 <= sensors[1]) && (1500 <= sensors[2]) && (1500 <= sensors[4])) {
        myData.w = 1; // клик
    } else {
        myData.w = 0; // нейтрал
    }
}


void init_base_pins(void) {
#ifdef OLD_VERSION
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    digitalWrite(LED3, 1);
    digitalWrite(LED4, 1);
#else
    pinMode(RX_1_LED, OUTPUT); // led
    pinMode(RX_3_LED, OUTPUT); // led
    pinMode(RX_4_LED, OUTPUT); // led
    digitalWrite(RX_1_LED, 1);
    digitalWrite(RX_3_LED, 1);
    digitalWrite(RX_4_LED, 1);
#endif

    pinMode(ENABLE, OUTPUT);
    digitalWrite(ENABLE, 1);
    pinMode(ON_VOLTAGE_MEAS, OUTPUT);
    digitalWrite(ON_VOLTAGE_MEAS, 1);
    pinMode(IN_VOLTAGE, ANALOG);
}


#ifdef MPU_ON
void mpu_init(void) {
    delay(1000);
    // pinMode(INT_MPU9250, INPUT);
    int status = IMU.begin();

    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
    } else {
        Serial.println("Status: ");
        Serial.println(status);
        // setting the accelerometer full scale range to +/-8G
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        // setting DLPF bandwidth to 20 Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
    }
}


void mpu_data_processing(void) {
    IMU.readSensor();
    float ax = IMU.getAccelX_mss();

#if (DEVICE_ID == 3)
    float ay = IMU.getAccelY_mss() + 4;
#else
    float ay = IMU.getAccelY_mss();
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


void mpu_get_print_accelerations(void) {
    IMU.readSensor();
    Serial.print((uint32_t) IMU.getAccelX_mss());
    Serial.print("\t");
    Serial.print((uint32_t) IMU.getAccelY_mss());
    Serial.print("\t");
    Serial.print((uint32_t) IMU.getAccelZ_mss());
    delay(250);
}
#endif /* MPU_ON */
