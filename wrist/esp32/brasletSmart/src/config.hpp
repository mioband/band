#pragma once

#define DEVICE_ID           3

// #define MPU_TESTING
#define MPU_ON
// #define SLEEP_MODE_TEST
#define DEEP_SLEEP_MODE_ON
// #define TEST_SIGNALS_READING

// #define OLD_VERSION


/* Digital pins */
#define WKUP                GPIO_NUM_13 // falling
#define ON_VOLTAGE_MEAS     GPIO_NUM_27
#define ENABLE              GPIO_NUM_23
/* ADC */
#define IN_VOLTAGE          GPIO_NUM_34
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
#define BYTES_TO_READ       12
#endif


typedef struct {
    int id; // must be unique for each sender board
    int x;
    int y;
    int t;
    int q;
    int w; // жесты
} struct_message;
