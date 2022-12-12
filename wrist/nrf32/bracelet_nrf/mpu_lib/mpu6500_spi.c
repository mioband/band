#include "mpu6500_spi.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

static const float g = 9.807f;
static const float _d2r = 3.14159265359f/180.0f;
static float _accelScale, _gyroScale;;
static float _axb, _ayb, _azb;
static float _gxb, _gyb, _gzb;
static float _axs = 1.0f;
static float _ays = 1.0f;
static float _azs = 1.0f;
static uint8_t _mpu_buffer[14];
static volatile uint8_t spi_xfer_done = 0;
const nrf_drv_spi_t spi_inst = NRF_DRV_SPI_INSTANCE(0);
nrf_drv_spi_config_t spi_cfg = NRF_DRV_SPI_DEFAULT_CONFIG;


void spi_event_handler(nrf_drv_spi_evt_t const * p_evt, void *p_context) {
    spi_xfer_done = 1;
}

static inline void mpu_cs_activate(void) {
    nrf_gpio_pin_clear(spi_cfg.ss_pin);
}

static inline void mpu_cs_deactivate(void) {
    nrf_gpio_pin_set(spi_cfg.ss_pin);
}

/* writes a byte to MPU6500 register given a register address and data */
static void write_register(uint8_t sub_addr, uint8_t data) {
    uint8_t tmp_arr[2] = {sub_addr, data};
    mpu_cs_activate();
    nrf_drv_spi_transfer(&spi_inst, tmp_arr, 2, NULL, 0);
    while (!spi_xfer_done);
    spi_xfer_done = 0;
    mpu_cs_deactivate();
    nrf_delay_ms(10);
}

/* reads acceleration and gyro registers from MPU6500 given a pointer to store data */
void read_acc_gyro_registers(uint8_t *dest) {
    uint8_t j = 6;
    uint8_t acc_addr = ACCEL_OUT;
    uint8_t gyr_addr = GYRO_OUT;
    for (uint8_t i = 0; i < 6; i++) {
        dest[i] = read_single_register(acc_addr);
        dest[j] = read_single_register(gyr_addr);
        acc_addr++;
        gyr_addr++;
        j++;
    }
}

/* reads one register from MPU6500 from given address */
uint8_t read_single_register(uint8_t subaddr) {
    uint8_t rx_buf[2];
    uint8_t tx_buf[2] = {subaddr | READWRITE_CMD, 0x00};
    mpu_cs_activate();
    nrf_drv_spi_transfer(&spi_inst, tx_buf, 2, rx_buf, 2);
    while (!spi_xfer_done);
    spi_xfer_done = 0;
    mpu_cs_deactivate();
    return rx_buf[1];
}

/* gets the MPU6500 WHO_AM_I register value, expected to be 0x70 */
uint8_t whoAmI(void) {
    return read_single_register(WHO_AM_I);
}

/* starts communication with the MPU-9250 */
uint8_t mpu_init(uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_sck, uint8_t pin_cs) {
    spi_cfg.ss_pin = pin_cs;
    spi_cfg.miso_pin = pin_miso;
    spi_cfg.mosi_pin = pin_mosi;
    spi_cfg.sck_pin = pin_sck;
    spi_cfg.frequency = NRF_DRV_SPI_FREQ_2M;
    spi_cfg.mode = NRF_SPI_MODE_3;
    if (nrf_drv_spi_init(&spi_inst, &spi_cfg, spi_event_handler, NULL) != NRF_SUCCESS) return 1;

    nrf_delay_ms(10);

    // check the WHO AM I byte, expected value is 0x70 (decimal 112) for MPU6500, 0x71 for MPU9250
    //if (whoAmI() != 0x70) return 2;/////////////////////////////////////////////////////////
    // select clock source to gyro
    write_register(PWR_MGMNT_1, CLOCK_SEL_PLL);
    // enable I2C master mode
    write_register(USER_CTRL, I2C_MST_EN);
    // set the I2C bus speed to 400 kHz
    write_register(I2C_MST_CTRL, I2C_MST_CLK);

    // reset the MPU6500
    write_register(PWR_MGMNT_1, PWR_RESET);
    // wait for MPU6500 to come back up
    nrf_delay_ms(10);

    // select clock source to gyro
    write_register(PWR_MGMNT_1, CLOCK_SEL_PLL);

    // enable accelerometer and gyro
    write_register(PWR_MGMNT_2, SEN_ENABLE);

    // setting accel range to 2G as default
    //write_register(ACCEL_CONFIG, ACCEL_FS_SEL_2G);
    mpu_set_accel_range(ACCEL_FS_SEL_2G);

    // setting the gyro range to 250DPS as default
    //write_register(GYRO_CONFIG, GYRO_FS_SEL_250DPS);
    mpu_set_gyro_range(GYRO_FS_SEL_250DPS);

    // setting bandwidth to 184Hz as default
    write_register(ACCEL_CONFIG2, DLPF_184);

    // setting gyro bandwidth to 184Hz
    write_register(MPU_CFG, DLPF_184);

    // setting the sample rate divider to 0 as default
    write_register(SMPDIV, 0x00);

    // enable I2C master mode
    write_register(USER_CTRL, I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    write_register(I2C_MST_CTRL, I2C_MST_CLK);

    // select clock source to gyro
    write_register(PWR_MGMNT_1, CLOCK_SEL_PLL);

    _axb = _ayb = _azb = 0;
    _gxb = _gyb = _gzb = 0;
    
    return 0; // successful init
}

/* sets the accelerometer full scale range to values other than default */
void mpu_set_accel_range(AccelRange range) {
    write_register(ACCEL_CONFIG, range);

    switch(range) {
        case ACCEL_RANGE_2G:
          _accelScale = g * 2.0f / 32767.5f;
          break; 
        case ACCEL_RANGE_4G:
            _accelScale = g * 4.0f / 32767.5f; // setting the accel scale to 4G
            break;
        case ACCEL_RANGE_8G:
            _accelScale = g * 8.0f / 32767.5f; // setting the accel scale to 8G
            break;
        case ACCEL_RANGE_16G:
            _accelScale = g * 16.0f / 32767.5f; // setting the accel scale to 16G
            break;
    }
}

/* sets the gyro full scale range to values other than default */
void mpu_set_gyro_range(GyroRange range) {
    write_register(GYRO_CONFIG, range);

    switch (range) {
        case GYRO_RANGE_250DPS:
            _gyroScale = 250.0f / 32767.5f * _d2r; // setting the gyro scale to 250DPS
            break;
        case GYRO_RANGE_500DPS:
            _gyroScale = 500.0f / 32767.5f * _d2r; // setting the gyro scale to 500DPS
            break;  
        case GYRO_RANGE_1000DPS:
            _gyroScale = 1000.0f / 32767.5f * _d2r; // setting the gyro scale to 1000DPS
            break;
        case GYRO_RANGE_2000DPS:
            _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
            break;
    }
}

/* sets the DLPF bandwidth to values other than default */
void mpu_set_DLPF_bandwidth(DLPFBandwidth bandwidth) {
    write_register(ACCEL_CONFIG2, bandwidth);
    write_register(MPU_CFG, bandwidth);
}

void set_accel_x_scale(float sc_val) {
    _axs = sc_val;
}

void set_accel_y_scale(float sc_val) {
    _ays = sc_val;
}

void set_accel_z_scale(float sc_val) {
    _azs = sc_val;
}

void set_accel_x_bias(float b_val) {
    _axb = b_val;
}

void set_accel_y_bias(float b_val) {
    _ayb = b_val;
}

void set_accel_z_bias(float b_val) {
    _azb = b_val;
}

/* read the data, each argiment should point to a array for x, y, and z */
void mpu_get_int_data(int16_t *acc_data, int16_t *gyro_data) {
    // grab the data from the MPU9250 and combine into 16 bit values
    read_acc_gyro_registers(_mpu_buffer);
    acc_data[0] = (((int16_t) _mpu_buffer[0]) << 8) | _mpu_buffer[1];
    acc_data[1] = (((int16_t) _mpu_buffer[2]) << 8) | _mpu_buffer[3];
    acc_data[2] = (((int16_t) _mpu_buffer[4]) << 8) | _mpu_buffer[5];
    gyro_data[0] = (((int16_t) _mpu_buffer[6]) << 8) | _mpu_buffer[7];
    gyro_data[1] = (((int16_t) _mpu_buffer[8]) << 8) | _mpu_buffer[9];
    gyro_data[2] = (((int16_t) _mpu_buffer[10]) << 8) | _mpu_buffer[11];

    /*acc_data[0] = (((int16_t) read_single_register(ACCEL_OUT)) << 8) | read_single_register(ACCEL_OUT + 1);
    acc_data[1] = (((int16_t) read_single_register(ACCEL_OUT + 2)) << 8) | read_single_register(ACCEL_OUT + 3);
    acc_data[2] = (((int16_t) read_single_register(ACCEL_OUT + 4)) << 8) | read_single_register(ACCEL_OUT + 5);
    gyro_data[0] = (((int16_t) read_single_register(GYRO_OUT)) << 8) | read_single_register(GYRO_OUT + 1);
    gyro_data[1] = (((int16_t) read_single_register(GYRO_OUT + 2)) << 8) | read_single_register(GYRO_OUT + 3);
    gyro_data[2] = (((int16_t) read_single_register(GYRO_OUT + 4)) << 8) | read_single_register(GYRO_OUT + 5);*/

}

void mpu_get_float_data(float *p_accel, float *p_gyro) {
    int16_t tmp_acc[3];
    int16_t tmp_gyr[3];
    mpu_get_int_data(tmp_acc, tmp_gyr);
    /* calculation of accelerations in m/(s*s) */
    p_accel[0] = (((float) tmp_acc[1] * _accelScale) - _axb) * _axs;
    p_accel[1] = (((float) tmp_acc[0] * _accelScale) - _ayb) * _ays;
    p_accel[2] = (((float) -tmp_acc[2] * _accelScale) - _azb) * _azs;

    /* calculation of rotation speed in rad/s */
    p_gyro[0] = ((float) tmp_gyr[1] * _gyroScale) - _gxb;
    p_gyro[1] = ((float) tmp_gyr[0] * _gyroScale) - _gyb;
    p_gyro[2] = ((float) -tmp_gyr[2] * _gyroScale) - _gzb;
}
