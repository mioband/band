#include <stdint.h>
#ifndef MPU6500_SPI_H_
#define MPU6500_SPI_H_

#define READWRITE_CMD           0x80
#define MULTIPLEBYTE_CMD        0x40

// MPU6500 registers
#define ACCEL_OUT               0x3B // 0x3B - 0x3F
#define GYRO_OUT                0x43 // 0x43 - 0x48
#define TEMP_OUT                0x41 // 0x41 - 0x42
#define EXT_SENS_DATA_00        0x49
#define ACCEL_CONFIG            0x1C
#define ACCEL_FS_SEL_2G         0x00
#define ACCEL_FS_SEL_4G         0x08
#define ACCEL_FS_SEL_8G         0x10
#define ACCEL_FS_SEL_16G        0x18
#define GYRO_CONFIG             0x1B
#define GYRO_FS_SEL_250DPS      0x00
#define GYRO_FS_SEL_500DPS      0x08
#define GYRO_FS_SEL_1000DPS     0x10
#define GYRO_FS_SEL_2000DPS     0x18
#define ACCEL_CONFIG2           0x1D
#define DLPF_184                0x01
#define DLPF_92                 0x02
#define DLPF_41                 0x03
#define DLPF_20                 0x04
#define DLPF_10                 0x05
#define DLPF_5                  0x06
#define MPU_CFG                 0x1A
#define SMPDIV                  0x19
#define INT_PIN_CFG             0x37
#define INT_ENABLE              0x38
#define INT_DISABLE             0x00
#define INT_PULSE_50US          0x00
#define INT_WOM_EN              0x40
#define INT_RAW_RDY_EN          0x01
#define PWR_MGMNT_1             0x6B
#define PWR_CYCLE               0x20
#define PWR_RESET               0x80
#define CLOCK_SEL_PLL           0x01
#define PWR_MGMNT_2             0x6C
#define SEN_ENABLE              0x00
#define DIS_GYRO                0x07
#define USER_CTRL               0x6A
#define I2C_MST_EN              0x20
#define I2C_MST_CLK             0x0D
#define I2C_MST_CTRL            0x24
#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_DO             0x63
#define I2C_SLV0_CTRL           0x27
#define I2C_SLV0_EN             0x80
#define I2C_READ_FLAG           0x80
#define MOT_DETECT_CTRL         0x69
#define ACCEL_INTEL_EN          0x80
#define ACCEL_INTEL_MODE        0x40
#define LP_ACCEL_ODR            0x1E
#define WOM_THR                 0x1F
#define WHO_AM_I                0x75
#define FIFO_EN                 0x23
#define FIFO_TEMP               0x80
#define FIFO_GYRO               0x70
#define FIFO_ACCEL              0x08
#define FIFO_COUNT              0x72
#define FIFO_READ               0x74


typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;

/* mpu initialization */
uint8_t mpu_init(uint8_t pin_mosi, uint8_t pin_miso, uint8_t pin_sck, uint8_t pin_cs);

/* read the data, each argiment should point to a array for x, y, and x */
void mpu_get_int_data(int16_t *acc_data, int16_t *gyro_data);

/* read all data and transform it to the float */
void mpu_get_float_data(float *p_accel, float *p_gyro);

/* sets the DLPF bandwidth to values other than default */
void mpu_set_DLPF_bandwidth(DLPFBandwidth bandwidth);

/* sets the gyro full scale range to values other than default */
void mpu_set_gyro_range(GyroRange range);

/* sets the accelerometer full scale range to values other than default */
void mpu_set_accel_range(AccelRange range);

void read_acc_gyro_registers(uint8_t *dest) ;

uint8_t read_single_register(uint8_t subaddr);

uint8_t whoAmI(void);

/* set scale factors to the axes */
void set_accel_x_scale(float sc_val);
void set_accel_y_scale(float sc_val);
void set_accel_z_scale(float sc_val);

/* set bias factors to the axes */
void set_accel_x_bias(float b_val);
void set_accel_y_bias(float b_val);
void set_accel_z_bias(float b_val);
#endif /* MPU6500_SPI_H_ */
