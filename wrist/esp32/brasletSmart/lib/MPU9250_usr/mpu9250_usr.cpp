#include "mpu9250_usr.h"

int32_t gyro_bias[3], accel_bias[3];
int32_t accel_bias_reg[3];
volatile uint32_t sysTick_Time;
uint8_t ctrl;
float fGX_Cal;
float fGY_Cal;
float fGZ_Cal;
float gyroX;
float gyroY;
float gyroZ;
float accelX;
float accelY;
float accelZ;
float gyroX_filtered;
float gyroY_filtered;
float gyroZ_filtered;
int i;
int is_initialized;

// переменные для калмана
float varVolt = 0; // среднее отклонение (расчет в программе)
float varProcess = 0.2; // скорость реакции на изменение (подбирается вручную)
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;
uint8_t test = 0x00;
uint8_t value = 0;

uint8_t sensor_io_read(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

void delay_usr(uint32_t delayTime){
	uint32_t startTime =  sysTick_Time;
	while ( (sysTick_Time - startTime) < delayTime );
}

uint8_t sensor_io_read(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
	return I2Cx_ReadData(Address, Register, Nbytes, Data);
}

void sensor_io_write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value) {
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
}

uint8_t I2Cx_ReadData(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    uint8_t result = Wire.endTransmission();
    if (result != 0) {
        return result;
    }

    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available()) {
        uint8_t d = Wire.read();
        if (index < Nbytes) {
        Data[index++] = d;
        }
    }
    return 0;
}

void I2Cx_WriteData(uint16_t Address, uint8_t Register, uint8_t Value) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Value);
    Wire.endTransmission();
}

void sensor_ini(void) {
	delay_usr(100);
	if ((read_id()==0xFF)||(read_id()==0x70)||(read_id()==0x48)) {

	}
}

uint8_t read_id(void) {
    uint8_t value;
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(WHO_AM_I_MPU9250);
    uint8_t result = Wire.endTransmission();
    if (result != 0) {
    return result;
    }
    Wire.requestFrom(MPU9250_ADDRESS, sizeof(byte));
    value = Wire.read();
    Serial.println("id = ");
    Serial.println(value);
    return value;
}

void MPU9250_init(void)
{
	//power up gyro
	MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	//delay(100);
	MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);

	MPU9250_writeReg(MPU9250_ADDRESS, CONFIG, 0x03);

	MPU9250_writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
	//MPU6050_SMPLRT_DIV
	//gyro config 500

	MPU9250_writeReg(MPU9250_ADDRESS, GYRO_CONFIG, 0x08);
	//accel config 8g
	MPU9250_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 0x10);

    Serial.println("Initialization OK..");

}

//void calibrateMPU9250(float * dest1, float * dest2)
void MPU9250_calibrate()
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    //int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay_usr(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay_usr(200);

    // Configure device for bias calculation
    MPU9250_writeReg(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    MPU9250_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    MPU9250_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    MPU9250_writeReg(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    MPU9250_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    MPU9250_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay_usr(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    MPU9250_writeReg(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    MPU9250_writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    MPU9250_writeReg(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    MPU9250_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    //uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    MPU9250_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    MPU9250_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay_usr(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    MPU9250_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    //readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = MPU9250_readReg16Bit(MPU9250_ADDRESS, FIFO_COUNTH);
    packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        MPU9250_readMulti(MPU9250_ADDRESS, FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4)       & 0xFF;
    data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4)       & 0xFF;

    // Push gyro biases to hardware registers
    MPU9250_writeReg(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    MPU9250_writeReg(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    MPU9250_writeReg(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    MPU9250_writeReg(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    MPU9250_writeReg(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    MPU9250_writeReg(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
//    dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
//    dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
//    dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.


     //int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
     //readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
     data[0] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_H);
     data[1] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_L);
     data[2] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_H);
     data[3] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_L);
     data[4] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_H);
     data[5] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_L);
     accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]  );
     accel_bias_reg[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]  );
     accel_bias_reg[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]  );
     //((uint16_t)buffer[0] << 8) + buffer[1]))
     //readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
     //accel_bias_reg[1] = (uint16_t)((uint16_t)data[2]) << 8) + data[3]);
     //readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
     //accel_bias_reg[2] = (uint16_t)((uint16_t)data[4]) << 8) + data[5]);
//     int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//     readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//     accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//     readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//     accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//     readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//     accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

     //uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
     uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    // for(ii = 0; ii < 3; ii++) {
    //     if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    // }

     // Construct total accelerometer bias, including calculated average accelerometer bias from above
     accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
     accel_bias_reg[1] -= (accel_bias[1] / 8);
     accel_bias_reg[2] -= (accel_bias[2] / 8);

     data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
     data[1] = (accel_bias_reg[0])      & 0xFF;
     data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
     data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
     data[3] = (accel_bias_reg[1])      & 0xFF;
     data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
     data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
     data[5] = (accel_bias_reg[2])      & 0xFF;
     data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

//     Apparently this is not working for the acceleration biases in the MPU-9250
//     Are we handling the temperature correction bit properly?
//     Push accelerometer biases to hardware registers
//     MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//     MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//     MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//     MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//     MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//     MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
    Serial.println("Calibrating OK..");

}

void MPU9250_getAllData(float* aX, float* aY, float* aZ, float* gX, float* gY, float* gZ)
{
	uint8_t buffer[12];
	buffer[0] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_XOUT_H);
	buffer[1] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_XOUT_L);
	buffer[2] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_YOUT_H);
	buffer[3] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_YOUT_L);
	buffer[4] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_ZOUT_H);
	buffer[5] = MPU9250_readReg(MPU9250_ADDRESS, GYRO_ZOUT_L);
	buffer[6] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H);
	buffer[7] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_XOUT_L);
	buffer[8] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_YOUT_H);
	buffer[9] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_YOUT_L);
	buffer[10] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_ZOUT_H);
	buffer[11] = MPU9250_readReg(MPU9250_ADDRESS, ACCEL_ZOUT_L);
	gyroX=(((int16_t)((uint16_t)buffer[0] << 8) + buffer[1]))/65.5f*3.14f/180.0f;
	gyroY=(((int16_t)((uint16_t)buffer[2] << 8) + buffer[3]))/65.5f*3.14f/180.0f;
	gyroZ=(((int16_t)((uint16_t)buffer[4] << 8) + buffer[5]))/65.5f*3.14f/180.0f;
//	accelX=((((int16_t)((uint16_t)buffer[6] << 8) + buffer[7])))/4096.0f*9.8f;
//	accelY=((((int16_t)((uint16_t)buffer[8] << 8) + buffer[9])))/4096.0f*9.8f;
	*aX=((((int16_t)((uint16_t)buffer[6] << 8) + buffer[7])))/4096.0f*9.8f;
	*aY=((((int16_t)((uint16_t)buffer[8] << 8) + buffer[9])))/4096.0f*9.8f;
	*aZ=(((int16_t)((uint16_t)buffer[10] << 8) + buffer[11]))/4096.0f*9.8f;
	// *gX = filter(gyroX);
	// *gY = filter(gyroY);
	// *gZ = filter(gyroZ);
    *gX = gyroX;
	*gY = gyroY;
	*gZ = gyroZ;
//	accelX=((((int16_t)((uint16_t)buffer[6] << 8) + buffer[7])));
//	accelY=((((int16_t)((uint16_t)buffer[8] << 8) + buffer[9])));
//	accelZ=((((int16_t)((uint16_t)buffer[10] << 8) + buffer[11])));
	//accelX_offset=(((int16_t)((uint16_t)buffer[6] << 8) + buffer[7]))/4096.0f*9.8f;
}

float filter(float val) { //функция фильтрации
	Pc = P + varProcess;
	G = Pc/(Pc + varVolt);
	P = (1-G)*Pc;
	Xp = Xe;
	Zp = Xp;
	Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
return(Xe);
}

void MPU9250_writeReg(uint16_t Address, uint8_t Register, uint8_t value)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(value);
    Wire.endTransmission();
}

// Write a 16-bit register
void MPU9250_writeReg16Bit(uint16_t Address, uint8_t Register, uint16_t value)
{
	uint8_t buf[2];
	buf[0] = (uint8_t) (value >> 8);
	buf[1] = (uint8_t) (value & 0xFF);
	Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(buf, 2);
    Wire.endTransmission();
}

// Write a 32-bit register
void MPU9250_writeReg32Bit(uint16_t Address, uint8_t Register, uint32_t value)
{
	uint8_t buf[4];
	buf[0] = (uint8_t) (value >> 24);
	buf[1] = (uint8_t) (value >> 16);
	buf[2] = (uint8_t) (value >> 8);
	buf[3] = (uint8_t) (value & 0xFF);
	Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(buf, 4);
    Wire.endTransmission();
}

// Read an 8-bit register
uint8_t MPU9250_readReg(uint16_t Address, uint8_t Register)
{
    uint8_t value;
    Wire.beginTransmission(Address);
    Wire.write(Register);
    uint8_t result = Wire.endTransmission();
    if (result != 0) {
        return result;
    }
    Wire.requestFrom(Address, sizeof(byte));
    value = Wire.read();
    return value;
}

// Read a 16-bit register
uint16_t MPU9250_readReg16Bit(uint16_t Address, uint8_t Register)
{
  uint16_t value;
  uint8_t buff[2];
  MPU9250_readMulti(Address, Register, buff, 2);
  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  value = tmp;
  return value;
}

// Read a 32-bit register
uint32_t MPU9250_readReg32Bit(uint16_t Addr, uint8_t reg)
{
  uint32_t value;
  uint8_t buff[4];
  MPU9250_readMulti(Addr, reg, buff, 4);
  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];
  value = tmp;
  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void MPU9250_writeMulti(uint16_t Address, uint8_t Register, uint8_t* src, uint8_t count)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    for (int i = 0; i < count; ++i)
    {
        Wire.write((uint8_t)src[i]);
    }
    
    Wire.endTransmission();
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
uint8_t MPU9250_readMulti(uint16_t Address, uint8_t Register, uint8_t * Data, uint8_t count)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    uint8_t result = Wire.endTransmission();
    if (result != 0) {
        return result;
    }

    Wire.requestFrom(Address, count);
    uint8_t index = 0;
    while (Wire.available()) {
        uint8_t d = Wire.read();
        if (index < count) {
            Data[index++] = d;
        }
    }
    return 0;
}

// void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE)
// {
//     MPU9250_writeReg(I2C_ADDRESS, RegAddr, 1);

//     while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
//         if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
//             //_Error_Handler(__FILE__, __LINE__);
//         }
//     }

//     while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
// }


