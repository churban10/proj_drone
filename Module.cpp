#include "Module.h"
#include "mbed.h"


//-----------------------------------------------------------------------------------------
//mpu9250 register      
int   MPU9250_ADDRESS                    = (0x68<<1), //0b11010000
      WHO_AM_I_MPU9250                   = 0x75,
      FIFO_EN                            = 0x23,
      SMPLRT_DIV                         = 0x19,
      CONFIG                             = 0x1A,
      GYRO_CONFIG                        = 0x1B,
      ACCEL_CONFIG                       = 0x1C,
      ACCEL_CONFIG2                      = 0x1D,     
      ACCEL_XOUT_H                       = 0x3B,
      TEMP_OUT_H                         = 0x41,
      GYRO_XOUT_H                        = 0x43,
      
      PWR_MGMT_1                         = 0x6B,
      PWR_MGMT_2                         = 0x6C,  
      INT_PIN_CFG                        = 0x37,     
      
      AK8963_ADDRESS                     = (0x0C<<1), //0b00001100 0x0C<<1
      WHO_AM_I_AK8963                    = 0x00; // should return 0x48
     
      
 extern float gyro_bias[3];
//------------------------------------------------------------------------------------------------

extern I2C i2c; //(I2C_SDA,I2C_SCL); //sda, scl I2C_SDA,I2C_SCL D4, D5 A4, A5
extern Serial pc; // D1D0

extern float accel_f[3],gyro_f[3];

void MPU9250_RESET(void) {
  // reset device
  // writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset
  // bit; toggle reset device

  char cmd[2];
  cmd[0] = PWR_MGMT_1; // status
  cmd[1] = 0x80;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  wait(0.1);
}

void WHO_AM_I(void) {
  char cmd[2];
  cmd[0] = WHO_AM_I_MPU9250;
  i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
  i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
  wait(0.1);
  uint8_t DEVICE_ID = cmd[0];
  pc.printf("IMU device id is  %d \n\r", DEVICE_ID);
  wait(1);
}

void MPU9250_INIT(void) {
  // Initialize MPU9250 device
  // wake up device
  // and clear sleep mode bit (6), enable all sensors
  // wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro;
  // should check for PLL ready interrupt

  // get stable time source
  // writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto selects the best
  // available clock source PLL if ready, else use the Internal oscillator

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set gyro bandwidth to 41 Hz, delay 5.9 ms
  // DLPF_CFG = bits 2:0 = 011; this sets gyro bandwidth to 41 Hz, delay 5.9 ms
  // and internal sampling rate at 1 kHz Maximum delay is 4.9 ms which is just
  // over a 200 Hz maximum rate
  // writeByte(MPU9250_ADDRESS, CONFIG, 0x03); //page 13 register map

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the
  // same rate set in CONFIG above

  char cmd[3];
  cmd[0] = PWR_MGMT_1; // reset
  cmd[1] = 0x80;
  i2c.write(MPU9250_ADDRESS, cmd, 2);
  pc.printf("MPU 1 \n\r");
  wait(0.1);

  cmd[0] = PWR_MGMT_1; // Auto selects the best available clock source PLL if
                       // ready, else use the Internal oscillator
  cmd[1] = 0x01;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 2 \n\r");
  wait(0.1);

  cmd[0] = CONFIG;
  cmd[1] = 0x03; // 41Hz gyro bandwidth, 1kHz internal sampling
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 3 \n\r");
  wait(0.1);

  // sample rate divisor for all sensors, 1000/(1+4)=200 Hz for Gyro
  cmd[0] = SMPLRT_DIV;
  cmd[1] = 0x04; // 0x04
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 4 \n\r");
  wait(0.1);

  /*// Set gyroscope full scale range //page 14
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  left-shifted into positions 4:3 uint8_t c; cmd[0] = GYRO_CONFIG; //status
   i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
   i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
   pc.printf("MPU 5 \n\r");
   c = cmd[0];
   */

  // writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits
  // [7:5] 0b1110000 writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); //
  // Clear AFS bits [4:3] 0b00011000 writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c |
  // 0b00010011); // Set scale range for the gyro

  cmd[0] = GYRO_CONFIG;
  cmd[1] = 0b00010100; // Gyro full scale 1000 deg/sec; Gyro DLPF Enable 0b00010000
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 6 \n\r");
  wait(0.1);

  // Set accelerometer configuration
  // Accel fulll sacle range +/- 4g
  cmd[0] = ACCEL_CONFIG;
  cmd[1] = 0b00001000; // Accel
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 8 \n\r");
  wait(0.1);

  // Set accelerometer sample rate configuration (Fast sampling)
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.13
  // kHz

  cmd[0] = ACCEL_CONFIG2;
  cmd[1] = 0b00010011; //0b00001100
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 10 \n\r");
  wait(0.1);
  // XYZ Gyro accel enable (default)
  cmd[0] = PWR_MGMT_2;
  cmd[1] = 0x00;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 11 \n\r");
  wait(0.1);

  // cmd[0] = FIFO_EN;
  // cmd[1] = 0b11111000;
  // i2c.write(MPU9250_ADDRESS, cmd, 2);
  // wait(0.1);

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS,
  // enable I2C_BYPASS_EN so additional chips can join the I2C bus and all can
  // be controlled by the mbed as master
  // writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  // writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0)
  // interrupt page 29
  // I2c bypass mode
  cmd[0] = INT_PIN_CFG;
  cmd[1] = 0x22; // 0x02
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 12 \n\r");
  wait(0.1);
}

void gyro_bias_f(void) {

  int16_t gyro1[3];
  pc.printf("Please keep still 5 seconds\n\r");
  for (int i = 0; i < 100; i++) {
    MPU9250_GET_GYRO(gyro1);
    gyro_bias[0] = gyro_bias[0] + gyro1[0] / 32.8;
    gyro_bias[1] = gyro_bias[1] + gyro1[1] / 32.8;
    gyro_bias[2] = gyro_bias[2] + gyro1[2] / 32.8;
    pc.printf("bias finding i= %d\n\r", i);
  }
  gyro_bias[0] = gyro_bias[0] / 100.0;
  gyro_bias[1] = gyro_bias[1] / 100.0;
  gyro_bias[2] = gyro_bias[2] / 100.0;
  pc.printf("bias finding completed %\n\r");
}

void MPU9250_GET_GYRO(int16_t *destination) {
  // uint8_t rawData[6];  // x/y/z gyro register data stored here
  // readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six
  // raw data registers sequentially into data array destination[0] =
  // (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB
  // into a signed 16-bit value destination[1] = (int16_t)(((int16_t)rawData[2]
  // << 8) | rawData[3]); destination[2] = (int16_t)(((int16_t)rawData[4] << 8)
  // | rawData[5]);

  char cmd[6];
  cmd[0] = GYRO_XOUT_H;
  i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
  i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
  destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
  destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
  destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);

  // pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1],
  // destination[2]);
}

void MPU9250_GET_ACCEL(int16_t *destination) {
  // uint8_t rawData[6];  // x/y/z gyro register data stored here
  // readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six
  // raw data registers sequentially into data array destination[0] =
  // (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB
  // into a signed 16-bit value destination[1] = (int16_t)(((int16_t)rawData[2]
  // << 8) | rawData[3]); destination[2] = (int16_t)(((int16_t)rawData[4] << 8)
  // | rawData[5]);

  char cmd[6];
  cmd[0] = ACCEL_XOUT_H;
  i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
  i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
  destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
  destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
  destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);

  // pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1],
  // destination[2]);
}
