#ifndef __MODULE_H__
#define __MODULE_H__
#include "mbed.h"

void WHO_AM_I(void);
void MPU9250_INIT(void);
void gyro_bias_f(void);
void gyro_bias_f(void);
void MPU9250_GET_GYRO(int16_t * destination);
void MPU9250_GET_ACCEL(int16_t * destination);

#endif
