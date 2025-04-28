#ifndef __imu_h__
#define __imu_h__

#include "Arduino.h"
#include "BMI160Gen.h"
#include "data.h"

extern const int irq_pin;
extern const int i2c_addr;
extern int calibrateOffsets;

extern BMI160GenClass BMI160;

extern data_t data;

void imu_init();
void imu_cali();
data_t imu_get_data(data_t data);
data_t imu_test_data(data_t data);

#endif /* __imu_h__ */