#include "imu.h"

void imu_init() {
    Wire.begin();
    // initialize device
    Serial.println("Initializing IMU device...");
    // BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
    BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
    uint8_t dev_id = BMI160.getDeviceID();
    Serial.print("DEVICE ID: ");
    Serial.println(dev_id, HEX);
}

void imu_cali() {
    if (VERBOSE) {
        Serial.println("Internal sensor offsets BEFORE calibration...");
        Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
        Serial.print("\t");  // -76
        Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
        Serial.print("\t");  // -235
        Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
        Serial.print("\t");  // 168
        Serial.print(BMI160.getGyroOffset(X_AXIS));
        Serial.print("\t");  // 0
        Serial.print(BMI160.getGyroOffset(Y_AXIS));
        Serial.print("\t");  // 0
        Serial.println(BMI160.getGyroOffset(Z_AXIS));
        Serial.println("About to calibrate. Make sure your board is stable and upright");
    }
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    BMI160.autoCalibrateGyroOffset();

    BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

    if (VERBOSE) {
        Serial.println("Internal sensor offsets AFTER calibration...");
        Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
        Serial.print("\t");  // -76
        Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
        Serial.print("\t");  // -2359
        Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
        Serial.print("\t");  // 1688
        Serial.print(BMI160.getGyroOffset(X_AXIS));
        Serial.print("\t");  // 0
        Serial.print(BMI160.getGyroOffset(Y_AXIS));
        Serial.print("\t");  // 0
        Serial.println(BMI160.getGyroOffset(Z_AXIS));
    }
}

data_t imu_get_data(data_t data) {
    data.index++;
    BMI160.readMotionSensor(data.imu.ax, data.imu.ay, data.imu.az, data.imu.gx, data.imu.gy, data.imu.gz);
    return data;
}

data_t imu_test_data(data_t data) {
    data.index++;
    data.imu.ax = 114;
    data.imu.ay = 514;
    data.imu.az = 1919;
    data.imu.gx = 810;
    data.imu.gy = 222;
    data.imu.gz = 333;
    return data;
}