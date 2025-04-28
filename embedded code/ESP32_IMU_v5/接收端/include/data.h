#ifndef __data_h__
#define __data_h__

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Structure example to send data
// Must match the receiver structure
typedef struct imu_data_t {
    int ax;
    int ay;
    int az;
    int gx;
    int gy;
    int gz;
} imu_data_t;

typedef struct data_t {
    uint32_t index = 0;
    int id = 0;
    imu_data_t imu;
} data_t;

typedef struct command_t {
    int command = 0;
    uint32_t index = 0;
    int id = 0;
} command_t;

extern const int ledPin;
extern bool blinkState;

extern data_t data;

extern int counter;
extern const int VERBOSE;

#endif  // __data_h__