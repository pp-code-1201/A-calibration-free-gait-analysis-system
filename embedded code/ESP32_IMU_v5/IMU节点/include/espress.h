#ifndef __espress_h__
#define __espress_h__

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "data.h"

extern uint8_t hostAddress[];
extern esp_now_peer_info_t peerInfo;
extern data_t data;
extern command_t command;
extern int status;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecvData(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void esp_init_peer();
bool esp_send_imu(uint8_t addr[], data_t data);

#endif