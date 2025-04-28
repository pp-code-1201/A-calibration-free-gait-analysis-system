#ifndef __espress_h__
#define __espress_h__

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "data.h"

extern uint8_t hostAddress[];
extern uint8_t peerAddress1[];
extern uint8_t peerAddress2[];
extern uint8_t peerAddress3[];
extern uint8_t peerAddress4[];
extern uint8_t peerAddress5[];
extern uint8_t peerAddress6[];

extern esp_now_peer_info_t peerInfo;
extern command_t command;

extern data_t data1;
extern data_t data2;
extern data_t data3;
extern data_t data4;
extern data_t data5;
extern data_t data6;
extern data_t datas[6];

void OnDataSentCheck(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecvData(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
int OnDataRecvCommand(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void esp_init_host();
bool esp_send_data(uint8_t addr[], data_t data);
bool esp_send_command(uint8_t addr[], command_t command);


#endif