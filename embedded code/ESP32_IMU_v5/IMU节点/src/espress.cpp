#include "espress.h"

#include "data.h"

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // callback function when data is sent
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecvData(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    /* This prints addr*/
    char macStr[18];
    // Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Serial.println(macStr);
    if (*incomingData == 1) {
        status = 1;
    } else if (*incomingData == 2) {
        status = 2;
    }
    // Serial.println(*incomingData);
}

int OnDataRecvCommand(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    /* This prints addr*/
    char macStr[18];
    // Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Serial.println(macStr);
    // Serial.println(*incomingData);
    return *incomingData;
}
void esp_init_peer() {
    // Add one host addr
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        // Serial.println("Suck ESP-NOW!");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, hostAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;  // For speed reason.

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        // Serial.println("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(OnDataRecvData);
}

bool esp_send_imu(uint8_t addr[], data_t data) {
    // send data to one addr
    esp_err_t result = esp_now_send(addr, (uint8_t *)&data, sizeof(data));
    if (result == ESP_OK) {
        return true;
    } else {
        return false;
    }
}
