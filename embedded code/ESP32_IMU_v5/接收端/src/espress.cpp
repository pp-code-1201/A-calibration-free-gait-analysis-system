#include "espress.h"

#include "data.h"

void OnDataSentCheck(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // callback function when data is sent
    char macStr[18];
    Serial.print("Packet to: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void esp_init_peer() {
    // Add one host addr
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Suck ESP-NOW!");
        return;
    }

    esp_now_register_send_cb(OnDataSentCheck);  // Only add this on host

    memcpy(peerInfo.peer_addr, hostAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;  // For speed reason.

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(OnDataRecvData);
}

void esp_init_host() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Suck ESP-NOW!");
        return;
    }

    esp_now_register_send_cb(OnDataSentCheck);  // Only add this on host

    // register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // register first peer
    memcpy(peerInfo.peer_addr, peerAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer1");
        return;
    }
    // register second peer
    memcpy(peerInfo.peer_addr, peerAddress2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer2");
        return;
    }
    // register third peer
    memcpy(peerInfo.peer_addr, peerAddress3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer3");
        return;
    }
    // register fourth peer
    memcpy(peerInfo.peer_addr, peerAddress4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer4");
        return;
    }
    // register fifth peer
    memcpy(peerInfo.peer_addr, peerAddress5, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer5");
        return;
    }
    // register sixth peer
    memcpy(peerInfo.peer_addr, peerAddress6, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer6");
        return;
    }
    Serial.println("ESP receiver success");
    esp_now_register_recv_cb(OnDataRecvData);
}

bool esp_send_command(uint8_t addr[], command_t command) {
    // send data to one addr
    esp_err_t result = esp_now_send(addr, (uint8_t *)&command, sizeof(command));
    if (result == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

bool esp_send_data(uint8_t addr[], data_t data) {
    // send data to one addr
    esp_err_t result = esp_now_send(addr, (uint8_t *)&data, sizeof(data));
    if (result == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

void OnDataRecvData(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    /* This prints addr*/
    counter++;
    if (VERBOSE) {
        char macStr[18];
        Serial.print("Packet received from: ");
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.println(macStr);
    }
    memcpy(&data, incomingData, sizeof(data));
    Serial.print("Recv: " + String(data.id) + "-" + String(data.index) + " ");
    Serial.print(String(data.imu.ax) + "," + String(data.imu.ay) + "," + String(data.imu.az) + "," + String(data.imu.gx) + "," + String(data.imu.gy) + "," + String(data.imu.gz));
    Serial.print("\n");
}

int OnDataRecvCommand(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    /* This prints addr*/
    if (VERBOSE) {
        char macStr[18];
        Serial.print("Packet received from: ");
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.println(macStr);
    }
    Serial.println(*incomingData);
    return *incomingData;
}
