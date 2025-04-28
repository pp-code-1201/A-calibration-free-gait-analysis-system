#include "Arduino.h"
#include "data.h"
#include "espress.h"

const int ledPin = D6;
bool blinkState = false;
int counter = 0;
const int VERBOSE = 0;

data_t data, data1, data2, data3, data4, data5, data6;
// A0:76:4E:40:1C:30
uint8_t hostAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x1C, 0x30};
// 34:85:18:07:06:10
uint8_t peerAddress1[] = {0x34, 0x85, 0x18, 0x07, 0x06, 0x10};
// 34:85:18:06:E9:E0
uint8_t peerAddress2[] = {0x34, 0x85, 0x18, 0x06, 0xE9, 0xE0};
// 34:85:18:05:61:90
uint8_t peerAddress3[] = {0x34, 0x85, 0x18, 0x05, 0x61, 0x90};
// 34:85:18:05:B6:E8
uint8_t peerAddress4[] = {0x34, 0x85, 0x18, 0x05, 0xB6, 0xE8};
// 34:85:18:05:65:04
uint8_t peerAddress5[] = {0x34, 0x85, 0x18, 0x05, 0x65, 0x04};
// 34:85:18:05:B6:7C
uint8_t peerAddress6[] = {0x34, 0x85, 0x18, 0x05, 0xB6, 0x7C};

esp_now_peer_info_t peerInfo;

data_t datas[6] = {data1, data2, data3, data4, data5, data6};

void starter() {
    while (!Serial.available()) {
    }
    command_t command;
    // read command from serial
    if (Serial.available()) {
        String command_ = Serial.readStringUntil('\n');
        if (command_ == "start") {
            Serial.println("Sending start command...");
            command.command = 1;
            esp_send_command(peerAddress1, command);
            esp_send_command(peerAddress2, command);
            esp_send_command(peerAddress3, command);
            esp_send_command(peerAddress4, command);
            esp_send_command(peerAddress5, command);
            esp_send_command(peerAddress6, command);
        } else if (command_ == "cali") {
            Serial.println("Calibrating...");
            command.command = 2;
            esp_send_command(peerAddress1, command);
            esp_send_command(peerAddress2, command);
            esp_send_command(peerAddress3, command);
            esp_send_command(peerAddress4, command);
            esp_send_command(peerAddress5, command);
            esp_send_command(peerAddress6, command);
            starter();
        } else {
            Serial.println("Invalid command");
            starter();
        }
    }
}

void setup() {
    Serial.begin(115200);  // initialize Serial communication
    pinMode(ledPin, OUTPUT);
    delay(2000);
    while (!Serial) {
        delay(10);
    }
    esp_init_host();
    delay(2000);
    Serial.print("====== Wait for command ====== \n");
    starter();  // Leave this as the last command in setup()
}

void loop() {
    // if (counter >= 6) {
    //     counter = 0;
    //     Serial.print("====== Get 6 datas ====== \n");
    // }
}
