#include "Arduino.h"
#include "espress.h"
#include "imu.h"

#define Test_Data 0
const int VERBOSE= 0;

const int device_idx = 1;

const int irq_pin = D3;
const int i2c_addr = 0x69;
const int ledPin = D6;
bool blinkState = false;
int status = 0;

int scl_ = SCL;
int sda_ = SDA;

data_t data;

// A0:76:4E:40:1C:30
uint8_t hostAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x1C, 0x30};

esp_now_peer_info_t peerInfo;

hw_timer_t *timer = NULL;
uint32_t flag = 0;

static void IRAM_ATTR Timer0_CallBack(void) {
    flag = 1;
}

void starter() {
    while (status != 1) {
        if (status == 0) {
            delay(10);
        }
        if (status == 2) {
            imu_cali();
            status = 0;
            starter();
        }
    }
    timerAlarmEnable(timer);
}

void setup() {
    Serial.begin(115200);  // initialize Serial communication
    delay(2000);

    pinMode(ledPin, OUTPUT);

    timer = timerBegin(0, 80, true);  // divide 80MHz by 80
    timerAttachInterrupt(timer, &Timer0_CallBack, true);
    timerAlarmWrite(timer, 10000, true);

    esp_init_peer();

    if (!Test_Data) {
        imu_init();
    }
    data.id = device_idx;

    starter();  // Leave this as the last command in setup()
}

void loop() {
    if (flag) {
        digitalWrite(ledPin, HIGH);
        if (Test_Data) {
            data = imu_test_data(data);
        } else {
            data = imu_get_data(data);
        }
        esp_send_imu(hostAddress, data);
        digitalWrite(ledPin, LOW);
        // flag--; // This is kinda dangerous
        flag = 0;
    }

    if (status == 9) {
        timerEnd(timer);
        timer = NULL;
        ESP.restart();
    }

}
