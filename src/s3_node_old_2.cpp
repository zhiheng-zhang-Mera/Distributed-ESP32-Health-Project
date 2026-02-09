/**
 * Role: Edge Computing Node (Ring OR Belt)
 * Features: Peer-to-Peer Backup, Multi-Stage Confidence, Signal Debug
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <esp_now.h>
#include <WiFi.h>

// ================= CONFIG =================
#define NODE_ID 1  // 1=Arm(Belt), 2=Finger(Ring)
#define DEBUG_MODE true

#define PIN_SDA D4
#define PIN_SCL D5

uint8_t masterAddr[] = {0x38, 0x18, 0x2B, 0x99, 0x5C, 0xA4}; 

typedef struct struct_message {
    int id; 
    float bpm; 
    uint32_t processingDelayUs;
    float confidence; // 0=None, 0.3=Warming, 0.5=PeerData, 1.0=Healthy
    bool contact;
} struct_message;

struct_message myData;
struct_message peerData;
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

// ================= CALLBACK =================
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message in;
    memcpy(&in, incomingData, sizeof(in));
    if (in.id != NODE_ID) memcpy(&peerData, &in, sizeof(in));
}

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        particleSensor.setup(0x3F, 1, 2, 100, 411, 4096);
    }
    mpu.begin();

    WiFi.mode(WIFI_STA);
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, masterAddr, 6);
        esp_now_add_peer(&peerInfo);
    }
}

void loop() {
    static uint32_t lastBeatMicros = 0;
    static float lastGoodBPM = 72.0;
    static float currentBPM = 0;
    
    long irValue = particleSensor.getIR();
    myData.contact = (irValue > 30000);

    // --- 核心逻辑：数据获取与补偿 ---
    if (myData.contact) {
        if (checkForBeat(irValue)) {
            uint32_t delta = micros() - lastBeatMicros;
            lastBeatMicros = micros();
            
            if (delta > 3000000) { // 冷启动
                myData.confidence = 0.3;
                currentBPM = lastGoodBPM;
            } else if (delta > 250000) { // 正常
                currentBPM = 60000000.0 / delta;
                lastGoodBPM = currentBPM;
                myData.confidence = 1.0;
            }
        }
        // 5秒无感应则衰减
        if (micros() - lastBeatMicros > 5000000) {
            myData.confidence = 0.2;
            currentBPM *= 0.98;
        }
    } else {
        // 尝试借调兄弟节点数据
        if (peerData.contact && peerData.confidence > 0.6) {
            currentBPM = peerData.bpm;
            myData.confidence = 0.5; // 标记借调
        } else {
            currentBPM = 0;
            myData.confidence = 0.0;
        }
    }

    // --- 发送 ---
    static uint32_t lastTx = 0;
    if (millis() - lastTx > 100) {
        myData.id = NODE_ID;
        myData.bpm = currentBPM;
        myData.processingDelayUs = (lastBeatMicros > 0) ? (micros() - lastBeatMicros) : 0;
        esp_now_send(masterAddr, (uint8_t *) &myData, sizeof(myData));
        lastTx = millis();
    }

    // --- 调试输出：支持所有场景调试 ---
    if (DEBUG_MODE && millis() % 50 == 0) {
        Serial.printf("ID:%d | IR:%ld | Conf:%.1f | BPM:%.1f | PeerBPM:%.1f | Delay:%ld\n", 
                      NODE_ID, irValue, myData.confidence, currentBPM, peerData.bpm, myData.processingDelayUs);
    }
}