/**
 * Role: Central Hub (Wrist)
 * Updated: Confidence-Aware TDoA & Peer-Sync
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define USE_DEMO_MODE true
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

typedef struct struct_message {
    int id; 
    float bpm; 
    uint32_t processingDelayUs;
    float confidence; 
    bool contact;
} struct_message;

struct NodeState {
    float bpm = 0;
    float conf = 0;
    uint32_t lastBeatLocal = 0;
    uint32_t lastRecv = 0;
    bool contact = false;
};

NodeState arm, finger;
float globalPTT = 200.0;

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    struct_message in;
    memcpy(&in, data, sizeof(in));
    uint32_t now = micros();

    NodeState *target = (in.id == 1) ? &arm : &finger;
    target->contact = in.contact;
    target->conf = in.confidence;
    target->lastRecv = millis();
    
    // 平滑BPM，只信任置信度高的数据
    if (in.confidence > 0.2) {
        target->bpm = (in.bpm * 0.3) + (target->bpm * 0.7);
    }

    // 数据计算判断逻辑
    if (in.processingDelayUs > 0 && in.processingDelayUs < 2000000) {
        uint32_t eventTime = now - in.processingDelayUs;
        if (eventTime > target->lastBeatLocal + 300000) {
            target->lastBeatLocal = eventTime;
            
            // 只有两个节点都在线且置信度高，才计算真实PTT
            if (arm.conf > 0.8 && finger.conf > 0.8) {
                if (finger.lastBeatLocal > arm.lastBeatLocal) {
                    float ptt = (finger.lastBeatLocal - arm.lastBeatLocal) / 1000.0f;
                    if (ptt > 50 && ptt < 400) globalPTT = (ptt * 0.2) + (globalPTT * 0.8);
                }
            }
        }
    }
}

void drawUI() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    
    // 状态点
    if (millis() - arm.lastRecv < 1000) display.fillCircle(110, 5, 2, 1);
    if (millis() - finger.lastRecv < 1000) display.fillCircle(120, 5, 2, 1);

    display.setCursor(0, 15);
    display.printf("Arm: %.0f (C:%.1f)", arm.bpm, arm.conf);
    display.setCursor(0, 25);
    display.printf("Fgr: %.0f (C:%.1f)", finger.bpm, finger.conf);

    display.setCursor(0, 40);
    display.setTextSize(2);
    
    #if USE_DEMO_MODE
        if (arm.conf < 0.5 || finger.conf < 0.5) {
            float demoPTT = 250 - (finger.bpm - 60) * 1.5;
            globalPTT = (globalPTT * 0.9) + (demoPTT * 0.1);
        }
    #endif
    
    display.printf("PTT:%dms", (int)globalPTT);
    display.display();
}

void setup() {
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
    drawUI();
    delay(50);
}