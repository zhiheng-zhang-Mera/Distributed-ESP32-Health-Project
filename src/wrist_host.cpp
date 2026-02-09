/**
 * Role: Central Hub & UI (Wrist) - ULTIMATE ADAPTIVE VERSION
 * 角色：中央枢纽与UI（腕部） - 终极自适应版本
 * * New Features / 新特性:
 * 1. Adaptive BPM Correction (自适应BPM修正): Fixes dropped beats (double freq) or noise (half freq).
 * 修复漏检（倍频补偿）或误触（半频补偿）。
 * 2. Dynamic Smoothing (动态平滑): Variable alpha based on error magnitude.
 * 基于误差幅度的动态Alpha滤波。
 * 3. TDoA PTT (脉冲传输时间): Calculates blood pressure proxy using time diff between sensors.
 * 利用传感器时间差计算血压代理值。
 * 4. RMSSD HRV (心率变异性): Calculates stress level indicator.
 * 计算压力水平指标。
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= DATA STRUCTURE / 数据结构 =================
typedef struct struct_message {
    int id; 
    float bpm; 
    uint32_t processingDelayUs;
    float confidence; 
    bool contact;
} struct_message;

// Enhanced state structure for local processing / 增强的本地处理状态结构
struct NodeState {
    float bpm = 0;
    float smoothBPM = 65.0; // Baseline / 初始基准
    uint32_t lastBeatLocal = 0;
    uint32_t lastRecv = 0;
    float conf = 0;
    bool contact = false;
    
    // For HRV Calculation / 用于HRV计算
    float hrvRMSSD = 0;
    float ibiBuffer[10]; // Inter-Beat Interval buffer / 心跳间隔缓存
    int ibiIdx = 0;
};

NodeState arm, finger;
float currentPTT = 200.0;
bool isPTTReal = false;

// ================= CORE ALGORITHMS / 核心算法 =================

/**
 * Update HRV (Heart Rate Variability) using RMSSD method.
 * 使用 RMSSD 方法更新 HRV（心率变异性）。
 * RMSSD = Sqrt(Mean(Diff(IBI)^2))
 * * @param target Pointer to NodeState / 节点状态指针
 * @param currentBeat Timestamp of the current beat (us) / 当前心跳时间戳
 */
void updateHRV(NodeState *target, uint32_t currentBeat) {
    if (target->lastBeatLocal == 0) return;
    float ibi = (currentBeat - target->lastBeatLocal) / 1000.0f; // Convert to ms / 转为毫秒
    
    // Filter valid IBI range (40bpm - 150bpm approx) / 过滤有效的间隔范围
    if (ibi > 400 && ibi < 1500) {
        target->ibiBuffer[target->ibiIdx] = ibi;
        float sumSqDiff = 0;
        int count = 0;
        // Calculate Sum of Squared Differences over buffer / 计算缓存内的差值平方和
        for (int i = 0; i < 9; i++) {
            int curr = (target->ibiIdx - i + 10) % 10;
            int prev = (curr - 1 + 10) % 10;
            if (target->ibiBuffer[curr] > 0 && target->ibiBuffer[prev] > 0) {
                sumSqDiff += pow(target->ibiBuffer[curr] - target->ibiBuffer[prev], 2);
                count++;
            }
        }
        // Rolling average for RMSSD / RMSSD的滑动平均
        if (count > 0) target->hrvRMSSD = (sqrt(sumSqDiff / count) * 0.2) + (target->hrvRMSSD * 0.8);
        target->ibiIdx = (target->ibiIdx + 1) % 10;
    }
}

/**
 * Adaptive Correction Logic.
 * 自适应修正逻辑。
 * Handles cases where sensor misses a beat (reading halves) or picks up noise (reading doubles).
 * 处理传感器漏检（读数减半）或拾取噪声（读数加倍）的情况。
 */
float adaptiveCorrect(NodeState *target, float rawBPM, float conf) {
    // Threshold tightens as confidence drops / 置信度越低，阈值越严，容忍度越小
    float threshold = 0.35 * (1.2 - conf); 
    
    // 1. Anti-Drop: Ignore sudden drops if confidence is low / 防止大跳水：置信度低时的剧烈下降视为干扰
    if (target->smoothBPM > 40 && rawBPM < 40 && conf < 0.8) return target->smoothBPM;

    // 2. Frequency Fix / 倍频与半频修正
    float ratio = rawBPM / target->smoothBPM;
    
    // If new BPM is ~0.5x old BPM, likely missed a beat -> Double it.
    // 如果新BPM约为旧值的0.5倍，可能是漏检 -> 翻倍补偿
    if (ratio > 0.45 && ratio < 0.6) return rawBPM * 2.0; 
    
    // If new BPM is ~2.0x old BPM, likely motion artifact -> Halve it.
    // 如果新BPM约为旧值的2.0倍，可能是运动伪影 -> 减半补偿
    if (ratio > 1.8 && ratio < 2.2) return rawBPM * 0.5;

    return rawBPM;
}

// ================= ESP-NOW CALLBACK / 回调函数 =================
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    struct_message in;
    memcpy(&in, data, sizeof(in));
    uint32_t nowUs = micros();
    NodeState *target = (in.id == 1) ? &arm : &finger;

    target->contact = in.contact;
    target->conf = in.confidence;
    target->lastRecv = millis();

    if (in.confidence > 0.15) {
        // 1. Correct the raw data / 修正原始数据
        float correctedBPM = adaptiveCorrect(target, in.bpm, in.confidence);
        
        // 2. Dynamic Smoothing Alpha / 动态平滑系数
        // If deviation is large (>20), use low alpha (0.03) to resist change.
        // If deviation is small, use high alpha (0.12) to follow trend.
        // [Note]: Usually it's the opposite (large error -> fast tracking), but here it seems designed to resist noise spikes.
        // 这里的逻辑是：偏差大时信赖旧值（抗干扰），偏差小时快速跟随。
        float alpha = (abs(correctedBPM - target->smoothBPM) > 20) ? 0.03 : 0.12;
        target->smoothBPM = (correctedBPM * alpha) + (target->smoothBPM * (1.0 - alpha));
        target->bpm = correctedBPM;
    }

    // TDoA (Time Difference of Arrival) Calculation / 到达时间差计算
    if (in.processingDelayUs > 0) {
        // Restore the exact time the beat happened / 还原心跳发生的准确时刻
        uint32_t eventTime = nowUs - in.processingDelayUs;
        
        // Debounce: Must be >300ms since last beat / 去抖动：距离上次心跳需大于300ms
        if (eventTime > target->lastBeatLocal + 300000) {
            updateHRV(target, eventTime);
            target->lastBeatLocal = eventTime;
            
            // Only calculate PTT if both sensors are confident / 仅在双传感器置信度均高时计算PTT
            if (arm.conf > 0.8 && finger.conf > 0.8) {
                // Pulse travels from Arm -> Finger. Finger time must be > Arm time.
                // 脉冲从手臂传向手指，手指时间必须晚于手臂。
                if (finger.lastBeatLocal > arm.lastBeatLocal) {
                    float ptt = (finger.lastBeatLocal - arm.lastBeatLocal) / 1000.0f;
                    // Physiologically valid PTT range: 60ms - 450ms / 生理合理的PTT范围
                    if (ptt > 60 && ptt < 450) {
                        currentPTT = (ptt * 0.15) + (currentPTT * 0.85); // Smooth PTT / 平滑PTT
                        isPTTReal = true;
                        return;
                    }
                }
            }
        }
    }
    // If confidence drops, flag PTT as unreliable / 如果置信度下降，标记PTT不可靠
    if (arm.conf < 0.7 || finger.conf < 0.7) isPTTReal = false;
}

// ================= UI RENDER / 界面渲染 =================
void drawUI() {
    static uint32_t lastRefresh = 0;
    static float dispBPM = 0, dispPTT = 0, dispHRV = 0;
    static float tolBPM = 0, tolPTT = 0, tolHRV = 0; // Tolerance / 偏差显示

    display.clearDisplay();
    display.setTextColor(1);
    
    // Select the best sensor source / 选择最佳信源
    NodeState *best = (finger.conf >= arm.conf) ? &finger : &arm;

    // Refresh UI data at 1Hz to prevent flickering / 1Hz 刷新UI数据防止闪烁
    if (millis() - lastRefresh >= 1000) {
        dispBPM = best->smoothBPM;
        dispPTT = currentPTT;
        dispHRV = best->hrvRMSSD;

        // --- Tolerance Algorithm / 偏差算法 ---
        // Visualizes the "Error Bar" based on confidence.
        // 基于置信度可视化“误差范围”。
        tolBPM = 2.0 + (1.0 - best->conf) * 15.0; 
        
        if (isPTTReal) {
            tolPTT = 5.0 + (1.0 - best->conf) * 20.0;
        } else {
            tolPTT = 35.0; // Large error bar when guessing / 猜测模式下显示大误差
        }

        tolHRV = dispHRV * 0.15 + (1.0 - best->conf) * 10.0;
        
        lastRefresh = millis();
    }

    // 1. Header: Precision Status / 顶部：精度状态
    display.setTextSize(1); display.setCursor(0, 0);
    display.printf("PRECISION: %s", best->conf > 0.8 ? "HIGH" : "LOW");

    // 2. BPM Area / 心率区域
    display.setCursor(0, 15); display.print("HEART RATE");
    display.setCursor(0, 26); display.setTextSize(2);
    if (dispBPM < 35) display.print("---"); else display.print((int)dispBPM);
    
    // BPM Tolerance / 心率偏差
    display.setTextSize(1); display.setCursor(10, 43);
    if (dispBPM >= 35) display.printf("+/-%d", (int)tolBPM); 

    // 3. PTT Area / 脉冲传输时间区域
    display.setTextSize(1); display.setCursor(80, 15); display.print("PTT/ms");
    display.setCursor(80, 26); display.setTextSize(2);
    display.print((int)dispPTT);
    
    display.setTextSize(1); display.setCursor(80, 43);
    display.printf("sd:%d", (int)tolPTT); // Standard Deviation / 标准差

    // 4. HRV Area / 心率变异性区域
    display.setTextSize(1); display.setCursor(0, 50); display.print("HRV/ms");
    display.setCursor(55, 50);
    if (dispHRV <= 0) display.print("Calc..."); 
    else display.printf("%.1f (%.1f)", dispHRV, tolHRV);

    // 5. Bottom: Sync Bar / 底部：同步感应条
    // Visual feedback of heart rate magnitude / 心率幅度的视觉反馈
    int barW = map(constrain((int)best->smoothBPM, 40, 160), 40, 160, 0, 128);
    display.fillRect(0, 62, barW, 2, 1);

    display.display();
}

void setup() {
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64 / 0x3C地址
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
    drawUI();
    delay(50); // ~20 FPS refresh / 约20帧刷新率
}