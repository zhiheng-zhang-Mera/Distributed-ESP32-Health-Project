/**
 * Role: Edge Computing Node (Ring OR Belt) - FULL INTEGRATED VERSION
 * 角色：边缘计算节点（指环 或 腰带） - 全功能集成版
 * * Combined Features / 集成功能:
 * 1. Peer Backup (兄弟节点备份): Get BPM from peer if local sensor fails.
 * 2. AGC Lock (自动增益锁定): Prevents brightness flicker during heartbeats.
 * 3. Motion Sense (运动检测): Reduces confidence when moving (MPU6050).
 * 4. AC Amplitude (交流分量振幅): Evaluates signal strength.
 * 5. Kalman Filter (卡尔曼滤波): Smooths BPM data scientifically.
 * * [VERSION NOTE / 版本说明]: 
 * Compared to old versions, this adds 'SimpleKalmanFilter' class and 'adjustBrightness' with locking logic.
 * 相比旧版，新增了卡尔曼滤波类和带锁定逻辑的亮度调节函数。
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <esp_now.h>
#include <WiFi.h>

// ================= CONFIG / 配置 =================
#define NODE_ID 1          // 1=Arm(Belt/腰带), 2=Finger(Ring/指环)
#define USE_KALMAN_FILTER  true 
#define DEBUG_MODE         true

#define PIN_SDA D4
#define PIN_SCL D5

// AGC Thresholds / 自动增益控制阈值
// Keeps IR signal within optimal range for the ADC.
const long TARGET_IR_MIN = 70000;
const long TARGET_IR_MAX = 150000;
byte currentPulseAmplitude = 0x3F; // Initial brightness / 初始亮度

// MAC Address of the Wrist Host / 腕部主机的MAC地址
uint8_t masterAddr[] = {0x38, 0x18, 0x2B, 0x99, 0x5C, 0xA4}; 

// ================= DATA STRUCTURE / 数据结构 =================
// Standard message structure for ESP-NOW / ESP-NOW 通讯结构体
typedef struct struct_message {
    int id;                // Node ID
    float bpm;             // Heart Rate
    uint32_t processingDelayUs; // Time delay from detection to transmission / 处理延迟(微秒)
    float confidence;      // Signal Quality: 0=None, 0.3=Warming, 0.5=Peer, 1.0=Healthy
    bool contact;          // Is sensor touching skin? / 是否接触皮肤
} struct_message;

struct_message myData;
struct_message peerData; 
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

// ================= KALMAN FILTER / 卡尔曼滤波器 =================
/**
 * Simple 1D Kalman Filter to smooth BPM values.
 * 简单的单维卡尔曼滤波器，用于平滑BPM数值。
 * * [NEW FEATURE / 新功能]: Not present in _old versions. Replaces simple averaging.
 * 旧版仅使用平均值，此版本引入卡尔曼滤波以应对噪声。
 */
class SimpleKalmanFilter {
public:
  /**
   * @param mea_e Measurement Uncertainty / 测量不确定度
   * @param est_e Estimation Uncertainty / 估计不确定度
   * @param q     Process Noise / 过程噪声 (Smaller = smoother but slower / 越小越平滑但响应变慢)
   */
  SimpleKalmanFilter(float mea_e, float est_e, float q) { _err_measure=mea_e; _err_estimate=est_e; _q=q; }
  
  float updateEstimate(float mea) {
    float _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    float _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
    _last_estimate=_current_estimate;
    return _current_estimate;
  }
private:
  float _err_measure; float _err_estimate; float _q;
  float _last_estimate=0;
};

// Initialized with Q=0.15 (Balanced smoothing) / 初始化参数 Q=0.15 (平衡平滑度)
SimpleKalmanFilter kf(2.0, 2.0, 0.15); 

// ================= HELPER FUNCTIONS / 辅助函数 =================

/**
 * Smart AGC with Locking Logic.
 * 带锁定逻辑的智能自动增益控制。
 * * @param currentIR Current IR value / 当前红外值
 * @param lock If true, prohibits adjustment (e.g., during a heartbeat) / 如果为真，禁止调节（例如在心跳过程中）
 * * [IMPROVEMENT / 改进]: Old version adjusted constantly, causing waveform distortion.
 * 旧版持续调节会导致波形失真，新版在心跳发生时“锁定”亮度。
 */
void adjustBrightness(long currentIR, bool lock) {
    if (lock) return; // Do not change brightness during a pulse event / 脉冲期间不调光
    static uint32_t lastAdjustTime = 0;
    if (millis() - lastAdjustTime < 1000) return; // Limit adjustment frequency / 限制调节频率
    
    bool changed = false;
    if (currentIR < TARGET_IR_MIN && currentPulseAmplitude < 0xFF) {
        currentPulseAmplitude += 5; changed = true; // Too dark, increase LED / 太暗，增加亮度
    } else if (currentIR > TARGET_IR_MAX && currentPulseAmplitude > 0x02) {
        currentPulseAmplitude -= 5; changed = true; // Too bright, decrease LED / 太亮，降低亮度
    }
    if (changed) {
        particleSensor.setPulseAmplitudeIR(currentPulseAmplitude);
        lastAdjustTime = millis();
    }
}

/**
 * Detect motion using MPU6050.
 * 使用 MPU6050 检测运动。
 * * @return true if significant motion is detected / 如果检测到明显运动返回真
 * * [NEW FEATURE / 新功能]: Used to lower confidence score when user is moving.
 * 用于在用户运动时降低置信度分数。
 */
bool isMotionDetected() {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    // Calculate vector sum of acceleration and subtract gravity (9.8) / 计算加速度矢量和并减去重力
    float diff = abs(sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z)) - 9.8);
    return (diff > 1.5); // Threshold 1.5 m/s^2 / 阈值 1.5
}

// AC Amplitude Calculation / 交流分量振幅计算
// Used to determine signal strength / 用于判断信号强度
#define AMP_WINDOW 50
long irBuffer[AMP_WINDOW];
int irBufIdx = 0;
long calculateAmplitude(long newIR) {
    irBuffer[irBufIdx] = newIR;
    irBufIdx = (irBufIdx + 1) % AMP_WINDOW;
    long minVal = 999999, maxVal = 0;
    for(int i=0; i<AMP_WINDOW; i++) {
        if(irBuffer[i] == 0) continue;
        if(irBuffer[i] < minVal) minVal = irBuffer[i];
        if(irBuffer[i] > maxVal) maxVal = irBuffer[i];
    }
    return (maxVal - minVal);
}

// ESP-NOW Receive Callback / ESP-NOW 接收回调
// Receives data from the 'Brother' node (e.g., Ring receives from Belt) / 接收“兄弟”节点的数据
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message in;
    memcpy(&in, incomingData, sizeof(in));
    if (in.id != NODE_ID) memcpy(&peerData, &in, sizeof(in));
}

// ================= SETUP / 初始化 =================
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);

    // Initialize Pulse Sensor / 初始化脉搏传感器
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        particleSensor.setup(currentPulseAmplitude, 1, 2, 100, 411, 4096);
    }
    mpu.begin(); // Initialize IMU / 初始化惯性单元

    // Setup WiFi & ESP-NOW / 设置 WiFi 和 ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, masterAddr, 6);
        esp_now_add_peer(&peerInfo);
    }
}

// ================= LOOP / 主循环 =================
void loop() {
    static uint32_t lastBeatTime = 0;
    static float lastGoodBPM = 72.0;
    static float historyFilteredBPM = 0;
    static long signalAmplitude = 0;

    long irValue = particleSensor.getIR();
    myData.contact = (irValue > 30000); // Simple contact check / 简单的佩戴检测

    if (myData.contact) {
        signalAmplitude = calculateAmplitude(irValue);
        
        // Lock AGC if a beat happened recently (3 sec window seems long, maybe typing error? Standard is usually during the pulse)
        // [Logic Note]: Here it locks AGC for 3s after a beat? This prevents AGC hunting between beats.
        // 锁定AGC逻辑：如果在最近3秒内有过心跳，则锁定AGC。防止心跳间隙调整增益。
        bool lockAGC = (micros() - lastBeatTime < 3000000); 
        adjustBrightness(irValue, lockAGC);

        if (checkForBeat(irValue)) { // External library function / 外部库函数检测心跳
            uint32_t now = micros();
            uint32_t delta = now - lastBeatTime;
            lastBeatTime = now;

            if (delta > 3000000) { // Cold start or long gap (>3s) / 冷启动或长间隙
                myData.confidence = 0.3;
                historyFilteredBPM = lastGoodBPM;
            } else if (delta > 250000) { // Normal Range ( < 240 BPM ) / 正常范围
                float instantBPM = 60000000.0 / delta;
                
                if (!isMotionDetected()) {
                    lastGoodBPM = instantBPM;
                    // Apply Kalman Filter / 应用卡尔曼滤波
                    historyFilteredBPM = kf.updateEstimate(instantBPM);
                    myData.confidence = 1.0;
                } else {
                    myData.confidence = 0.7; // Motion detected, lower confidence / 检测到运动，降低置信度
                }
            }
        }
        // Timeout check / 超时检测
        if (micros() - lastBeatTime > 5000000) { // > 5 seconds no beat / 超过5秒无心跳
            myData.confidence = 0.2;
            historyFilteredBPM *= 0.98; // Slowly decay BPM / BPM缓慢衰减
        }
    } else {
        // Core Compensation: Use Peer Data / 核心补偿：使用兄弟节点数据
        // If I am not touching skin, but my brother node is healthy, use his data.
        if (peerData.contact && peerData.confidence > 0.6) {
            historyFilteredBPM = peerData.bpm;
            myData.confidence = 0.5; // Confidence 0.5 indicates "Borrowed Data" / 0.5代表“借用数据”
        } else {
            historyFilteredBPM = 0;
            myData.confidence = 0.0;
        }
    }

    // Timer for Transmission (10Hz) / 发送定时器 (10Hz)
    static uint32_t lastTx = 0;
    if (millis() - lastTx > 100) {
        myData.id = NODE_ID;
        myData.bpm = historyFilteredBPM;
        // Calculate delay for TDoA (Time Difference of Arrival) / 计算处理延迟用于TDoA
        myData.processingDelayUs = (lastBeatTime > 0) ? (micros() - lastBeatTime) : 0;
        esp_now_send(masterAddr, (uint8_t *) &myData, sizeof(myData));
        lastTx = millis();
    }

    // Debug Output / 调试输出
    if (DEBUG_MODE) {
        static uint32_t lastDebugPrint = 0;
        if(millis() - lastDebugPrint >= 150) {
            lastDebugPrint = millis();
            Serial.printf("IR:%ld,BPM:%.1f,Amp:%ld,Conf:%.1f,Delay:%ld,Peer:%.1f\n", 
                      irValue, historyFilteredBPM, signalAmplitude, myData.confidence, 
                      myData.processingDelayUs, peerData.bpm);
        }
    }
}