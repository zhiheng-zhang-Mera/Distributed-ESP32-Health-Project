# Distributed WBAN Biometric System: Edge AI & Sensor Fusion
# 基于边缘智能与传感器融合的分布式无线体域网系统

## Abstract / 摘要

This project presents a **low-cost, distributed Wireless Body Area Network (WBAN)** designed to democratize advanced biometric monitoring. Instead of relying on expensive, medical-grade hardware, this research explores the potential of **Edge AI algorithms** to extract reliable physiological metrics (BPM, HRV, PTT-based Blood Pressure trends) from cost-effective consumer sensors (MAX30102).

By leveraging a multi-node architecture (Ring, Belt, Wrist) and **sensor fusion techniques**, the system compensates for the inherent hardware limitations and motion artifacts, achieving robust continuous monitoring. This work serves as a proof-of-concept for **software-defined reliability** in resource-constrained IoT health applications.

本项目提出了一种**低成本、分布式的无线体域网 (WBAN)**，旨在普及高级生物体征监测。本研究不依赖昂贵的医疗级硬件，而是探索利用 **边缘AI算法 (Edge AI)** 从低成本消费级传感器 (MAX30102) 中提取可靠生理指标（BPM, HRV, 基于 PTT 的血压趋势）的潜力。

通过利用多节点架构（指环、腰带、腕部）和 **传感器融合技术**，系统成功补偿了固有的硬件局限性和运动伪影，实现了鲁棒的连续监测。本项目作为概念验证，展示了在资源受限的 IoT 健康应用中实现 **软件定义可靠性** 的可能性。

---

## System Architecture / 系统架构

The system prioritizes **decentralized processing** to reduce latency and bandwidth.
系统优先考虑**去中心化处理**，以降低延迟和带宽占用。

| Role / 角色 | Hardware / 硬件 | Edge Intelligence / 边缘智能 | Source Code |
| --- | --- | --- | --- |
| **Node A (Finger)** | ESP32-S3 + MAX30102 + MPU6050 | Kalman Filtering, Motion Detection, Smart AGC | `src/s3_node.cpp` |
| **Node B (Arm)** | ESP32-S3 + MAX30102 + MPU6050 | Redundant Sensing, Peer Backup Logic | `src/s3_node.cpp` |
| **Host (Wrist)** | ESP32 + OLED Display | TDoA Calculation, Adaptive Signal Correction, UI | `src/wrist_host.cpp` |

---

## Key Research Contributions / 核心研究贡献

### 1. Algorithm-Driven Cost Reduction (算法驱动的成本降低)

* **Philosophy:** High precision usually comes with high cost. This project challenges this trade-off by using **algorithmic compensation** to enhance the performance of generic sensors ($5 MAX30102) to approach the utility of premium devices.
* **Implementation:** A 1D **Kalman Filter** () runs locally on edge nodes to model and predict heart rate trajectories, effectively filtering out the high noise floor inherent to cheap optical sensors.
* **理念**：高精度通常伴随着高成本。本项目通过**算法补偿**挑战这一权衡，通过软件提升通用传感器（$5 的 MAX30102）的性能，使其逼近高端设备的效用。
* **实现**：在边缘节点本地运行 1D **卡尔曼滤波器** () 来建模和预测心率轨迹，有效滤除廉价光学传感器固有的高底噪。

### 2. Distributed Sensor Fusion & Resilience (分布式传感器融合与韧性)

* **Mechanism:** The system implements a **Peer-to-Peer Backup Protocol**. If the Ring node detects poor signal quality (due to skin contact loss or texture issues), it seamlessly switches to the Arm node's data stream.
* **Benefit:** This ensures **Zero-Downtime Monitoring**, a critical feature for wearable health devices that single-point sensors cannot achieve.
* **机制**：系统实现了**点对点备份协议**。如果指环节点检测到信号质量差（由于接触不良或皮肤纹理问题），它会无缝切换到手臂节点的数据流。
* **优势**：这确保了**零停机监测**，这是单点传感器无法实现的，对可穿戴健康设备至关重要。

### 3. Edge-Based Motion Artifact Rejection (基于边缘的运动伪影抑制)

* **Logic:** Utilizing the MPU6050 accelerometer, the system calculates the dynamic acceleration vector. When , the `confidence` score of the biometric data is actively penalized.
* **Outcome:** Prevents motion-induced noise from being misinterpreted as arrhythmia or hypertensive events.
* **逻辑**：利用 MPU6050 加速度计，系统计算动态加速度矢量。当  时，主动惩罚生物识别数据的 `置信度` 分数。
* **结果**：防止运动产生的噪声被误读为心律失常或高血压事件。

---

## Critical Analysis & Constraints / 批判性分析与局限性

### 1. Sensor Accuracy vs. Physiology (传感器精度与生理学)

* **Constraint:** The MAX30102 is a consumer-grade sensor. Its sensitivity is physically limited by **skin heterogeneity** (e.g., tattoos, scars, heavy epidermal texture), resulting in an intrinsic accuracy of **~70%** compared to benchmarks from smart heart rate band data.
* **Mitigation:** The project treats individual sensor readings as "votes" rather than absolute truths. The central host aggregates these votes using a **confidence-weighted averaging algorithm** to synthesize a more reliable estimation.
* **限制**：MAX30102 是消费级传感器。其灵敏度受到**皮肤异质性**（如纹身、伤疤、厚表皮纹理）的物理限制，导致其与心率手环数据基准相比的固有准确率仅为 **~70%**。
* **缓解**：本项目将单个传感器的读数视为“投票”而非绝对真理。中央主机使用**置信度加权平均算法**聚合这些投票，以合成更可靠的估计值。

### 2. Biological Limits of PTT/HRV (PTT/HRV 的生物学极限)

* **Constraint:** Pulse Transit Time (PTT) and HRV are highly sensitive to hemodynamic changes caused by limb movement (hydrostatic pressure shifts). During active motion, these metrics **deviate significantly from the physiological safety range**, rendering them medically unusable.
* **Safety Lock:** The system implements a strict **Motion Lock**. When the IMU detects movement, PTT and HRV calculations are frozen to prevent false alarms.
* **限制**：脉冲传输时间 (PTT) 和 HRV 对肢体运动引起的血流动力学变化（流体静压偏移）高度敏感。在主动运动期间，这些指标会**显著偏离生理安全范围**，导致医学上不可用。
* **安全锁**：系统实施了严格的**运动锁定**。当 IMU 检测到运动时，冻结 PTT 和 HRV 计算以防止误报。

### 3. Power Consumption & Optimization (能耗与优化)

* **Current Status:** As an algorithmic proof-of-concept, this iteration **does not implement power management strategies**. The ESP32 radios remain active (Rx/Tx) to maintain low-latency TDoA synchronization, resulting in high power draw (~100mA+).
* **Future Work:** Future research will focus on energy-efficient protocols, such as implementing **Wake-up Radios (WuR)** or **TDMA-based duty cycling** to extend battery life without compromising synchronization accuracy.
* **现状**：作为算法概念验证，当前迭代**未实施电源管理策略**。ESP32 射频保持开启 (Rx/Tx) 以维持低延迟 TDoA 同步，导致高功耗 (~100mA+)。
* **未来工作**：未来的研究将集中在节能协议上，例如实施**唤醒无线电 (WuR)** 或 **基于 TDMA 的占空比循环**，在不牺牲同步精度的前提下延长电池寿命。

---
