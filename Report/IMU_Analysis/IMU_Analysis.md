# IMU Function Analysis: BNO055 vs. BNO085 Performance Evaluation (Updated)

## 1. Overview
The newer generation IMU, **BNO085**, was expected to outperform the legacy **BNO055**. However, during initial experiments, its data output exhibited periodic data freezes and jagged curves. Following a series of systematic evaluations, we have successfully identified and resolved these bottlenecks.

---

## 2. Potential Issues & Experimental Results

### 2.1 BLE Buffer Overflow & Transmission Congestion
* **Status:** **Resolved.**  
* **Details:** Implemented an **STM32-side timestamping mechanism** and downsampled telemetry to **25 Hz**. This allows chronological reconstruction on the PC side while preventing application-level buffer bursts.

### 2.2 I2C Bandwidth & Interface Contention
* **Status:** **Dismissed.**  
* **Details:** STM32 reads data significantly faster than the IMU's native output rate; bandwidth is not the bottleneck.

### 2.3 I2C Port Racing (Hardware Interference)
* **Status:** **Dismissed.**  
* **Details:** Constant value segments persisted even when only a single IMU was active, ruling out bus contention.

### 2.4 Sensor Fusion Modes
* **Status:** **Inconclusive.**  
* **Details:** Both "Rotation Vector" and "Game Rotation Vector" modes produced identical data curves and suffered from the same freezing issue under the old library.

### 2.5 Library Optimization & Implementation (Root Cause Identified)
* **Status:** **Resolved.**  **Details:** The third-party BNO08x library was identified as the root cause of the data freezes and jagged artifacts. By replacing it with the **Official BNO085 SH2 Library**, we achieved a continuous, high-fidelity data stream without interruptions.

### 2.6 Comparative Performance: BNO085 vs. BNO055
* **Status:** **Validated.**
* **Observation:** Overall, the data demonstrates that while both sensors follow the same physical trend, the **BNO085 (phi)** exhibits a significantly higher dynamic range and sensitivity, capturing motion peaks that the BNO055 tends to attenuate due to internal over-filtering. 
* **Findings:** Furthermore, the angular velocity signal (dphi) from the BNO085 is remarkably smoother and cleaner than that of the BNO055, which suffers from severe noise spikes and requires aggressive filtering to be usable.

---

## 3. Conclusion & Final Assessment
The investigation into the BNO085’s performance degradation is now complete. The switch to the official SH-2 library has unlocked the sensor's full potential, providing a more "physics-accurate" representation of the system's dynamics.

**Key Takeaways:**
1. **Accuracy:** BNO085 is more sensitive to rapid motion, avoiding the "peak-shaving" effect seen in BNO055.
2. **Signal Quality:** The internal fusion engine of BNO085 delivers a much cleaner angular velocity signal, essential for precision closed-loop control.
3. **Primary Sensor:** BNO085 is confirmed as the primary feedback source, while BNO055 remains as a secondary reference.

**Final Status:** High-bandwidth, low-noise feedback achieved. Ready for advanced control algorithm integration.