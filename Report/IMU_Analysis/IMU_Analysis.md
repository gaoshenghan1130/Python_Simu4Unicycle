# IMU Function Analysis: BNO055 vs. BNO085 Performance Degradation

## 1. Overview
The newer generation IMU, **BNO085**, was expected to outperform the legacy **BNO055**. However, during practical experiments, its data output occasionally exhibited worse performance (specifically, periodic data freezes and jagged curves). 

Guided by instructions from Mate, this document analyzes the root causes of this phenomenon by evaluating four potential hardware and software bottlenecks.

---

## 2. Potential Issues & Experimental Results

### 2.1 BLE Buffer Overflow & Transmission Congestion
* **Initial Hypothesis:** The high-frequency telemetry logging (Gamma, $d\phi$, etc.) at 100 Hz bursts the application-level BLE buffer, or Windows enforces a restrictive 30 ms connection interval, causing a 2/3 data loss.
* **Investigation:** Disabled BLE completely and switched to Serial Wire Viewer (SWV) logging for baseline verification. Later analysis revealed the issue was not data loss, but an **80 ms ~ 100 ms cyclic burst accumulation** on the PC host side.
* **Resolution / Status:** **Resolved.** 1. Implemented an **STM32-side timestamping mechanism** (recording the midpoint time of the IMU measurement) to allow correct chronological reconstruction on the PC.
  2. Downsampled the BLE transmission frequency to **25 Hz**, which optimized bandwidth without sacrificing visual resolution for human analysis.
  * *Detailed Report:* See [IMU analysis - BLE optimization](BLE_Optimization.md) & [BLE buffer overflow analysis](BLE_buffer.md)

### 2.2 I2C Bandwidth & Interface Contention
* **Initial Hypothesis:** The I2C bus lacks sufficient bandwidth to handle two IMUs simultaneously, leading to periodic data freezes.
* **Investigation:** Disabled the maximum packet reading safety limit in each loop to log the accumulated packets over time.
* **Resolution / Status:** **Dismissed.** The STM32 reads data significantly faster than the IMU's native output rate. I2C bandwidth is not the bottleneck.
  * *Detailed Report:* See [IMU Analysis - I2C bandwidth](I2C_bandwidth.md)

### 2.3 I2C Port Racing (Hardware Interference)
* **Initial Hypothesis:** Hardware crosstalk or bus contention occurs because the two IMUs operate on different I2C ports.
* **Investigation:** Isolated the hardware by disabling one IMU at a time (first BNO085, then BNO055) and observed the signal output.
* **Resolution / Status:** **Dismissed.** The constant value segments (data freezes) persisted identically even when only a single IMU was active.
  * *Detailed Report:* See [IMU function analysis - I2C ports racing](I2C_ports_racing.md)

### 2.4 Sensor Fusion Modes (Game Rotation Vector vs. Rotation Vector)
* **Initial Hypothesis:** Magnetometer integration in standard Rotation Vector mode might introduce processing latency or noise compared to the gyro-copter/accelerometer-only Game Rotation Vector mode.
* **Investigation:** Compared the output waveforms of both modes.
* **Resolution / Status:** **Inconclusive.** Both modes produced nearly identical data curves and suffered from the same constant value freezing issue.
  * *Detailed Report:* See [IMU analysis - Rotation Vector mode VS Game Rotation Vector mode](Rotation_Vector_mode.md)

### 2.5 Chip Initialization Frequency & Library Limitations
* **Initial Hypothesis:** 1. Clock and sampling frequency configurations specified during chip initialization heavily impact output quality. Current tweaks show both IMUs underperforming compared to the standalone legacy configuration.
  2. The third-party [STM32 I2C library for BNO08x](https://www.grozeaion.com/electronics/stm32/stm32-i2c-library-for-bno08x-9-axis-imu/) being used lacks professional optimization (e.g., compared to the CAN bus implementation).
* **Resolution / Status:** **Open / Next Steps.** The data freeze issue is likely linked to the IMU driver's register polling efficiency or internal sensor configuration. 

---

## 3. Conclusion & Next Steps
While the BLE visualization issue has been fully resolved via STM32-side timestamping and 25 Hz downsampling, the **periodic constant-value data freeze** is still present and is decoupled from BLE, I2C bandwidth, or port racing.

**Action Items:**
1. Refactor or replace the current BNO085 I2C library with a more robust, interrupt-driven, or DMA-backed official driver implementation.
2. Review the BNO085 initialization registers, specifically focusing on its internal sensor report rates and power management states.