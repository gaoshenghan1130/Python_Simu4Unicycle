# Estimated Static and Dynamic Roll/Pitch Error of Representative IMUs

**Updated:** August 4, 2026

This table compares the **roll/pitch angle error produced by each IMU/AHRS module itself**. It does not consider the controller, mechanical system, or any later implementation.

The values marked **Official** are quoted directly from the manufacturer. Values marked **Estimated** are engineering estimates based on published experiments or a closely comparable product. Different manufacturers use different statistical definitions, so RMS, 1\(\sigma\), and nominal values are retained explicitly instead of being treated as identical.

## Static and Dynamic Angle-Error Table

| IMU / AHRS | Estimated static roll/pitch error | Estimated dynamic roll/pitch error | Why this numerical estimate is reasonable | Source |
|---|---:|---:|---|---|
| **BNO085 – Game Rotation Vector** | **\(1.5^\circ\) nominal** (Official) | **\(2.5^\circ\) nominal** (Official) | CEVA gives a \(1.5^\circ\) static and \(2.5^\circ\) dynamic **non-heading error** for Game Rotation Vector. Non-heading error is the closest published BNO085 metric for roll/pitch. The values were generated from models of 210 characterized physical devices subjected to simulated motion. | [CEVA BNO08X datasheet, Performance Characteristics](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf) |
| **BNO085 – standard Rotation Vector** | **\(2.0^\circ\) nominal** (Official) | **\(3.5^\circ\) nominal** (Official) | CEVA directly lists these two Rotation Vector errors. This output also uses the magnetometer, so the full rotation error includes heading/environmental effects and is larger than the Game Rotation Vector non-heading error. CEVA additionally states that practical Rotation Vector accuracy is typically about \(5^\circ\) when environmental effects are included. | [CEVA BNO08X datasheet, Performance Characteristics](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf) |
| **BNO055** | **\(1.0^\circ\)** (Estimated) | **\(2.0^\circ\)** (Estimated) | Bosch does not publish fused roll/pitch accuracy. Independent turntable tests reported pitch errors of approximately \(0.53^\circ\)–\(0.86^\circ\) and roll errors of \(0.44^\circ\)–\(1.41^\circ\). Another experimental system observed less than \(1^\circ\) under static conditions and less than \(2^\circ\) dynamically. Therefore, \(1.0^\circ\) static and \(2.0^\circ\) dynamic are reasonable rounded engineering estimates. | [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf), [IEEE experimental comparison](https://ieeexplore.ieee.org/document/8119388/), [Sensors experimental study](https://www.mdpi.com/1424-8220/21/6/2068) |
| **Xsens MTi-320** | **\(0.5^\circ\)** (Official) | **\(1.0^\circ\)** (Estimated) | Xsens lists \(0.5^\circ\) roll/pitch accuracy but does not publish a separate dynamic number in the MTi-320 leaflet. A factor of two is used because the industrial VectorNav VN-100 has the same \(0.5^\circ\) static accuracy and an official \(1.0^\circ\) dynamic accuracy. The estimate is also conservative relative to the better MTi-630R. | [Xsens MTi-320 leaflet](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-320.pdf), [VectorNav VN-100 specifications](https://www.vectornav.com/products/detail/vn-100) |
| **VectorNav VN-100** | **\(0.5^\circ\) RMS** (Official) | **\(1.0^\circ\) RMS** (Official) | Both numbers are published directly by VectorNav. The dynamic value is exactly twice the static value. | [VectorNav VN-100 specifications](https://www.vectornav.com/products/detail/vn-100) |
| **Xsens MTi-630R** | **\(0.20^\circ\) RMS** (Official) | **\(0.25^\circ\) RMS** (Official) | Xsens publishes separate values: \(0.20^\circ\) static RMS and \(0.25^\circ\) dynamic RMS for a car moving at \(25\,\mathrm{m/s}\). No additional estimation is required. | [Xsens MTi-600-series specifications](https://mtidocs.movella.com/sensor-specifications-2), [MTi-630R leaflet](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630R.pdf) |
| **SBG Ellipse-A** | **\(0.10^\circ\), 1\(\sigma\)** (Official) | **\(0.40^\circ\), 1\(\sigma\)** (Official) | SBG publishes both values directly. They are specified as 1\(\sigma\) errors over the full temperature range; the dynamic value was evaluated under the manufacturer's representative marine dynamics. | [SBG Ellipse-A datasheet](https://www.sbg-systems.com/wp-content/uploads/SBG-Ellipse-A-MK067EN.pdf) |
| **VectorNav VN-110** | **\(0.05^\circ\) RMS** (Official) | **\(1.0^\circ\) RMS** (Official) | VectorNav directly publishes \(0.05^\circ\) static and \(1.0^\circ\) dynamic pitch/roll accuracy. The manufacturer notes that the dynamic value is typical and that velocity aiding is required under sustained linear acceleration. | [VectorNav VN-110 specifications](https://www.vectornav.com/products/detail/vn-110) |
| **SBG Ekinox-A** | **\(0.02^\circ\)** (Estimated from official system specification) | **\(0.02^\circ\)** (Official system value) | SBG lists one \(0.02^\circ\) roll/pitch accuracy value and describes Ekinox-A as an MRU that provides accurate roll/pitch under dynamic conditions. Because the published value already targets dynamic operation and no separate static value is given, using \(0.02^\circ\) for both is a conservative working estimate for the device specification. | [SBG Ekinox-A product page](https://www.sbg-systems.com/ahrs-mru/ekinox-a/) |

## Recommended Numbers for a Direct Comparison

If each device must be represented by exactly one static and one dynamic angle-error number, use:

| IMU / AHRS | Static error | Dynamic error |
|---|---:|---:|
| BNO085 – Game Rotation Vector | \(1.5^\circ\) | \(2.5^\circ\) |
| BNO085 – standard Rotation Vector | \(2.0^\circ\) | \(3.5^\circ\) |
| BNO055 | \(1.0^\circ\) | \(2.0^\circ\) |
| Xsens MTi-320 | \(0.5^\circ\) | \(1.0^\circ\) |
| VectorNav VN-100 | \(0.5^\circ\) RMS | \(1.0^\circ\) RMS |
| Xsens MTi-630R | \(0.20^\circ\) RMS | \(0.25^\circ\) RMS |
| SBG Ellipse-A | \(0.10^\circ\), 1\(\sigma\) | \(0.40^\circ\), 1\(\sigma\) |
| VectorNav VN-110 | \(0.05^\circ\) RMS | \(1.0^\circ\) RMS |
| SBG Ekinox-A | \(0.02^\circ\) | \(0.02^\circ\) |

## Important Interpretation Notes

### BNO085: use \(1.5^\circ\) static and \(2.5^\circ\) dynamic for roll/pitch

For a comparison focused on lean angle rather than heading, the most relevant official BNO085 numbers are the Game Rotation Vector non-heading errors:

\[
\boxed{
e_{\mathrm{static}}\approx1.5^\circ,
\qquad
e_{\mathrm{dynamic}}\approx2.5^\circ.
}
\]

The standard Rotation Vector values, \(2.0^\circ\) static and \(3.5^\circ\) dynamic, include the full magnetometer-referenced orientation and are therefore less specific to roll/pitch.

The BNO08X datasheet also lists gyroscope nominal dynamic accuracy = \(3.1^\circ/\mathrm{s}\). This is an angular-rate error, not an angle error. 

### Statistical definitions are not identical

- **RMS** is a root-mean-square error.
- **1\(\sigma\)** is one standard deviation.
- **Nominal error** is the manufacturer's representative performance value, but it is not automatically a 1\(\sigma\) or 3\(\sigma\) limit, but we can assume that the nominal error is approximately a 3\(\sigma\) value, optimistically.

## Compact Conclusion

For roll/pitch angle accuracy, the representative static/dynamic pairs are approximately:

\[
\begin{array}{c|c|c}
\text{Class} & \text{Static error} & \text{Dynamic error}\\
\hline
\text{Consumer BNO-class} & 1.0^\circ\text{–}2.0^\circ & 2.0^\circ\text{–}3.5^\circ\\
\text{Industrial AHRS} & 0.1^\circ\text{–}0.5^\circ & 0.25^\circ\text{–}1.0^\circ\\
\text{High-end AHRS/MRU} & 0.02^\circ\text{–}0.05^\circ & 0.02^\circ\text{–}1.0^\circ
\end{array}
\]

