# IMU function analysis - I2C ports racing

This file deals with doubts about where the two I2C ports will interfere with each other.

For reference, when both IMU are used:

![image](asset/disable_ble.png)

WHen disabling BNO085:

![image](asset/disable_bno085.png)

When disabling BNO055:

![image](asset/disable_bno055.png)

Constant value still persists. So this doesn't seem to be an issue.