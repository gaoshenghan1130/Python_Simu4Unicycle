# IMU Analysis - I2C bandwidth

It is also possible that the I2C bandwidth is not sufficient for the two IMUs to work together, which may cause a periodical data freeze. To check whether this is the issue, I will disable the safety limit of the I2C max packet reading each loop, and record the packets accumulated over time.


It turns out that this is not the issue, as STM32 seems to be able to read much faster than the IMU actual update rate.
