# IMU function analysis between BNO055 and BNO085

## Brief

For the IMU performance, the newer IMY BNO085 is expected to have better performance. However, during the experiments, the data output is sometimes worse. Here I will give some analysis to get the exact reason for this phenomenon with the instructions from Mate.

## Potential issues

1. BLE buffer overflow: One reason might be the fast logging data of gamma and dphi. For this I I will disable BLE and only log through SWV. ([BLE buffer overflow analysis](BLE_buffer.md))

2. I2C ports racing: Another reason might be the two IMUs are using different I2C ports interfering with each other. For this I will disable one IMU at a time and check the data output. ([I2C ports racing analysis](I2C_ports_racing.md))

3. Another factor is that the frequency configuration when initializing the chip impact significantly on the data output, as the previous email noted. For this I can try to tune it so both of them looks better, however they looks both worse than BNO055, so I don't have much hope.

4. Finally, while investigating these issues, I have also begun doubting the library I'm using. Currently I'm using the BNO085 library from [here](https://www.grozeaion.com/electronics/stm32/stm32-i2c-library-for-bno08x-9-axis-imu/). The way he coded it isn't very professional compared to other libraries I used before, such as the one for Can bus, so may be we can try another library, but it will take some extra time. 