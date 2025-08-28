# BMI088 
>#### <u>参考于： 第7课-BMI088与CAN通讯】https://www.bilibili.com/video/BV1sK411C7qH?vd_source=70089cf19d6ed075d77314bb56aeea3c</u>

## 函数说明
1.主要函数全部写在`bmi088_`文件的`bmi088.c`之中。变量声明和函数声明在`bmi088.h`中。
2.`main.c`文件中在初始化调用了`BMI088_init()`和`HAL_FLASH_Unlock()`
```c
  /* USER CODE BEGIN 2 */
  BMI088_init();
  HAL_FLASH_Unlock();
  /* USER CODE END 2 */
```
以及在循环中调``MI088_read_gyro()``、 ``BMI088_read_accel()``和 ``BMI088_read_temperature()``
```c
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BMI088_read_gyro();
    BMI088_read_accel();
    BMI088_read_temperature();
  }
```
## 变量说明 （R标正着放，东北天坐标系）
主要使用变量有四个，分别是:`gyro_zero_calib_flag` `bmi088_temperature`、`bmi088_gyro_deg[3]`和`bmi088_acce[3]`

1.`gyro_zero_calib_flag`：进入陀螺仪校准的标志位。用于判断是否已经进行过零点校准。

2.`bmi088_temperature`：保存温度数据。用于获取BMI088的温度数据。

3.`bmi088_gyro_deg[3]`：保存陀螺仪数据。用于获取BMI088的陀螺仪数据。
`bmi088_gyro_deg[0]`为X轴，`bmi088_gyro_deg[1]`为Y轴，`bmi088_gyro_deg[2]`为Z轴。

4.`bmi088_acce[3]`：保存加速度数据。用于获取BMI088的加速度数据。
`bmi088_acce[0]`为X轴，`bmi088_acce[1]`为Y轴，`bmi088_acce[2]`为Z轴。

# 注意！！
# 没加Mahony，Kalman
># <u>参考图坐标不一定对！！！</u>