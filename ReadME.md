# BMI088 
>#### <u>�ο��ڣ� ��7��-BMI088��CANͨѶ��https://www.bilibili.com/video/BV1sK411C7qH?vd_source=70089cf19d6ed075d77314bb56aeea3c</u>

## ����˵��
1.��Ҫ����ȫ��д��`bmi088_`�ļ���`bmi088.c`֮�С����������ͺ���������`bmi088.h`�С�
2.`main.c`�ļ����ڳ�ʼ��������`BMI088_init()`��`HAL_FLASH_Unlock()`
```c
  /* USER CODE BEGIN 2 */
  BMI088_init();
  HAL_FLASH_Unlock();
  /* USER CODE END 2 */
```
�Լ���ѭ���е�``MI088_read_gyro()``�� ``BMI088_read_accel()``�� ``BMI088_read_temperature()``
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
## ����˵�� ��R�����ŷţ�����������ϵ��
��Ҫʹ�ñ������ĸ����ֱ���:`gyro_zero_calib_flag` `bmi088_temperature`��`bmi088_gyro_deg[3]`��`bmi088_acce[3]`

1.`gyro_zero_calib_flag`������������У׼�ı�־λ�������ж��Ƿ��Ѿ����й����У׼��

2.`bmi088_temperature`�������¶����ݡ����ڻ�ȡBMI088���¶����ݡ�

3.`bmi088_gyro_deg[3]`���������������ݡ����ڻ�ȡBMI088�����������ݡ�
`bmi088_gyro_deg[0]`ΪX�ᣬ`bmi088_gyro_deg[1]`ΪY�ᣬ`bmi088_gyro_deg[2]`ΪZ�ᡣ

4.`bmi088_acce[3]`��������ٶ����ݡ����ڻ�ȡBMI088�ļ��ٶ����ݡ�
`bmi088_acce[0]`ΪX�ᣬ`bmi088_acce[1]`ΪY�ᣬ`bmi088_acce[2]`ΪZ�ᡣ

# ע�⣡��
# û��Mahony��Kalman
># <u>�ο�ͼ���겻һ���ԣ�����</u>