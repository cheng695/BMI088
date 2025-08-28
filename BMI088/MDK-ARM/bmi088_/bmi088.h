#ifndef __BMI088_H
#define __BMI088_H

#include "spi.h"

#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080D0000) //STM32F407 扇区10初始地址
#define GYRO_FLASH_ADDRESS ADDR_FLASH_SECTOR_10 //选好gyro的选区
#define BMI088_ACCEL_CS1_GPIO_Port GPIOA
#define BMI088_ACCEL_CS1_Pin GPIO_PIN_4
#define BMI088_GYRO_CS1_GPIO_Port GPIOB
#define BMI088_GYRO_CS1_Pin GPIO_PIN_0

//变量声明
extern uint8_t gyro_zero_calib_flag;//进入陀螺仪校准的标志位
extern volatile float bmi088_gyro[3];
extern volatile float bmi088_gyro_deg[3];
extern float bmi088_accel[3];
extern float bmi088_accel_last[3];
extern float bmi088_temperature;

extern float bmi088_gyro_zero[3];
extern float bmi088_gyro_deg_zero[3];

extern float bmi088_gyro_calib[3];
extern float bmi088_gyro_deg_calib[3];

//函数声明
extern uint8_t BMI088_read_write_byte(uint8_t tx_data);
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);
extern void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t BMI088_accel_read_single_reg(uint8_t reg);
extern void BMI088_accel_read_multiple_reg(uint8_t reg, uint8_t *buf, uint16_t num);
extern void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t BMI088_gyro_read_single_reg(uint8_t reg);
extern void BMI088_gyro_read_multiple_reg(uint8_t reg, uint8_t *buf, uint16_t num);
extern void BMI088_init(void);
extern void BMI088_calib_gyro_zero(volatile float gyro[], volatile float gyro_deg[]);
extern void BMI088_read_gyro(void);
extern void BMI088_read_accel(void);
extern void BMI088_read_temperature(void);

#endif
