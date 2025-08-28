/**
 * @file bmi088.c
 * @author ï�� ��26�����������ة��
 * @brief C��BMI088��ȡ ���ٶȡ��¶ȡ����ٶ�
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "bmi088.h"
#include "stm32f4xx_hal_flash.h"


/**
 *  ���������Ǳ궨����ı�־λ
 *  ���ڴ�ľ��Ե�ַ 0x10000000 ������һ������ʧ�Ե�У׼��־��
 * �ñ�־������ϵͳ�������ж��Ƿ���Ҫ����ִ�������ǵ���ƫУ׼���Ӷ����ⲻ��Ҫ���ظ�У׼��ʵ�ֿ���������Ӧ��״̬�ĳ־û���
 */
uint8_t gyro_zero_calib_flag __attribute__((at(0x10000000))) = 1; 

/**
 * �� �� ���� BMI088_read_write_byte()
 * ����˵���� ��BMI088����һ�ֽ�����
 * ��������� �����ֽ�
 * ���ز����� �����ֽ�
 */
uint8_t BMI088_read_write_byte(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 1000);
    //ͨ�� SPI ���������ʹӻ���ͨ�ţ�������� SPI ͨ�ųɹ����򷵻� HAL_OK
    return rx_data;
}

/**
 * �� �� ���� BMI088_ACCEL_NS_L()
 * ����˵���� BMI088оƬ���ٶ�ʹ���������ͣ���Ч��
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_ACCEL_NS_L(void)
{ 
    HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_RESET);
}

/**
 * �� �� ���� BMI088_ACCEL_NS_H()
 * ����˵���� BMI088оƬ���ٶ�ʹ���������ߣ���Ч��
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_ACCEL_NS_H(void)
{ 
    HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_SET);
}

/**
 * �� �� ���� BMI088_GYRO_NS_L()
 * ����˵���� BMI088оƬ������ʹ���������ͣ���Ч��
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_GYRO_NS_L(void)
{ 
    HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_RESET);
}

/**
 * �� �� ���� BMI088_GYRO_NS_H()
 * ����˵���� BMI088оƬ������ʹ���������ߣ���Ч��
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_GYRO_NS_H(void)
{ 
    HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_SET);
}

/**
 * �� �� ���� BMI088_accel_write_single_reg()
 * ����˵���� д��BMI088���ٶ�һ���Ĵ���
 * ��������� reg �Ĵ�����ַ�� data д������
 * ���ز����� ��
 */
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
    BMI088_ACCEL_NS_H();
}

/**
 * �� �� ���� BMI088_accel_read_single_reg()
 * ����˵���� ��ȡBMI088���ٶ�һ���Ĵ���
 * ��������� reg �Ĵ�����ַ
 * ���ز����� ��ȡ����
 */
uint8_t BMI088_accel_read_single_reg(uint8_t reg)
{
    uint8_t tmp;

    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg|0x80);
    BMI088_read_write_byte(0x55);
    tmp = BMI088_read_write_byte(0x55);
    BMI088_ACCEL_NS_H();
    
    return tmp;
}

/**
 * �� �� ���� BMI088_accel_read_multiple_reg()
 * ����˵���� ��ȡBMI088���ٶȶ���Ĵ���
 * ��������� reg ��ʼ�Ĵ�����ַ�� buf ��ȡ���ݻ������� num ��ȡ����
 * ���ز����� ��
 */
void BMI088_accel_read_multiple_reg(uint8_t reg, uint8_t *buf, uint16_t num)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg|0x80);
    BMI088_read_write_byte(0x55);
    while(num!=0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        num--;
    }
    BMI088_ACCEL_NS_H(); 
}

/**
 * �� �� ���� BMI088_gyro_write_single_reg()
 * ����˵���� д��BMI088������һ���Ĵ���
 * ��������� reg �Ĵ�����ַ�� data д������
 * ���ز����� ��
 */
void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
    BMI088_GYRO_NS_H();
}

/**
 * �� �� ���� BMI088_gyro_read_single_reg()
 * ����˵���� ��ȡBMI088������һ���Ĵ���
 * ��������� reg �Ĵ�����ַ
 * ���ز����� ��ȡ����
 */
uint8_t BMI088_gyro_read_single_reg(uint8_t reg)
{
    uint8_t tmp;

    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg|0x80);
    tmp = BMI088_read_write_byte(0x55);
    BMI088_GYRO_NS_H();

    return tmp;
}

/**
 * �� �� ���� BMI088_gyro_read_multiple_reg()
 * ����˵���� ��ȡBMI088�����Ƕ���Ĵ���
 * ��������� reg ��ʼ�Ĵ�����ַ�� buf ��ȡ���ݻ������� num ��ȡ����
 * ���ز����� ��
 */
void BMI088_gyro_read_multiple_reg(uint8_t reg, uint8_t *buf, uint16_t num)
{
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg|0x80);
    while(num!=0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        num--;
    }
    BMI088_GYRO_NS_H();
}

/**
 * ��������Ʈ����
 */
float bmi088_gyro_zero[3] __attribute__((at(0x10000060))) = {0.0f};
float bmi088_gyro_deg_zero[3] __attribute__((at(0x10000070))) = {0.0f};

/**
 * �� �� ���� BMI088_init()
 * ����˵���� ��ʼ��BMI088�Ĵ������ã����ٶ� ����+-24g Ƶ��800Hz / ������ ����+-2000dps Ƶ��1000Hz��
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_init(void)
{
    uint8_t res = 0;

    BMI088_accel_write_single_reg(0x7E, 0xB6); //������
    HAL_Delay(100);
    res = BMI088_accel_read_single_reg(0x00); HAL_Delay(150);
    res = BMI088_accel_read_single_reg(0x00); HAL_Delay(150);
    
    BMI088_accel_write_single_reg(0x40, 0xAB); HAL_Delay(150); //ACC CONF 1110 1010 800Hz,230/200Hz���ü��ٶȼ�Ƶ��Ϊ800hz
    BMI088_accel_write_single_reg(0x41, 0x03); HAL_Delay(150); //ACC RANGE ʿ24g���ü��ٶȼƲ�����Χ
    BMI088_accel_write_single_reg(0x53, 0x08); HAL_Delay(150); //INT1_IO_CONF 0000 1000 INT1 as output pp acitiv_low
    BMI088_accel_write_single_reg(0x58, 0x04); HAL_Delay(150); //INT_MAP_DATA 0000 0100 INT1 drdy interrupt
    BMI088_accel_write_single_reg(0x7C, 0x00); HAL_Delay(150); //power config-acitveģʽ
    BMI088_accel_write_single_reg(0x7D, 0x04); HAL_Delay(150); //power config-acc enable

    BMI088_gyro_write_single_reg(0x14, 0xB6); //soft_rest
    HAL_Delay(100);
    res = BMI088_gyro_read_single_reg(0x00); HAL_Delay(150);

    BMI088_gyro_write_single_reg(0X0F, 0x00); HAL_Delay(10); //GYRO_RANGE 00-2000dps; 01-1000dps; 02-500dps; 03-250dps; 04-125dps
    BMI088_gyro_write_single_reg(0X10, 0x82); HAL_Delay(10); //GYRO_ODR,BANDWIDTH 00-2000Hz,532Hz; 01-2000Hz; 02-1000Hz,116Hz; 03-400Hz,47Hz; 04-200Hz,23Hz;
    BMI088_gyro_write_single_reg(0X11, 0x00); HAL_Delay(10); //GYRO_LPM1 normal mode
    BMI088_gyro_write_single_reg(0X15, 0x80); HAL_Delay(10); //GYRO_INT_CTRL 1000 0000 enable drdy
    BMI088_gyro_write_single_reg(0X16, 0x00); HAL_Delay(10); //INT_IO_CONF 0000 0000 pp active_low
    BMI088_gyro_write_single_reg(0X18, 0x01); HAL_Delay(10); //INT_IO_MAP 0000 0001 drdy to INT3

    //read flash
    uint32_t *ptr = (uint32_t*)bmi088_gyro_zero;
    for (int i = 0; i < 3; i++) 
    {
        ptr[i] = *(__IO uint32_t*)(GYRO_FLASH_ADDRESS + i * 4);
    }

    ptr = (uint32_t*)bmi088_gyro_deg_zero;
    for (int i = 0; i < 3; i++) 
    {
        ptr[i] = *(__IO uint32_t*)(GYRO_FLASH_ADDRESS + 12 + i * 4);
    }
}

/**
 * �� �� ���� BMI088_calib_gyro_zero()
 * ����˵���� �궨BMI088���������ǵ���Ʈֵ
 * ��������� ���������ǻ���ֵ�����������ǽǶ�ֵ
 * ���ز����� ��
 */
void BMI088_calib_gyro_zero(volatile float gyro[], volatile float gyro_deg[])
{
    uint16_t n;
    float sum[3]={0.0f,0.0f,0.0f};
    float sum_deg[3]={0.0f,0.0f,0.0f};

    for(n=0; n<1000; n++)
    {
        sum[0] += gyro[0];
        sum[1] += gyro[1];
        sum[2] += gyro[2];

        sum_deg[0] += gyro_deg[0];
        sum_deg[1] += gyro_deg[1];
        sum_deg[2] += gyro_deg[2];

        HAL_Delay(20);
    }

    //������������Ʈ����
    bmi088_gyro_zero[0] = sum[0]/1000.0f;
    bmi088_gyro_zero[1] = sum[1]/1000.0f;
    bmi088_gyro_zero[2] = sum[2]/1000.0f;

    bmi088_gyro_deg_zero[0] = sum_deg[0]/1000.0f;
    bmi088_gyro_deg_zero[1] = sum_deg[1]/1000.0f;
    bmi088_gyro_deg_zero[2] = sum_deg[2]/1000.0f;

    //write flash
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_10;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) 
    {
        Error_Handler();
    }

    // д�� gyro_zero
    uint32_t *data_ptr = (uint32_t*)bmi088_gyro_zero;
    for (int i = 0; i < 3; i++) 
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, GYRO_FLASH_ADDRESS + i * 4, data_ptr[i]) != HAL_OK) 
        {
            Error_Handler();
        }
    }

    // д�� gyro_deg_zero
    data_ptr = (uint32_t*)bmi088_gyro_deg_zero;
    for (int i = 0; i < 3; i++) 
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, GYRO_FLASH_ADDRESS + 12 + i * 4, data_ptr[i]) != HAL_OK) 
        {
            Error_Handler();
        }
    }
}

//BMI088�ɼ�����
volatile float bmi088_gyro[3] __attribute__((at(0x10000010))) = {0.0f};
volatile float bmi088_gyro_deg[3] __attribute__((at(0x10000020))) = {0.0f};
float bmi088_accel[3] __attribute__((at(0x10000030))) = {0.0f};
float bmi088_accel_last[3] __attribute__((at(0x10000040))) = {0.0f};
float bmi088_temperature __attribute__((at(0x10000050))) = 0.0f;

//У׼������������
float bmi088_gyro_calib[3] __attribute__((at(0x10000080))) = {0.0f};
float bmi088_gyro_deg_calib[3] __attribute__((at(0x10000090))) = {0.0f};

/**
 * �� �� ���� BMI088_read_gyro()
 * ����˵���� ��ȡBMI088��������������
 * ��������� ��
 * ���ز����� ��
 */
void  BMI088_read_gyro(void)
{
    uint8_t buf[6];
    int16_t bmi088_raw_tmp;

    BMI088_gyro_read_multiple_reg(0x02, buf, 6);

    bmi088_raw_tmp = (int16_t)((buf[1]<<8)|buf[0]);
    bmi088_gyro[0] = bmi088_raw_tmp * 0.00106526443603169529841533860381f; //((tmp/32768)*(2000*pi/180))rad/s
    bmi088_gyro_deg[0] = bmi088_raw_tmp * 0.06103515625f; //((tmp/32768)*2000)deg/s

    bmi088_raw_tmp = (int16_t)((buf[3]<<8)|buf[2]);
    bmi088_gyro[1] = bmi088_raw_tmp * 0.00106526443603169529841533860381f; //((tmp/32768)*(2000*pi/180))rad/s
    bmi088_gyro_deg[1] = bmi088_raw_tmp * 0.06103515625f; //((tmp/32768)*2000)deg/s

    bmi088_raw_tmp = (int16_t)((buf[5]<<8)|buf[4]);
    bmi088_gyro[2] = bmi088_raw_tmp * 0.00106526443603169529841533860381f; //((tmp/32768)*(2000*pi/180))rad/s
    bmi088_gyro_deg[2] = bmi088_raw_tmp * 0.06103515625f; //((tmp/32768)*2000)deg/s

    //��ȥ�궨����Ʈֵ
    bmi088_gyro_calib[0] = bmi088_gyro[0]-bmi088_gyro_zero[0];
    bmi088_gyro_calib[1] = bmi088_gyro[1]-bmi088_gyro_zero[1];
    bmi088_gyro_calib[2] = bmi088_gyro[2]-bmi088_gyro_zero[2];

    bmi088_gyro_deg_calib[0] = bmi088_gyro_deg[0]-bmi088_gyro_deg_zero[0];
    bmi088_gyro_deg_calib[1] = bmi088_gyro_deg[1]-bmi088_gyro_deg_zero[1];
    bmi088_gyro_deg_calib[2] = bmi088_gyro_deg[2]-bmi088_gyro_deg_zero[2];
}

/**
 * �� �� ���� BMI088_read_accel()
 * ����˵���� ��ȡBMI088������ٶȼ�����
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_read_accel(void)
{
    uint8_t buf[6];
    int16_t bmi088_raw_tmp;

    BMI088_accel_read_multiple_reg(0x12, buf, 6);

    bmi088_raw_tmp = (int16_t)((buf[1]<<8)|buf[0]);
    bmi088_accel[0] = bmi088_raw_tmp * 0.00718260498046875f; //(tmp/32768)*24*9.80665m/s2
    bmi088_raw_tmp = (int16_t)((buf[3]<<8)|buf[2]);
    bmi088_accel[1] = bmi088_raw_tmp * 0.00718260498046875f; //(tmp/32768)*24*9.80665m/s2
    bmi088_raw_tmp = (int16_t)((buf[5]<<8)|buf[4]);
    bmi088_accel[2] = bmi088_raw_tmp * 0.00718260498046875f; //(tmp/32768)*24*9.80665m/s2    

    //һ�׵�ͨ�˲�
    bmi088_accel[0] = bmi088_accel[0]*0.3f + bmi088_accel_last[0]*0.7f;
    bmi088_accel[1] = bmi088_accel[1]*0.3f + bmi088_accel_last[1]*0.7f;
    bmi088_accel[2] = bmi088_accel[2]*0.3f + bmi088_accel_last[2]*0.7f;

    bmi088_accel_last[0] = bmi088_accel[0];
    bmi088_accel_last[1] = bmi088_accel[1];
    bmi088_accel_last[2] = bmi088_accel[2];
}

/**
 * �� �� ���� BMI088_read_temperature() 
 * ����˵���� ��ȡBMI088�¶�����
 * ��������� ��
 * ���ز����� ��
 */
void BMI088_read_temperature(void)
{
    uint8_t buf[2];
    int16_t bmi088_raw_tmp;

    BMI088_accel_read_multiple_reg(0x22, buf, 2);

    bmi088_raw_tmp = (int16_t)((buf[0]<<3)|buf[1]>>5);
    if(bmi088_raw_tmp > 1023) {bmi088_raw_tmp =2048;}
    bmi088_temperature = bmi088_raw_tmp * 0.125f + 23.0f;
}
