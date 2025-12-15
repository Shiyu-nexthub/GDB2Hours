/**
 * @file    xv7001bb.c
 * @brief   XV7001BB 陀螺仪驱动实现
 * @details 实现 XV7001BB 陀螺仪的初始化、配置、数据读取等功能
 * 
 * 依赖:
 *   - spi_gyro.h/c: SPI2 底层驱动
 * 
 * 使用流程:
 *   1. 调用 XV7001_Init() 初始化
 *   2. 配置滤波器和输出格式
 *   3. 循环读取角速度数据
 */

#include "xv7001bb.h"

/* ==================== 私有函数声明 ==================== */

static void XV7001_DelayMs(uint32_t ms);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief  初始化 XV7001BB 陀螺仪
 * @note   初始化流程:
 *         1. 初始化 SPI 接口
 *         2. 配置 4 线 SPI 模式
 *         3. 配置温度输出格式 (12位)
 *         4. 配置角速度输出 (16位使能)
 *         5. 设置默认低通滤波器 (200Hz)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_Init(void)
{
    HAL_StatusTypeDef status;
    
    /* ========== 步骤1: 初始化 SPI2 接口 ========== */
    status = SPI_Gyro_Init();
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* 等待芯片稳定 */
    XV7001_DelayMs(10);
    
    /* ========== 步骤2: 配置 4 线 SPI 模式 ========== */
    status = XV7001_ConfigSPIMode();
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤3: 配置温度输出格式 (12位) ========== */
    status = XV7001_ConfigTempFormat();
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤4: 配置角速度输出 (16位 + 使能) ========== */
    status = XV7001_ConfigRateOutput(XV7001_RATE_16BIT_ENABLE);
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤5: 设置默认低通滤波器 (200Hz) ========== */
    status = XV7001_SetLPF(XV7001_LPF_200HZ);
    
    return status;
}

/**
 * @brief  软件复位 XV7001BB
 * @note   写入任意值到软件复位寄存器触发复位
 *         复位后需等待芯片重新初始化
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_SoftReset(void)
{
    HAL_StatusTypeDef status;
    
    /* 写入任意值触发软复位 */
    status = SPI_Gyro_WriteRegister(XV7001_REG_SOFT_RST, 0x01);
    
    /* 等待复位完成 (典型值约 100ms) */
    XV7001_DelayMs(100);
    
    return status;
}

/**
 * @brief  读取设备状态寄存器
 * @retval uint8_t 状态寄存器原始值
 */
uint8_t XV7001_GetStatus(void)
{
    return SPI_Gyro_ReadRegister(XV7001_REG_STATUS);
}

/**
 * @brief  检查数据是否准备好
 * @note   检查状态寄存器的 PROC_OK 位 (Bit3)
 * @retval uint8_t 1=数据准备好, 0=未准备好
 */
uint8_t XV7001_IsDataReady(void)
{
    uint8_t status = XV7001_GetStatus();
    return (status & XV7001_STATUS_PROC_OK) ? 1 : 0;
}

/**
 * @brief  获取设备工作状态码
 * @note   从状态寄存器提取低3位状态码
 * @retval uint8_t 状态码
 *         - 0: 休眠状态
 *         - 1: 正在退出休眠
 *         - 2: 待机状态
 *         - 4: 上电复位后状态
 */
uint8_t XV7001_GetState(void)
{
    uint8_t status = XV7001_GetStatus();
    return (status & XV7001_STATUS_MASK);
}

/* ==================== 电源管理函数 ==================== */

/**
 * @brief  使芯片进入休眠模式
 * @note   写入任意值到休眠进入寄存器
 *         休眠模式下功耗最低，但无法测量
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_EnterSleep(void)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_SLEEP_IN, 0x01);
}

/**
 * @brief  唤醒芯片 (退出休眠)
 * @note   写入任意值到休眠退出寄存器
 *         唤醒后需等待芯片稳定
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ExitSleep(void)
{
    HAL_StatusTypeDef status;
    
    status = SPI_Gyro_WriteRegister(XV7001_REG_SLEEP_OUT, 0x01);
    
    /* 等待唤醒完成 */
    XV7001_DelayMs(50);
    
    return status;
}

/**
 * @brief  进入待机模式
 * @note   待机模式功耗介于工作和休眠之间
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_EnterStandby(void)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_STANDBY, 0x01);
}

/* ==================== 滤波器配置函数 ==================== */

/**
 * @brief  设置低通滤波器截止频率
 * @param  lpfConfig: 滤波器配置值
 *         - XV7001_LPF_100HZ (0x06): 100Hz，噪声低，响应慢
 *         - XV7001_LPF_200HZ (0x07): 200Hz，平衡选择
 *         - XV7001_LPF_400HZ (0x08): 400Hz，响应快，噪声大
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_SetLPF(uint8_t lpfConfig)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_DSP_CTL2, lpfConfig);
}

/**
 * @brief  复位数字滤波器
 * @note   清除滤波器内部状态，用于快速响应场景
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ResetFilter(void)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_FILTER_RST, 0x01);
}

/* ==================== 数据输出配置函数 ==================== */

/**
 * @brief  配置角速度输出格式
 * @param  config: 输出配置值
 *         - XV7001_RATE_16BIT_ENABLE (0x01): 16位输出并使能
 *         - XV7001_RATE_24BIT_ENABLE (0x05): 24位输出并使能
 * @retval HAL_StatusTypeDef
 * 
 * 配置说明:
 *   寄存器 0x0B 的位定义:
 *   - Bit2: 1=24位输出, 0=16位输出
 *   - Bit0: 1=使能输出, 0=禁止输出
 */
HAL_StatusTypeDef XV7001_ConfigRateOutput(uint8_t config)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_RATE_CTL, config);
}

/**
 * @brief  配置温度输出格式
 * @note   设置为 12 位温度输出格式
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ConfigTempFormat(void)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_TEMP_FMT, XV7001_TEMP_12BIT);
}

/**
 * @brief  配置 SPI 接口模式
 * @note   设置为 4 线 SPI 模式 (标准全双工)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ConfigSPIMode(void)
{
    return SPI_Gyro_WriteRegister(XV7001_REG_IF_CTL, XV7001_SPI_4WIRE);
}

/* ==================== 数据读取函数 ==================== */

/**
 * @brief  读取角速度数据 (16位模式)
 * @note   读取 2 字节角速度数据，组合为 16 位有符号整数
 * @retval int16_t 角速度原始值
 * 
 * 数据格式:
 *   第一次读取: 高字节 [15:8]
 *   第二次读取: 低字节 [7:0]
 */
int16_t XV7001_ReadRate16(void)
{
    uint8_t highByte, lowByte;
    
    /* 选中芯片 */
    GYRO_NSS_LOW();
    
    /* 发送读取角速度寄存器命令 */
    SPI_Gyro_TransmitReceive(XV7001_SPI_READ | XV7001_REG_RATE);
    
    /* 读取高字节 */
    highByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 读取低字节 */
    lowByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 取消选中 */
    GYRO_NSS_HIGH();
    
    /* 组合为 16 位有符号整数 */
    return (int16_t)((highByte << 8) | lowByte);
}

/**
 * @brief  读取角速度数据 (24位模式)
 * @note   读取 3 字节角速度数据，符号扩展为 32 位
 * @retval int32_t 角速度原始值 (24位有符号扩展到32位)
 * 
 * 数据格式:
 *   第一次读取: 高字节 [23:16]
 *   第二次读取: 中字节 [15:8]
 *   第三次读取: 低字节 [7:0]
 */
int32_t XV7001_ReadRate24(void)
{
    uint8_t highByte, midByte, lowByte;
    int32_t result;
    
    /* 选中芯片 */
    GYRO_NSS_LOW();
    
    /* 发送读取角速度寄存器命令 */
    SPI_Gyro_TransmitReceive(XV7001_REG_RATE | 0x80);
    
    /* 读取高字节 */
    highByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 读取中字节 */
    midByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 读取低字节 */
    lowByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 取消选中 */
    GYRO_NSS_HIGH();
    
    /* 组合为 24 位值 */
    result = ((int32_t)highByte << 16) | ((int32_t)midByte << 8) | lowByte;
    
    /* 24 位有符号数扩展到 32 位 */
    /* 如果最高位 (bit23) 为 1，则为负数，需要符号扩展 */
    if (result & 0x800000)
    {
        result |= 0xFF000000;  /* 高 8 位填充 1 */
    }
    
    return result;
}

/**
 * @brief  读取温度数据
 * @note   读取 2 字节，包含 12 位温度数据
 * @retval int16_t 温度原始值 (12位)
 * 
 * 温度计算 (参考):
 *   实际温度 = (原始值 - 偏移) × 比例因子
 *   具体公式请参考 XV7001BB 数据手册
 */
int16_t XV7001_ReadTemperature(void)
{
    uint8_t highByte, lowByte;
    
    /* 选中芯片 */
    GYRO_NSS_LOW();
    
    /* 发送读取温度寄存器命令 */
    SPI_Gyro_TransmitReceive(XV7001_REG_TEMP | 0x80);
    
    /* 读取高字节 */
    highByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 读取低字节 */
    lowByte = SPI_Gyro_TransmitReceive(0x00);
    
    /* 取消选中 */
    GYRO_NSS_HIGH();
    
    /* 组合为 10 位值 */
    /* 数据格式: buf[0]=[D9:D2] 高8位, buf[1]=[D1:D0]在最高2位 */
    /* raw = (buf[0] << 2) | ((buf[1] >> 6) & 0x03) */
    return (int16_t)((highByte << 2) | ((lowByte >> 6) & 0x03));
}

/* ==================== 校准函数 ==================== */

/**
 * @brief  执行零点校准
 * @note   零点校准流程:
 *         1. 读取 DSP_CTL3 当前值
 *         2. 设置 Bit6 使能校准命令
 *         3. 写入零点校准触发命令
 *         4. 等待校准完成
 *         校准期间芯片必须保持静止!
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ZeroCalibration(void)
{
    HAL_StatusTypeDef status;
    uint8_t dspCtl3;
    
    /* ========== 步骤1: 读取 DSP_CTL3 当前值 ========== */
    dspCtl3 = SPI_Gyro_ReadRegister(XV7001_REG_DSP_CTL3);
    
    /* ========== 步骤2: 设置 Bit6 使能校准命令 ========== */
    dspCtl3 |= XV7001_CAL_ENABLE_BIT;
    status = SPI_Gyro_WriteRegister(XV7001_REG_DSP_CTL3, dspCtl3);
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤3: 写入零点校准触发命令 ========== */
    status = SPI_Gyro_WriteRegister(XV7001_REG_ZERO_CAL, XV7001_CAL_TRIGGER);
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤4: 等待校准完成 ========== */
    /* 校准时间取决于滤波器设置，典型值约 500ms */
    XV7001_DelayMs(500);
    
    /* ========== 步骤5: 清除校准使能位 (可选) ========== */
    dspCtl3 &= ~XV7001_CAL_ENABLE_BIT;
    status = SPI_Gyro_WriteRegister(XV7001_REG_DSP_CTL3, dspCtl3);
    
    return status;
}

/* ==================== 数据转换函数实现 ==================== */

/**
 * @brief  将16位原始角速度转换为实际角速度 (°/s)
 * @param  rawRate: 16位原始角速度值
 * @retval float 实际角速度 (单位: °/s)
 * 
 * 转换公式:
 *   实际角速度 = 原始值 / 280.0
 *   1 LSB = 1/280 °/s ≈ 0.00357 °/s
 */
float XV7001_ConvertRate16(int16_t rawRate)
{
    return (float)rawRate / XV7001_RATE_SCALE_16BIT;
}

/**
 * @brief  将24位原始角速度转换为实际角速度 (°/s)
 * @param  rawRate: 24位原始角速度值 (已扩展为32位)
 * @retval float 实际角速度 (单位: °/s)
 * 
 * 转换公式:
 *   实际角速度 = 原始值 / 71680.0
 *   1 LSB = 1/71680 °/s ≈ 0.0000139 °/s
 */
float XV7001_ConvertRate24(int32_t rawRate)
{
    return (float)rawRate / XV7001_RATE_SCALE_24BIT;
}

/**
 * @brief  将12位原始温度转换为实际温度 (°C)
 * @param  rawTemp: 12位原始温度值
 * @retval float 实际温度 (单位: °C)
 * 
 * 转换公式:
 *   实际温度 = (原始值 - 偏移量) / 16.0
 *   1 LSB = 1/16 °C = 0.0625°C
 *   偏移量约为 6 (可根据实际情况调整)
 */
float XV7001_ConvertTemperature(int16_t rawTemp)
{
    return ((float)rawTemp - XV7001_TEMP_OFFSET) / XV7001_TEMP_SCALE;
}

/**
 * @brief  读取并转换角速度 (16位模式，返回°/s)
 * @retval float 实际角速度 (单位: °/s)
 * 
 * 使用示例:
 *   float angularVel = XV7001_GetAngularVelocity16();
 *   // angularVel 现在是以 °/s 为单位的角速度
 */
float XV7001_GetAngularVelocity16(void)
{
    int16_t rawRate = XV7001_ReadRate16();
    return XV7001_ConvertRate16(rawRate);
}

/**
 * @brief  读取并转换角速度 (24位模式，返回°/s)
 * @retval float 实际角速度 (单位: °/s)
 */
float XV7001_GetAngularVelocity24(void)
{
    int32_t rawRate = XV7001_ReadRate24();
    return XV7001_ConvertRate24(rawRate);
}

/**
 * @brief  读取并转换温度 (返回°C)
 * @retval float 实际温度 (单位: °C)
 * 
 * 使用示例:
 *   float tempC = XV7001_GetTemperatureCelsius();
 *   // tempC 现在是以 °C 为单位的温度
 */
float XV7001_GetTemperatureCelsius(void)
{
    int16_t rawTemp = XV7001_ReadTemperature();
    return XV7001_ConvertTemperature(rawTemp);
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  毫秒延时函数
 * @param  ms: 延时毫秒数
 * @note   使用 HAL_Delay 实现
 */
static void XV7001_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
