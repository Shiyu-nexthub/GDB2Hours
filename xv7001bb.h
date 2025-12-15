/**
 * @file    xv7001bb.h
 * @brief   XV7001BB 陀螺仪驱动头文件
 * @details 提供 XV7001BB 陀螺仪的寄存器定义和功能接口
 * 
 * XV7001BB 简介:
 *   - 高精度数字输出陀螺仪
 *   - SPI 接口通信 (Mode 3)
 *   - 支持 16/24 位角速度输出
 *   - 内置数字低通滤波器
 *   - 支持零点校准功能
 */

#ifndef XV7001BB_H
#define XV7001BB_H

#include <stm32f1xx_hal.h>
#include "spi_gyro.h"

/* ============================================================================
 *                          XV7001BB SPI 通信协议定义
 * ============================================================================ */

/**
 * @brief  SPI 命令前缀定义
 * @note   XV7001BB 寄存器地址为 5 位 (0x00-0x1F)
 *         SPI 通信时需在地址前加命令位:
 *         - 读取: 地址 | 0x80 (最高位置1)
 *         - 写入: 地址 | 0x00 (最高位置0，即地址本身)
 * 
 * 使用示例:
 *   读状态寄存器 (0x04): 发送 XV7001_SPI_READ | 0x04 = 0x84
 *   写退出休眠 (0x06):   发送 XV7001_SPI_WRITE | 0x06 = 0x06
 */
#define XV7001_SPI_READ             0x80    /* 读命令前缀: 地址 | 0x80 */
#define XV7001_SPI_WRITE            0x00    /* 写命令前缀: 地址 | 0x00 */

/* ============================================================================
 *                          XV7001BB 寄存器地址定义
 * ============================================================================ */

/* -------------------- 控制类寄存器 -------------------- */

/**
 * @brief  DSP 控制寄存器 1 - 滤波器基本配置
 * @note   地址: 0x01
 *         用于配置数字滤波器的基本参数
 */
#define XV7001_REG_DSP_CTL1             0x01

/**
 * @brief  DSP 控制寄存器 2 - 低通滤波器截止频率设置
 * @note   地址: 0x02
 *         写入值决定截止频率:
 *         - 0x06: 100Hz 截止频率 (噪声低，响应慢)
 *         - 0x07: 200Hz 截止频率 (平衡)
 *         - 0x08: 400Hz 截止频率 (响应快，噪声大)
 */
#define XV7001_REG_DSP_CTL2             0x02
#define XV7001_LPF_100HZ                0x06    /* 低通滤波器 100Hz 截止 */
#define XV7001_LPF_200HZ                0x07    /* 低通滤波器 200Hz 截止 */
#define XV7001_LPF_400HZ                0x08    /* 低通滤波器 400Hz 截止 */

/**
 * @brief  DSP 控制寄存器 3 - 采样模式和校准使能
 * @note   地址: 0x03
 *         Bit6: 校准命令使能位，置1后才能执行零点校准
 */
#define XV7001_REG_DSP_CTL3             0x03
#define XV7001_CAL_ENABLE_BIT           (1 << 6)    /* Bit6: 校准命令使能位 */

/* -------------------- 状态寄存器 -------------------- */

/**
 * @brief  状态寄存器 - 读取设备当前状态
 * @note   地址: 0x04
 *         Bit3 (PROC_OK): 为1表示数据准备好可读取
 *         Bit2-0 (状态码): 设备工作模式
 *           - 0: 休眠状态
 *           - 1: 正在退出休眠
 *           - 2: 待机状态
 *           - 4: 上电复位后状态
 */
#define XV7001_REG_STATUS               0x04
#define XV7001_STATUS_PROC_OK           (1 << 3)    /* Bit3: 数据准备好标志 */
#define XV7001_STATUS_MASK              0x07        /* Bit2-0: 状态码掩码 */
#define XV7001_STATE_SLEEP              0x00        /* 状态码 0: 休眠状态 */
#define XV7001_STATE_WAKING             0x01        /* 状态码 1: 正在退出休眠 */
#define XV7001_STATE_STANDBY            0x02        /* 状态码 2: 待机状态 */
#define XV7001_STATE_POWER_ON           0x04        /* 状态码 4: 上电复位后状态 */

/* -------------------- 电源管理寄存器 -------------------- */

/**
 * @brief  进入休眠寄存器
 * @note   地址: 0x05
 *         写入任意值使芯片进入休眠模式 (低功耗)
 */
#define XV7001_REG_SLEEP_IN             0x05

/**
 * @brief  退出休眠寄存器
 * @note   地址: 0x06
 *         写入任意值唤醒芯片
 */
#define XV7001_REG_SLEEP_OUT            0x06

/**
 * @brief  待机模式寄存器
 * @note   地址: 0x07
 *         用于控制待机模式
 */
#define XV7001_REG_STANDBY              0x07

/* -------------------- 数据输出寄存器 -------------------- */

/**
 * @brief  温度读取寄存器
 * @note   地址: 0x08
 *         读取返回 2 字节，包含 12 位温度数据
 */
#define XV7001_REG_TEMP                 0x08

/**
 * @brief  软件复位寄存器
 * @note   地址: 0x09
 *         写入任意值触发芯片软复位
 */
#define XV7001_REG_SOFT_RST             0x09

/**
 * @brief  角速度读取寄存器 (最重要的寄存器)
 * @note   地址: 0x0A
 *         读取陀螺仪测量的角速度数据
 *         根据配置返回 16 位或 24 位数据
 */
#define XV7001_REG_RATE                 0x0A

/* -------------------- 输出控制寄存器 -------------------- */

/**
 * @brief  角速度输出控制寄存器
 * @note   地址: 0x0B
 *         Bit2: 输出位数选择 (1=24位, 0=16位)
 *         Bit0: 输出使能 (1=开启)
 *         示例: 写入 0x05 表示 24 位输出并使能
 */
#define XV7001_REG_RATE_CTL             0x0B
#define XV7001_RATE_24BIT               (1 << 2)    /* Bit2: 24位输出模式 */
#define XV7001_RATE_16BIT               0x00        /* 16位输出模式 (默认) */
#define XV7001_RATE_ENABLE              (1 << 0)    /* Bit0: 角速度输出使能 */
#define XV7001_RATE_24BIT_ENABLE        0x05        /* 24位输出 + 使能 (0b00000101) */
#define XV7001_RATE_16BIT_ENABLE        0x01        /* 16位输出 + 使能 (0b00000001) */

/* -------------------- 校准与复位寄存器 -------------------- */

/**
 * @brief  零点校准寄存器
 * @note   地址: 0x0C
 *         写入 0x01 触发零点校准
 *         注意: 需先在 DSP_CTL3 中使能校准位 (Bit6)
 */
#define XV7001_REG_ZERO_CAL             0x0C
#define XV7001_CAL_TRIGGER              0x01        /* 触发零点校准命令 */

/**
 * @brief  滤波器复位寄存器
 * @note   地址: 0x0D
 *         写入任意值清除数字滤波器内部状态
 */
#define XV7001_REG_FILTER_RST           0x0D

/* -------------------- 格式配置寄存器 -------------------- */

/**
 * @brief  温度格式寄存器
 * @note   地址: 0x1C
 *         写入 0x60 配置为 12 位温度输出格式
 */
#define XV7001_REG_TEMP_FMT             0x1C
#define XV7001_TEMP_12BIT               0x60        /* 12位温度输出格式 */

/**
 * @brief  接口控制寄存器
 * @note   地址: 0x1F
 *         写入 0x00 配置为 4 线 SPI 模式
 */
#define XV7001_REG_IF_CTL               0x1F
#define XV7001_SPI_4WIRE                0x00        /* 4线 SPI 模式 */

/* ============================================================================
 *                          XV7001BB 数据转换常量
 * ============================================================================ */

/* -------------------- 角速度转换常量 -------------------- */

/**
 * @brief  16位模式角速度转换系数
 * @note   1 LSB = 1/280 °/s
 *         实际角速度 (°/s) = 原始值 / 280.0
 */
#define XV7001_RATE_SCALE_16BIT         280.0f

/**
 * @brief  24位模式角速度转换系数
 * @note   1 LSB = 1/71680 °/s
 *         24位比16位多8位，相当于 280 × 256 = 71680
 *         实际角速度 (°/s) = 原始值 / 71680.0
 */
#define XV7001_RATE_SCALE_24BIT         71680.0f

/* -------------------- 温度转换常量 -------------------- */

/**
 * @brief  温度转换比例因子
 * @note   12位温度输出: 1 LSB = 1/16 °C = 0.0625°C
 */
#define XV7001_TEMP_SCALE               16.0f

/**
 * @brief  温度转换偏移量
 * @note   转换公式: °C = (原始值 - 偏移量) / 16
 *         根据实际校准: raw≈98 时应为室温 25°C
 *         偏移量 = 98 - 25×16 = -302
 */
#define XV7001_TEMP_OFFSET              (-302.0f)

/**
 * @brief  默认初始温度 (°C)
 * @note   用于初始化时的默认温度值
 */
#define XV7001_TEMP_DEFAULT             25.0f

/* ============================================================================
 *                          XV7001BB 功能接口函数
 * ============================================================================ */

/**
 * @brief  初始化 XV7001BB 陀螺仪
 * @note   配置 SPI 接口、设置默认参数
 * @retval HAL_StatusTypeDef
 *         - HAL_OK: 初始化成功
 *         - HAL_ERROR: 初始化失败
 */
HAL_StatusTypeDef XV7001_Init(void);

/**
 * @brief  软件复位 XV7001BB
 * @note   触发芯片软复位，恢复默认状态
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_SoftReset(void);

/**
 * @brief  读取设备状态
 * @retval uint8_t 状态寄存器值
 *         使用 XV7001_STATUS_xxx 宏解析
 */
uint8_t XV7001_GetStatus(void);

/**
 * @brief  检查数据是否准备好
 * @retval uint8_t 1=数据准备好, 0=未准备好
 */
uint8_t XV7001_IsDataReady(void);

/**
 * @brief  获取设备工作状态码
 * @retval uint8_t 状态码 (0=休眠, 1=唤醒中, 2=待机, 4=上电)
 */
uint8_t XV7001_GetState(void);

/* -------------------- 电源管理函数 -------------------- */

/**
 * @brief  使芯片进入休眠模式
 * @note   低功耗模式，需要唤醒后才能工作
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_EnterSleep(void);

/**
 * @brief  唤醒芯片 (退出休眠)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ExitSleep(void);

/**
 * @brief  进入待机模式
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_EnterStandby(void);

/* -------------------- 滤波器配置函数 -------------------- */

/**
 * @brief  设置低通滤波器截止频率
 * @param  lpfConfig: 滤波器配置值
 *         - XV7001_LPF_100HZ: 100Hz 截止
 *         - XV7001_LPF_200HZ: 200Hz 截止
 *         - XV7001_LPF_400HZ: 400Hz 截止
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_SetLPF(uint8_t lpfConfig);

/**
 * @brief  复位数字滤波器
 * @note   清除滤波器内部状态
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ResetFilter(void);

/* -------------------- 数据输出配置函数 -------------------- */

/**
 * @brief  配置角速度输出格式
 * @param  config: 输出配置值
 *         - XV7001_RATE_16BIT_ENABLE: 16位输出并使能
 *         - XV7001_RATE_24BIT_ENABLE: 24位输出并使能
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ConfigRateOutput(uint8_t config);

/**
 * @brief  配置温度输出格式
 * @note   设置为 12 位温度输出
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ConfigTempFormat(void);

/**
 * @brief  配置 SPI 接口模式
 * @note   设置为 4 线 SPI 模式
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ConfigSPIMode(void);

/* -------------------- 数据读取函数 -------------------- */

/**
 * @brief  读取角速度数据 (16位)
 * @retval int16_t 角速度原始值
 */
int16_t XV7001_ReadRate16(void);

/**
 * @brief  读取角速度数据 (24位)
 * @retval int32_t 角速度原始值 (24位有符号扩展到32位)
 */
int32_t XV7001_ReadRate24(void);

/**
 * @brief  读取温度数据
 * @retval int16_t 温度原始值 (12位)
 */
int16_t XV7001_ReadTemperature(void);

/* -------------------- 校准函数 -------------------- */

/**
 * @brief  执行零点校准
 * @note   自动使能校准位并触发校准
 *         校准期间芯片需保持静止
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef XV7001_ZeroCalibration(void);

/* -------------------- 数据转换函数 -------------------- */

/**
 * @brief  将16位原始角速度转换为实际角速度 (°/s)
 * @param  rawRate: 16位原始角速度值
 * @retval float 实际角速度 (单位: °/s)
 */
float XV7001_ConvertRate16(int16_t rawRate);

/**
 * @brief  将24位原始角速度转换为实际角速度 (°/s)
 * @param  rawRate: 24位原始角速度值 (已扩展为32位)
 * @retval float 实际角速度 (单位: °/s)
 */
float XV7001_ConvertRate24(int32_t rawRate);

/**
 * @brief  将12位原始温度转换为实际温度 (°C)
 * @param  rawTemp: 12位原始温度值
 * @retval float 实际温度 (单位: °C)
 */
float XV7001_ConvertTemperature(int16_t rawTemp);

/**
 * @brief  读取并转换角速度 (16位模式，返回°/s)
 * @retval float 实际角速度 (单位: °/s)
 */
float XV7001_GetAngularVelocity16(void);

/**
 * @brief  读取并转换角速度 (24位模式，返回°/s)
 * @retval float 实际角速度 (单位: °/s)
 */
float XV7001_GetAngularVelocity24(void);

/**
 * @brief  读取并转换温度 (返回°C)
 * @retval float 实际温度 (单位: °C)
 */
float XV7001_GetTemperatureCelsius(void);

#endif /* XV7001BB_H */
