/**
 * @file    gyro_bias.h
 * @brief   陀螺仪零漂校准模块
 * @details 实现两阶段零漂校准:
 *          1. 初始阶段: 静止采样400次取平均值
 *          2. 运行阶段: 检测静止状态，EMA动态更新零漂
 * 
 * 使用流程:
 *   1. 调用 GyroBias_Init() 初始化
 *   2. 在主循环中调用 GyroBias_Update() 更新
 *   3. 调用 GyroBias_GetBias() 获取当前零漂值
 *   4. 使用 (rawRate - bias) 得到校准后的角速度
 */

#ifndef GYRO_BIAS_H
#define GYRO_BIAS_H

#include <stdint.h>

/* ============================================================================
 *                          零漂校准常量定义
 * ============================================================================ */

/**
 * @brief  初始校准采样次数
 * @note   静止状态下采集400个样本计算平均值
 */
#define BIAS_INIT_SAMPLE_COUNT          400

/**
 * @brief  初始校准超时时间 (毫秒)
 * @note   超过此时间强制完成初始校准
 *         400次 × 10ms = 4000ms，设置5秒超时
 */
#define BIAS_INIT_TIMEOUT_MS            5000

/**
 * @brief  EMA 平滑系数 (0~1)
 * @note   较小的值使零漂更新更平滑但响应较慢
 *         alpha = 0.01 表示新值权重1%，旧值权重99%
 */
#define BIAS_EMA_ALPHA                  0.01f

/**
 * @brief  静止检测阈值 (°/s)
 * @note   角速度绝对值小于此阈值认为静止
 */
#define BIAS_STATIONARY_THRESHOLD       0.5f

/**
 * @brief  静止检测连续次数
 * @note   连续多少次检测到静止才认为真正静止
 */
#define BIAS_STATIONARY_COUNT           50

/* ============================================================================
 *                          校准状态定义
 * ============================================================================ */

/**
 * @brief  校准状态枚举
 */
typedef enum {
    BIAS_STATE_INIT = 0,    /* 初始校准阶段 */
    BIAS_STATE_RUNNING      /* 运行阶段 (动态更新) */
} GyroBiasState_t;

/* ============================================================================
 *                          零漂校准功能接口
 * ============================================================================ */

/**
 * @brief  初始化零漂校准模块
 * @note   清零所有状态，准备开始初始校准
 * @retval None
 */
void GyroBias_Init(void);

/**
 * @brief  更新零漂校准
 * @param  rawRate: 原始角速度值 (单位: °/s)
 * @note   初始阶段: 累加样本计算平均值
 *         运行阶段: 检测静止状态，EMA更新零漂
 * @retval None
 */
void GyroBias_Update(float rawRate);

/**
 * @brief  获取当前零漂值
 * @retval float 零漂值 (单位: °/s)
 */
float GyroBias_GetBias(void);

/**
 * @brief  获取校准后的角速度
 * @param  rawRate: 原始角速度值 (单位: °/s)
 * @retval float 校准后角速度 = rawRate - bias
 */
float GyroBias_GetCalibratedRate(float rawRate);

/**
 * @brief  获取当前校准状态
 * @retval GyroBiasState_t 校准状态
 */
GyroBiasState_t GyroBias_GetState(void);

/**
 * @brief  检查初始校准是否完成
 * @retval uint8_t 1=完成, 0=未完成
 */
uint8_t GyroBias_IsCalibrated(void);

/**
 * @brief  强制完成初始校准
 * @note   用于超时或需要立即开始运行的情况
 * @retval None
 */
void GyroBias_ForceComplete(void);

/**
 * @brief  重置校准 (重新开始初始校准)
 * @retval None
 */
void GyroBias_Reset(void);

#endif /* GYRO_BIAS_H */
