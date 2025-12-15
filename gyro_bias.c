/**
 * @file    gyro_bias.c
 * @brief   陀螺仪零漂校准模块实现
 * @details 两阶段零漂校准:
 *          1. 初始阶段: 采集400个样本，计算平均值作为初始零漂
 *          2. 运行阶段: 检测静止状态，使用EMA动态更新零漂
 * 
 * EMA (指数移动平均) 公式:
 *   bias_new = alpha * sample + (1 - alpha) * bias_old
 *   alpha 越小，平滑效果越好，但响应越慢
 */

#include "gyro_bias.h"

/* ==================== 私有变量 ==================== */

/**
 * @brief  当前零漂值 (单位: °/s)
 */
static volatile float s_Bias = 0.0f;

/**
 * @brief  当前校准状态
 */
static volatile GyroBiasState_t s_State = BIAS_STATE_INIT;

/**
 * @brief  初始校准采样计数
 */
static volatile uint32_t s_SampleCount = 0;

/**
 * @brief  初始校准采样累加和
 */
static volatile float s_SampleSum = 0.0f;

/**
 * @brief  静止检测计数器
 */
static volatile uint32_t s_StationaryCount = 0;

/**
 * @brief  初始校准开始时间 (用于超时检测)
 */
static volatile uint32_t s_InitStartTick = 0;

/**
 * @brief  模块初始化标志
 */
static volatile uint8_t s_Initialized = 0;

/* ==================== 私有函数声明 ==================== */

static float fabsf_local(float x);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief  初始化零漂校准模块
 */
void GyroBias_Init(void)
{
    s_Bias = 0.0f;
    s_State = BIAS_STATE_INIT;
    s_SampleCount = 0;
    s_SampleSum = 0.0f;
    s_StationaryCount = 0;
    s_InitStartTick = 0;  /* 将在第一次Update时设置 */
    s_Initialized = 1;
}

/**
 * @brief  更新零漂校准
 * @param  rawRate: 原始角速度值 (单位: °/s)
 * 
 * 初始阶段:
 *   累加采样值，达到400次后计算平均值
 *   超时则强制完成
 * 
 * 运行阶段:
 *   检测静止状态 (角速度 < 阈值)
 *   连续静止50次后，使用EMA更新零漂
 */
void GyroBias_Update(float rawRate)
{
    float calibratedRate;
    
    /* 未初始化则先初始化 */
    if (!s_Initialized)
    {
        GyroBias_Init();
    }
    
    switch (s_State)
    {
        case BIAS_STATE_INIT:
            /* ========== 初始校准阶段 ========== */
            
            /* 累加样本 */
            s_SampleSum += rawRate;
            s_SampleCount++;
            
            /* 检查是否达到采样次数 */
            if (s_SampleCount >= BIAS_INIT_SAMPLE_COUNT)
            {
                /* 计算平均值作为初始零漂 */
                s_Bias = s_SampleSum / (float)s_SampleCount;
                s_State = BIAS_STATE_RUNNING;
            }
            break;
            
        case BIAS_STATE_RUNNING:
            /* ========== 运行阶段 (动态更新) ========== */
            
            /* 计算校准后的角速度 */
            calibratedRate = rawRate - s_Bias;
            
            /* 检测是否静止 */
            if (fabsf_local(calibratedRate) < BIAS_STATIONARY_THRESHOLD)
            {
                /* 静止计数增加 */
                s_StationaryCount++;
                
                /* 连续静止足够次数，使用EMA更新零漂 */
                if (s_StationaryCount >= BIAS_STATIONARY_COUNT)
                {
                    /* EMA 公式: bias = alpha * rawRate + (1-alpha) * bias */
                    s_Bias = BIAS_EMA_ALPHA * rawRate + (1.0f - BIAS_EMA_ALPHA) * s_Bias;
                }
            }
            else
            {
                /* 非静止，重置计数器 */
                s_StationaryCount = 0;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  获取当前零漂值
 * @retval float 零漂值 (单位: °/s)
 */
float GyroBias_GetBias(void)
{
    return s_Bias;
}

/**
 * @brief  获取校准后的角速度
 * @param  rawRate: 原始角速度值 (单位: °/s)
 * @retval float 校准后角速度 = rawRate - bias
 */
float GyroBias_GetCalibratedRate(float rawRate)
{
    return rawRate - s_Bias;
}

/**
 * @brief  获取当前校准状态
 * @retval GyroBiasState_t 校准状态
 */
GyroBiasState_t GyroBias_GetState(void)
{
    return s_State;
}

/**
 * @brief  检查初始校准是否完成
 * @retval uint8_t 1=完成, 0=未完成
 */
uint8_t GyroBias_IsCalibrated(void)
{
    return (s_State == BIAS_STATE_RUNNING) ? 1 : 0;
}

/**
 * @brief  强制完成初始校准
 * @note   使用当前已采集的样本计算平均值
 */
void GyroBias_ForceComplete(void)
{
    if (s_State == BIAS_STATE_INIT)
    {
        if (s_SampleCount > 0)
        {
            /* 使用已有样本计算平均值 */
            s_Bias = s_SampleSum / (float)s_SampleCount;
        }
        s_State = BIAS_STATE_RUNNING;
    }
}

/**
 * @brief  重置校准 (重新开始初始校准)
 */
void GyroBias_Reset(void)
{
    GyroBias_Init();
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  本地浮点绝对值函数
 * @param  x: 输入值
 * @retval float 绝对值
 */
static float fabsf_local(float x)
{
    return (x < 0.0f) ? (-x) : x;
}
