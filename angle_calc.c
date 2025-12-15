/**
 * @file    angle_calc.c
 * @brief   角度计算模块实现
 * @details 使用梯形积分法将角速度(°/s)转换为角度(°)
 * 
 * 梯形积分法原理:
 *   angle += (prevRate + currentRate) / 2 * dt
 *   比矩形积分法更精确，减少积分误差
 * 
 * 使用示例:
 *   Angle_Init();                           // 初始化
 *   while (1) {
 *       float rate = XV7001_GetAngularVelocity16();
 *       Angle_Update(rate);                 // 更新角度
 *       float angle = Angle_GetAngle();     // 获取角度
 *       vTaskDelay(pdMS_TO_TICKS(10));      // 10ms 周期
 *   }
 */

#include "angle_calc.h"

/* ==================== 私有变量 ==================== */

/**
 * @brief  当前累积角度 (单位: °)
 */
static volatile float s_Angle = 0.0f;

/**
 * @brief  上一次角速度值 (单位: °/s)
 * @note   用于梯形积分计算
 */
static volatile float s_PrevRate = 0.0f;

/**
 * @brief  模块初始化标志
 */
static volatile uint8_t s_Initialized = 0;

/* ==================== 公有函数实现 ==================== */

/**
 * @brief  初始化角度计算模块
 * @note   清零角度值和上一次角速度值
 */
void Angle_Init(void)
{
    s_Angle = 0.0f;
    s_PrevRate = 0.0f;
    s_Initialized = 1;
}

/**
 * @brief  更新角度值 (梯形积分法)
 * @param  currentRate: 当前角速度 (单位: °/s)
 * 
 * 梯形积分公式:
 *   angle += (prevRate + currentRate) / 2 * dt
 * 
 * 其中:
 *   - prevRate: 上一次采样的角速度
 *   - currentRate: 当前采样的角速度
 *   - dt: 采样周期 (ANGLE_UPDATE_PERIOD_S = 0.01s)
 * 
 * 优点:
 *   相比矩形积分法 (angle += rate * dt)，
 *   梯形积分法使用两次采样的平均值，精度更高
 */
void Angle_Update(float currentRate)
{
    float deltaAngle;
    
    /* 如果未初始化，先初始化 */
    if (!s_Initialized)
    {
        Angle_Init();
    }
    
    /* 梯形积分: 使用上次和本次角速度的平均值 */
    /* 乘以校准因子修正角度 */
    deltaAngle = (s_PrevRate + currentRate) * 0.5f * ANGLE_UPDATE_PERIOD_S * ANGLE_CALIBRATION_FACTOR;
    
    /* 累加角度 */
    s_Angle += deltaAngle;
    
    /* 保存当前角速度供下次使用 */
    s_PrevRate = currentRate;
}

/**
 * @brief  获取当前角度值
 * @retval float 当前角度 (单位: °)
 */
float Angle_GetAngle(void)
{
    return s_Angle;
}

/**
 * @brief  重置角度值为指定值
 * @param  angle: 目标角度 (单位: °)
 */
void Angle_SetAngle(float angle)
{
    s_Angle = angle;
}

/**
 * @brief  重置角度值为零
 */
void Angle_Reset(void)
{
    s_Angle = 0.0f;
    s_PrevRate = 0.0f;
}

/**
 * @brief  获取归一化角度 (-180° ~ +180°)
 * @retval float 归一化后的角度
 * 
 * 算法:
 *   1. 先对 360° 取模
 *   2. 如果 > 180°，减去 360°
 *   3. 如果 < -180°，加上 360°
 */
float Angle_GetNormalized180(void)
{
    float angle = s_Angle;
    
    /* 先归一化到 0° ~ 360° */
    while (angle >= ANGLE_RANGE_360)
    {
        angle -= ANGLE_RANGE_360;
    }
    while (angle < 0.0f)
    {
        angle += ANGLE_RANGE_360;
    }
    
    /* 再转换到 -180° ~ +180° */
    if (angle > ANGLE_RANGE_180)
    {
        angle -= ANGLE_RANGE_360;
    }
    
    return angle;
}

/**
 * @brief  获取归一化角度 (0° ~ 360°)
 * @retval float 归一化后的角度
 */
float Angle_GetNormalized360(void)
{
    float angle = s_Angle;
    
    /* 归一化到 0° ~ 360° */
    while (angle >= ANGLE_RANGE_360)
    {
        angle -= ANGLE_RANGE_360;
    }
    while (angle < 0.0f)
    {
        angle += ANGLE_RANGE_360;
    }
    
    return angle;
}
