/**
 * @file    angle_calc.h
 * @brief   角度计算模块 - 基于陀螺仪角速度的角度积分
 * @details 使用梯形积分法将角速度(°/s)转换为角度(°)
 * 
 * 依赖:
 *   - xv7001bb.h/c: XV7001BB 陀螺仪驱动
 *   - FreeRTOS: 任务调度
 * 
 * 使用流程:
 *   1. 调用 Angle_Init() 初始化模块
 *   2. 在任务中周期性调用 Angle_Update() 更新角度
 *   3. 调用 Angle_GetAngle() 获取当前角度
 */

#ifndef ANGLE_CALC_H
#define ANGLE_CALC_H

#include <stdint.h>

/* ============================================================================
 *                          角度计算常量定义
 * ============================================================================ */

/**
 * @brief  角度更新周期 (毫秒)
 * @note   与陀螺仪采样率匹配，10ms = 100Hz
 */
#define ANGLE_UPDATE_PERIOD_MS          10

/**
 * @brief  角度更新周期 (秒)
 * @note   用于积分计算: dt = 10ms = 0.01s
 */
#define ANGLE_UPDATE_PERIOD_S           0.01f

/**
 * @brief  角度校准因子
 * @note   根据 XV7001BB 数据手册校准
 *         校准因子 = 4.0424
 */
#define ANGLE_CALIBRATION_FACTOR        4.0424f

/**
 * @brief  角度范围限制 (可选)
 * @note   用于将角度限制在 -180° ~ +180° 或 0° ~ 360°
 */
#define ANGLE_RANGE_180                 180.0f
#define ANGLE_RANGE_360                 360.0f

/* ============================================================================
 *                          角度计算功能接口
 * ============================================================================ */

/**
 * @brief  初始化角度计算模块
 * @note   清零角度值和上一次角速度值
 * @retval None
 */
void Angle_Init(void);

/**
 * @brief  更新角度值 (梯形积分法)
 * @param  currentRate: 当前角速度 (单位: °/s)
 * @note   积分公式: angle += (prevRate + currentRate) / 2 * dt
 *         需要周期性调用，周期为 ANGLE_UPDATE_PERIOD_MS
 * @retval None
 */
void Angle_Update(float currentRate);

/**
 * @brief  获取当前角度值
 * @retval float 当前角度 (单位: °)
 */
float Angle_GetAngle(void);

/**
 * @brief  重置角度值为指定值
 * @param  angle: 目标角度 (单位: °)
 * @note   用于角度校准或归零
 * @retval None
 */
void Angle_SetAngle(float angle);

/**
 * @brief  重置角度值为零
 * @note   等同于 Angle_SetAngle(0.0f)
 * @retval None
 */
void Angle_Reset(void);

/**
 * @brief  获取归一化角度 (-180° ~ +180°)
 * @retval float 归一化后的角度
 */
float Angle_GetNormalized180(void);

/**
 * @brief  获取归一化角度 (0° ~ 360°)
 * @retval float 归一化后的角度
 */
float Angle_GetNormalized360(void);

#endif /* ANGLE_CALC_H */
