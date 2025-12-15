/**
 * @file    task_main.c
 * @brief   STM32F103C8T6 FreeRTOS LED 闪烁项目
 * @details LED 连接到 PB9，正常状态 1Hz 闪烁，错误状态 10Hz 快闪
 * 
 * 硬件配置：
 *   - MCU: STM32F103C8T6
 *   - LED: PB9 (推挽输出)
 *   - 时钟: HSI (8MHz) + PLL -> 64MHz
 * 
 * 功能说明：
 *   1. 系统时钟配置为 64MHz (HSI + PLL)
 *   2. GPIOB Pin9 配置为推挽输出
 *   3. FreeRTOS 任务控制 LED 闪烁
 *   4. 正常模式: 500ms 间隔 (1Hz)
 *   5. 错误模式: 50ms 间隔 (10Hz)
 */

#include <stm32f1xx_hal.h>
#include <stm32_hal_legacy.h>
#include <FreeRTOS.h>
#include <task.h>
#include "xv7001bb.h"
#include "angle_calc.h"
#include "gyro_bias.h"

/* ==================== LED 引脚定义 ==================== */
#define LED_PIN         GPIO_PIN_9              /* LED 连接到 PB9 */
#define LED_PORT        GPIOB                   /* LED 端口 GPIOB */
#define LED_CLK_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE  /* GPIOB 时钟使能宏 */

/* ==================== 闪烁时间定义 (毫秒) ==================== */
#define BLINK_NORMAL_MS     500     /* 正常模式: 1Hz = 500ms 开 + 500ms 关 */
#define BLINK_ERROR_MS      50      /* 错误模式: 10Hz = 50ms 开 + 50ms 关 */

/* ==================== 系统状态定义 ==================== */
typedef enum {
    STATE_NORMAL = 0,   /* 正常状态 */
    STATE_ERROR         /* 错误状态 */
} SystemState_t;

/* 全局系统状态变量 (默认为正常状态) */
static volatile SystemState_t g_SystemState = STATE_NORMAL;

/* ==================== 函数声明 ==================== */
static void SystemClock_Config(void);   /* 系统时钟配置函数 */
static void LED_GPIO_Init(void);        /* LED GPIO 初始化函数 */
static void LED_Task(void *pvParameters); /* LED 闪烁任务函数 */
static void Gyro_Task(void *pvParameters); /* 陀螺仪读取任务函数 */

/* 全局变量: 存储陀螺仪读取的转换后数值 (供调试查看) */
static volatile float g_AngularRate = 0.0f;      /* 角速度 (单位: °/s) */
static volatile float g_Temperature = 25.0f;    /* 温度 (单位: °C), 初始值 25°C */
static volatile float g_Angle = 0.0f;           /* 角度 (单位: °), 积分得到 */
static volatile float g_GyroBias = 0.0f;        /* 零漂 (单位: °/s) */

/* FreeRTOS 移植层外部函数声明 */
extern void xPortSysTickHandler(void);

/**
 * @brief  SysTick 中断处理函数 - HAL 库和 FreeRTOS 共享
 * @note   此函数在每个系统滴答 (1ms) 时被调用
 *         需要同时服务 HAL 库的时基和 FreeRTOS 的任务调度
 */
void SysTick_Handler(void)
{
    /* 步骤1: 增加 HAL 库的时间基准计数 */
    HAL_IncTick();
    
    /* 步骤2: 如果 FreeRTOS 调度器已启动，则调用 FreeRTOS 的 SysTick 处理函数 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/**
 * @brief  系统时钟配置函数 - 使用 HSI + PLL
 * @note   时钟配置路径: HSI (8MHz) -> HSI/2 (4MHz) -> PLL×16 -> SYSCLK (64MHz)
 * @retval None
 * 
 * 时钟树说明:
 *   HSI 内部高速时钟: 8MHz (无需外部晶振)
 *   PLL 输入: HSI/2 = 4MHz
 *   PLL 倍频: ×16
 *   系统时钟 SYSCLK: 4MHz × 16 = 64MHz
 *   AHB 总线 HCLK: 64MHz (不分频)
 *   APB1 总线: 32MHz (2 分频，最大 36MHz)
 *   APB2 总线: 64MHz (不分频)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* ========== 步骤1: 配置振荡器 (HSI) 和 PLL ========== */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  /* 选择 HSI 作为时钟源 */
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    /* 使能 HSI */
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; /* HSI 校准值 */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                /* 使能 PLL */
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;   /* PLL 时钟源: HSI/2 = 4MHz */
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;               /* PLL 倍频系数: ×16 = 64MHz */
    
    /* 应用振荡器配置 */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* 配置失败，设置错误状态 (LED 将快闪) */
        g_SystemState = STATE_ERROR;
        return;
    }

    /* ========== 步骤2: 配置系统时钟和总线分频 ========== */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   /* 系统时钟源选择 PLL */
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          /* AHB 不分频: HCLK = 64MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           /* APB1 2分频: 32MHz (最大36MHz) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           /* APB2 不分频: 64MHz */
    
    /* 应用时钟配置，Flash 延迟设置为 2 个等待周期 (64MHz 需要) */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        /* 配置失败，设置错误状态 */
        g_SystemState = STATE_ERROR;
        return;
    }
}

/**
 * @brief  LED GPIO 初始化函数
 * @note   将 GPIOB Pin9 配置为推挽输出模式用于驱动 LED
 * @retval None
 * 
 * GPIO 配置说明:
 *   引脚: PB9
 *   模式: 推挽输出 (Push-Pull)
 *   速度: 低速 (LED 闪烁不需要高速)
 *   上下拉: 无
 */
static void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ========== 步骤1: 使能 GPIOB 时钟 ========== */
    LED_CLK_ENABLE();

    /* ========== 步骤2: 配置 PB9 为推挽输出 ========== */
    GPIO_InitStruct.Pin = LED_PIN;                  /* 选择 Pin9 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     /* 推挽输出模式 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* 低速模式 (节省功耗) */
    GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 无上下拉电阻 */
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* ========== 步骤3: 设置初始状态为低电平 (LED 熄灭) ========== */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  设置系统状态 (供外部模块调用)
 * @param  state: 系统状态
 *         - STATE_NORMAL: 正常状态 (LED 1Hz 闪烁)
 *         - STATE_ERROR:  错误状态 (LED 10Hz 快闪)
 * @retval None
 * 
 * 使用示例:
 *   SetSystemState(STATE_ERROR);  // 切换到错误状态
 *   SetSystemState(STATE_NORMAL); // 恢复正常状态
 */
void SetSystemState(SystemState_t state)
{
    g_SystemState = state;
}

/**
 * @brief  LED 闪烁任务 (FreeRTOS 任务函数)
 * @param  pvParameters: 任务参数 (本任务未使用)
 * @retval None
 * 
 * 任务功能:
 *   根据系统状态控制 LED 闪烁频率:
 *   - 正常状态: 1Hz 闪烁 (500ms 间隔)
 *   - 错误状态: 10Hz 快闪 (50ms 间隔)
 * 
 * 实现方式:
 *   使用 vTaskDelayUntil() 实现精确的周期性延时
 *   避免累积误差，确保闪烁频率准确
 */
static void LED_Task(void *pvParameters)
{
    (void)pvParameters;  /* 未使用参数，避免编译警告 */
    TickType_t xLastWakeTime;  /* 记录上次唤醒时间 */
    uint32_t blinkDelay;       /* 闪烁延时 (毫秒) */

    /* ========== 初始化: 获取当前系统滴答计数 ========== */
    xLastWakeTime = xTaskGetTickCount();

    /* ========== 任务主循环 ========== */
    for (;;)
    {
        /* 步骤1: 根据系统状态选择闪烁间隔 */
        blinkDelay = (g_SystemState == STATE_ERROR) ? BLINK_ERROR_MS : BLINK_NORMAL_MS;

        /* 步骤2: 翻转 LED 状态 (亮->灭 或 灭->亮) */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        /* 步骤3: 精确延时到下一个周期 */
        /* vTaskDelayUntil 会自动计算延时，避免累积误差 */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(blinkDelay));
    }
}

/**
 * @brief  陀螺仪读取任务 (FreeRTOS 任务函数)
 * @param  pvParameters: 任务参数 (本任务未使用)
 * @retval None
 * 
 * 任务功能:
 *   1. 初始化 XV7001BB 陀螺仪
 *   2. 周期性读取角速度和温度数据
 *   3. 读取失败时设置错误状态 (LED 快闪)
 * 
 * 调试方法:
 *   在调试器中查看全局变量:
 *   - g_AngularRate: 角速度 (°/s)
 *   - g_Temperature: 温度 (°C)
 */
static void Gyro_Task(void *pvParameters)
{
    (void)pvParameters;  /* 未使用参数，避免编译警告 */
    HAL_StatusTypeDef status;
    
    /* ========== 步骤1: 初始化陀螺仪 ========== */
    status = XV7001_Init();
    if (status != HAL_OK)
    {
        /* 初始化失败，设置错误状态 */
        g_SystemState = STATE_ERROR;
    }
    
    /* ========== 步骤2: 初始化角度计算模块 ========== */
    Angle_Init();
    
    /* ========== 步骤3: 初始化零漂校准模块 ========== */
    GyroBias_Init();
    
    /* ========== 步骤4: 任务主循环 ========== */
    for (;;)
    {
        /* 检查数据是否准备好 */
        if (XV7001_IsDataReady())
        {
            /* 读取并转换角速度 (16位模式, 单位: °/s) */
            float rawRate = XV7001_GetAngularVelocity16();
            
            /* 更新零漂校准 */
            GyroBias_Update(rawRate);
            g_GyroBias = GyroBias_GetBias();
            
            /* 获取校准后的角速度 (去除零漂) */
            g_AngularRate = GyroBias_GetCalibratedRate(rawRate);
            
            /* 更新角度 (梯形积分法，使用校准后的角速度) */
            Angle_Update(g_AngularRate);
            g_Angle = Angle_GetAngle();
            
            /* 读取并转换温度 (单位: °C) */
            g_Temperature = XV7001_GetTemperatureCelsius();
            
            /* 数据读取成功，确保正常状态 */
            if (g_SystemState != STATE_NORMAL)
            {
                g_SystemState = STATE_NORMAL;
            }
        }
        
        /* 延时 10ms (100Hz 采样率) */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  主函数 - 程序入口
 * @retval int (实际不会返回)
 * 
 * 执行流程:
 *   1. 初始化 HAL 库
 *   2. 配置系统时钟为 64MHz
 *   3. 初始化 LED GPIO
 *   4. 创建 FreeRTOS LED 任务
 *   5. 启动 FreeRTOS 调度器
 *   6. 进入死循环 (正常情况下不会执行到)
 */
int main(void)
{
    /* ========== 步骤1: 初始化 HAL 库 ========== */
    /* 配置 Flash 预取、指令缓存、数据缓存 */
    /* 初始化 SysTick 作为时间基准 (1ms) */
    HAL_Init();

    /* ========== 步骤2: 配置系统时钟 (HSI + PLL -> 64MHz) ========== */
    SystemClock_Config();

    /* ========== 步骤3: 初始化 LED GPIO (PB9) ========== */
    LED_GPIO_Init();

    /* ========== 步骤4: 创建 LED 闪烁任务 ========== */
    /* 任务名称: "LED_Task" */
    /* 堆栈大小: configMINIMAL_STACK_SIZE (128 字) */
    /* 任务参数: NULL (无参数) */
    /* 优先级: tskIDLE_PRIORITY + 1 (比空闲任务高一级) */
    /* 任务句柄: NULL (不需要保存句柄) */
    if (xTaskCreate(LED_Task,
                    "LED_Task",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS)
    {
        /* 任务创建失败，设置错误状态 */
        g_SystemState = STATE_ERROR;
    }

    /* ========== 步骤5: 创建陀螺仪读取任务 ========== */
    /* 任务名称: "Gyro_Task" */
    /* 堆栈大小: configMINIMAL_STACK_SIZE * 2 (需要更多栈空间) */
    /* 优先级: tskIDLE_PRIORITY + 2 (比 LED 任务高) */
    if (xTaskCreate(Gyro_Task,
                    "Gyro_Task",
                    configMINIMAL_STACK_SIZE * 2,
                    NULL,
                    tskIDLE_PRIORITY + 2,
                    NULL) != pdPASS)
    {
        /* 任务创建失败，设置错误状态 */
        g_SystemState = STATE_ERROR;
    }

    /* ========== 步骤6: 启动 FreeRTOS 调度器 ========== */
    /* 调度器启动后，将开始执行任务 */
    /* 此函数不会返回 (除非堆内存不足) */
    vTaskStartScheduler();

    /* ========== 步骤6: 死循环 (理论上不会执行到这里) ========== */
    /* 如果执行到这里，说明调度器启动失败 (通常是堆内存不足) */
    for (;;)
    {
        /* 空循环 */
    }
}
