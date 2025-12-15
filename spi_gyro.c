/**
 * @file    spi_gyro.c
 * @brief   SPI2 驱动实现 - 用于 XV7001BB 陀螺仪通信
 * @details 实现 SPI2 初始化、数据收发、寄存器读写等功能
 * 
 * SPI2 外设说明:
 *   STM32F103 的 SPI2 挂载在 APB1 总线上
 *   APB1 时钟 = 32MHz (在 task_main.c 中配置)
 *   波特率 = APB1 / 分频系数
 * 
 * XV7001BB 陀螺仪 SPI 时序要求:
 *   - SPI Mode 3: CPOL=1 (空闲时钟高电平), CPHA=1 (第二边沿采样)
 *   - 最大时钟频率: 1MHz
 *   - 数据格式: 8-bit, MSB First
 */

#include "spi_gyro.h"

/* ==================== 私有变量 ==================== */

/**
 * @brief  SPI2 句柄
 * @note   HAL 库使用句柄管理外设，所有 SPI 操作都通过此句柄进行
 */
static SPI_HandleTypeDef hspi2;

/* ==================== 私有函数声明 ==================== */

static void SPI_Gyro_GPIO_Init(void);
static void SPI_Gyro_MspInit(void);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief  初始化 SPI2 外设
 * @note   配置流程:
 *         1. 初始化 GPIO (NSS, SCK, MISO, MOSI)
 *         2. 配置 SPI2 参数 (Mode 3, 8-bit, MSB First)
 *         3. 使能 SPI2 外设
 * @retval HAL_StatusTypeDef
 *         - HAL_OK: 初始化成功
 *         - HAL_ERROR: 初始化失败
 * 
 * SPI Mode 3 时序说明:
 *   CPOL=1: 空闲状态时，SCK 保持高电平
 *   CPHA=1: 数据在 SCK 的第二个边沿 (下降沿) 被采样
 *           数据在 SCK 的第一个边沿 (上升沿) 被移出
 * 
 * 波特率计算:
 *   APB1 时钟 = 32MHz
 *   分频系数 = 32 (SPI_BAUDRATEPRESCALER_32)
 *   波特率 = 32MHz / 32 = 1MHz
 */
HAL_StatusTypeDef SPI_Gyro_Init(void)
{
    HAL_StatusTypeDef status;
    
    /* ========== 步骤1: 初始化 SPI 相关 GPIO ========== */
    SPI_Gyro_GPIO_Init();
    
    /* ========== 步骤2: 配置 SPI2 参数 ========== */
    
    /* 选择 SPI2 外设 */
    hspi2.Instance = SPI2;
    
    /* 主从模式: 主模式 (STM32 作为主设备控制时钟) */
    hspi2.Init.Mode = SPI_MODE_MASTER;
    
    /* 通信方向: 全双工 (同时收发) */
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    
    /* 数据位宽: 8-bit */
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    
    /* 时钟极性 CPOL=1: 空闲时 SCK 为高电平 */
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    
    /* 时钟相位 CPHA=1: 第二个边沿采样数据 */
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    
    /* NSS 管理: 软件控制 (使用 GPIO 控制片选) */
    hspi2.Init.NSS = SPI_NSS_SOFT;
    
    /* 波特率分频: 32 分频 -> 32MHz / 32 = 1MHz */
    /* 可选值: 2, 4, 8, 16, 32, 64, 128, 256 */
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    
    /* 数据传输顺序: MSB First (高位先传) */
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    
    /* TI 模式: 禁用 (使用 Motorola 模式) */
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    
    /* CRC 校验: 禁用 */
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;  /* CRC 多项式 (禁用时无效) */
    
    /* ========== 步骤3: 初始化 SPI2 外设 ========== */
    status = HAL_SPI_Init(&hspi2);
    
    if (status != HAL_OK)
    {
        /* 初始化失败 */
        return status;
    }
    
    /* ========== 步骤4: 确保 NSS 初始状态为高电平 (不选中) ========== */
    GYRO_NSS_HIGH();
    
    return HAL_OK;
}

/**
 * @brief  SPI 发送并接收单字节 (全双工)
 * @param  txData: 要发送的字节
 * @retval 接收到的字节
 * 
 * 工作原理:
 *   SPI 全双工通信时，发送和接收同时进行
 *   每发送一个字节，同时会接收一个字节
 *   即使只想读取数据，也需要发送一个字节 (通常发送 0x00 或 0xFF)
 * 
 * 注意:
 *   此函数不处理 NSS 片选，调用前需手动拉低 NSS
 */
uint8_t SPI_Gyro_TransmitReceive(uint8_t txData)
{
    uint8_t rxData = 0;
    
    /* 使用 HAL 库的全双工传输函数 */
    /* 发送 1 字节，同时接收 1 字节 */
    HAL_SPI_TransmitReceive(&hspi2, &txData, &rxData, 1, GYRO_SPI_TIMEOUT);
    
    return rxData;
}

/**
 * @brief  SPI 发送并接收多字节缓冲区
 * @param  pTxData: 发送数据缓冲区指针
 * @param  pRxData: 接收数据缓冲区指针
 * @param  length: 传输长度 (字节数)
 * @retval HAL_StatusTypeDef
 * 
 * 使用场景:
 *   - 批量读取陀螺仪数据
 *   - 连续读取多个寄存器
 * 
 * 注意:
 *   pTxData 和 pRxData 可以指向同一缓冲区
 *   此函数不处理 NSS 片选
 */
HAL_StatusTypeDef SPI_Gyro_TransmitReceiveBuffer(uint8_t *pTxData, uint8_t *pRxData, uint16_t length)
{
    return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, length, GYRO_SPI_TIMEOUT);
}

/**
 * @brief  读取陀螺仪寄存器
 * @param  regAddr: 寄存器地址 (7-bit)
 * @retval 寄存器值
 * 
 * XV7001BB 读操作时序:
 *   1. 拉低 NSS (选中芯片)
 *   2. 发送寄存器地址 (最高位为 1 表示读操作)
 *   3. 发送 0x00 (虚拟字节)，同时接收寄存器数据
 *   4. 拉高 NSS (结束通信)
 * 
 * 地址格式:
 *   Bit7: R/W 位 (1=读, 0=写)
 *   Bit6-0: 寄存器地址
 */
uint8_t SPI_Gyro_ReadRegister(uint8_t regAddr)
{
    uint8_t rxData;
    
    /* 步骤1: 拉低 NSS，选中陀螺仪 */
    GYRO_NSS_LOW();
    
    /* 步骤2: 发送读命令 (地址 | 0x80 表示读操作) */
    SPI_Gyro_TransmitReceive(regAddr | 0x80);
    
    /* 步骤3: 发送虚拟字节，接收寄存器数据 */
    rxData = SPI_Gyro_TransmitReceive(0x00);
    
    /* 步骤4: 拉高 NSS，结束通信 */
    GYRO_NSS_HIGH();
    
    return rxData;
}

/**
 * @brief  写入陀螺仪寄存器
 * @param  regAddr: 寄存器地址 (7-bit)
 * @param  value: 要写入的值
 * @retval HAL_StatusTypeDef
 * 
 * XV7001BB 写操作时序:
 *   1. 拉低 NSS (选中芯片)
 *   2. 发送寄存器地址 (最高位为 0 表示写操作)
 *   3. 发送要写入的数据
 *   4. 拉高 NSS (结束通信)
 * 
 * 地址格式:
 *   Bit7: R/W 位 (1=读, 0=写)
 *   Bit6-0: 寄存器地址
 */
HAL_StatusTypeDef SPI_Gyro_WriteRegister(uint8_t regAddr, uint8_t value)
{
    /* 步骤1: 拉低 NSS，选中陀螺仪 */
    GYRO_NSS_LOW();
    
    /* 步骤2: 发送写命令 (地址 & 0x7F 确保最高位为 0) */
    SPI_Gyro_TransmitReceive(regAddr & 0x7F);
    
    /* 步骤3: 发送要写入的数据 */
    SPI_Gyro_TransmitReceive(value);
    
    /* 步骤4: 拉高 NSS，结束通信 */
    GYRO_NSS_HIGH();
    
    return HAL_OK;
}

/**
 * @brief  获取 SPI2 句柄指针
 * @retval SPI_HandleTypeDef* SPI2 句柄指针
 * @note   供需要直接操作 SPI 的模块使用
 */
SPI_HandleTypeDef* SPI_Gyro_GetHandle(void)
{
    return &hspi2;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  初始化 SPI2 相关 GPIO
 * @note   配置 PB12-PB15 用于 SPI2 通信
 * 
 * GPIO 配置说明:
 *   PB12 (NSS):  推挽输出，软件控制片选
 *   PB13 (SCK):  复用推挽输出，SPI 时钟
 *   PB14 (MISO): 浮空输入，SPI 数据输入
 *   PB15 (MOSI): 复用推挽输出，SPI 数据输出
 */
static void SPI_Gyro_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ========== 步骤1: 使能 GPIO 时钟 ========== */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ========== 步骤2: 使能 SPI2 时钟 ========== */
    __HAL_RCC_SPI2_CLK_ENABLE();
    
    /* ========== 步骤3: 配置 NSS 引脚 (PB12) ========== */
    /* NSS 使用软件控制，配置为普通推挽输出 */
    GPIO_InitStruct.Pin = GYRO_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     /* 推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 无上下拉 */
    HAL_GPIO_Init(GYRO_NSS_PORT, &GPIO_InitStruct);
    
    /* NSS 初始状态为高电平 (不选中从设备) */
    HAL_GPIO_WritePin(GYRO_NSS_PORT, GYRO_NSS_PIN, GPIO_PIN_SET);
    
    /* ========== 步骤4: 配置 SCK 引脚 (PB13) ========== */
    /* SCK 为 SPI 时钟输出，配置为复用推挽输出 */
    GPIO_InitStruct.Pin = GYRO_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* 复用推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速 */
    HAL_GPIO_Init(GYRO_SCK_PORT, &GPIO_InitStruct);
    
    /* ========== 步骤5: 配置 MOSI 引脚 (PB15) ========== */
    /* MOSI 为 SPI 数据输出，配置为复用推挽输出 */
    GPIO_InitStruct.Pin = GYRO_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* 复用推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速 */
    HAL_GPIO_Init(GYRO_MOSI_PORT, &GPIO_InitStruct);
    
    /* ========== 步骤6: 配置 MISO 引脚 (PB14) ========== */
    /* MISO 为 SPI 数据输入，配置为浮空输入 */
    GPIO_InitStruct.Pin = GYRO_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         /* 输入模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 浮空输入 */
    HAL_GPIO_Init(GYRO_MISO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  SPI MSP 初始化回调函数
 * @note   此函数被 HAL_SPI_Init() 内部调用
 *         用于初始化底层硬件 (GPIO, 时钟等)
 *         由于我们在 SPI_Gyro_GPIO_Init() 中已完成 GPIO 初始化，
 *         此函数留空或用于其他底层配置
 */
static void SPI_Gyro_MspInit(void)
{
    /* GPIO 和时钟已在 SPI_Gyro_GPIO_Init() 中初始化 */
    /* 此函数可用于配置 DMA 或中断 (本例未使用) */
}
