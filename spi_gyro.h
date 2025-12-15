/**
 * @file    spi_gyro.h
 * @brief   SPI2 驱动头文件 - 用于 XV7001BB 陀螺仪通信
 * @details 配置 SPI2 为全双工主模式，与 XV7001BB 陀螺仪进行数据交换
 * 
 * 硬件连接:
 *   - PB12: NSS  (片选信号，软件控制)
 *   - PB13: SCK  (SPI 时钟)
 *   - PB14: MISO (主入从出，接收数据)
 *   - PB15: MOSI (主出从入，发送数据)
 * 
 * SPI 配置:
 *   - 模式: Mode 3 (CPOL=1, CPHA=1)
 *   - 波特率: ≤1MHz
 *   - 数据位: 8-bit
 *   - 字节序: MSB First
 *   - 工作模式: 全双工主模式
 */

#ifndef SPI_GYRO_H
#define SPI_GYRO_H

#include <stm32f1xx_hal.h>

/* ==================== 硬件引脚定义 ==================== */

/* NSS 片选引脚 (软件控制) */
#define GYRO_NSS_PIN            GPIO_PIN_12         /* PB12 */
#define GYRO_NSS_PORT           GPIOB               /* GPIOB */

/* SCK 时钟引脚 */
#define GYRO_SCK_PIN            GPIO_PIN_13         /* PB13 */
#define GYRO_SCK_PORT           GPIOB               /* GPIOB */

/* MISO 数据输入引脚 */
#define GYRO_MISO_PIN           GPIO_PIN_14         /* PB14 */
#define GYRO_MISO_PORT          GPIOB               /* GPIOB */

/* MOSI 数据输出引脚 */
#define GYRO_MOSI_PIN           GPIO_PIN_15         /* PB15 */
#define GYRO_MOSI_PORT          GPIOB               /* GPIOB */

/* ==================== NSS 片选控制宏 ==================== */

/**
 * @brief  片选信号拉低 (选中从设备，开始通信)
 * @note   XV7001BB 在 NSS 低电平时被选中
 */
#define GYRO_NSS_LOW()          HAL_GPIO_WritePin(GYRO_NSS_PORT, GYRO_NSS_PIN, GPIO_PIN_RESET)

/**
 * @brief  片选信号拉高 (取消选中，结束通信)
 * @note   通信结束后必须拉高 NSS
 */
#define GYRO_NSS_HIGH()         HAL_GPIO_WritePin(GYRO_NSS_PORT, GYRO_NSS_PIN, GPIO_PIN_SET)

/* ==================== SPI 超时设置 ==================== */

#define GYRO_SPI_TIMEOUT        100                 /* SPI 操作超时时间 (毫秒) */

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化 SPI2 外设和相关 GPIO
 * @note   配置 SPI2 为 Mode 3，波特率 ≤1MHz，8-bit 数据
 * @retval HAL_StatusTypeDef
 *         - HAL_OK: 初始化成功
 *         - HAL_ERROR: 初始化失败
 */
HAL_StatusTypeDef SPI_Gyro_Init(void);

/**
 * @brief  SPI 发送并接收单字节数据 (全双工)
 * @param  txData: 要发送的数据字节
 * @retval 接收到的数据字节
 * @note   SPI 是同步通信，发送和接收同时进行
 */
uint8_t SPI_Gyro_TransmitReceive(uint8_t txData);

/**
 * @brief  SPI 发送并接收多字节数据
 * @param  pTxData: 发送数据缓冲区指针
 * @param  pRxData: 接收数据缓冲区指针
 * @param  length: 数据长度 (字节数)
 * @retval HAL_StatusTypeDef
 *         - HAL_OK: 传输成功
 *         - HAL_ERROR: 传输失败
 *         - HAL_TIMEOUT: 传输超时
 */
HAL_StatusTypeDef SPI_Gyro_TransmitReceiveBuffer(uint8_t *pTxData, uint8_t *pRxData, uint16_t length);

/**
 * @brief  读取陀螺仪寄存器
 * @param  regAddr: 寄存器地址
 * @retval 寄存器值
 * @note   自动处理 NSS 片选信号
 */
uint8_t SPI_Gyro_ReadRegister(uint8_t regAddr);

/**
 * @brief  写入陀螺仪寄存器
 * @param  regAddr: 寄存器地址
 * @param  value: 要写入的值
 * @retval HAL_StatusTypeDef
 * @note   自动处理 NSS 片选信号
 */
HAL_StatusTypeDef SPI_Gyro_WriteRegister(uint8_t regAddr, uint8_t value);

/**
 * @brief  获取 SPI2 句柄指针
 * @retval SPI_HandleTypeDef* SPI2 句柄指针
 * @note   供外部模块直接访问 SPI 句柄 (如需要)
 */
SPI_HandleTypeDef* SPI_Gyro_GetHandle(void);

#endif /* SPI_GYRO_H */
