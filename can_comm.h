/**
 * @file    can_comm.h
 * @brief   CAN 通信驱动头文件 - 用于与上位机通信
 * @details 配置 CAN 外设实现 STM32 与上位机的数据交换
 * 
 * 硬件连接:
 *   - PA11: CAN_RX (CAN 接收引脚)
 *   - PA12: CAN_TX (CAN 发送引脚)
 * 
 * CAN 配置:
 *   - 波特率: 1000kbps (1Mbps)
 *   - 工作模式: Normal (正常模式)
 *   - 自动重传: 启用
 *   - 接收 FIFO: FIFO0
 * 
 * 通信协议:
 *   发送方向 (STM32 -> 上位机):
 *     - 0x123: 角度数据
 *     - 0x124: 角速度数据
 *     - 0x125: 温度数据
 *   接收方向 (上位机 -> STM32):
 *     - 0x666: 控制命令
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stm32f1xx_hal.h>

/* ==================== CAN 引脚定义 ==================== */

#define CAN_RX_PIN          GPIO_PIN_11         /* PA11: CAN 接收 */
#define CAN_TX_PIN          GPIO_PIN_12         /* PA12: CAN 发送 */
#define CAN_GPIO_PORT       GPIOA               /* CAN 引脚所在端口 */

/* ==================== CAN ID 定义 ==================== */

/**
 * @brief  发送数据 CAN ID (STM32 -> 上位机)
 */
#define CAN_ID_TX_ANGLE         0x123           /* 角度数据 ID */
#define CAN_ID_TX_ANGULAR_VEL   0x124           /* 角速度数据 ID */
#define CAN_ID_TX_TEMPERATURE   0x125           /* 温度数据 ID */

/**
 * @brief  接收命令 CAN ID (上位机 -> STM32)
 */
#define CAN_ID_RX_COMMAND       0x666           /* 控制命令 ID */

/* ==================== CAN 数据长度定义 ==================== */

#define CAN_DATA_LENGTH_MAX     8               /* CAN 数据最大长度 (字节) */

/* ==================== 接收命令类型定义 ==================== */

/**
 * @brief  上位机命令类型枚举
 * @note   根据实际需求扩展
 */
typedef enum {
    CMD_NONE = 0x00,            /* 无命令 */
    CMD_START = 0x01,           /* 启动命令 */
    CMD_STOP = 0x02,            /* 停止命令 */
    CMD_RESET = 0x03,           /* 复位命令 */
    CMD_SET_MODE = 0x04,        /* 设置模式命令 */
    CMD_CALIBRATE = 0x05        /* 校准命令 */
} CAN_CommandType_t;

/* ==================== CAN 接收数据结构体 ==================== */

/**
 * @brief  CAN 接收数据结构体
 * @note   存储从上位机接收到的命令和数据
 */
typedef struct {
    uint32_t id;                        /* 接收到的 CAN ID */
    uint8_t data[CAN_DATA_LENGTH_MAX];  /* 接收数据缓冲区 */
    uint8_t length;                     /* 数据长度 */
    uint8_t newDataFlag;                /* 新数据标志 (1=有新数据) */
} CAN_RxMessage_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化 CAN 外设
 * @note   配置 CAN 为 1Mbps，Normal 模式，启用自动重传
 * @retval HAL_StatusTypeDef
 *         - HAL_OK: 初始化成功
 *         - HAL_ERROR: 初始化失败
 */
HAL_StatusTypeDef CAN_Comm_Init(void);

/**
 * @brief  配置 CAN 过滤器
 * @note   设置过滤器只接收指定 ID 的消息
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_FilterConfig(void);

/**
 * @brief  启动 CAN 通信
 * @note   启动 CAN 外设并使能接收中断
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_Start(void);

/**
 * @brief  发送角度数据
 * @param  angle: 角度值 (可以是 int16_t 或 float，根据协议定义)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendAngle(int16_t angle);

/**
 * @brief  发送角速度数据
 * @param  angularVel: 角速度值
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendAngularVelocity(int16_t angularVel);

/**
 * @brief  发送温度数据
 * @param  temperature: 温度值 (单位: 0.1°C)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendTemperature(int16_t temperature);

/**
 * @brief  发送通用 CAN 消息
 * @param  canId: CAN 标识符
 * @param  pData: 数据指针
 * @param  length: 数据长度 (1-8 字节)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_Transmit(uint32_t canId, uint8_t *pData, uint8_t length);

/**
 * @brief  获取接收到的消息
 * @param  pRxMsg: 接收消息结构体指针
 * @retval uint8_t 1=有新消息, 0=无新消息
 */
uint8_t CAN_Comm_GetReceivedMessage(CAN_RxMessage_t *pRxMsg);

/**
 * @brief  解析接收到的命令
 * @param  pRxMsg: 接收消息结构体指针
 * @retval CAN_CommandType_t 命令类型
 */
CAN_CommandType_t CAN_Comm_ParseCommand(CAN_RxMessage_t *pRxMsg);

/**
 * @brief  获取 CAN 句柄指针
 * @retval CAN_HandleTypeDef* CAN 句柄指针
 */
CAN_HandleTypeDef* CAN_Comm_GetHandle(void);

/**
 * @brief  CAN 接收中断回调函数 (在中断中调用)
 * @note   此函数由 HAL 库自动调用，用于处理接收到的消息
 */
void CAN_Comm_RxCallback(void);

#endif /* CAN_COMM_H */
