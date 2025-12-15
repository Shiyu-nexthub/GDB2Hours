/**
 * @file    can_comm.c
 * @brief   CAN 通信驱动实现 - 用于与上位机通信
 * @details 实现 CAN 初始化、数据发送、命令接收等功能
 * 
 * CAN 外设说明:
 *   STM32F103 的 CAN 挂载在 APB1 总线上
 *   APB1 时钟 = 32MHz (在 task_main.c 中配置为 64MHz 系统时钟的一半)
 * 
 * 波特率计算:
 *   CAN 波特率 = APB1 / (Prescaler × (BS1 + BS2 + 1))
 *   目标波特率: 1Mbps
 *   APB1 = 32MHz
 *   Prescaler = 4
 *   BS1 = 5, BS2 = 2
 *   波特率 = 32MHz / (4 × (5 + 2 + 1)) = 32MHz / 32 = 1Mbps
 */

#include "can_comm.h"

/* ==================== 私有变量 ==================== */

/**
 * @brief  CAN 句柄
 * @note   HAL 库使用句柄管理外设，所有 CAN 操作都通过此句柄进行
 */
static CAN_HandleTypeDef hcan;

/**
 * @brief  CAN 发送消息头结构体
 * @note   存储发送消息的配置 (ID, 长度等)
 */
static CAN_TxHeaderTypeDef txHeader;

/**
 * @brief  CAN 接收消息头结构体
 */
static CAN_RxHeaderTypeDef rxHeader;

/**
 * @brief  接收数据缓冲区
 */
static CAN_RxMessage_t rxMessage;

/**
 * @brief  发送邮箱变量
 */
static uint32_t txMailbox;

/* ==================== 私有函数声明 ==================== */

static void CAN_Comm_GPIO_Init(void);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief  初始化 CAN 外设
 * @note   配置流程:
 *         1. 初始化 GPIO (PA11-RX, PA12-TX)
 *         2. 配置 CAN 参数 (波特率、模式等)
 *         3. 初始化 CAN 外设
 * @retval HAL_StatusTypeDef
 * 
 * CAN 波特率计算详解:
 *   STM32F103 CAN 时钟来源: APB1 (32MHz)
 *   
 *   波特率公式:
 *   BaudRate = APB1_CLK / (Prescaler × (SJW + BS1 + BS2))
 *   
 *   本配置:
 *     Prescaler = 4
 *     SJW = 1 (同步跳转宽度)
 *     BS1 = 5 (时间段1)
 *     BS2 = 2 (时间段2)
 *     
 *   波特率 = 32MHz / (4 × (1 + 5 + 2)) = 32MHz / 32 = 1Mbps
 *   
 *   采样点位置 = (1 + BS1) / (1 + BS1 + BS2) = 6/8 = 75%
 */
HAL_StatusTypeDef CAN_Comm_Init(void)
{
    HAL_StatusTypeDef status;
    
    /* ========== 步骤1: 初始化 CAN 相关 GPIO ========== */
    CAN_Comm_GPIO_Init();
    
    /* ========== 步骤2: 配置 CAN 参数 ========== */
    
    /* 选择 CAN1 外设 (STM32F103C8T6 只有 CAN1) */
    hcan.Instance = CAN1;
    
    /* 波特率预分频器: 4 */
    hcan.Init.Prescaler = 4;
    
    /* 工作模式: 正常模式 */
    /* 可选: CAN_MODE_NORMAL (正常), CAN_MODE_LOOPBACK (回环测试) */
    hcan.Init.Mode = CAN_MODE_NORMAL;
    
    /* 同步跳转宽度 (SJW): 1 个时间单位 */
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    
    /* 时间段1 (BS1): 5 个时间单位 */
    hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
    
    /* 时间段2 (BS2): 2 个时间单位 */
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    
    /* 时间触发通信模式: 禁用 */
    hcan.Init.TimeTriggeredMode = DISABLE;
    
    /* 自动离线管理: 禁用 */
    /* 启用后，CAN 会在检测到总线错误后自动恢复 */
    hcan.Init.AutoBusOff = DISABLE;
    
    /* 自动唤醒模式: 禁用 */
    hcan.Init.AutoWakeUp = DISABLE;
    
    /* 自动重传: 启用 */
    /* 发送失败时自动重新发送 */
    hcan.Init.AutoRetransmission = ENABLE;
    
    /* 接收 FIFO 锁定模式: 禁用 */
    /* 禁用时，新消息会覆盖旧消息 */
    hcan.Init.ReceiveFifoLocked = DISABLE;
    
    /* 发送 FIFO 优先级: 禁用 (按请求顺序发送) */
    hcan.Init.TransmitFifoPriority = DISABLE;
    
    /* ========== 步骤3: 初始化 CAN 外设 ========== */
    status = HAL_CAN_Init(&hcan);
    
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤4: 配置发送消息头默认值 ========== */
    txHeader.StdId = 0;                     /* 标准 ID (11位) */
    txHeader.ExtId = 0;                     /* 扩展 ID (29位，本例不使用) */
    txHeader.IDE = CAN_ID_STD;              /* 使用标准 ID */
    txHeader.RTR = CAN_RTR_DATA;            /* 数据帧 (非远程帧) */
    txHeader.DLC = 8;                       /* 数据长度: 8 字节 */
    txHeader.TransmitGlobalTime = DISABLE;  /* 禁用时间戳 */
    
    /* 清空接收消息缓冲区 */
    rxMessage.newDataFlag = 0;
    
    return HAL_OK;
}

/**
 * @brief  配置 CAN 过滤器
 * @note   设置过滤器接收指定 ID 的消息
 * @retval HAL_StatusTypeDef
 * 
 * 过滤器配置说明:
 *   本例使用 32 位掩码模式
 *   过滤器设置为接收所有消息 (掩码全为 0)
 *   可根据需要修改为只接收特定 ID
 * 
 * 过滤器模式:
 *   - 标识符列表模式: 只接收指定 ID
 *   - 掩码模式: 根据掩码过滤 ID
 */
HAL_StatusTypeDef CAN_Comm_FilterConfig(void)
{
    CAN_FilterTypeDef filterConfig;
    
    /* ========== 配置过滤器参数 ========== */
    
    /* 过滤器编号: 0 (STM32F103 有 14 个过滤器: 0-13) */
    filterConfig.FilterBank = 0;
    
    /* 过滤器模式: 掩码模式 */
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    
    /* 过滤器位宽: 32 位 */
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    
    /* 过滤器 ID: 接收命令 ID (0x666) */
    /* 左移 5 位是因为标准 ID 在 CAN 寄存器中的位置 */
    filterConfig.FilterIdHigh = (CAN_ID_RX_COMMAND << 5) & 0xFFFF;
    filterConfig.FilterIdLow = 0x0000;
    
    /* 过滤器掩码: 全 0 表示接收所有 ID (不过滤) */
    /* 如果只想接收 0x666，将掩码设为 0x7FF << 5 */
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    
    /* 关联到 FIFO0 */
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    
    /* 启用过滤器 */
    filterConfig.FilterActivation = ENABLE;
    
    /* 从过滤器 0 开始 (用于单 CAN 控制器) */
    filterConfig.SlaveStartFilterBank = 14;
    
    /* ========== 应用过滤器配置 ========== */
    return HAL_CAN_ConfigFilter(&hcan, &filterConfig);
}

/**
 * @brief  启动 CAN 通信
 * @note   启动 CAN 外设并使能接收中断
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_Start(void)
{
    HAL_StatusTypeDef status;
    
    /* ========== 步骤1: 启动 CAN 外设 ========== */
    status = HAL_CAN_Start(&hcan);
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* ========== 步骤2: 使能 FIFO0 消息挂起中断 ========== */
    /* 当 FIFO0 收到消息时触发中断 */
    status = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    return status;
}

/**
 * @brief  发送角度数据
 * @param  angle: 角度值 (单位根据协议定义，如 0.01°)
 * @retval HAL_StatusTypeDef
 * 
 * 数据格式 (8 字节):
 *   Byte 0-1: 角度值 (int16_t, 小端序)
 *   Byte 2-7: 保留
 */
HAL_StatusTypeDef CAN_Comm_SendAngle(int16_t angle)
{
    uint8_t txData[8] = {0};
    
    /* 将角度值拆分为两个字节 (小端序) */
    txData[0] = (uint8_t)(angle & 0xFF);        /* 低字节 */
    txData[1] = (uint8_t)((angle >> 8) & 0xFF); /* 高字节 */
    
    return CAN_Comm_Transmit(CAN_ID_TX_ANGLE, txData, 8);
}

/**
 * @brief  发送角速度数据
 * @param  angularVel: 角速度值 (单位根据协议定义，如 0.01°/s)
 * @retval HAL_StatusTypeDef
 * 
 * 数据格式 (8 字节):
 *   Byte 0-1: 角速度值 (int16_t, 小端序)
 *   Byte 2-7: 保留
 */
HAL_StatusTypeDef CAN_Comm_SendAngularVelocity(int16_t angularVel)
{
    uint8_t txData[8] = {0};
    
    /* 将角速度值拆分为两个字节 (小端序) */
    txData[0] = (uint8_t)(angularVel & 0xFF);
    txData[1] = (uint8_t)((angularVel >> 8) & 0xFF);
    
    return CAN_Comm_Transmit(CAN_ID_TX_ANGULAR_VEL, txData, 8);
}

/**
 * @brief  发送温度数据
 * @param  temperature: 温度值 (单位: 0.1°C)
 * @retval HAL_StatusTypeDef
 * 
 * 数据格式 (8 字节):
 *   Byte 0-1: 温度值 (int16_t, 小端序)
 *   Byte 2-7: 保留
 */
HAL_StatusTypeDef CAN_Comm_SendTemperature(int16_t temperature)
{
    uint8_t txData[8] = {0};
    
    /* 将温度值拆分为两个字节 (小端序) */
    txData[0] = (uint8_t)(temperature & 0xFF);
    txData[1] = (uint8_t)((temperature >> 8) & 0xFF);
    
    return CAN_Comm_Transmit(CAN_ID_TX_TEMPERATURE, txData, 8);
}

/**
 * @brief  发送通用 CAN 消息
 * @param  canId: CAN 标识符 (11 位标准 ID)
 * @param  pData: 数据指针
 * @param  length: 数据长度 (1-8 字节)
 * @retval HAL_StatusTypeDef
 * 
 * 发送流程:
 *   1. 检查发送邮箱是否空闲
 *   2. 配置发送消息头 (ID, 长度等)
 *   3. 将消息添加到发送邮箱
 *   4. 等待发送完成 (可选)
 */
HAL_StatusTypeDef CAN_Comm_Transmit(uint32_t canId, uint8_t *pData, uint8_t length)
{
    /* 步骤1: 检查是否有空闲的发送邮箱 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        /* 没有空闲邮箱，返回忙状态 */
        return HAL_BUSY;
    }
    
    /* 步骤2: 配置发送消息头 */
    txHeader.StdId = canId;         /* 设置标准 ID */
    txHeader.DLC = length;          /* 设置数据长度 */
    
    /* 步骤3: 将消息添加到发送邮箱 */
    return HAL_CAN_AddTxMessage(&hcan, &txHeader, pData, &txMailbox);
}

/**
 * @brief  获取接收到的消息
 * @param  pRxMsg: 接收消息结构体指针
 * @retval uint8_t 1=有新消息, 0=无新消息
 * 
 * 使用方法:
 *   CAN_RxMessage_t msg;
 *   if (CAN_Comm_GetReceivedMessage(&msg)) {
 *       // 处理接收到的消息
 *   }
 */
uint8_t CAN_Comm_GetReceivedMessage(CAN_RxMessage_t *pRxMsg)
{
    if (rxMessage.newDataFlag)
    {
        /* 复制接收到的消息 */
        pRxMsg->id = rxMessage.id;
        pRxMsg->length = rxMessage.length;
        for (uint8_t i = 0; i < rxMessage.length; i++)
        {
            pRxMsg->data[i] = rxMessage.data[i];
        }
        
        /* 清除新数据标志 */
        rxMessage.newDataFlag = 0;
        pRxMsg->newDataFlag = 1;
        
        return 1;
    }
    
    return 0;
}

/**
 * @brief  解析接收到的命令
 * @param  pRxMsg: 接收消息结构体指针
 * @retval CAN_CommandType_t 命令类型
 * 
 * 命令格式:
 *   Byte 0: 命令类型
 *   Byte 1-7: 命令参数 (根据命令类型定义)
 */
CAN_CommandType_t CAN_Comm_ParseCommand(CAN_RxMessage_t *pRxMsg)
{
    /* 检查是否是命令 ID */
    if (pRxMsg->id != CAN_ID_RX_COMMAND)
    {
        return CMD_NONE;
    }
    
    /* 检查数据长度 */
    if (pRxMsg->length < 1)
    {
        return CMD_NONE;
    }
    
    /* 返回命令类型 (第一个字节) */
    return (CAN_CommandType_t)pRxMsg->data[0];
}

/**
 * @brief  获取 CAN 句柄指针
 * @retval CAN_HandleTypeDef* CAN 句柄指针
 */
CAN_HandleTypeDef* CAN_Comm_GetHandle(void)
{
    return &hcan;
}

/**
 * @brief  CAN 接收中断回调函数
 * @note   此函数由 HAL_CAN_IRQHandler 调用
 *         当 FIFO0 收到消息时触发
 */
void CAN_Comm_RxCallback(void)
{
    uint8_t rxData[8];
    
    /* 从 FIFO0 读取消息 */
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        /* 存储接收到的消息 */
        rxMessage.id = rxHeader.StdId;
        rxMessage.length = rxHeader.DLC;
        
        for (uint8_t i = 0; i < rxHeader.DLC; i++)
        {
            rxMessage.data[i] = rxData[i];
        }
        
        /* 设置新数据标志 */
        rxMessage.newDataFlag = 1;
    }
}

/**
 * @brief  HAL CAN 接收 FIFO0 消息挂起回调函数
 * @note   HAL 库自动调用此函数
 * @param  hcan: CAN 句柄指针
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* 调用接收处理函数 */
    CAN_Comm_RxCallback();
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  初始化 CAN 相关 GPIO
 * @note   配置 PA11 (CAN_RX) 和 PA12 (CAN_TX)
 * 
 * GPIO 配置说明:
 *   PA11 (CAN_RX): 浮空输入 (接收数据)
 *   PA12 (CAN_TX): 复用推挽输出 (发送数据)
 */
static void CAN_Comm_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ========== 步骤1: 使能 GPIO 和 CAN 时钟 ========== */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    /* ========== 步骤2: 配置 CAN_TX 引脚 (PA12) ========== */
    /* CAN_TX 为数据输出，配置为复用推挽输出 */
    GPIO_InitStruct.Pin = CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* 复用推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速 */
    HAL_GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStruct);
    
    /* ========== 步骤3: 配置 CAN_RX 引脚 (PA11) ========== */
    /* CAN_RX 为数据输入，配置为浮空输入 */
    GPIO_InitStruct.Pin = CAN_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         /* 输入模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 浮空输入 */
    HAL_GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  CAN1 接收中断处理函数
 * @note   需要在 stm32f1xx_it.c 中调用或直接使用此函数
 *         只有在调用 CAN_Comm_Init() 和 CAN_Comm_Start() 后才能启用此中断
 * 
 * 使用方法:
 *   在 stm32f1xx_it.c 中添加:
 *   extern CAN_HandleTypeDef* CAN_Comm_GetHandle(void);
 *   void USB_LP_CAN1_RX0_IRQHandler(void) {
 *       HAL_CAN_IRQHandler(CAN_Comm_GetHandle());
 *   }
 */
/* 注释掉此函数，避免未初始化时触发异常
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}
*/
