#pragma once
#include "can_drv.h"
#include "stm32g0xx_hal_fdcan.h"
#include <string.h>
#include "stm32g0xx_hal_conf.h"

volatile uint16_t top = 0;
volatile uint16_t tail = 0;
volatile can_pack FDCAN_RX_FIFO[FIFO_LENGTH];

void FDCAN_Filter_Init(FDCAN_HandleTypeDef* hfdcan){
    FDCAN_FilterTypeDef can_filter_init_structure;

    // 配置fifo0全通滤波器
    can_filter_init_structure.IdType = FDCAN_STANDARD_ID;
    can_filter_init_structure.FilterIndex = 0;
    can_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
    can_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_init_structure.FilterID1 = 0x00000000;
    can_filter_init_structure.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &can_filter_init_structure);

    // 全局滤波器, 直接拒绝不符合规则的标准数据帧, 扩展数据帧, 标准遥控帧, 扩展遥控帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

void FDCAN_Init(FDCAN_HandleTypeDef* hfdcan){
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void FDCAN_Receiver_IQRHandler(FDCAN_HandleTypeDef* hfdcan){
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    can_pack dataPack;
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
    if (RxHeader.DataLength == FDCAN_DLC_BYTES_8){
        dataPack.canid = RxHeader.Identifier;
        memcpy(dataPack.data, RxData, 8);
        FDCAN_RX_FIFO[tail] = dataPack;
        FDCAN_RX_FIFO[tail] = dataPack;           // 写入当前尾部
    tail = (tail + 1) % FIFO_LENGTH;          // 尾部指针后移
    if (tail == top) {                        // 如果缓冲区已满
    top = (top + 1) % FIFO_LENGTH;        // 覆盖最旧的数据（丢弃一帧）
    }
    }
}

uint8_t FDCAN_GetMessage(uint32_t* CANID, uint8_t* data){
    if (top != tail){
        *CANID = FDCAN_RX_FIFO[top].canid;
        memcpy(data, FDCAN_RX_FIFO[top].data, 8);
        top = (top + 1) % FIFO_LENGTH;
        return 1;
    }else{
        return 0;
    }
}

/**
  * @brief FDCAN Rx FIFO 0新消息回调
  * @param hfdcan: FDCAN句柄
  * @param RxFifo0ITs: 中断类型
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_Receiver_IQRHandler(hfdcan);
    }
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0)
    {
        // FIFO0满中断处理（可选）
        // __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_FF0);
    }
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
    {
        // 消息丢失中断处理（可选）
        // 可以在此添加错误计数或警告标志
    }
}

/**
  * @brief FDCAN Rx FIFO 1新消息回调
  * @param hfdcan: FDCAN句柄
  * @param RxFifo1ITs: 中断类型
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    
    // ADC转换完成回调函数
    // 可以在此处理ADC采样数据，例如读取ADC值并进行相应的处理
}



