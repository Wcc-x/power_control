 
#include "can_drv.h"
#include "stm32g0xx_hal_fdcan.h"
#include <string.h>
#include "stm32g0xx_hal_conf.h"
#include "main.h"
//定义adc转换
#define V_BUS_THRESHOLD     2580    // IN0 母线 >21V
#define V_BUS_UNDER_THRESHOLD  2460 // IN0 母线 <21V
#define V_BLD_THRESHOLD     1861    // IN1 泄放 >1.5V
#define V_BLD_UNDER_THRESHOLD  1700 // IN1 泄放 <1.5V
//ADC全局标志
uint8_t ADC_BusOver21V_Flag = 0;
uint8_t ADC_BldOver15V_Flag = 0;
uint8_t ADC_Bus21V_Keep3s_OK = 0;


// 四个电机的数据
motor_data_t motor[4];
uint32_t total_voltage;//四个电机电压总和，具体是什么可以自己改
uint32_t measure_target_data;//这个是一定值，具体是什么自己填写

volatile uint16_t top = 0;
volatile uint16_t tail = 0;
volatile can_pack FDCAN_RX_FIFO[FIFO_LENGTH];

// ADC转换通道索引
static uint8_t adc_channel_index = 0;

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
        memcpy(data, (uint8_t*)FDCAN_RX_FIFO[top].data, 8);
        top = (top + 1) % FIFO_LENGTH;
        return 1;
    }
    else{
        return 0;
    }
}

/**
  * @brief FDCAN Rx FIFO 0新消息回调,同时计算电压总和
  * @param hfdcan: FDCAN句柄
  * @param RxFifo0ITs: 中断类型
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_Receiver_IQRHandler(hfdcan);
        uint32_t canid;
        uint8_t data[8];
        while (FDCAN_GetMessage(&canid, data))
        {
            // 根据CANID区分电机
            switch(canid)
            {
                case 0x101:  // 电机1
                    motor[0].voltage = (data[0] << 8) | data[1];
                    motor[0].current = (data[2] << 8) | data[3];
                    motor[0].temperature = data[4];
                    motor[0].status = data[5];
                    break;
                    
                case 0x102:  // 电机2
                    motor[1].voltage = (data[0] << 8) | data[1];
                    motor[1].current = (data[2] << 8) | data[3];
                    motor[1].temperature = data[4];
                    motor[1].status = data[5];
                    break;
                    
                case 0x103:  // 电机3
                    motor[2].voltage = (data[0] << 8) | data[1];
                    motor[2].current = (data[2] << 8) | data[3];
                    motor[2].temperature = data[4];
                    motor[2].status = data[5];
                    break;
                    
                case 0x104:  // 电机4
                    motor[3].voltage = (data[0] << 8) | data[1];
                    motor[3].current = (data[2] << 8) | data[3];
                    motor[3].temperature = data[4];
                    motor[3].status = data[5];
                    break;
            }
        }
        total_voltage = motor[0].voltage + motor[1].voltage + motor[2].voltage + motor[3].voltage;
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







// 这是你已经写好的回调，中断完成后会自动进入这里

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) // 多ADC时要判断
    {
        uint32_t channel = HAL_ADC_GetCurrentChannel(hadc);
        uint32_t val = HAL_ADC_GetValue(hadc);
        
        // 你的通道判断和逻辑
        //channel0是母线电压，1是实际电压
        if (channel == ADC_CHANNEL_0) {
            if(val >= V_BUS_THRESHOLD){
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            }
            else if(val < V_BUS_UNDER_THRESHOLD){
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            }
        }
        else if (channel == ADC_CHANNEL_1) {
            
            // 处理BLD电压
        }
    }
}
    