#include "main.h"
#include "stm32g0xx_hal_fdcan.h"
#include <stdint.h>

#define FIFO_LENGTH 128

typedef struct
{   
    uint32_t canid;
    uint8_t data[8];
} can_pack;

void FDCAN_Filter_Init(FDCAN_HandleTypeDef* hfdcan);
void FDCAN_Init(FDCAN_HandleTypeDef* hfdcan);
void FDCAN_Receiver_IQRHandler(FDCAN_HandleTypeDef* hfdcan);

// 回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern volatile uint16_t top;
extern volatile uint16_t tail;
extern volatile can_pack FDCAN_RX_FIFO[FIFO_LENGTH];
uint8_t FDCAN_GetMessage(uint32_t* CANID, uint8_t* data);