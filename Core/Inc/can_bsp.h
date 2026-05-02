#include "main.h"
#include "stm32g0xx_hal.h"
#include <stdint.h>
typedef struct
{
    uint32_t id;          // CAN message ID
    uint8_t data[8];      // CAN message data (up to 8 bytes)
    uint8_t length;       // Length of the data (0-8)
} CAN_Message;
void CAN_PARSE(void)
{

}