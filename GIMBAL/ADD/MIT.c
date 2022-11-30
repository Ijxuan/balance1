#include "MIT.h"

int MIT_MODE_TEXT=0;
MIT_t text_moto;


void MIT_MODE(uint8_t MODE)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(MODE)
    {
        case CMD_MOTOR_MODE:
            buf[7] = 0xFC;
            break;
        
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        
        default:
        return; /* 直接退出函数 */
    }
CAN_SendData(&hcan2,CAN_ID_STD,TEST_MIT_SLAVE_ID,buf);
}












