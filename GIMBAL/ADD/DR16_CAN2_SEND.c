#include "DR16_CAN2_SEND.h"
#include "DR16_RECIVE.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

//uint8_t DR16_SEND_part_one[9];
//uint8_t DR16_SEND_part_two[9];
//uint8_t DR16_SEND_part_three[9];


#if send_any_time==1
//任何时候都会以固定频率发送
uint8_t DR16_SEND_all[24];  // 3x8=24

void DR16_send_master_control()
{
	
for(int i=0;i<18;i++ )//0到17位有效,一共18位
{
	DR16_SEND_all[i]=DR16Buffer[i];
}

DR16_send_part_one();
osDelay(1);
DR16_send_part_two();
osDelay(1);
DR16_send_part_three();


}


void DR16_send_part_one()
{
	CAN_SendData(&hcan2,CAN_ID_STD,DR16_SEND_PART_ONE,&DR16_SEND_all[0]);



}
void DR16_send_part_two()
{
	CAN_SendData(&hcan2,CAN_ID_STD,DR16_SEND_PART_TWO,&DR16_SEND_all[8]);



}
void DR16_send_part_three()
{
	
	
	CAN_SendData(&hcan2,CAN_ID_STD,DR16_SEND_PART_THREE,&DR16_SEND_all[16]);



}



#endif



