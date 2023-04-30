#include "CAN2_SEND_RM.h"
#include "MY_CHASSIS_CONTROL.h"

uint8_t js_SEND_all[33];  // 4x8=32

void SPEED_CHANGE_SEND_control()
{
//	for(int i=0;i<7;i++ )//0到6位有效,一共7位
//{
//	js_SEND_all[i]=ext_shoot_data.data.dataBuff[i];
//}
	CAN_SendData(&hcan2,CAN_ID_STD,SPEED_CHANGE_SEND_ID,&js_SEND_all[0]);

}

void JS_send_SHOOT_control()
{
	for(int i=0;i<7;i++ )//0到6位有效,一共7位
{
	js_SEND_all[i]=ext_shoot_data.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_SHOOT_ID,&js_SEND_all[0]);

}
void JS_send_HURT_control()
{

	js_SEND_all[0]=ext_robot_hurt.data.dataBuff[0];

	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HURT_ID,&js_SEND_all[0]);

}

void JS_send_STATUS_control()
{

	for(int i=0;i<27;i++ )//0到26位有效,一共27位
{
	js_SEND_all[i]=ext_game_robot_state.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_ONE,&js_SEND_all[0]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_TWO,&js_SEND_all[8]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_THREE,&js_SEND_all[16]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_FOUR,&js_SEND_all[24]);
osDelay(1);

}

void JS_send_HEAT_control()
{

	for(int i=0;i<18;i++ )//0到17位有效,一共18位
{
	js_SEND_all[i]=ext_power_heat_data.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HEAT_ID_ONE,&js_SEND_all[0]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HEAT_ID_TWO,&js_SEND_all[8]);

}

void PLACE_send_control()
{

if(in_END==1)//不在轨道中间段,在末端
{
//	//先右后左
//	if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
//	{
//	js_SEND_all[0]=1;
//	js_SEND_all[7]=0;
//	
//	}
//	if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
//	{
//	js_SEND_all[0]=0;
//	js_SEND_all[7]=1;
//	}

}
else//在轨道中间段
{
	js_SEND_all[0]=0;
	js_SEND_all[7]=0;

}

	js_SEND_all[1]=0;
	js_SEND_all[2]=0;

if(in_MID==1)//在轨道中间段
{
	js_SEND_all[3]=1;
	js_SEND_all[4]=1;
}
else//不在轨道中间段
{
	js_SEND_all[3]=0;
	js_SEND_all[4]=0;	
}

	js_SEND_all[5]=0;
	js_SEND_all[6]=0;


	
	
//	CAN_SendData(&hcan2,CAN_ID_STD,PLACE_SEND_ID,&js_SEND_all[0]);


}

