#include "DR16_RECIVE.h"
#include "stdint.h"
#include "usbd_cdc_if.h"

#include "usart.h"
#include "my_positionPID_bate.h"
//#include "GM6020.h"
//#include "DJI_C_IMU.h"
#include "M3508.h"
//#include "MY_CHASSIS_CONTROL.h"
#include "my_IncrementPID_bate.h"
#include "Vision.h"
#include "MY_CLOUD_CONTROL.h"
#include "calibrate_task.h"
#include "MY_SHOOT_CONTROL.h"

#include "bmi088driver.h"
#include "spinning_top_examine.h"
#include "Vision_Control.h"

//#include "GM6020_Motor.h"
//#include "control.h"

static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
ext_shoot_data_t ext_shoot_data;
ext_robot_hurt_t ext_robot_hurt;     
ext_game_robot_status_t ext_game_robot_state;//状态  剩余血量
ext_power_heat_data_t ext_power_heat_data;//能量 热量
ext_game_robot_HP_t ext_game_robot_HP;//场上所有机器人的血量
ext_game_status_t      ext_game_status;//比赛阶段  准备 倒计时 开始......
uint8_t CHASSIS_place[8];
uint8_t JSBuffer[8];

//uint8_t DR16Buffer[DR16BufferNumber];
uint8_t DR16Buffer[22];

DR16_t DR16 = DR16_GroundInit;


uint8_t testdatatosend[50];//匿名上位机发送数据

void usart1_dr16_init(void)
{
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
//HAL_DMA_Start(huart1,(uint32_t)&USART1->DR,(uint32_t)DR16Buffer,DR16BufferNumber);
	
//		/*清空标志位然后使能USART的中断*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE(&huart3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
//	/*开启DMA传输（但是不开启DMA中断）*/
	USART_Receive_DMA_NO_IT(&huart3,DR16Buffer,DR16BufferNumber);
}


//		&huart1
void DR_16hander(UART_HandleTypeDef *huart)
	
{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
	   __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		//if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
		{
			DR16.DR16_Process(DR16Buffer);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	
	
	
	
	
	
}


void DR16_Process(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.rc.ch0 -= 1024;

	DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
		DR16.rc.ch1 -= 1024;

	DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
		DR16.rc.ch2 -= 1024;

	DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
		DR16.rc.ch3 -= 1024;
	DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
	DR16.mouse.x = (pData[6]) | (pData[7] << 8);
	DR16.mouse.y = (pData[8]) | (pData[9] << 8);
	DR16.mouse.z = (pData[10]) | (pData[11] << 8);
	DR16.mouse.keyLeft = pData[12];
	DR16.mouse.keyRight = pData[13];
	DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

	//your control code ….
	DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
		DR16.rc.ch4_DW -= 1024;
	ch4_DW_total_2+=DR16.rc.ch4_DW;
	DR16.infoUpdateFrame++;


	/* prevent remote control zero deviation */
	if (DR16.rc.ch0 <= 5 && DR16.rc.ch0 >= -5)
		DR16.rc.ch0 = 0;
	if (DR16.rc.ch1 <= 5 && DR16.rc.ch1 >= -5)
		DR16.rc.ch1 = 0;
	if (DR16.rc.ch2 <= 20 && DR16.rc.ch2 >= -20)
		DR16.rc.ch2 = 0;
	if (DR16.rc.ch3 <= 20 && DR16.rc.ch3 >= -20)
		DR16.rc.ch3 = 0;
	if (DR16.rc.ch4_DW <= 20 && DR16.rc.ch4_DW >= -20)
		DR16.rc.ch4_DW = 0;
	
	
	CH0_TOTAL+=DR16.rc.ch0;
	CH1_TOTAL+=DR16.rc.ch1;
    CH2_TOTAL+=DR16.rc.ch2;
	CH3_TOTAL+=DR16.rc.ch3;
//				if(DR16.rc.s_left==3)
////				targe_angle=-20000;
//				if(DR16.rc.s_left==1)
////				targe_angle=20000;
//				if(DR16.rc.s_left==2)
//				targe_angle+=(DR16.rc.ch0/660.0)*300;

//				mubiaosudu3=(DR16.rc.ch1/660.0)*300;

//		if (DR16.rc.ch3 >600 )
//targe_angle=20000;
//		if (DR16.rc.ch3 <-600 )
//targe_angle=-20000;
					

	
//targe_angle+=(DR16.rc.ch3/660.0)*300;
//	DR16_Export_data.ControlSwitch.Left = (RemotePole_e)DR16.rc.s_left;
//	DR16_Export_data.ControlSwitch.Right = (RemotePole_e)DR16.rc.s_right;
//	
///* 	RemoteMode_Update();//对控制来源、运动模式进行更新。*/
//	RemoteControl_Update();//计算机器人运动目标值。 

			}



/**
  * @Data    2019-02-19 15:46
  * @brief   USART_DMA接收开启和重定向
  * @param   void
  * @retval  void
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

		/*检测当前huart状态*/
		if(huart->RxState == HAL_UART_STATE_READY)
		{
			/*输入的地址或者数据有问题的话*/
			if((pData == NULL) || (Size == 0))
			{
					return HAL_ERROR;
			}
			
			/*huart里面对应的Rx变量重定向*/
			huart->pRxBuffPtr = pData;
			huart->RxXferSize = Size;
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			/*开启huart1上的RX_DMA*/
			HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
			
			/*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			
			
		}
		else
		{
			return HAL_BUSY;
		}

		return HAL_OK;
}

int16_t data1=1;
int16_t data2=2;
int32_t data3=3;
//int16_t 
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
int32_t data4=4;

uint8_t i=0;
uint8_t p=0; 
int32_t send_d_32[8];
int16_t send_d_16[3];

int32_t send_data1=1;
int32_t send_data2=2;

int32_t send_data3=3;
int32_t send_data4=4;
int32_t send_data5=5;

int16_t send_data6=6;
int16_t send_data7=7;

int32_t send_data8=8;
int32_t send_data9=9;
int32_t send_data10=10;

int16_t send_data11=11;

/*需求：目标角度，当前角度-int32_t
位置环P_OUT,I_OUT,D_OU-Tint32_t
速度环P_OUT,I_OUT,D_OUT-int32_t

目标速度，当前速度-int16_t
输出给电机的值-int16_t
           28  6
数据长度：4*7+2*3=38
*/
void NM_swj(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF1;
	testdatatosend[_cnt++]=34;
	if(1)
	{
			#if 0//发送陀螺仪数据  YAW PITCH
	p=0;
			send_d_32[p++]=tire_L_TARGE_speed;//当前角度		1
			send_d_32[p++]=M3508s[3].realSpeed;//最终目标角度		2

			send_d_32[p++]=send_to_tire_L;//视觉数据		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= tire_L_TARGE_speed_FAKE;//I_OUT 4		4PID_YES

			send_d_32[p++]=tire_R_TARGE_speed_FAKE;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-10;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*-100;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//输出电压      8

			send_d_16[p++]=yaw_trage_speed*100000;//目标角度       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1在校准 0不在		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
			#if 1//发送陀螺仪数据  YAW PITCH
	p=0;
			send_d_32[p++]=tire_L_TARGE_speed;//当前角度		1
			send_d_32[p++]=M3508s[3].realSpeed;//最终目标角度		2

			send_d_32[p++]=-tire_R_TARGE_speed;//视觉数据		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= -M3508s[2].realSpeed;//I_OUT 4		4PID_YES

			send_d_32[p++]=tire_R_TARGE_speed_FAKE;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-10;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*-100;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//输出电压      8

			send_d_16[p++]=yaw_trage_speed*100000;//目标角度       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1在校准 0不在		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
}



for(p=0;p<7;p++)
	{
testdatatosend[_cnt++]=BYTE0(send_d_32[p]);
testdatatosend[_cnt++]=BYTE1(send_d_32[p]);
testdatatosend[_cnt++]=BYTE2(send_d_32[p]);
testdatatosend[_cnt++]=BYTE3(send_d_32[p]);
	}	
	for(p=0;p<3;p++)
	{
testdatatosend[_cnt++]=BYTE0(send_d_16[p]);
testdatatosend[_cnt++]=BYTE1(send_d_16[p]);
	}
	
	
sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

//	HAL_UART_Transmit_DMA(&huart1,&testdatatosend[0],_cnt);//4pin

	CDC_Transmit_FS(&testdatatosend[0],_cnt);

//		for (uint8_t i = 0; i < _cnt; i++)
//	{
//		while ((USART6->SR & 0X40) == 0);
//		USART6->DR = testdatatosend[i];
//	}//三PIN接口
}


void NM_swj2(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF2;
	testdatatosend[_cnt++]=10;
//	testdatatosend[_cnt++]=0;
//	
//	testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//			testdatatosend[_cnt++]=BYTE1(mubiaosudu3);

//	testdatatosend[_cnt++]=BYTE0(my_6020array[1].realSpeed);
//		testdatatosend[_cnt++]=BYTE1(my_6020array[1].realSpeed);
	//位置环参数
//	data3=GM_6020_speed.p_out;
//	data4=GM_6020_speed.i_out;
	
//testdatatosend[_cnt++]=BYTE0(targe_angle);
//testdatatosend[_cnt++]=BYTE1(targe_angle);

testdatatosend[_cnt++]=BYTE0(data1);
testdatatosend[_cnt++]=BYTE1(data1);//真实速度

testdatatosend[_cnt++]=BYTE0(data3);
testdatatosend[_cnt++]=BYTE1(data3);
testdatatosend[_cnt++]=BYTE2(data3);
testdatatosend[_cnt++]=BYTE3(data3);//speed p——out

testdatatosend[_cnt++]=BYTE0(data4);
testdatatosend[_cnt++]=BYTE1(data4);
testdatatosend[_cnt++]=BYTE2(data4);
testdatatosend[_cnt++]=BYTE3(data4);//speed i——out
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//testdatatosend[_cnt++]=BYTE1(mubiaosudu3);//输出



//	testdatatosend[_cnt++]=BYTE0(data3);
//testdatatosend[_cnt++]=BYTE1(data3);
//testdatatosend[_cnt++]=BYTE2(data3);
//testdatatosend[_cnt++]=BYTE3(data3);//目标角度
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.d_out);
//	testdatatosend[_cnt++]=BYTE3(mubiaosudu3);
//	

sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

//	HAL_UART_Transmit_DMA(&huart6,&testdatatosend[0],_cnt);


}





