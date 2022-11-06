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
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//当前角度		1
			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//最终目标角度		2

			send_d_32[p++]=0;//视觉数据		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= PITCH_trage_angle*-50000;//I_OUT 4		4PID_YES

			send_d_32[p++]=VisionData.RawData.Depth*10;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-50000;//I_OUT		6
			send_d_32[p++]=VisionData.RawData.Beat*1111;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//输出电压      8

			send_d_16[p++]=yaw_trage_speed*100000;//目标角度       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1在校准 0不在		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
		#if 0//发送DR16数据
	p=0;
			send_d_32[p++]=yaw_trage_speed*1000;//目标角度		1
			send_d_32[p++]=PITCH_MAX_angle*1000;//当前角度		2

			send_d_32[p++]=PITCH_trage_angle*1000;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= PITCH_MIN_angle*1000;//I_OUT 4		4PID_YES
			send_d_32[p++]=allow_angle*1000;//P_OUT		5
			send_d_32[p++]=DR16.rc.ch1;//I_OUT		6
			send_d_32[p++]=DR16.rc.ch2;//D_OUT  	7
	p=0;
			send_d_16[p++]=DR16.rc.ch3;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//输出电压		10
														//保留到小数点后四位
#endif	

		#if 0//发送DR16数据
	p=0;
			send_d_32[p++]=CH0_TOTAL;//目标角度		1
			send_d_32[p++]=CH1_TOTAL;//当前角度		2

			send_d_32[p++]=CH2_TOTAL;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CH3_TOTAL;//I_OUT 4		4PID_YES
			send_d_32[p++]=DR16.rc.ch0;//P_OUT		5
			send_d_32[p++]=DR16.rc.ch1;//I_OUT		6
			send_d_32[p++]=DR16.rc.ch2;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//输出电压		10
														//保留到小数点后四位
#endif
		#if 0//确定正负号
	//首先,用手逆时针旋转云台,看看陀螺仪的值是增大还是减小   阶跃是在现有基础上增加的话
	//误差值是目标值减当前值,阶跃后误差值应为正
	//用手逆时针旋转云台,看看速度值是否为正
	//两种情况:速度值为正,则说明PID过程中不需要变号,即角度环的Kp Ki为正
	//为负,则说明PID过程中需要变号,即角度环的Kp Ki为负
	//......
	//
	//化简这个流程
	//
	//观测到的现实物理量(参与PID计算的):
	//陀螺仪的角度值   陀螺仪的速度值  电机控制值的正反决定的正反
	//
	//如果朝同一方向转动 陀螺仪的角度值增大,同时陀螺仪的速度值也为正(物理上)
	//那么说明在误差值为正的时候             需要算出的目标速度也应该为正(程序上)
	//                                		                即角度环Kp,Ki,Kd应为正
	//
	//反之                                       陀螺仪的速度值为负(物理上)
	//说明在误差值为正的时候,              需要算出一个负的速度目标值(程序上)
	//                                		                即角度环Kp,Ki,Kd应为负
	//开环控制,给一个正1000~3000的电流值
	//如果陀螺仪的速度显示为正,则速度环Kp,Ki,Kd应为正
	//如果陀螺仪的速度显示为负,则速度环Kp,Ki,Kd应为负
	
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//目标角度		1
			send_d_32[p++]=yaw_trage_angle*1000;//当前角度		2
	
	//从上往下看的逆时针,陀螺仪值增大,误差值为正:目标-当前
	
	//从上往下看的逆时针旋转,陀螺仪速度值为正,
	
	
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Yaw_IMU_Angle_pid.result;//I_OUT 4		4PID_YES

			send_d_32[p++]=Yaw_IMU_Speed_pid.Measure;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//输出电压		10
														//保留到小数点后四位
#endif
	#if 0//发送云台数据  YAW 视觉
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//当前角度		1
			send_d_32[p++]=yaw_trage_angle*1000;//最终目标角度		2

			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//视觉数据		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*10000;//I_OUT 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Measure*10000;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-50000;//I_OUT		6
			send_d_32[p++]=VisionData.RawData.Beat*1111;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//输出电压      8

			send_d_16[p++]=yaw_trage_speed*100000;//目标角度       	9
			send_d_16[p++]=send_to_yaw*1111;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
	#if 0//发送陀螺仪温度数据 YAW 陀螺仪
	p=0;
			send_d_32[p++]=Yaw_IMU_Speed_pid.Target*10000;//当前角度		1
			send_d_32[p++]=DJIC_IMU.total_yaw*10000;//最终目标角度		2

			send_d_32[p++]=Yaw_IMU_Speed_pid.Measure*10000;//视觉数据		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= Yaw_IMU_Speed_pid.Error*1000;//I_OUT 4		4PID_YES

			send_d_32[p++]=Vision_RawData_Yaw_Angle*10000;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Yaw_IMU_Angle_pid.result;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=TEMPERATURE_PID_OUT;//输出电压      8

			send_d_16[p++]=bmi088_real_data.temp*10;//目标角度       	9
			send_d_16[p++]=send_to_yaw;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
#if USE_MOTOR_angle==0  //PITCH使用电机角度

//发送云台数据 YAW 陀螺仪 666
	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//目标角度		1
			send_d_32[p++]=GM6020s[3].totalAngle;//当前角度		2

			send_d_32[p++]=DJIC_IMU.total_pitch*1000000;//视觉数据		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= send_to_pitch;//发 4		4PID_YES

			send_d_32[p++]=GM6020s[3].realCurrent;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=DR16.rc.s_left;//输出电压      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*热量 角度误差允许 视觉发射指令是连续 不在轨道末端 所有条件全部满足*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta
#endif

	#if 0//发送云台数据  PITCH 陀螺仪
//先确定哪个轴是PITCH轴(陀螺仪有三个轴)
//保证水平时值为0；向上为正，抬头为负
//
//然后就确定上下机械限位,这个是读GM6020的值，读完以后填到动态限位函数
//用手动一动，看看边界值是否正确
//然后就可以正式开始调参了，先把目标角度设为0
//
//向上的时候速度为正,到不了0°!
//角度环参数为正
//正值正转,
//速度环参数为正
p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//目标角度		1
			send_d_32[p++]=PITCH_IMU_Angle_pid.Measure*10000;//当前角度		2

			send_d_32[p++]=PITCH_IMU_Angle_pid.Error*10000;//P_OUT		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Vision_RawData_Pitch_Angle*-10000;//I_OUT 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=PITCH_IMU_Speed_pid.Target*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.Measure*10000;//I_OUT		6
			send_d_32[p++]=PITCH_IMU_Speed_pid.Differential;//D_OUT  	7
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//输出电压      8

			send_d_16[p++]=PITCH_IMU_Angle_pid.Target;//目标角度       	9
			send_d_16[p++]=send_to_pitch;//当前角度		10
														//保留到小数点后四位
#endif
	#if 0//发送云台数据  与视觉 双轴 陀螺仪
//先确定哪个轴是PITCH轴(陀螺仪有三个轴)
//保证水平时值为0；向上为正，抬头为负
//
//然后就确定上下机械限位,这个是读GM6020的值，读完以后填到动态限位函数
//用手动一动，看看边界值是否正确
//然后就可以正式开始调参了，先把目标角度设为0
//
//向上的时候速度为正,到不了0°!
//角度环参数为正
//正值正转,
//速度环参数为正
p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//目标角度		1
			send_d_32[p++]=PITCH_trage_angle*10000;//当前角度		2

			send_d_32[p++]=PITCH_IMU_Angle_pid.Error*10000;//P_OUT		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Vision_RawData_Pitch_Angle*-10000;//I_OUT 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=DJIC_IMU.pitch*10000;//P_OUT		5
			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//I_OUT		6
			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//D_OUT  	7
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//输出电压      8

			send_d_16[p++]=PITCH_IMU_Angle_pid.Target;//目标角度       	9
			send_d_16[p++]=send_to_pitch;//当前角度		10
														//保留到小数点后四位
#endif
#if 0//发送底盘3508数据
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=CHASSIS_trage_angle;//目标角度		1
			send_d_32[p++]=M3508s[3].totalAngle;//当前角度		2

			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.Target;//目标速度		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CHASSIS_MOTOR_SPEED_pid.Measure;//当前速度 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.Proportion;//P_OUT		5
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.I_Output;//I_OUT		6
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.D_Output;//D_OUT  	7
	p=0;
			send_d_16[p++]=CHASSIS_MOTOR_SPEED_pid.result;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//当前角度		10

#endif
#if 0//发送摩擦轮3508数据
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=6700;//目标角度		1
			send_d_32[p++]=-M3508s[2].realSpeed;//当前角度		2

			send_d_32[p++]=0;//目标速度		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= M3508s[3].realSpeed;//左摩擦轮 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//当前角度		10

#endif
#if 0//发送光电数据
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=M3508s[3].realSpeed;//目标角度		1
			send_d_32[p++]=HWswitch_R;//当前角度		2

			send_d_32[p++]=HWswitch_L;//目标速度		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= M3508s[3].totalAngle;//当前速度 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//当前角度		10

#endif


#if 0//发送底盘位置数据
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=CHASSIS_trage_angle;//目标角度		1
			send_d_32[p++]=CHASSIS_L_MAX;//当前角度		2

			send_d_32[p++]=M3508s[3].totalAngle;//目标速度		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CHASSIS_R_MIN;//当前速度 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//输出电压      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//目标速度     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//当前速度		10
			send_d_32[p++]=CHASSIS_MID;//P_OUT		5
			send_d_32[p++]=DEBUFF*100;//I_OUT		6
			send_d_32[p++]=speed_change;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=M3508s[3].realSpeed;//目标角度       	9
			send_d_16[p++]=RANDOM_CHASSIS.number;//随机数		10

#endif

#if 0//发送底盘位置数据
	p=0;
			ENCODER_M_MID=(ENCODER_L_MAX+ENCODER_R_MIN)/2;
			send_d_32[p++]=CHASSIS_L_MAX;//轨道左边界值		1
			send_d_32[p++]=M3508s[3].totalAngle;//在轨位置		2

			send_d_32[p++]=CHASSIS_R_MIN;//轨道右边界值		3 

			send_d_32[p++]=ENCODER_L_MAX;//当前速度 4		4PID_YES

			send_d_32[p++]=Chassis_Encoder.totalLine;//P_OUT		5
			send_d_32[p++]=ENCODER_R_MIN;//I_OUT		6
			send_d_32[p++]=ENCODER_M_MID;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//随机数		10

#endif

#if 0//发送编码器位置数据
	p=0;
			ENCODER_M_MID=(ENCODER_L_MAX+ENCODER_R_MIN)/2;
			send_d_32[p++]=ENCODER_ARRIVE_MAX;//轨道左边界值		1
			send_d_32[p++]=Chassis_Encoder.totalLine;//在轨位置		2

			send_d_32[p++]=ENCODER_ARRIVE_MIN;//轨道右边界值		3 

			send_d_32[p++]=ENCODER_L_MAX;//当前速度 4		4PID_YES

			send_d_32[p++]=Chassis_Encoder.totalLine;//P_OUT		5
			send_d_32[p++]=ENCODER_R_MIN;//I_OUT		6
			send_d_32[p++]=ENCODER_M_MID;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=speed_change_times;//随机数		10

#endif

#if 0//发送拨盘数据
	p=0;
			send_d_32[p++]=Driver_I_PID.Target;//目标速度		1
			send_d_32[p++]=M3508s[1].realSpeed;//当前速度		2

			send_d_32[p++]=M3508s[1].totalAngle;//轨道右边界值		3 

			send_d_32[p++]=M3508s[1].realAngle;//当前速度 4		4PID_YES

			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=Driver_I_PID.Proportion;//输出电压      8

			send_d_16[p++]=Driver_I_PID.Differential;//目标角度       	9
			send_d_16[p++]=Driver_I_PID.result;//随机数		10

#endif

#if 0//发送拨盘数据
send_data10=M3508s[2].realSpeed;
	p=0;
			send_d_32[p++]=0;//目标速度		1
			send_d_32[p++]=send_data10;//当前速度		2

			send_d_32[p++]=0;//轨道右边界值		3 

			send_d_32[p++]=0;//当前速度 4		4PID_YES

			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8

			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//随机数		10

#endif
}
if(1)
{
	#if 1//发送自动开火数据  中
	p=0;

			send_d_32[p++]=M3508s[1].totalAngle;//拨盘当前角度		1
			send_d_32[p++]=Driver_ANGLE_pid.Target;//拨盘目标角度		2

			send_d_32[p++]=DJIC_IMU.total_yaw*10000;//最终目标角度		2
	
			send_d_32[p++]= -M3508s[3].realSpeed;//左摩擦轮 4		4PID_YES
			send_d_32[p++]=ext_shoot_data.data.bullet_speed*10;//拨盘目标角度		5
			send_d_32[p++]=ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;//拨盘当前角度		6
			send_d_32[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//视觉发射指令  	7
	p=0;
	
			send_d_16[p++]=vision_shoot_times;//射击    指令次数      8
			send_d_16[p++]=SHOOT_STOP_time;//停止射击   指令次数       	9
			send_d_16[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//枪口1热量		10

#endif
#if 0//发送摩擦轮数据  中
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=SHOOT_L_speed;//目标角度		1
			send_d_32[p++]=M3508s[2].realSpeed;//当前角度		2

			send_d_32[p++]=-SHOOT_R_speed;//目标速度		3 
			send_d_32[p++]= -M3508s[3].realSpeed;//左摩擦轮 4		4PID_YES
			send_d_32[p++]=Driver_ANGLE_pid.Target;//拨盘目标角度		5
			send_d_32[p++]=ext_game_robot_state.data.shooter_id1_17mm_cooling_rate;//拨盘当前角度		6
			send_d_32[p++]=ext_shoot_data.data.bullet_speed*100;//D_OUT  	7
	p=0;
			send_d_16[p++]=JS_RC_TIMES;//输出电压      8
			send_d_16[p++]=send_to_SHOOT_L;//目标角度       	9
			send_d_16[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//当前角度		10

#endif
#if 0//发送左摩擦轮数据  
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=SHOOT_L_I_PID.Proportion;//目标速度		1
			send_d_32[p++]=SHOOT_L_I_PID.Increment_Output;//当前速度		2

			send_d_32[p++]=SHOOT_L_I_PID.D_Output;//速度误差		3 
			send_d_32[p++]= SHOOT_R_I_PID.Proportion;//有误差时,输出拉满了吗 4		4PID_YES
			send_d_32[p++]=SHOOT_R_I_PID.Increment_Output;//拨盘目标角度		5
			send_d_32[p++]=SHOOT_R_I_PID.D_Output;//拨盘当前角度		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=send_to_SHOOT_L;//输出电压      8
			send_d_16[p++]=SHOOT_L_I_PID.Target;//目标角度       	9
			send_d_16[p++]=SHOOT_L_I_PID.Measure;//当前角度		10

#endif
#if 0//发送左摩擦轮数据  
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//目标角度		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//当前角度		2
			send_d_32[p++]=6700;//目标速度		1
			send_d_32[p++]=SHOOT_L_I_PID.Measure;//当前速度		2

			send_d_32[p++]=SHOOT_L_I_PID.Error;//速度误差		3 
			send_d_32[p++]= send_to_SHOOT_L;//有误差时,输出拉满了吗 4		4PID_YES
			send_d_32[p++]=0;//拨盘目标角度		5
			send_d_32[p++]=Driver_ANGLE_pid.Measure;//拨盘当前角度		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//输出电压      8
			send_d_16[p++]=0;//目标角度       	9
			send_d_16[p++]=0;//当前角度		10

#endif

	#if 0//发送卡尔曼数据 YAW 陀螺仪 666
	p=0;
			send_d_32[p++]=PITCH_Angle_pid.Error*10000;//当前角度		1
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//最终目标角度		2

			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//视觉原始数据		333333333333 

			send_d_32[p++]= PITCH_TRAGET_ANGLE_TEMP*10000;// 4 之前用这个值做目标值

			send_d_32[p++]=PITCH_IMU_Speed_pid.P_Output*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.I_Output*10000;//I_OUT	666666666666
			send_d_32[p++]=PITCH_IMU_Speed_pid.D_Output*10000;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//输出电压      8

			send_d_16[p++]=auto_shoot_condition_show;///*热量 角度误差允许 视觉发射指令是连续 不在轨道末端 所有条件全部满足*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta
#endif
	#if 0//发送卡尔曼数据 YAW 陀螺仪 666
	p=0;
			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//当前角度		1
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//最终目标角度		2

			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//视觉原始数据		333333333333 

			send_d_32[p++]= PITCH_TRAGET_ANGLE_TEMP*10000;// 4 之前用这个值做目标值

			send_d_32[p++]=PITCH_IMU_Speed_pid.P_Output*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.I_Output*10000;//I_OUT	666666666666
			send_d_32[p++]=PITCH_IMU_Speed_pid.D_Output*10000;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//输出电压      8

			send_d_16[p++]=auto_shoot_condition_show;///*热量 角度误差允许 视觉发射指令是连续 不在轨道末端 所有条件全部满足*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta
#endif

#if 0//视觉联调时要看的

	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//PITCH 目标角度		1
			send_d_32[p++]=GM6020s[3].totalAngle;//PITCH   当前角度		2

			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//YAW 角度误差		333333333333 

			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*100;//YAW 速度目标 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Error*100;//YAW 速度误差           P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=VisionData.RawData.Beat*10+VisionData.RawData.Armour;//自瞄,自动开火      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*热量 角度误差允许 视觉发射指令是连续 不在轨道末端 所有条件全部满足*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//输出电压		10
														//保留到小数点后四位558 320 660   bjTlta

#endif
#if 0//调视觉YAW时要看的

	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//目标角度		1
			send_d_32[p++]=GM6020s[3].totalAngle;//当前角度		2

			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//视觉数据		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*100;//发 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Error*100;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 角度换的输出值,看有木有更大
	p=0;
			send_d_16[p++]=VisionData.RawData.Beat*10+VisionData.RawData.Armour;//输出电压      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*热量 角度误差允许 视觉发射指令是连续 不在轨道末端 所有条件全部满足*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//输出电压		10
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

	HAL_UART_Transmit_DMA(&huart1,&testdatatosend[0],_cnt);//4pin

//	CDC_Transmit_FS(&testdatatosend[0],_cnt);

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





