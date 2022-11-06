#include "MY_CHASSIS_CONTROL.h"
#include "math.h"
//#include "rng.h"

#define HW_SWITCH_JR 240000

bool CHASSIS_L_MAX_new=0;//左右边界值是否更新
bool CHASSIS_R_MIN_new=0;
bool Random_CHASSIS_CHOOSE=0;//是否选择随机模式
bool Cruise_CHASSIS_CHOOSE=1;//是否选择巡航模式

void CHASSIS_CONTROUL(void)
{
	#if PID_CHASSIS_MOTOR
			#if 0	//底盘随机与巡航运动
					if(DR16.rc.s_left==3)//自动控制
					{
						

						
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//只有当边界值更新完了才会  真正开始巡航	
				{
				if(DR16.rc.ch4_DW<=-400)//拨上
				{
					
				Random_CHASSIS_CHOOSE=1;//是选择随机模式
				Cruise_CHASSIS_CHOOSE=0;
				}
				
				if(DR16.rc.ch4_DW>=400)//拨下
				{
				Cruise_CHASSIS_CHOOSE=1;//是选择巡航模式
				Random_CHASSIS_CHOOSE=0;
					
				}
//				if(Cruise_CHASSIS_CHOOSE==1)//是选择巡航模式
//				Cruise_CHASSIS();//巡航模式
//				if(Random_CHASSIS_CHOOSE==1)//是选择巡航模式
//				Random_CHASSIS();//随机模式
				
				CHASSIS_trage_speed=0;//锁死
					
				}
				else
				{
					if( HWswitch_L==0)// 左光电感应到了，向右运动
					CHASSIS_trage_angle=-9990000;
					else if(HWswitch_R==0)//	右光电感应到了，向左运动
					CHASSIS_trage_angle=9990000;
				
					P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle

					CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//双环
//					else 				//默认向左运动
//					CHASSIS_trage_angle=990000;		
				}

					}
//			CHASSIS_MOTOR_ANGLE_pid.Max_result=1200;
			#endif
//					if(DR16.rc.s_left==1)//遥控器控制  左上
//					{
//					CHASSIS_trage_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*CHASSIS_MAX_SPEED;//遥控器给速度目标值 二选一		
//					}
//					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
//					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed,M3508s[3].realSpeed);
//			send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;
#endif
	
//	if(Chassis_Encoder.totalLine>ENCODER_ARRIVE_MAX)
//		ENCODER_ARRIVE_MAX=Chassis_Encoder.totalLine;
//	if(Chassis_Encoder.totalLine<ENCODER_ARRIVE_MIN)
//		ENCODER_ARRIVE_MIN=Chassis_Encoder.totalLine;
	
}

void switch_change(void)
{
//			HWswitch_L			 = HAL_GPIO_ReadPin(GPIOA,HWswitch_1_Pin);
//			HWswitch_R   		 = HAL_GPIO_ReadPin(GPIOA,HWswitch_2_Pin);
//			//光电变化检测  函数
//	if(HWswitch_L!=HWswitch_L_last	)
//	{
//			if(HWswitch_L_last==1)//		0<--1
//			{
//				CHASSIS_L_MAX=M3508s[3].totalAngle+HW_SWITCH_JR;
//				ENCODER_L_MAX=Chassis_Encoder.totalLine+8850;
//				CHASSIS_L_MAX_new=1;//边界值已更新
//			}
//		
//	}
//	if(HWswitch_R!=HWswitch_R_last	)
//	{
//			if(HWswitch_R_last==1)//		1-->0
//			{
//				CHASSIS_R_MIN=M3508s[3].totalAngle-HW_SWITCH_JR;
//				ENCODER_R_MIN=Chassis_Encoder.totalLine-8978;
//				CHASSIS_R_MIN_new=1;//边界值已更新

//			}
//		
//	}	
//	
//			HWswitch_L_last		=HWswitch_L;
//			HWswitch_R_last		=HWswitch_R;
//	
	
}


Random_t RANDOM_CHASSIS;

const uint16_t Random_CHANGE_times = 500; //500ms间隔采样
const uint8_t Random_Proportion = 50;      //随机概率占比
const uint16_t Random_CHANGE_speed = 3000;      //再次变向要达到这个速度以上

//随机模式
void Random_CHASSIS(void)
{
//    if (abs(CHASSIS_trage_speed) != 3500)
//    {
//        CHASSIS_trage_speed = 3500;//随机运动的基础速度
//    }//随机运动   初始化速度   以Random_Velocity做变向运动
//    RANDOM_CHASSIS.number = Get_RandomNumbers_Range(0, 100);
//					if(M3508s[3].totalAngle>(CHASSIS_R_MIN+100000)&&M3508s[3].totalAngle<(CHASSIS_L_MAX-100000))//做实验确定多远变向 负十万到正十万

//    RANDOM_CHASSIS.sampling++;
	
	#if 1
	if(	abs(M3508s[3].realSpeed) >Random_CHANGE_speed	)
	    RANDOM_CHASSIS.sampling++;

	#endif
	
//					if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
//				{
//			CHASSIS_trage_speed=3500;
//			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
//				}
//			
//			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//轨道边界变向 负十万到正十万
//			{
//				CHASSIS_trage_speed=-3500;
//			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
//			}
//	
//    if (RANDOM_CHASSIS.sampling == Random_CHANGE_times)
//    {
//        if (RANDOM_CHASSIS.number >= Random_Proportion)//是否变向
//        {
//            CHASSIS_trage_speed = -CHASSIS_trage_speed;
//			speed_change_times++;
//        }
//        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
//    }
//	


	
}


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;

//	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}


void Cruise_CHASSIS(void)//		cruise	巡航
{
						
//			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
//			CHASSIS_trage_angle=990000;
//			
//			
//			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//做实验确定多远变向 负十万到正十万
//			CHASSIS_trage_angle=-990000;
//			
//			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
//			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//双环
//			
//				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
//				//CHASSIS_MID-CHASSIS_R_MIN   一半行程
//				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
//				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//最多减慢百分之70
//				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//现在是中间快，两边慢
//	
	
	
	
//    if (fabs(Chassis.Velocity.temp_Speed) != Cruise_Velocity)
//    {
//        Chassis.Velocity.temp_Speed = Cruise_Velocity;
//    }
}





Encoder_t Chassis_Encoder;


void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
	
}





















