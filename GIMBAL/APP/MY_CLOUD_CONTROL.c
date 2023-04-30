#include "MY_CLOUD_CONTROL.h"
#include "bsp_buzzer.h"
#include "Vision_Control.h"
#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"
#include "RM_JudgeSystem.h"

cloud_control_mode cloud_mode;
float YAW_TRAGET_ANGLE_TEMP;
float PITCH_TRAGET_ANGLE_TEMP;
float PITCH_TRAGET_ANGLE_TEMP_EM;

bool simulation_target_yaw_is_stop=0;
int pitch_motor_tagert=5000;

 bool our_outpost_is_live=1;
CHASSIS_KEY key_message;

Encoder_new_t Chassis_Encoder_new;

/*5100-6255  5150-6150*/
int in_end_times=0;

#if USE_PITCH_BC==1
int pitch_bc=0;

#endif
void cloud_control(void)
{
	/**/
		if(key_message.game_state_progress==3)
		key_message.our_outpost_is_live=1;//倒计时阶段确认前哨战存活
		
//	key_message.game_state_progress=ext_game_status.data.game_progress;/*从裁判系统获取真实的比赛状态*/
		/*or*/
	key_message.game_state_progress=4;/*手动给定比赛状态*/
		
		
		
	if(key_message.game_state_progress==4)
	{//比赛阶段为比赛中就会判断前哨站是否存活
	
	 if(ext_game_robot_HP.data.blue_outpost_HP>0 )
 {
	key_message.blue_outpost_is_live=1; 
 }
 else//前哨判断
 {
	key_message.blue_outpost_is_live=0; 
 }
 if(ext_game_robot_HP.data.red_outpost_HP>0 )
 {
	key_message.red_outpost_is_live=1; 
 }
 else//前哨判断
 {
	key_message.red_outpost_is_live=0; 
 } 
if(ext_game_robot_state.data.robot_id== 7)//自己是红色
{
	if(	key_message.red_outpost_is_live==1)
    key_message.our_outpost_is_live=1;
	else
    key_message.our_outpost_is_live=0;
		
}
if(ext_game_robot_state.data.robot_id== 107)//自己蓝色
{
	if(	key_message.blue_outpost_is_live==1)
    key_message.our_outpost_is_live=1;
	else
    key_message.our_outpost_is_live=0;
		
}
	}
	
	
	
	
//	
//								if(DR16.rc.s_left==1)//YAW轴控制挡位
//							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW轴遥控器控制
//							if(DR16.rc.ch4_DW<=-400)//拨上
//							yaw_trage_angle=(CLOUD_enable_imu+200.0);//陀螺仪角速度最大为140?
//							if(DR16.rc.ch4_DW>=400)//拨下
//							yaw_trage_angle=(CLOUD_enable_imu-200.0);
//							
//							/*
//							所以目标阶跃要大于140/0.03   4666
//							//还要考虑29000的输出多久才能到140这个最大速度
//							//先给10000的阶跃吧
//							现在是800
//							*/
//							

//							}
///*					else
//					从-10000到30000，最开始20000是上升加速段，然后是10000的220，最后是10000的减速		
//					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

//*/
//	
//


cloud_control_mode_choose();
 scan_cloud();
// if(DR16.rc.s_left!=3)
//{
//simulation_target_yaw=	DJIC_IMU.total_yaw;
//}DR16.rc.s_left==3
if(DR16.rc.s_left==3)//左中,手动取假
{
       if(DR16.rc.s_right==2)//关摩擦轮
	   {
	   if(DR16.rc.ch4_DW<=-600)//拨上
		{
	   							Buzzer.mode = One_times;
jia_ZJ_PITCH=DJIC_IMU.total_pitch;
	   whether_use_fake_armor=1;
			fake_armor_init_angle_6020=GM6020s[0].readAngle;
			fake_armor_init_place_encoder=Chassis_Encoder_new.data.totalLine;
//			fake_armor_init_vision_deepth=VisionData.RawData.Depth;
		}
	   
	   }
}
if(DR16.rc.s_left==1)//左上,自瞄取假
{
       if(DR16.rc.s_right==2||DR16.rc.s_right==3)//关自动开火
	   {
	   if(DR16.rc.ch4_DW<=-600)//拨上
		{
						if(vision_beats_give_to_jia==1&&VisionData.RawData.Armour==1)
			{
				if(fabs(Vision_RawData_Pitch_Angle)<1.0)
				{
	   							Buzzer.mode = One_times;
jia_ZJ_PITCH=DJIC_IMU.total_pitch;
					
				}
			}
//			if(
//	   							Buzzer.mode = One_times;
//jia_ZJ_PITCH=DJIC_IMU.total_pitch;
	   
		}
	   
	   }
}
if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//左中右上
{
							if(DR16.rc.ch4_DW<=-400)//拨上
							simulation_target_yaw_is_stop=1;//陀螺仪角速度最大为140?
							if(DR16.rc.ch4_DW>=400)//拨下
							simulation_target_yaw_is_stop=0;//陀螺仪角速度最大为140?
	}
 if(DR16.rc.s_left==3)
{
	if(DR16.rc.s_right==1)//左中右上
	{
		if(simulation_target_yaw_is_stop==0)
	simulation_target_yaw+= 0.036f;   // 18度/1000毫秒
//		else
//		{
//		simulation_target_yaw=
//		}
	}
	else
	{
		simulation_target_yaw=	DJIC_IMU.total_yaw;
	}
	
}
else
{
		simulation_target_yaw=	DJIC_IMU.total_yaw;

}
fake_armor_yaw();
							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	
//							PITCH_trage_angle=0;//保持水平位置
							imu_angle();//去仿真做测试！！！  已做
//									PITCH_trage_angle=28;

							//陀螺仪零漂限幅
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}


void cloud_control_mode_choose(void)
{
	if(DR16.rc.s_left == 1)//左上时
	{
//		if(VisionData.RawData.Armour==1)

		if(Armour_lose_time>=200)//丢失200帧装甲 退出扫描开始自瞄
		{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);			
			
		}
		else
		{
			cloud_mode.control_mode_NOW=vision_mode;
			cloud_mode.control_mode_LAST=aoto_scan_mode;
			P_PID_Parameter_Clear(&Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&Yaw_IMU_Speed_pid);			
		}

		
	}
//	else if(DR16.rc.s_left == 3&&DR16.rc.s_right == 3)
//	{
////				cloud_mode.control_mode_NOW=vision_mode;

//	}
	#if YAW_TEXT==0

	else
	{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);	
			}

#endif		

#if YAW_TEXT==1
			else{
if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//左中右上
{
			cloud_mode.control_mode_NOW=vision_mode;
			cloud_mode.control_mode_LAST=aoto_scan_mode;
}
else
{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);
}
				}

#endif		
	
	
}


bool YAW_TARGE_ANGLE_ADD=1;
int arrive_targe_angle=0;
int TEXT_targe_SPEED=400;
int8_t ch0_z_f=1;
void YAW_PID()
{
	
//									if(DR16.rc.s_left==1)//YAW轴控制挡位
//							{
//	if(DJIC_IMU.total_yaw>(CLOUD_enable_imu+360.0))	
//	{
//		
////TEXT_targe_SPEED		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=0;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(DJIC_IMU.total_yaw<(CLOUD_enable_imu-360.0))	
//	{
//		
//		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=1;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(cloud_text_add==1)//增加
//	{
//yaw_trage_speed=TEXT_targe_SPEED;
//	}	
//	if(cloud_text_add==0)//角度减小,速度为负
//	{
//yaw_trage_speed=-TEXT_targe_SPEED;
//	}	
//	
////if(cloud_enable==0)
////{
////}
//							}	
		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //失能保护
		{
CLOUD_enable_moto=GM6020s[0].totalAngle;
CLOUD_enable_imu=DJIC_IMU.total_yaw;
		}
//						#if PID_MOTOR//YAW轴电机角度
//					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

//					yaw_trage_speed=Yaw_Angle_pid.result;//阶跃


//							yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;//左手推速度
//					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);//用电机做速度闭环
//	
//			                   send_to_yaw=Yaw_Speed_pid.result;

//					#endif
//							
					#if PID_YAW_IMU//YAW轴陀螺仪
									if(DR16.rc.s_left==3)//YAW轴控制挡位
							{
									#if YAW_TEXT==0

								if(cloud_mode.control_mode_NOW==aoto_scan_mode)//扫描PID
								{
									if(DR16.rc.ch0>0)
										ch0_z_f=1;
									if(DR16.rc.ch0<0)
										ch0_z_f=-1;
							yaw_trage_angle-=(DR16.rc.ch0/660.0)*(DR16.rc.ch0/660.0)*ch0_z_f/0.6;//YAW轴遥控器控制
								YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw;
								PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch;
									PITCH_TRAGET_ANGLE_TEMP_EM=GM6020s[3].totalAngle;
								}
//								if(cloud_mode.control_mode_NOW==vision_mode)//扫描PID
//								{
//								yaw_trage_angle=simulation_target_yaw;
//								}
//							CH0_TOTAL_in_con+=	DR16.rc.ch0;
//								if(DR16.rc.ch0!=0)
//								dr16_controul_times++;
								#endif
										#if YAW_TEXT==1
								if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//左中右上
								{
								yaw_trage_angle=simulation_target_yaw;
								}
								else
								{
								yaw_trage_angle-=(DR16.rc.ch0/660.0)/2.0;//YAW轴遥控器控制

								}
								#endif

							}
									if(DR16.rc.s_left==1)//YAW轴控制挡位
							{
//								if(VisionData.RawData.Armour==1)
//							yaw_trage_angle-=Vision_RawData_Yaw_Angle;//YAW轴遥控器控制
								
				if(cloud_mode.control_mode_NOW==vision_mode)//视觉PID
				{
//					yaw_trage_angle=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;//YAW轴遥控器控制
					yaw_trage_angle=YAW_TRAGET_ANGLE_TEMP;//YAW轴视觉控制
													

				}			
//								else
//								yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW轴遥控器控制

							}							
//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//调试过程中这个值要不断更新
							
				if(cloud_mode.control_mode_NOW==aoto_scan_mode)//扫描PID
				{
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
							
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
				}
				
				if(cloud_mode.control_mode_NOW==vision_mode)//自瞄PID
				{
					P_PID_bate(&VISION_Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=VISION_Yaw_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
//							yaw_trage_speed=Vision_RawData_Yaw_Angle ;
					P_PID_bate(&VISION_Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);	
		                   send_to_yaw=VISION_Yaw_IMU_Speed_pid.result;
				
					if(YAW_MOTION_STATE==12)
					{
				Vision_Control_Cloud();
						
//										P_PID_bate(&VISION_Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
	
					}
				
				}
				
				
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
//	
//	
	
}


int8_t ch1_z_f=1;

void PITCH_PID()
{
//	#if PID_PITCH_MOTOR      //PITCH轴电机角度
//if(PITCH_trage_angle>7125)
//	PITCH_trage_angle=7125;
//if(PITCH_trage_angle<6435)
//	PITCH_trage_angle=6435;
//					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
////陀螺仪的速度值会有小数
//					PITCH_trage_speed=PITCH_Angle_pid.result;//外环的结果给内环  二选一
////					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
//				
//				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
//				send_to_pitch=PITCH_Speed_pid.result;
//#endif	
//	
//	#if PID_PITCH_IMU		//PITCH轴陀螺仪
							if(DR16.rc.s_left==3)//PITCH轴控制挡位
							{
								if(DR16.rc.ch1>0)
									ch1_z_f=1;
								if(DR16.rc.ch1<0)
									ch1_z_f=-1;
							PITCH_trage_angle+=(DR16.rc.ch1*1.0/660.0)*(DR16.rc.ch1*1.0/660.0)*ch1_z_f*0.3;//遥控器给速度目标值 二选一
                            PITCH_trage_angle_motor+=(DR16.rc.ch1*1.0/660.0)*1.0;
 ////							if(DR16.rc.ch4_DW<=-400)//拨上
////							PITCH_trage_angle=PITCH_MAX_angle-10;
////							if(DR16.rc.ch4_DW>=400)//拨下
////							PITCH_trage_angle=PITCH_MIN_angle+10;
//							
							}
														if(DR16.rc.s_left==1)//PITCH轴控制挡位
							{
//								if(VisionData.RawData.Armour==1)
//							PITCH_trage_angle=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;//YAW轴遥控器控制
												if(cloud_mode.control_mode_NOW==vision_mode)//视觉PID
												{
								PITCH_trage_angle=PITCH_TRAGET_ANGLE_TEMP;
//								else
//								PITCH_trage_angle+=(DR16.rc.ch1/660.0)*0.4;//YAW轴遥控器控制
							PITCH_trage_angle_motor=PITCH_TRAGET_ANGLE_TEMP_EM;
												}
							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
							
#if use_new_gimbal==0
							if(PITCH_trage_angle_motor>5080)
PITCH_trage_angle_motor=5080;
							if(PITCH_trage_angle_motor<3900)
PITCH_trage_angle_motor=3900;
	#endif
#if use_new_gimbal==1
							if(PITCH_trage_angle_motor>6500)
PITCH_trage_angle_motor=6500;
							if(PITCH_trage_angle_motor<5330)
PITCH_trage_angle_motor=5330;
	#endif								

					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle_motor,GM6020s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle

					#if USE_MOTOR_angle==1
					PITCH_trage_speed=PITCH_Angle_pid.result;//外环的结果给内环  二选一

/*y (前馈输出)= 600.26*[(机械角度-3871.5)/53.912] - 7542*/
							
#if use_new_gimbal==0
	send_to_pitch_before=(GM6020s[3].totalAngle-3871.5)*600.26/53.912-7542;

#endif
#if use_new_gimbal==1
send_to_pitch_before=0;
#endif	
							
					#endif		
												#if USE_MOTOR_angle==0
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//外环的结果给内环  二选一

//					PITCH_trage_speed=PITCH_Angle_pid.result;//外环的结果给内环  二选一

#endif		
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
//				send_to_pitch=PITCH_IMU_Speed_pid.result;//先去做实验
send_to_pitch=send_to_pitch_before+PITCH_IMU_Speed_pid.result;
							if(send_to_pitch>29000)
							{send_to_pitch=29000;}
							if(send_to_pitch<-29000)
							{send_to_pitch=-29000;}
							
							
							
#if USE_PITCH_BC==1
							if(PITCH_trage_angle_motor<4492&&PITCH_trage_angle_motor>3899)
send_to_pitch=PITCH_IMU_Speed_pid.result+
							(PITCH_trage_angle_motor*PITCH_trage_angle_motor*PITCH_trage_angle_motor*-2.5419
							+PITCH_trage_angle_motor*PITCH_trage_angle_motor*62.982f
							-PITCH_trage_angle_motor*745.77+768.52);
/*y = 1.9764x2 - 342.62x + 146.26    R2 = 0.967
*/
#endif
//#endif
//	
}



void imu_angle()
{
//	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7990-GM6020s[3].totalAngle)/8196.0*360.0;
//	PITCH_MIN_angle=DJIC_IMU.total_pitch+(7450-GM6020s[3].totalAngle)/8196.0*360.0;
	

#if use_new_gimbal==0
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(5080-GM6020s[3].totalAngle)/8191.0f*360.0f;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(3900-GM6020s[3].totalAngle)/8191.0f*360.0f;//3900
			allow_angle=	PITCH_MAX_angle-PITCH_MIN_angle;
#endif
#if use_new_gimbal==1
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(6500-GM6020s[3].totalAngle)/8191.0f*360.0f;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(5330-GM6020s[3].totalAngle)/8191.0f*360.0f;//3900
			allow_angle=	PITCH_MAX_angle-PITCH_MIN_angle;	
#endif
}

//2022-4-14:
//
//上边界-下边界

//
//使劲抬头:陀螺仪值为15    6020值为4408      5170
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为-39.5      6020值为3154    3820

//2022-3-27:
//
//上边界-下边界

//
//使劲抬头:陀螺仪值为23    6020值为8026
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为-31      6020值为6756
//2022-3-18:
//
//上边界-下边界

//
//使劲抬头:陀螺仪值为-1.1    6020值为4008
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为-44      6020值为3028

//
//3070-3950
//3100-3930
//8017-7044
//8000-7400
//上边界-下边界

//
//使劲抬头:陀螺仪值为6.5    6020值为8022
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为50      6020值为7020


//扫描函数
//V1.0
//功能描述：
//匀速扫描
//拨到左中右中档位开始扫描运动
//拨到左中右下，停止，并清楚时间累加值
//t=0时，PITCH轴运动到中间，然后向下运动
//完成，但是只有YAW轴是丝滑开启，PITCH轴时默认行程中间开启扫描,PITCH与YAW轴速度绑定
//V1.1
//PITCH与YAW轴速度解除绑定,可以分别设置


		 bool scan_i_PITCH=1;
		 int scan_percent_PITCH=500;//0到1000,百分比
		 int scan_time=0;

		 int scan_percent_YAW=500;//0到1000,百分比
	 float YAW_START_ANGLE;//S扫描开始时YAW轴角度

void scan_cloud(void)
{
//	if(DR16.rc.s_left==3)//控制挡位-扫描
			if(DR16.rc.s_left==1)//控制挡位-扫描
	{
//		if(DR16.rc.s_right==3)//控制挡位-扫描开始
		if(Armour_lose_time>1500)//视觉150ms没锁到装甲板-开始扫描
		{
		 int scan_speed_PITCH=2;//PITCH轴扫描速度,最小为1
		int scan_speed_YWA=35;//YAW轴扫描速度,最小为1


		static bool scan_i_YAW=0;
			scan_time++;
if(scan_time%scan_speed_PITCH==0)//是扫描速度的整数倍
{
	if(scan_percent_PITCH>0&&scan_percent_PITCH<500)
	{
		if(scan_i_PITCH==0)//0增大
		{
		scan_percent_PITCH++;
		}
		else//1减小
		{
		scan_percent_PITCH--;
		}
	}
	else if(scan_percent_PITCH<1)//超出下边界,切换到增大模式
	{
		scan_i_PITCH=0;
		scan_percent_PITCH=1;
	}
	else//超出上边界,切换到减小模式
	{
		scan_i_PITCH=1;
				scan_percent_PITCH=499;

	}
	
}
if(scan_time%scan_speed_YWA==0)//是扫描速度的整数倍  scan_percent_YAW在-500到1000之间波动
{
	if(scan_percent_YAW>-500&&scan_percent_YAW<1000)
	{
		if(scan_i_YAW==0)//0增大
		{
		scan_percent_YAW++;
		}
		else//1减小
		{
		scan_percent_YAW--;
		}
	}
	else if(scan_percent_YAW<-499)//超出下边界,切换到增大模式
	{
		scan_i_YAW=0;
		scan_percent_YAW=-499;
	}
	else//超出上边界,切换到减小模式
	{
		scan_i_YAW=1;
				scan_percent_YAW=999;
	}
	
}
#if 1
/*新哨兵云台是5000-1300,之前是5000-7000?*/
	if(key_message.game_state_progress==4)
	{
		if(key_message.our_outpost_is_live==1)
		{
	if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<1300)//4700-4900
	{
				scan_i_YAW=1;//增大模式
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>5000&&GM6020s[0].readAngle<=8191) //3700-4000
	{
				scan_i_YAW=0;//减小模式
							Buzzer.mode = One_times;

	}	
         }
		else if(key_message.our_outpost_is_live==0)//前哨站死亡
		{
		if(laoliu_gjiwo_times_ago>10000)//有老六偷袭我背后,10秒内扫360度
		{
			if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<2100)//4700-4900
	{
				scan_i_YAW=1;//增大模式
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>4500&&GM6020s[0].readAngle<=8191) //3700-4000
	{
				scan_i_YAW=0;//减小模式
							Buzzer.mode = One_times;

	}	
		
		}
		}
	
	////打符角度5820    左3701   7748

	}
	
if	(in_END_L==1)
{
		in_end_times++;
	if(in_end_times<5000)//5秒内
	{
	if(GM6020s[0].readAngle>3842&&GM6020s[0].readAngle<4900)//4700-4900
	{
				scan_i_YAW=1;//增大模式
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>2687&&GM6020s[0].readAngle<=3842) //3700-4000
	{
				scan_i_YAW=0;//减小模式
							Buzzer.mode = One_times;

	}
}
//	if(GM6020s[0].readAngle>=4330&&GM6020s[0].readAngle<=4700)//4000-4700
//	{
//					scan_i_YAW=0;//增大模式
//		YAW_START_ANGLE=DJIC_IMU.total_yaw+400;
//		scan_percent_YAW=0;
//				Buzzer.mode = One_times;

//		
//	}
//	if(GM6020s[0].readAngle>=4000&&GM6020s[0].readAngle<=4329)//4000-4329
//	{
//				scan_i_YAW=1;//减小模式
//		YAW_START_ANGLE=DJIC_IMU.total_yaw-500;
//		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

//	}
}
if	(in_END_R==1)
{		
	in_end_times++;
	if(in_end_times<5000)//5秒内
	{
		if(GM6020s[0].readAngle>0&&GM6020s[0].readAngle<1200)//500-1200
	{
				scan_i_YAW=1;//增大模式
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>7000&&GM6020s[0].readAngle<8191) //7000-7700
	{
				scan_i_YAW=0;//减小模式
							Buzzer.mode = One_times;

	}
}
	if(in_END_R==0&&in_END_L==0)
	{//不在左右边界
	in_end_times=0;
	}
//	if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<=1000)//0-500
//	{
//					scan_i_YAW=0;//增大模式
//		YAW_START_ANGLE=DJIC_IMU.total_yaw+500;
//		scan_percent_YAW=0;
//		Buzzer.mode = One_times;

// //  7888
//	}
//	if(GM6020s[0].readAngle>=7000&&GM6020s[0].readAngle<=8191)//4000-4329
//	{
//				scan_i_YAW=1;//减小模式
//		YAW_START_ANGLE=DJIC_IMU.total_yaw-400;
//		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

//	}
}
#endif	
#if 0
if	(in_END_R==1)
{
	if(GM6020s[0].readAngle>4700&&GM6020s[0].readAngle<4900)//4700-4900
	{
				scan_i_YAW=0;//增大模式
	}
	if(GM6020s[0].readAngle>3700&&GM6020s[0].readAngle<4000) //3700-4000
	{
				scan_i_YAW=1;//减小模式
	}
	if(GM6020s[0].readAngle>=4330&&GM6020s[0].readAngle<=4700)//4000-4700
	{
					scan_i_YAW=0;//增大模式
		YAW_START_ANGLE=DJIC_IMU.total_yaw+400;
		scan_percent_YAW=0;
				Buzzer.mode = One_times;

		
	}
	if(GM6020s[0].readAngle>=4000&&GM6020s[0].readAngle<=4329)//4000-4329
	{
				scan_i_YAW=1;//减小模式
		YAW_START_ANGLE=DJIC_IMU.total_yaw-500-720;
		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

	}
}
if	(in_END_L==1)
{		

	if(GM6020s[0].readAngle>500&&GM6020s[0].readAngle<1200)//500-1200
	{
				scan_i_YAW=0;//增大模式
	}
	if(GM6020s[0].readAngle>7000&&GM6020s[0].readAngle<7700) //7000-7700
	{
				scan_i_YAW=1;//减小模式
	}
	if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<=500)//0-500
	{
					scan_i_YAW=0;//增大模式
		YAW_START_ANGLE=DJIC_IMU.total_yaw+500;
		scan_percent_YAW=0;
//		Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>=7700&&GM6020s[0].readAngle<=8191)//4000-4329
	{
				scan_i_YAW=1;//减小模式
		YAW_START_ANGLE=DJIC_IMU.total_yaw-500-720;
		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

	}
}
#endif			


PITCH_trage_angle=PITCH_MIN_angle+(allow_angle)*0.8f*(scan_percent_PITCH/500.0f);//PITCH
yaw_trage_angle=YAW_START_ANGLE+720*(scan_percent_YAW/1000.0f);//YAW轴转一圈多一点
								YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw;
								PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch;
								PITCH_TRAGET_ANGLE_TEMP_EM=GM6020s[3].totalAngle;

#if use_new_gimbal==0
PITCH_trage_angle_motor=3900+1180*(scan_percent_PITCH/500.0f);
#endif
#if use_new_gimbal==1
PITCH_trage_angle_motor=5200+1000*(scan_percent_PITCH/500.0f);
#endif	
		}
		else //视觉锁到装甲板-扫描结束
		{
			scan_time=0;
		if(Armour_lose_time>1400)//控制挡位-扫描开始  丢失超过1.4秒 将扫描目标值变成当前角度,方便丝滑切换扫描模式
		{
YAW_START_ANGLE=DJIC_IMU.total_yaw;//丝滑开始扫描
//scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*500	;	
scan_percent_YAW=0;	
#if use_new_gimbal==0
scan_percent_PITCH=(GM6020s[3].totalAngle-3900)/1180.0f*500	;//用电机角度	
#endif
#if use_new_gimbal==1
scan_percent_PITCH=(GM6020s[3].totalAngle-5150)/1180.0f*500	;//用电机角度	
#endif	
		}
		}

	}
	else//不在扫描档位
	{
		PITCH_TRAGET_ANGLE_TEMP_EM=GM6020s[3].totalAngle;
		scan_percent_PITCH=(GM6020s[3].totalAngle-3900)/1180.0f*500	;//用电机角度		

		scan_time=0;
		YAW_START_ANGLE=DJIC_IMU.total_yaw;//丝滑开始扫描
//		scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*1000	;
scan_percent_YAW=0;
	}
		
	

	
	
}

