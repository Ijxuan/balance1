#include "MY_CHASSIS_CONTROL.h"
#include "math.h"
//#include "rng.h"

#define HW_SWITCH_JR 240000

bool CHASSIS_L_MAX_new=0;//���ұ߽�ֵ�Ƿ����
bool CHASSIS_R_MIN_new=0;
bool Random_CHASSIS_CHOOSE=0;//�Ƿ�ѡ�����ģʽ
bool Cruise_CHASSIS_CHOOSE=1;//�Ƿ�ѡ��Ѳ��ģʽ

void CHASSIS_CONTROUL(void)
{
	#if PID_CHASSIS_MOTOR
			#if 0	//���������Ѳ���˶�
					if(DR16.rc.s_left==3)//�Զ�����
					{
						

						
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
				if(DR16.rc.ch4_DW<=-400)//����
				{
					
				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
				Cruise_CHASSIS_CHOOSE=0;
				}
				
				if(DR16.rc.ch4_DW>=400)//����
				{
				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
				Random_CHASSIS_CHOOSE=0;
					
				}
//				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
//				if(Random_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Random_CHASSIS();//���ģʽ
				
				CHASSIS_trage_speed=0;//����
					
				}
				else
				{
					if( HWswitch_L==0)// �����Ӧ���ˣ������˶�
					CHASSIS_trage_angle=-9990000;
					else if(HWswitch_R==0)//	�ҹ���Ӧ���ˣ������˶�
					CHASSIS_trage_angle=9990000;
				
					P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle

					CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//˫��
//					else 				//Ĭ�������˶�
//					CHASSIS_trage_angle=990000;		
				}

					}
//			CHASSIS_MOTOR_ANGLE_pid.Max_result=1200;
			#endif
//					if(DR16.rc.s_left==1)//ң��������  ����
//					{
//					CHASSIS_trage_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*CHASSIS_MAX_SPEED;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
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
//			//���仯���  ����
//	if(HWswitch_L!=HWswitch_L_last	)
//	{
//			if(HWswitch_L_last==1)//		0<--1
//			{
//				CHASSIS_L_MAX=M3508s[3].totalAngle+HW_SWITCH_JR;
//				ENCODER_L_MAX=Chassis_Encoder.totalLine+8850;
//				CHASSIS_L_MAX_new=1;//�߽�ֵ�Ѹ���
//			}
//		
//	}
//	if(HWswitch_R!=HWswitch_R_last	)
//	{
//			if(HWswitch_R_last==1)//		1-->0
//			{
//				CHASSIS_R_MIN=M3508s[3].totalAngle-HW_SWITCH_JR;
//				ENCODER_R_MIN=Chassis_Encoder.totalLine-8978;
//				CHASSIS_R_MIN_new=1;//�߽�ֵ�Ѹ���

//			}
//		
//	}	
//	
//			HWswitch_L_last		=HWswitch_L;
//			HWswitch_R_last		=HWswitch_R;
//	
	
}


Random_t RANDOM_CHASSIS;

const uint16_t Random_CHANGE_times = 500; //500ms�������
const uint8_t Random_Proportion = 50;      //�������ռ��
const uint16_t Random_CHANGE_speed = 3000;      //�ٴα���Ҫ�ﵽ����ٶ�����

//���ģʽ
void Random_CHASSIS(void)
{
//    if (abs(CHASSIS_trage_speed) != 3500)
//    {
//        CHASSIS_trage_speed = 3500;//����˶��Ļ����ٶ�
//    }//����˶�   ��ʼ���ٶ�   ��Random_Velocity�������˶�
//    RANDOM_CHASSIS.number = Get_RandomNumbers_Range(0, 100);
//					if(M3508s[3].totalAngle>(CHASSIS_R_MIN+100000)&&M3508s[3].totalAngle<(CHASSIS_L_MAX-100000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��

//    RANDOM_CHASSIS.sampling++;
	
	#if 1
	if(	abs(M3508s[3].realSpeed) >Random_CHANGE_speed	)
	    RANDOM_CHASSIS.sampling++;

	#endif
	
//					if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
//				{
//			CHASSIS_trage_speed=3500;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//				}
//			
//			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//����߽���� ��ʮ����ʮ��
//			{
//				CHASSIS_trage_speed=-3500;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//			}
//	
//    if (RANDOM_CHASSIS.sampling == Random_CHANGE_times)
//    {
//        if (RANDOM_CHASSIS.number >= Random_Proportion)//�Ƿ����
//        {
//            CHASSIS_trage_speed = -CHASSIS_trage_speed;
//			speed_change_times++;
//        }
//        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//    }
//	


	
}


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;

//	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}


void Cruise_CHASSIS(void)//		cruise	Ѳ��
{
						
//			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
//			CHASSIS_trage_angle=990000;
//			
//			
//			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��
//			CHASSIS_trage_angle=-990000;
//			
//			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
//			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//˫��
//			
//				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
//				//CHASSIS_MID-CHASSIS_R_MIN   һ���г�
//				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
//				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//�������ٷ�֮70
//				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//�������м�죬������
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





















