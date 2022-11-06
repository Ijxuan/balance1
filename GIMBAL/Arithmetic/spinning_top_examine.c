#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"
#include "INS_task.h"
#include "Vision.h"
//#include "stdio.h"       /* cos�����������ͷ�ļ�math.h */
 #include <math.h>
 #include "User_typedef.h"
#include "DR16_RECIVE.h"
#include "MY_CLOUD_CONTROL.h"

/**
  ****************************(C) COPYRIGHT 2022 YL****************************
  * @file       spinning_top_examine.c/h
  * @brief      
  *             
  *            
  *             ��Ҫͨ������Ч���ж�С����
  *             
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     5-26-2022       JQX              1. 
  *  V2.0.0     
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YL****************************
  */
int examine_run_times=0;
int8_t YAW_MOTION_STATE=0;//�˶�״̬
/* 
    1 δ��⵽��仯,�ڵ�0������
	
    2 ��⵽����ֵ������ֵ ��ʼ��һ���������
    3 ��⵽����ֵ������ֵ ��ʼ�ڶ������������
    4 ��⵽����ֵ������ֵ ��ʼ�������������

    6 ��⵽��СֵС����ֵ ��ʼ��һ���������
    7 ��⵽��СֵС����ֵ ��ʼ�ڶ����������
    8 ��⵽��СֵС����ֵ ��ʼ�������������

    12  �Ѿ���С����ģʽ��
*/

int examine_sampling_period=30;//С���ݼ�������,�������ý���һ�β���
float examine_sampling_extent=1.2;//�仯�ĽǶȴ��ڶ��ٶȲ����ȥ
float total_yaw_last_time;//��һʱ�̵ĽǶ�
float total_yaw_change;//�仯�ĽǶ�

int yaw_angle_is_add=0;
int yaw_angle_is_small_change=0;//���ڿ�ʼ��,С���ȱ仯

int yaw_add_judge_threshold=4;//�ж���ֵ
int vision_is_lock_in_1s=0;//һ����������״̬
int over_time=9999;
void S_T_examine(void)
{
	
	
	
	
	
	
	if(DR16.rc.s_left==1)
	{
	examine_run_times++;//�����������
		if(VisionData.RawData.Armour==1)
		{
		 vision_is_lock_in_1s=0;     //ʶ����װ��
		}
		else{
		vision_is_lock_in_1s++;//��ʧװ�׶���ms
		}
	if(vision_is_lock_in_1s<1000)
	{
if(examine_run_times%examine_sampling_period==0)
{	
	
	total_yaw_change=DJIC_IMU.total_yaw-total_yaw_last_time;

//	if((DJIC_IMU.total_yaw-total_yaw_last_time)>examine_sampling_extent)
	if(YAW_MOTION_STATE==1)
	{
	if(total_yaw_change>examine_sampling_extent)
	{
		YAW_MOTION_STATE=2;
		over_time=examine_run_times+200;//0.2���˳�
//		printf("��⵽�����仯,��ʼ��һ���������		");
	}
	if(total_yaw_change<-examine_sampling_extent)
	{
		YAW_MOTION_STATE=6;
		over_time=examine_run_times+200;//0.2���˳�
//		printf("��⵽�����仯,��ʼ��һ���������		");
	}	
	}
			if(YAW_MOTION_STATE==8)
	{
		if(total_yaw_change>0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=12;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
					over_time=examine_run_times+6000;//3���˳�

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==7)
	{
		if(total_yaw_change>0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=8;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
					over_time=examine_run_times+200;//5���˳�

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==6)
	{
		if(total_yaw_change>0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽��һ�����ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=7;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("��һ�����ڽ���	");
//			printf("����С����ģʽ	");
//			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
			if(YAW_MOTION_STATE==4)
	{
		if(total_yaw_change<-0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
			YAW_MOTION_STATE=12;
								over_time=examine_run_times+6000;//3���˳�
//					over_time=examine_run_times+200;//0.2���˳�

		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==3)
	{
		if(total_yaw_change<-0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
			YAW_MOTION_STATE=4;
//								over_time=examine_run_times+3000;//5���˳�
//					over_time=examine_run_times+200;//0.2���˳�

		}
//		printf("\r\n");
	}
	if(YAW_MOTION_STATE==2)
	{
		if(total_yaw_change<-0.3f)
		{
			yaw_angle_is_small_change++;
//			printf("��⵽��һ�����ڵ�%d��С�仯	",yaw_angle_is_small_change);
					over_time=examine_run_times+200;//0.2���˳�

		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=3;//��ʼ�ڶ������������
//			printf("��һ�����ڽ���	");
			yaw_angle_is_small_change=0;
		}
//		printf("\r\n");

	}
		if(YAW_MOTION_STATE==2||YAW_MOTION_STATE==3||YAW_MOTION_STATE==6||YAW_MOTION_STATE==7||YAW_MOTION_STATE==4||YAW_MOTION_STATE==8)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("��ⳬʱ,�˳����ģʽ	");
//					printf("\r\n");

		}
			
		}
				if(YAW_MOTION_STATE==12)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("��С����ģʽ����10��,�˳�С����ģʽ	");
//					printf("\r\n");

		}
			
		}
	if(yaw_angle_is_add>yaw_add_judge_threshold)
	{
		yaw_angle_is_add++;	
	}
	
total_yaw_last_time=DJIC_IMU.total_yaw;	
}

	}		


	}
	else{
	examine_run_times=0;//������
		over_time=5000;
	}
	
	

	
	
}



/*
�����ˮƽ��,�ڲ�����pitch��仯�������,YAW����һ��������

��һ����ʱ֪����:ָ���װ��ʱ,ǹ�ں͹�����еĽǶ�:��ʲô��?

������(�����ϻ�����Ҫ�������ֵ:����ǰ����ֵ,����ƽ��ʱ������ֵ�Ĳ�ֵ(�ǵ�ȡ���?������))  
    ������һ�� �����ϻ��������������ֵ�ľ���

Ȼ����ǹ�����װ�׵ľ���,����Ӿ��ᷢ����(mm)?

��װ�׵�����Ĵ�ֱ������3379mm


���ݴ˿̵ı�����ֵ,�Ϳ������ǹ�ڴ�ֱ����ָ���װ�׵ı�������ֵ

���Զ���,��ס��װ�׺�:
��¼���¼���ֵ:

12��ǰ��������ֵ
��ǰYAW��6020��ֵ,��Ҫ��Ȧ��,Ҫreal_angle
   --���ֵ����5820,˵����ǰ�ڼ�װ�׵��ұ�

3�Ӿ����ͻ��������

4�ı��Ƿ��¼��װ�׵ı�־λ

�����:��ǰλ���µ�----
��ǰ������ֵ�µ�,ָ���װ�׵ĽǶ�ֵ(6020����ֵ)-Ӧ����3701-7748
����Ŀ��Ƕ�(�������ܽǶ�)

�м����:

��ֱ��ı�����ֵ
��װ�׾��봹ֱ��ľ���  ----ͼֽ����3379mm


������תһȦ,����3999
ֱ��50mm  �ܳ� 50*paimm
50*pai(mm)=3999(������)
157.07963267948966192313216916398=3999(������)

1������=0.03927972810189788995327136013103mm
1mm=3999/50/pai=25.458424696979577909590646789067(������)

1(������)=50*pai/3999(mm)

	////����Ƕ�5820    ��3701   7748


*/
float temp_angle=0;
float temp_rus=0;

float SJX_D=0;//�����εױ�_��Ҫ�����Ӿ���Ⱥ���̨�Ƕ�ȷ��,ͨ������������

void fake_armor_yaw(void)
{
if(whether_use_fake_armor==1)
{//��ʼ���˼�װ��ֵ
	
if(fake_armor_init_angle_6020<5820)//�ڼ�װ�׵��ұ�
{
 fake_armor_vertical_place_encoder= 
	fake_armor_init_place_encoder+
	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0) *fake_armor_init_vision_deepth/*mm*/
	
	*25.458424696979577909590646789067;//�����Ϊ��
//	temp_rus=	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0) *fake_armor_init_vision_deepth/*mm*/;
temp_angle=(fake_armor_init_angle_6020-3710)/8192.0*360.0;
	temp_rus=cos(temp_angle*pai/180.0);
//	temp_rus=cos(temp_angle);
	
	fake_armor_vertica_distance=fake_armor_init_vision_deepth
	*sin((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0);//б�߳�sin�Ƕȵõ���ֱ����;
	
}

if(fake_armor_init_angle_6020>=5820)//�ڼ�װ�׵����
{
 fake_armor_vertical_place_encoder= 
	fake_armor_init_place_encoder+
	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0) *fake_armor_init_vision_deepth/*mm*/
	*3999/50.0/pai;//�����Ϊ��

	fake_armor_vertica_distance=fake_armor_init_vision_deepth
	*sin((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0);//б�߳�sin�Ƕȵõ���ֱ����;
}
whether_use_fake_armor=2;

}

if(whether_use_fake_armor==2)
{
//������װ�װ��ұ�
	fake_armor_now_angle_IMU=atan(3379*(fake_armor_vertical_place_encoder-Chassis_Encoder_new.data.totalLine)
	*0.03927972810189788995327136013103f);
	YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO=3701+fake_armor_now_angle_IMU/pai/2.0*8192;
	YAW_TRAGET_ANGLE_TEMP_FAKE_IMU=DJIC_IMU.total_yaw+(GM6020s[0].readAngle-YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO)/8191.0*360.0;
}


}







