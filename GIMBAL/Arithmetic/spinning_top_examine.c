#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"
#include "INS_task.h"
#include "Vision.h"
//#include "stdio.h"       /* cos函数必须包含头文件math.h */
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
  *             主要通过自瞄效果判断小陀螺
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
int8_t YAW_MOTION_STATE=0;//运动状态
/* 
    1 未检测到大变化,在第0个周期
	
    2 检测到增大值大于阈值 开始第一个检测周期
    3 检测到增大值大于阈值 开始第二个个检测周期
    4 检测到增大值大于阈值 开始第三个检测周期

    6 检测到减小值小于阈值 开始第一个检测周期
    7 检测到减小值小于阈值 开始第二个检测周期
    8 检测到减小值小于阈值 开始第三个检测周期

    12  已经在小陀螺模式了
*/

int examine_sampling_period=30;//小陀螺检测的周期,即间隔多久进行一次采样
float examine_sampling_extent=1.2;//变化的角度大于多少度才算进去
float total_yaw_last_time;//上一时刻的角度
float total_yaw_change;//变化的角度

int yaw_angle_is_add=0;
int yaw_angle_is_small_change=0;//周期开始后,小幅度变化

int yaw_add_judge_threshold=4;//判断阈值
int vision_is_lock_in_1s=0;//一秒内是锁定状态
int over_time=9999;
void S_T_examine(void)
{
	
	
	
	
	
	
	if(DR16.rc.s_left==1)
	{
	examine_run_times++;//这个档有自瞄
		if(VisionData.RawData.Armour==1)
		{
		 vision_is_lock_in_1s=0;     //识别到了装甲
		}
		else{
		vision_is_lock_in_1s++;//丢失装甲多少ms
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
		over_time=examine_run_times+200;//0.2秒退出
//		printf("检测到正向大变化,开始第一个检测周期		");
	}
	if(total_yaw_change<-examine_sampling_extent)
	{
		YAW_MOTION_STATE=6;
		over_time=examine_run_times+200;//0.2秒退出
//		printf("检测到反向大变化,开始第一个检测周期		");
	}	
	}
			if(YAW_MOTION_STATE==8)
	{
		if(total_yaw_change>0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=12;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
					over_time=examine_run_times+6000;//3秒退出

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==7)
	{
		if(total_yaw_change>0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=8;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
					over_time=examine_run_times+200;//5秒退出

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==6)
	{
		if(total_yaw_change>0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第一个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=7;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第一个周期结束	");
//			printf("进入小陀螺模式	");
//			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
			if(YAW_MOTION_STATE==4)
	{
		if(total_yaw_change<-0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
			YAW_MOTION_STATE=12;
								over_time=examine_run_times+6000;//3秒退出
//					over_time=examine_run_times+200;//0.2秒退出

		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==3)
	{
		if(total_yaw_change<-0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
			YAW_MOTION_STATE=4;
//								over_time=examine_run_times+3000;//5秒退出
//					over_time=examine_run_times+200;//0.2秒退出

		}
//		printf("\r\n");
	}
	if(YAW_MOTION_STATE==2)
	{
		if(total_yaw_change<-0.3f)
		{
			yaw_angle_is_small_change++;
//			printf("检测到第一个周期第%d个小变化	",yaw_angle_is_small_change);
					over_time=examine_run_times+200;//0.2秒退出

		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=3;//开始第二个个检测周期
//			printf("第一个周期结束	");
			yaw_angle_is_small_change=0;
		}
//		printf("\r\n");

	}
		if(YAW_MOTION_STATE==2||YAW_MOTION_STATE==3||YAW_MOTION_STATE==6||YAW_MOTION_STATE==7||YAW_MOTION_STATE==4||YAW_MOTION_STATE==8)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("检测超时,退出检测模式	");
//					printf("\r\n");

		}
			
		}
				if(YAW_MOTION_STATE==12)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("在小陀螺模式超过10秒,退出小陀螺模式	");
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
	examine_run_times=0;//有自瞄
		over_time=5000;
	}
	
	

	
	
}



/*
如果是水平的,在不考虑pitch轴变化的情况下,YAW轴有一个三角形

有一个角时知道的:指向假装甲时,枪口和轨道所夹的角度:用什么算?

陀螺仪(本质上还是需要电机码盘值:即当前码盘值,与轨道平行时的码盘值的差值(记得取锐角?好像不用))  
    分析了一下 本质上还是依靠电机码盘值的精度

然后是枪口离假装甲的距离,这个视觉会发给我(mm)?

假装甲到轨道的垂直距离是3379mm


根据此刻的编码器值,就可以算出枪口垂直与轨道指向假装甲的编码器的值

在自动档,锁住假装甲后:
记录以下几个值:

12当前编码器的值
当前YAW轴6020的值,不要总圈数,要real_angle
   --这个值大于5820,说明当前在假装甲的右边

3视觉发送回来的深度

4改变是否记录假装甲的标志位

计算出:当前位置下的----
当前编码器值下的,指向假装甲的角度值(6020码盘值)-应该在3701-7748
最终目标角度(陀螺仪总角度)

中间变量:

垂直点的编码器值
假装甲距离垂直点的距离  ----图纸上是3379mm


编码器转一圈,增加3999
直径50mm  周长 50*paimm
50*pai(mm)=3999(编码器)
157.07963267948966192313216916398=3999(编码器)

1编码器=0.03927972810189788995327136013103mm
1mm=3999/50/pai=25.458424696979577909590646789067(编码器)

1(编码器)=50*pai/3999(mm)

	////打符角度5820    左3701   7748


*/
float temp_angle=0;
float temp_rus=0;

float SJX_D=0;//三角形底边_主要依靠视觉深度和云台角度确认,通过编码器更新

void fake_armor_yaw(void)
{
if(whether_use_fake_armor==1)
{//初始化了假装甲值
	
if(fake_armor_init_angle_6020<5820)//在假装甲的右边
{
 fake_armor_vertical_place_encoder= 
	fake_armor_init_place_encoder+
	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0) *fake_armor_init_vision_deepth/*mm*/
	
	*25.458424696979577909590646789067;//往左边为正
//	temp_rus=	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0) *fake_armor_init_vision_deepth/*mm*/;
temp_angle=(fake_armor_init_angle_6020-3710)/8192.0*360.0;
	temp_rus=cos(temp_angle*pai/180.0);
//	temp_rus=cos(temp_angle);
	
	fake_armor_vertica_distance=fake_armor_init_vision_deepth
	*sin((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0);//斜边乘sin角度得到垂直距离;
	
}

if(fake_armor_init_angle_6020>=5820)//在假装甲的左边
{
 fake_armor_vertical_place_encoder= 
	fake_armor_init_place_encoder+
	cos((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0) *fake_armor_init_vision_deepth/*mm*/
	*3999/50.0/pai;//往左边为正

	fake_armor_vertica_distance=fake_armor_init_vision_deepth
	*sin((fake_armor_init_angle_6020-3710)/8192.0*360.0*pai/180.0);//斜边乘sin角度得到垂直距离;
}
whether_use_fake_armor=2;

}

if(whether_use_fake_armor==2)
{
//假如在装甲板右边
	fake_armor_now_angle_IMU=atan(3379*(fake_armor_vertical_place_encoder-Chassis_Encoder_new.data.totalLine)
	*0.03927972810189788995327136013103f);
	YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO=3701+fake_armor_now_angle_IMU/pai/2.0*8192;
	YAW_TRAGET_ANGLE_TEMP_FAKE_IMU=DJIC_IMU.total_yaw+(GM6020s[0].readAngle-YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO)/8191.0*360.0;
}


}







