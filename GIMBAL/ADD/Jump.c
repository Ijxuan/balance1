#include "Jump.h"





int DW_FOR_jump = 0;

int jump_step=1;//步骤

int jump_times = 0;
int next_jump_times = 0;



void jump_text()
{

DW_FOR_jump=DR16.rc.ch4_DW;

	
	
if(jump_step==4&&DW_FOR_jump==0)
{
jump_step=1;//松手才能调下一次
}



if(jump_step==3&&next_jump_times>3000)
{
jump_step=4;//三秒后才能跳下一次;
}


switch (jump_step)
	{
	case 0:

	break;
	case 1:
jump_height_R=0;
if(DW_FOR_jump>200)//拨轮拨下
{
jump_step=2;//如果是第一步,就开始整个流程
}	
jump_times=0;//时间清零
	break;
	case 2:
		
jump_height_R=40-engine_body_height_R;//伸腿到35的高度
if(R_C_Y_NOW>34)//实际伸腿到33的高度  就收腿  
	//实际上在快达到目标时，大概还有5-6cm时，力矩就会开始急速衰减
{
jump_step=3;
}
jump_times++;//记录整个过程用了多少秒
	
	break;
	case 3:
		jump_times++;//记录整个过程用了多少秒
	
jump_height_R=0;
	if(R_C_Y_NOW<R_Y-1)
	{//受到冲击已经缩腿了
	Buzzer.mode = One_times;
	}
	next_jump_times++;
	break;
	case 4:
jump_height_R=0;
	jump_times++;//记录整个过程用了多少秒
	next_jump_times=0;

	break;
	default:
		
		break;
	}
jump_height_L=jump_height_R;
}














