#include "Jump.h"





int DW_FOR_jump = 0;

int jump_step=1;//����

int jump_times = 0;
int next_jump_times = 0;



void jump_text()
{

DW_FOR_jump=DR16.rc.ch4_DW;

	
	
if(jump_step==4&&DW_FOR_jump==0)
{
jump_step=1;//���ֲ��ܵ���һ��
}



if(jump_step==3&&next_jump_times>3000)
{
jump_step=4;//������������һ��;
}


switch (jump_step)
	{
	case 0:

	break;
	case 1:
jump_height_R=0;
if(DW_FOR_jump>200)//���ֲ���
{
jump_step=2;//����ǵ�һ��,�Ϳ�ʼ��������
}	
jump_times=0;//ʱ������
	break;
	case 2:
		
jump_height_R=40-engine_body_height_R;//���ȵ�35�ĸ߶�
if(R_C_Y_NOW>34)//ʵ�����ȵ�33�ĸ߶�  ������  
	//ʵ�����ڿ�ﵽĿ��ʱ����Ż���5-6cmʱ�����ؾͻῪʼ����˥��
{
jump_step=3;
}
jump_times++;//��¼�����������˶�����
	
	break;
	case 3:
		jump_times++;//��¼�����������˶�����
	
jump_height_R=0;
	if(R_C_Y_NOW<R_Y-1)
	{//�ܵ�����Ѿ�������
	Buzzer.mode = One_times;
	}
	next_jump_times++;
	break;
	case 4:
jump_height_R=0;
	jump_times++;//��¼�����������˶�����
	next_jump_times=0;

	break;
	default:
		
		break;
	}
jump_height_L=jump_height_R;
}














