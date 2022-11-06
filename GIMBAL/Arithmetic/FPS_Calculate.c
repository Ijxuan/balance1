#include "FPS_Calculate.h"


FPS_ALL_type FPS_ALL;

static uint32_t FPS_Calculate(uint32_t deltaTime)
{
	return (1.0f / (double)(deltaTime)) * 1000.0f; // ��������ת��Ϊ��������������о��ȶ�ʧ
}
//�õ��豸�����ݴ���֡��
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS)
{
	time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;	  //��ȡ��ǰϵͳ��ʱ�ӽ���
	*FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime); //����õ����ε�ʱ�ӽ��ļ�࣬��ת��λ���ȣ����õ�FPS
	time->Last_WorldTime = time->WorldTime;
	
	
	
	
}



























