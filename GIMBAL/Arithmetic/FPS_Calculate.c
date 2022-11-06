#include "FPS_Calculate.h"


FPS_ALL_type FPS_ALL;

static uint32_t FPS_Calculate(uint32_t deltaTime)
{
	return (1.0f / (double)(deltaTime)) * 1000.0f; // 别忘了先转换为浮点数，否则会有精度丢失
}
//得到设备的数据传输帧率
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS)
{
	time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;	  //获取当前系统的时钟节拍
	*FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime); //计算得到两次的时钟节拍间距，并转单位精度，即得到FPS
	time->Last_WorldTime = time->WorldTime;
	
	
	
	
}



























