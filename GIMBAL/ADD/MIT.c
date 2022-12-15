#include "MIT.h"
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "math.h"

int MIT_MODE_TEXT=1;
MIT_t text_moto;
int16_t sendto_MIT_TEXT=1;
float send_to_MIT_text=0;//发送给电机的值
/**/
int MIT_ANGLE_JD_LAST;//上一时刻电机角度
int MIT_ANGLE_JD_LAST_LAST;//上一时刻电机角度

int MIT_ANGLE_JD_CHANGE;//两个时刻电机角度的变化值

int MIT_SPEED_BY_ANGLE;//根据两次角度之差算出的速度
int MIT_SPEED_BY_ANGLE_TEMP;//根据两次角度之差算出的速度 临时

int MIT_SPEED_NEW;//临时

	int i_for_speed=1;//多久计算一次速度

/* 把buf中的内容通过CAN接口发送出去 */
static void CanTransmit(uint8_t *buf, uint8_t len,uint32_t id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox;
    
    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = id;     /* 指定标准标识符，该值在0x00-0x7FF */
        TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
        TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
        TxHead.DLC      = len;              /* 指定将要传输的帧长度 */
        
        if(HAL_CAN_AddTxMessage(&hcan2, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
        {
            
        }
    }
}


void MIT_MODE(uint8_t MODE)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(MODE)
    {
        case CMD_MOTOR_MODE:
            buf[7] = 0xFC;
            break;
        
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        
        default:
        return; /* 直接退出函数 */
    }
CAN_SendData(&hcan2,CAN_ID_STD,TEST_MIT_SLAVE_ID,buf);
//	CanTransmit(buf,6,TEST_MIT_SLAVE_ID);
}



float uint_to_float(int x_int, float x_min, float x_max, int bits)
	
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
   
}
/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}


/**
  * @brief  Can总线发送控制参数
  * @param
  * @retval 

  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* 通过CAN接口把buf中的内容发送出去 */
//    CanTransmit(buf, sizeof(buf));

CAN_SendData(&hcan2,CAN_ID_STD,TEST_MIT_SLAVE_ID,buf);


}

float position_text=0;//目标角度
float position_text_TEMP=0;//目标角度

float position_HD_text=0;//目标角度-弧度制

float speed_text=0;//目标速度
float speed_HD_text=0;//目标速度-弧度制

float kp_text=0;//角度系数 5  3
float kv_text=0;//速度系数 1  1
float NJ_text=0;//目标扭矩


Ramp_Struct MIT_P;//目标位置斜坡

void MIT_controul(void)
{
position_text+=DR16.rc.ch1/1100.0f;
//	speed_HD_text=DR16.rc.ch3/18.0f;//遥控器速度给弧度的目标值+-35
	

//if(position_text_TEMP>-1)position_text_TEMP=-1;
//	
//if(position_text_TEMP<-90)position_text_TEMP=-90;
//MIT_P.Target_Value=position_text_TEMP;//斜坡目标值
//	MIT_P.Current_Value=position_text;//斜坡当前值
//	position_text=Ramp_Function(&MIT_P);
	
if(position_text>-1)position_text=-1;
	
if(position_text<-90)position_text=-90;
	
	position_HD_text=position_text/Angle_turn_Radian;

	speed_HD_text=P_PID_bate(&MIT_TEXT,position_HD_text,text_moto.position_end);//用弧度制做PID闭环

	speed_text=speed_HD_text*Angle_turn_Radian;//目标速度转角度方便观测
	
	speed_text_v();
if(text_moto.ANGLE_JD>-2)send_to_MIT_text=0;
	
if(text_moto.ANGLE_JD<-89)send_to_MIT_text=0;	
	
CanComm_SendControlPara(position_HD_text,speed_HD_text,kp_text,kv_text,send_to_MIT_text);
//CanComm_SendControlPara(position_HD_text,speed_HD_text,0,0,0);

	
/*
float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
*/
	
}

int speed_add_or_fall;//速度是增大还是减小; 1是增加 2是减小
float target_speed_text=0;//测试用目标速度
float target_speed_text_value=140;//测试用目标速度数值,必须为正值

void speed_text_v(void)
{
if(text_moto.ANGLE_JD>-10)//
{
speed_add_or_fall=2;//开始减小
}
if(text_moto.ANGLE_JD<-80)//
{
speed_add_or_fall=1;//开始增大
}
target_speed_text_value=fabs(target_speed_text_value);
if(speed_add_or_fall==1)
{
target_speed_text=target_speed_text_value;//正速度
}
if(speed_add_or_fall==2)
{
target_speed_text=-target_speed_text_value;//负速度
}

send_to_MIT_text=P_PID_bate(&MIT_SPEED_TEXT,target_speed_text,text_moto.SPEED_JD);

}
