#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#include "User_math.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x09//测试用电机SLAVEid
#define TEST_MIT_MASTER_ID 0x0A//测试用电机MASTERid

//#define MIT_A_SLAVE_ID 0x01//测试用电机SLAVEid
//#define MIT_A_MASTER_ID 0x02//测试用电机MASTERid

//#define MIT_B_SLAVE_ID 0x03//测试用电机SLAVEid
//#define MIT_B_MASTER_ID 0x04//测试用电机MASTERid

//#define MIT_C_SLAVE_ID 0x05//测试用电机SLAVEid
//#define MIT_C_MASTER_ID 0x06//测试用电机MASTERid

//#define MIT_D_SLAVE_ID 0x07//测试用电机SLAVEid
//#define MIT_D_MASTER_ID 0x08//测试用电机MASTERid

#define MIT_A_SLAVE_ID 0x01//测试用电机SLAVEid
#define MIT_A_MASTER_ID 0x05//测试用电机MASTERid

#define MIT_B_SLAVE_ID 0x02//测试用电机SLAVEid
#define MIT_B_MASTER_ID 0x06//测试用电机MASTERid

#define MIT_C_SLAVE_ID 0x03//测试用电机SLAVEid
#define MIT_C_MASTER_ID 0x07//测试用电机MASTERid

#define MIT_D_SLAVE_ID 0x04//测试用电机SLAVEid
#define MIT_D_MASTER_ID 0x08//测试用电机MASTERid
// #define P_MIN -0.0f//测试电机位置限幅
// #define P_MAX 98.0f
// #define V_MIN -10.0f
// #define V_MAX 10.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -18.0f
// #define T_MAX 18.0f

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f    // N-m
#define T_MAX 18.0f

#define Angle_turn_Radian 57.295779513082320876798154814105f
//弧度转角度要用到
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))



//#define Angle_turn_Radian_MIT 57.295779513082320876798154814105f

typedef struct
{


	int position; //MIT位置

	uint16_t velocity; //MIT速度
	int current;  //MIT电流
	float position_end; //MIT最终位置

	float velocity_end; //MIT最终速度
	int MIT_SPEED_TEMP;//速度中间变量
	
	float current_end;  //MIT最终电流
		float ANGLE_JD; //角度 
			float SPEED_JD; //角度 

int RC_TIMES;//接收次数
int TX_TIMES;//发送次数
	
float send_to_MIT;//发送给电机的值
	float send_to_MIT_position;//发送给电机的位置目标值
	float send_to_MIT_speed;//发送给电机的位置速度值

float	target_position;//目标角度
float	target_position_end;//最终目标角度

float	target_speed;//目标角度

	float	kp;//角度*kp
	float	kp_temp;//角度*kp l临时
	float	kv;//速度*kv
	float	kv_temp;//速度*kv l临时

uint8_t MIT_RAW_DATA[8];//接收到的电机发来的原始数据

	float MIT_TZG;//抬最高
	float MIT_TSZ;//腿伸直
	float MIT_TZG_ARRIVE;//抬最高-实际达到值

} MIT_t;
typedef struct
{


float x_speed;
float y_speed;
float swing_link_SPEED;//摆杆长度变化的速度

	float fai_0;//90-摆杆角度
	float fai_1;//A点与x轴的夹角 钝角
	float fai_2;//B点与x轴的夹角 锐角
	float fai_3;//D点与x轴的夹角 钝角
	float fai_4;//E点与x轴的夹角 锐角

		float T_A;//A点电机力矩
		float T_E;//E点电机力矩

} connecting_rod_t;//连杆机构



extern MIT_t text_moto;
extern MIT_t MIT_A;
extern MIT_t MIT_B;
extern MIT_t MIT_C;
extern MIT_t MIT_D;

extern connecting_rod_t right_leg;//连杆机构
extern connecting_rod_t Left_leg;

extern int MIT_MODE_TEXT;
extern int16_t sendto_MIT_TEXT;//发送给电机 为1时发 为0时不发
extern float position_text;

extern float position_text;//目标角度
extern float position_HD_text;//目标角度-弧度制

extern float speed_text;//目标速度
extern float speed_HD_text;//目标速度-弧度制

extern float kp_text;//角度系数
extern float kv_text;//速度系数
extern float NJ_text;//目标扭矩
extern Ramp_Struct MIT_P;//目标位置斜坡
extern float position_text_TEMP;//目标角度
extern float target_speed_text_value;//测试用目标速度数值,必须为正值
extern float send_to_MIT_text;//发送给电机的值
extern float target_speed_text;//测试用目标速度

extern int MIT_ANGLE_JD_LAST;//上一时刻电机角度
extern int MIT_SPEED_BY_ANGLE;//根据两次角度之差算出的速度
extern	int i_for_speed;//多久计算一次速度
extern int MIT_ANGLE_JD_CHANGE;//两个时刻电机角度的变化值
extern int MIT_SPEED_BY_ANGLE_TEMP;//根据两次角度之差算出的速度 临时
extern int MIT_ANGLE_JD_LAST_LAST;//上一时刻电机角度
extern int MIT_SPEED_NEW;//临时
extern float target_position_text_PID;//PID测试用目标位置
extern float liftoff_R;//右边离地高度
extern float liftoff_L;//左边离地高度

extern float MAX_OUT;//最大输出
extern Ramp_Struct SEND_TO_MIT_MAX;//
extern float send_to_MIT_damping;//发送给电机的值的衰减

extern int MIT_DISABLE_TIMES;//电机失能时间累计
extern int MIT_ENABLE_TIMES;//电机使能时间累计

extern int run_MIT_ENTER_MOTO_MODE_times;

extern float MIT_Bias_R;/*腿部倾斜值*/
extern float MIT_Bias_L;/*腿部倾斜值*/
extern float pitch_kp;/*pitch轴太灵敏了,需要衰减一下*/
extern Ramp_Struct liftoff_temp;//离地高度斜坡

extern float L_X;//左x目标位置
extern float L_Y;//左Y目标位置
extern float R_X;//右x目标位置
extern float R_Y;//右Y目标位置
extern float engine_body_height_R;//右边机体高度
extern float engine_body_height_L;//左边机体高度
extern float jump_height_R; // 右边跳跃高度
extern float jump_height_L; // 左边跳跃高度

extern int R_speed_new_FOR_MIT;
extern int L_speed_new_FOR_MIT;

extern float L_C_X_NOW;//此时此刻左轮的坐标（x轴）
extern float L_C_Y_NOW;//此时此刻左轮的坐标（y轴）
extern float R_C_X_NOW;//此时此刻右轮的坐标（x轴）
extern float R_C_Y_NOW;//此时此刻右轮的坐标（y轴）
void MIT_MODE(uint8_t MODE);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t id);
void CanComm_SendControlPara_CAN2(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t id);

//static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits); 
void MIT_controul(void);
void speed_text_v(void);
void MIT_PID_INIT(void);
void MIT_B_controul(void);
void ALL_MIT_ENTER_MOTO_MODE(void);
void DISABLE_ALL_MIT(void);
void MIT_A_controul(void);
void MIT_C_controul(void);
void MIT_D_controul(void);
void MIT_calibration(void);
void get_MIT_tg_angle_for_bais(void);
void get_MIT_tg_angle_for_liftoff(void);

#endif
