#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#include "User_math.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x01//�����õ��SLAVEid
#define TEST_MIT_MASTER_ID 0x02//�����õ��MASTERid

// #define P_MIN -0.0f//���Ե��λ���޷�
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
#define T_MIN -18.0f
#define T_MAX 18.0f

#define Angle_turn_Radian 57.295779513082320876798154814105f
//����ת�Ƕ�Ҫ�õ�
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

//#define Angle_turn_Radian_MIT 57.295779513082320876798154814105f

typedef struct
{


	int position; //MITλ��

	uint16_t velocity; //MIT�ٶ�
	int current;  //MIT����
	float position_end; //MIT����λ��

	float velocity_end; //MIT�����ٶ�
	float current_end;  //MIT���յ���
		float ANGLE_JD; //�Ƕ� 
			float SPEED_JD; //�Ƕ� 

	int16_t turnCount;	//ת����Ȧ��
	
uint8_t MIT_RAW_DATA[8];//���յ��ĵ��������ԭʼ����

} MIT_t;
extern MIT_t text_moto;
extern int MIT_MODE_TEXT;
extern int16_t sendto_MIT_TEXT;//���͸���� Ϊ1ʱ�� Ϊ0ʱ����
extern float position_text;

extern float position_text;//Ŀ��Ƕ�
extern float position_HD_text;//Ŀ��Ƕ�-������

extern float speed_text;//Ŀ���ٶ�
extern float speed_HD_text;//Ŀ���ٶ�-������

extern float kp_text;//�Ƕ�ϵ��
extern float kv_text;//�ٶ�ϵ��
extern float NJ_text;//Ŀ��Ť��
extern Ramp_Struct MIT_P;//Ŀ��λ��б��
extern float position_text_TEMP;//Ŀ��Ƕ�

void MIT_MODE(uint8_t MODE);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t);
//static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
void MIT_controul(void);

#endif
