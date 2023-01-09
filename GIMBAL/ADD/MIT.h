#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#include "User_math.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x09//�����õ��SLAVEid
#define TEST_MIT_MASTER_ID 0x0A//�����õ��MASTERid

#define MIT_A_SLAVE_ID 0x01//�����õ��SLAVEid
#define MIT_A_MASTER_ID 0x02//�����õ��MASTERid

#define MIT_B_SLAVE_ID 0x03//�����õ��SLAVEid
#define MIT_B_MASTER_ID 0x04//�����õ��MASTERid

#define MIT_C_SLAVE_ID 0x05//�����õ��SLAVEid
#define MIT_C_MASTER_ID 0x06//�����õ��MASTERid

#define MIT_D_SLAVE_ID 0x07//�����õ��SLAVEid
#define MIT_D_MASTER_ID 0x08//�����õ��MASTERid
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
	int MIT_SPEED_TEMP;//�ٶ��м����
	
	float current_end;  //MIT���յ���
		float ANGLE_JD; //�Ƕ� 
			float SPEED_JD; //�Ƕ� 

int RC_TIMES;//���մ���
int TX_TIMES;//���ʹ���
	
float send_to_MIT;//���͸������ֵ
float	target_position;//Ŀ��Ƕ�
uint8_t MIT_RAW_DATA[8];//���յ��ĵ��������ԭʼ����

	float MIT_TZG;//̧���
	float MIT_TSZ;//����ֱ
} MIT_t;




extern MIT_t text_moto;
extern MIT_t MIT_A;
extern MIT_t MIT_B;
extern MIT_t MIT_C;
extern MIT_t MIT_D;


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
extern float target_speed_text_value;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ
extern float send_to_MIT_text;//���͸������ֵ
extern float target_speed_text;//������Ŀ���ٶ�

extern int MIT_ANGLE_JD_LAST;//��һʱ�̵���Ƕ�
extern int MIT_SPEED_BY_ANGLE;//�������νǶ�֮��������ٶ�
extern	int i_for_speed;//��ü���һ���ٶ�
extern int MIT_ANGLE_JD_CHANGE;//����ʱ�̵���Ƕȵı仯ֵ
extern int MIT_SPEED_BY_ANGLE_TEMP;//�������νǶ�֮��������ٶ� ��ʱ
extern int MIT_ANGLE_JD_LAST_LAST;//��һʱ�̵���Ƕ�
extern int MIT_SPEED_NEW;//��ʱ
extern float target_position_text_PID;//PID������Ŀ��λ��
extern float liftoff_R;//�ұ���ظ߶�
extern float liftoff_L;//�����ظ߶�

extern float MAX_OUT;//������
extern Ramp_Struct SEND_TO_MIT_MAX;//
extern float send_to_MIT_damping;//���͸������ֵ��˥��

extern int MIT_DISABLE_TIMES;//���ʧ��ʱ���ۼ�
extern int MIT_ENABLE_TIMES;//���ʹ��ʱ���ۼ�

extern int run_MIT_ENTER_MOTO_MODE_times;


void MIT_MODE(uint8_t MODE);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t id);
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

#endif
