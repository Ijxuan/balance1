#ifndef LQR_TEST_H
#define LQR_TEST_H

#include "MIT.h"
#include "math.h"
typedef float fp32;


//reducation of 3508 motor
//m3508����ļ��ٱ�
#define M3508_MOTOR_REDUCATION 19.2032f

//m3508 rpm change to chassis speed
//m3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00040490766f
//0.00004144660047

//m3508 rpm change to motor angular velocity
//m3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.0054533f

//m3508 current change to motor torque
//m3508ת�ص���(-16384~16384)תΪ�ɵ�����ת��(N.m)�ı���
//c=20/16384*0.3��0.3Ϊת�س���(N.m/A)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//single chassis motor max torque
//�������̵���������
#define MAX_WHEEL_TORQUE 5.5f

//chassis forward or back max speed
//�����˶��������ǰ���˶��ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1.0f


//LQR feedback parameter
//LQR��������ϵ�� -Ĭ��
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2554f
//#define LQR_K3 -10.1436f
//#define LQR_K4 -1.5177f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.6863f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//�����ĳ�11.65KG
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2542f
//#define LQR_K3 -11.7836f
//#define LQR_K4 -1.4369f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.5377f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//�����ĳ�11.65KG ���������ĳ�0.3KG
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2528f
//#define LQR_K3 -11.5491f
//#define LQR_K4 -1.3836f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.4304f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//�����ĳ�11.65KG ���������ĳ�0.3KG  �ڳ� 15cm
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2529f//����̥�ٶ�
//#define LQR_K3 -13.7787f//�˻���Ƕ�
//#define LQR_K4 -2.3758f//�˻�����ٶ�
//#define LQR_K15 2.2361f
//#define LQR_K16 0.6126f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//�����ĳ�7.026KG ���������ĳ�0.5KG 
//����ת����������λΪkg*m^2   1488.358
//�ڳ� 111.81mm 
//����ת����������λΪkg*m^2     137252.709
//%��y���ת������ ת�����      160792.682

#define LQR_K1 -1.0000f//û�õ�
#define LQR_K2 -2.7490f//����̥�ٶ�
#define LQR_K3 -9.5940f//�˻���Ƕ�
#define LQR_K4 -1.6526f//�˻�����ٶ�
#define LQR_K15 2.2361f//YAW�Ƕ�
#define LQR_K16 0.4105f//YAW���ٶ�
#define LQR_K25 -LQR_K15
#define LQR_K26 -LQR_K16

#define M_LQR_K1 -0.0224f//û�õ�
#define M_LQR_K2 -1.7462f//����̥�ٶ�
#define M_LQR_K3 -9.5951f//�˻���Ƕ�
#define M_LQR_K4 -1.3175f//�˻�����ٶ�
#define M_LQR_K15 2.2361f//YAW�Ƕ�
#define M_LQR_K16 0.4105f//YAW���ٶ�
#define M_LQR_K25 -LQR_K15
#define M_LQR_K26 -LQR_K16

extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

#define PII					3.14159265358979f

#define rad_format(Ang) loop_fp32_constrain((Ang), -PII, PII)


//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0015f
//ң������yawң�ˣ�max 660��ת���ɳ�����תĿ��Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000003f

//typedef __packed struct
//{
//    fp32 input;        //��������
//    fp32 out;          //�˲����������
//    fp32 num[1];       //�˲�����
//    fp32 frame_period; //�˲���ʱ���� ��λ s
//} first_order_filter_type_t;

//���̵�����ݽṹ��
typedef struct
{
//  const chassis_motor_measure_t *chassis_motor_measure;
  fp32 speed;           //�������λ���ٶ�
	fp32 omg;             //����������ת�ٶ�
	fp32 torque;          //����������
	fp32 torque_set;      //�����������趨ֵ
  int16_t give_current; //������Ƶ����趨ֵ
} chassis_motor_t;

//�����˶����ݽṹ��
typedef struct
{
//  const RC_ctrl_t *chassis_RC;               //��ȡң����ָ��
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  const fp32 *chassis_INS_angle_speed;       //��ȡ�����ǽ��������ת���ٶ�ָ��
//	chassis_mode_e chassis_mode;               //���̿���ģʽ״̬��
//  chassis_mode_e last_chassis_mode;          //�����ϴο���ģʽ״̬��
  chassis_motor_t motor_chassis[2];          //���̵������

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //�����ٶȣ�ǰ��Ϊ������Ϊ������λm/s
  fp32 omg;                         //���̺ϳ��ֵĽ��ٶȣ���λrad/s
  fp32 vx_set;                      //�����ٶ��趨ֵ��ǰ��Ϊ������Ϊ������λm/s
  fp32 chassis_yaw_set;             //����yaw��Ƕ��趨ֵ
  fp32 delta_angle;                 //����yaw��Ƕ��趨ֵ��yaw��Ƕȵ�ǰֵ֮��

  fp32 vx_max_speed;                //ǰ����������ٶȣ���λm/s
  fp32 vx_min_speed;                //���˷�������ٶȣ���λm/s
  fp32 chassis_yaw;                 //���������Ƿ����ĵ�ǰyaw�Ƕ�
  fp32 chassis_pitch;               //���������Ƿ����ĵ�ǰpitch�Ƕ�
  fp32 chassis_roll;                //���������Ƿ����ĵ�ǰroll�Ƕ�
  fp32 chassis_yaw_speed;           //���������Ƿ����ĵ�ǰyaw���ٶ�
	fp32 chassis_pitch_speed;         //���������Ƿ����ĵ�ǰpitch���ٶ�
	fp32 chassis_roll_speed;          //���������Ƿ����ĵ�ǰroll���ٶ�
	
	fp32 chassis_position_tg;          //����λ��Ŀ��
	fp32 chassis_position_now;          //����λ�õ�ǰ

} chassis_move_t;




extern float  Nm_L_test;
extern float  Nm_R_test;

extern int send_to_L_test;
extern int send_to_R_test;
extern float K3_OUT;
extern float K4_OUT;
extern float K2_OUT;
extern float TARGET_SPEED_POSITION;
extern float LQR_TARGET_position;
extern float pitch_cut_off_angle;//��ֹ��ǳ�������ǶȾ�û�ٶ���
extern float speed_damping_p;//˥��ϵ��
extern float LQRweiyi_text;//LQRλ������ȷ��
extern float TARGET_SPEED_POSITION_V2;
extern float LQRweiyi_PO_TG;//lqrλ��Ŀ��
extern float LQRweiyi_SPEED_TG;//LQR�ٶ�Ŀ��
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector);
void LQR_TEST_CON(void);
void get_speed_by_position_V1(void);
double encoderToDistance(int encoderCount) ;
void LQR_target_position(void);
void get_speed_by_position_V2(void);

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
#endif