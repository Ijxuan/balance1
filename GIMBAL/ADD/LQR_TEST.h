#ifndef LQR_TEST_H
#define LQR_TEST_H

#include "MIT.h"
#include "math.h"
typedef float fp32;


//reducation of 3508 motor
//m3508电机的减速比
#define M3508_MOTOR_REDUCATION 19.2032f

//m3508 rpm change to chassis speed
//m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00040490766f
//0.00004144660047

//m3508 rpm change to motor angular velocity
//m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.0054533f

//m3508 current change to motor torque
//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
//c=20/16384*0.3，0.3为转矩常数(N.m/A)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//single chassis motor max torque
//单个底盘电机最大力矩
#define MAX_WHEEL_TORQUE 5.5f

//chassis forward or back max speed
//底盘运动过程最大前后运动速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1.0f


//LQR feedback parameter
//LQR反馈增益系数 -默认
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2554f
//#define LQR_K3 -10.1436f
//#define LQR_K4 -1.5177f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.6863f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//总量改成11.65KG
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2542f
//#define LQR_K3 -11.7836f
//#define LQR_K4 -1.4369f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.5377f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//总量改成11.65KG 车轮质量改成0.3KG
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2528f
//#define LQR_K3 -11.5491f
//#define LQR_K4 -1.3836f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.4304f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//总量改成11.65KG 车轮质量改成0.3KG  摆长 15cm
//#define LQR_K1 -0.0224f
//#define LQR_K2 -2.2529f//乘轮胎速度
//#define LQR_K3 -13.7787f//乘机体角度
//#define LQR_K4 -2.3758f//乘机体角速度
//#define LQR_K15 2.2361f
//#define LQR_K16 0.6126f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16

//总量改成7.026KG 车轮质量改成0.5KG 
//车轮转动惯量，单位为kg*m^2   1488.358
//摆长 111.81mm 
//车体转动惯量，单位为kg*m^2     137252.709
//%绕y轴的转动惯量 转向惯量      160792.682

#define LQR_K1 -1.0000f//没用到
#define LQR_K2 -2.7490f//乘轮胎速度
#define LQR_K3 -9.5940f//乘机体角度
#define LQR_K4 -1.6526f//乘机体角速度
#define LQR_K15 2.2361f//YAW角度
#define LQR_K16 0.4105f//YAW角速度
#define LQR_K25 -LQR_K15
#define LQR_K26 -LQR_K16

#define M_LQR_K1 -0.0224f//没用到
#define M_LQR_K2 -1.7462f//乘轮胎速度
#define M_LQR_K3 -9.5951f//乘机体角度
#define M_LQR_K4 -1.3175f//乘机体角速度
#define M_LQR_K15 2.2361f//YAW角度
#define M_LQR_K16 0.4105f//YAW角速度
#define M_LQR_K25 -LQR_K15
#define M_LQR_K26 -LQR_K16

extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

#define PII					3.14159265358979f

#define rad_format(Ang) loop_fp32_constrain((Ang), -PII, PII)


//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0015f
//遥控器的yaw遥杆（max 660）转化成车体旋转目标角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000003f

//typedef __packed struct
//{
//    fp32 input;        //输入数据
//    fp32 out;          //滤波输出的数据
//    fp32 num[1];       //滤波参数
//    fp32 frame_period; //滤波的时间间隔 单位 s
//} first_order_filter_type_t;

//底盘电机数据结构体
typedef struct
{
//  const chassis_motor_measure_t *chassis_motor_measure;
  fp32 speed;           //电机轮轴位移速度
	fp32 omg;             //电机输出轴旋转速度
	fp32 torque;          //电机输出力矩
	fp32 torque_set;      //电机输出力矩设定值
  int16_t give_current; //电机控制电流设定值
} chassis_motor_t;

//底盘运动数据结构体
typedef struct
{
//  const RC_ctrl_t *chassis_RC;               //获取遥控器指针
  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  const fp32 *chassis_INS_angle_speed;       //获取陀螺仪解算出的旋转角速度指针
//	chassis_mode_e chassis_mode;               //底盘控制模式状态机
//  chassis_mode_e last_chassis_mode;          //底盘上次控制模式状态机
  chassis_motor_t motor_chassis[2];          //底盘电机数据

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值

  fp32 vx;                          //底盘速度，前进为正后退为负，单位m/s
  fp32 omg;                         //底盘合成轮的角速度，单位rad/s
  fp32 vx_set;                      //底盘速度设定值，前进为正后退为负，单位m/s
  fp32 chassis_yaw_set;             //底盘yaw轴角度设定值
  fp32 delta_angle;                 //底盘yaw轴角度设定值与yaw轴角度当前值之差

  fp32 vx_max_speed;                //前进方向最大速度，单位m/s
  fp32 vx_min_speed;                //后退方向最大速度，单位m/s
  fp32 chassis_yaw;                 //底盘陀螺仪反馈的当前yaw角度
  fp32 chassis_pitch;               //底盘陀螺仪反馈的当前pitch角度
  fp32 chassis_roll;                //底盘陀螺仪反馈的当前roll角度
  fp32 chassis_yaw_speed;           //底盘陀螺仪反馈的当前yaw角速度
	fp32 chassis_pitch_speed;         //底盘陀螺仪反馈的当前pitch角速度
	fp32 chassis_roll_speed;          //底盘陀螺仪反馈的当前roll角速度
	
	fp32 chassis_position_tg;          //底盘位置目标
	fp32 chassis_position_now;          //底盘位置当前

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
extern float pitch_cut_off_angle;//截止倾角超过这个角度就没速度了
extern float speed_damping_p;//衰减系数
extern float LQRweiyi_text;//LQR位移数据确定
extern float TARGET_SPEED_POSITION_V2;
extern float LQRweiyi_PO_TG;//lqr位移目标
extern float LQRweiyi_SPEED_TG;//LQR速度目标
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector);
void LQR_TEST_CON(void);
void get_speed_by_position_V1(void);
double encoderToDistance(int encoderCount) ;
void LQR_target_position(void);
void get_speed_by_position_V2(void);

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
#endif