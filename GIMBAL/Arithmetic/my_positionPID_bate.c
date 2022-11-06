#include "my_positionPID_bate.h"


#if PID_MOTOR
P_PID_t Yaw_Angle_pid;
P_PID_t Yaw_Speed_pid;
#endif
#if PID_YAW_IMU
P_PID_t Yaw_IMU_Angle_pid;
P_PID_t Yaw_IMU_Speed_pid;
#endif
#if VISION_PID_YAW_IMU
P_PID_t VISION_Yaw_IMU_Angle_pid;
P_PID_t VISION_Yaw_IMU_Speed_pid;
#endif

#if PID_PITCH_MOTOR
P_PID_t PITCH_Angle_pid;
P_PID_t PITCH_Speed_pid;
#endif
#if PID_PITCH_IMU
P_PID_t PITCH_IMU_Angle_pid;
P_PID_t PITCH_IMU_Speed_pid;
#endif
#if PID_CHASSIS_MOTOR
P_PID_t CHASSIS_MOTOR_ANGLE_pid;
P_PID_t CHASSIS_MOTOR_SPEED_pid;
#endif



P_PID_t Driver_ANGLE_pid;//����PID
P_PID_t Driver_SPEED_pid;

int PID_YES=0;


void P_PID_Parameter_Init(P_PID_t *P_PID, float Kp, float Ki, float Kd,
                          float epsilon, 
//float max_error, float min_error,
//                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result)
{
    P_PID->Kp = Kp;
    P_PID->Ki = Ki;
    P_PID->Kd = Kd;

    P_PID->Target = 0;
    P_PID->Measure = 0;
    P_PID->Error = P_PID->Target - P_PID->Measure;

    P_PID->Epsilon = epsilon;
//    P_PID->max_error = max_error;
//    P_PID->min_error = min_error;

    P_PID->Proportion = 0;
    P_PID->Integral = 0;
    P_PID->Differential = 0;

//    P_PID->alpha = alpha;
    P_PID->D_Output = 0;

    P_PID->Max_antiwindup = Max_antiwindup;
    P_PID->Min_antiwindup = Min_antiwindup;

    P_PID->result = 0;
    P_PID->Max_result = Max_result;
    P_PID->Min_result = Min_result;

    P_PID->LastError = P_PID->Error;
    P_PID->PreError = P_PID->LastError;
    P_PID->D_Last_Output = 0;
}

//���ַ���ϵ���µ�ȷ��
static uint16_t P_PID_BetaGeneration(float error, float epsilon)
{
    uint16_t beta = 0;
    if (abs(error) <= epsilon)
    {
        beta = 1;
    }
    return beta;
}	

                      
float P_PID_bate(P_PID_t *P_PID, float target, float measure)
{
    uint16_t beta; //���ַ���ϵ��
    //float factor; //����ֲ���

    P_PID->Target = target;   //Ŀ��ֵ
    P_PID->Measure = measure; //����ֵ

    P_PID->Error = target - measure; 

    P_PID->Proportion = P_PID->Kp *P_PID->Error;  //һ��	
//	P_PID->Proportion = P_PID->Kp *P_PID->Error*P_PID->Error; //����     
//if(P_PID->Error<0)
//	P_PID->Proportion=-P_PID->Proportion;
    P_PID->Differential = P_PID->Error - P_PID->LastError; 

//    P_PID->D_Output = P_PID->Kd * (1 - P_PID->alpha) * P_PID->Differential +
//									   	 P_PID->alpha  * P_PID->D_Last_Output;
    P_PID->D_Output = P_PID->Kd * (P_PID->Error - P_PID->LastError);

    beta = P_PID_BetaGeneration(P_PID->Error, P_PID->Epsilon);
PID_YES=beta;
		if(beta==1)
		{
			
	if (P_PID->Integral > P_PID->Max_antiwindup)
        {
            if (P_PID->Error <= 0)
            {
//
								P_PID->Integral += P_PID->Error;

//                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
            }
        }
        else if (P_PID->Integral < P_PID->Min_antiwindup)
        {
            if (P_PID->Error >= 0)
            {
//			  P_PID->Integral += P_PID->Ki *P_PID->Error ;
				P_PID->Integral += P_PID->Error;

//                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
            }
        }
        else
        {
//				P_PID->Integral += P_PID->Ki *P_PID->Error;
				P_PID->Integral += P_PID->Error;
//            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2; //����ֵ
        }
		P_PID->I_Output=	P_PID->Ki * P_PID->Integral   ;
			
        P_PID->result =  P_PID->Proportion + 
					     P_PID->I_Output   + 
						 P_PID->D_Output; //PID������
		}
		else
		{
		        P_PID->result =  P_PID->Proportion + 
									P_PID->D_Output; //PID������
		
		}
    /*	
    if (P_PID->result > P_PID->Max_result)
    {
        if (P_PID->Error <= 0)
        {
            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
        }
    }
    else if (P_PID->result < P_PID->Min_result)
    {
        if (P_PID->Error >= 0)
        {
            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
        }
    }
    else
    {
        P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2; //����ֵ
    }
	*/

//beta=1;
    //factor = P_PID_Variable_Integral_Coefficient(P_PID->Error, P_PID->max_error, P_PID->min_error);

//    if (beta == 0) //ִ��PD����
//    {
//        P_PID->result = P_PID->Kp * P_PID->Proportion + P_PID->D_Output; //PD������
//    }
//    else//ִ��PID����
//    {

//        if (P_PID->Integral > P_PID->Max_antiwindup)
//        {
//            if (P_PID->Error <= 0)
//            {
//                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
//            }
//        }
//        else if (P_PID->Integral < P_PID->Min_antiwindup)
//        {
//            if (P_PID->Error >= 0)
//            {
//                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
//            }
//        }
//        else
//        {

//            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2; //����ֵ
//        }

//        P_PID->result = P_PID->Kp * P_PID->Proportion + P_PID->Ki * P_PID->Integral + P_PID->D_Output; //PID������
//    }


    if (P_PID->result > P_PID->Max_result)
    {
        P_PID->result = P_PID->Max_result;
    }
    else if (P_PID->result < P_PID->Min_result)
    {
        P_PID->result = P_PID->Min_result;
    }

    P_PID->PreError = P_PID->LastError; //ǰ����ƫ��
    P_PID->LastError = P_PID->Error;    //ǰһ��ƫ��

    P_PID->D_Last_Output = P_PID->D_Output; 

    return P_PID->result;
}





//�л�pidʱ�����
void P_PID_Parameter_Clear(P_PID_t *P_PID)
{
		P_PID->Error = 0;					//����������

	P_PID->LastError = 0;				//����ϴε�ƫ��ֵ
	P_PID->PreError = 0;				//������ϴε�ƫ��ֵ
	P_PID->Integral = 0;				//��������ۼ�
	P_PID->D_Last_Output = 0;		//����ϴε�΢�����
	P_PID->result = 0;					//����������
	
}


