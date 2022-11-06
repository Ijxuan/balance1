#include "my_IncrementPID_bate.h"

//挂钩函数指针
I_PID_FUN_t I_PID_FUN = I_PID_HOOK_FUN;

I_PID_t CHASSIS_I_PID;

I_PID_t SHOOT_L_I_PID;
I_PID_t SHOOT_R_I_PID;

I_PID_t Driver_I_PID;


//积分分离系数的确定
static uint16_t I_PID_BetaGeneration(float error, float epsilon)
{
    uint16_t beta = 0;

    if (fabs(error) <= epsilon)
    {
        beta = 1;
    }

    return beta;
}
/*

//变积分系数的确定
static float I_PID_Variable_Integral_Coefficient(float error, float max_error, float min_error)
{
    float factor = 0.0f;

    if (Get_Absolute(error) < min_error)
    {
        factor = 1.0f;
    }
    else if (Get_Absolute(error) > max_error)
    {
        factor = 0.0f;
    }
    else
    {
        factor = 0.75f; //可以是函数关系式
    }
		
    return factor;
}
*/

void I_PID_Parameter_Init(I_PID_t *I_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result)
{
    I_PID->Kp = Kp;
    I_PID->Ki = Ki;
    I_PID->Kd = Kd;

    I_PID->Target = 0;
    I_PID->Measure = 0;
    I_PID->Error = I_PID->Target - I_PID->Measure;

    I_PID->Epsilon = epsilon;
    I_PID->max_error = max_error;
    I_PID->min_error = min_error;

    I_PID->Proportion = 0;
    I_PID->Integral = 0;
    I_PID->Differential = 0;

    I_PID->alpha = alpha;
    I_PID->D_Output = 0;

    I_PID->Max_antiwindup = Max_antiwindup;
    I_PID->Min_antiwindup = Min_antiwindup;

    I_PID->Increment_Output = 0;
    I_PID->result = 0;
    I_PID->Max_result = Max_result;
    I_PID->Min_result = Min_result;

    I_PID->LastError = I_PID->Error;
    I_PID->PreError = I_PID->LastError;
    I_PID->D_Last_Output = 0;
}

float I_PID_Regulation(I_PID_t *I_PID, float target, float measure)
{
    //积分分离系数
    uint16_t beta;
    //变积分系数
    //uint16_t factor;

    I_PID->Target = target;   //目标值
    I_PID->Measure = measure; //测量值

    I_PID->Error = I_PID->Target - I_PID->Measure; //偏差值

    I_PID->Proportion = I_PID->Error - I_PID->LastError;                         //比例项
    I_PID->Differential = I_PID->Error - 2 * I_PID->LastError + I_PID->PreError; //微分项

    I_PID->D_Output = I_PID->Kd * (1 - I_PID->alpha) * I_PID->Differential + I_PID->alpha * I_PID->D_Last_Output;

    beta = I_PID_BetaGeneration(I_PID->Error, I_PID->Epsilon);

    //factor = I_PID_Variable_Integral_Coefficient(I_PID->Error,I_PID->max_error,I_PID->min_error);
    if (beta == 0)
    {
        I_PID->Increment_Output = I_PID->Kp * I_PID->Proportion + I_PID->Kd * I_PID->Differential; 
    }
    else
    {
        if (I_PID->result > I_PID->Max_antiwindup)
        {
            if (I_PID->Error <= 0)
            {
                I_PID->Integral = (I_PID->Error + I_PID->LastError) / 2;
            }
        }
        else if (I_PID->result < I_PID->Min_antiwindup)
        {
            if (I_PID->Error >= 0)
            {
                I_PID->Integral = (I_PID->Error + I_PID->LastError) / 2;
            }
        }
        else
        {
            I_PID->Integral = (I_PID->Error + I_PID->LastError) / 2;
        }

        I_PID->Increment_Output = I_PID->Kp * I_PID->Proportion + I_PID->Ki * I_PID->Integral + I_PID->Kd * I_PID->Differential; 
    }

    I_PID->result += I_PID->Increment_Output; //最终输出

    //输出限幅
    if (I_PID->result > I_PID->Max_result)
    {
        I_PID->result = I_PID->Max_result;
    }
    else if (I_PID->result < I_PID->Min_result)
    {
        I_PID->result = I_PID->Min_result;
    }

    I_PID->PreError = I_PID->LastError; //上上次偏差
    I_PID->LastError = I_PID->Error;    //上次偏差

    I_PID->D_Last_Output = I_PID->D_Output;

    return I_PID->result;
}


void I_PID_Parameter_Clear(I_PID_t *I_PID)
{
    I_PID->LastError = 0;     
    I_PID->PreError = 0;      
    I_PID->Integral = 0;      
    I_PID->D_Last_Output = 0; 
    I_PID->result = 0;        
		I_PID->Measure = 0;
		I_PID->Target = 0;
}
