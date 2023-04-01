#ifndef __keyBoard_to_vjoy_H
#define __keyBoard_to_vjoy_H


#include "main.h"

#define TIME_KeyMouse_Press 1 //������ʱ����Ϊ ���¡�
//������֮����Ϊ ����
#define TIME_KeyMouse_LongPress 30 //������ʱ����Ϊ ����

typedef enum
{
    Click_Press = 2,  //����
    Long_Press = 3, //����
    No_Press = 1 //�ɿ�
} Press_static_e;

typedef   struct
    {

        uint32_t Press_TIMES;//���¶��                //�����±�־
        Press_static_e Press_static;          //��������־
        uint8_t Press_static_last_time;           //����־-��һʱ��
        uint8_t Click_Press_wait_use;           //�����ȴ�ʹ��

    } keyBoard_PRESS;       //���Ķ��������
	
typedef	struct {
		float ch_WS;//��W��S��������ɵ�ͨ��
		float ch_AD;//��A��D��������ɵ�ͨ��
		float ch_MOUSE;//�������������ɵ�ͨ��
		float ch_GB;//��G��B��������ɵ�ͨ��

	}vjoy;//����ҡ��
extern float rate_add;
extern float rate_decrease;
extern	vjoy vjoy_TEST;//����ҡ�˲���
extern	keyBoard_PRESS keyBoard_W;//W���Ľṹ��

void keyBoard_WASD (void);

#endif













