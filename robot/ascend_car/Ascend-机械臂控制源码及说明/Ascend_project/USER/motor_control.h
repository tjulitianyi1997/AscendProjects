#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include "include.h"

#define SERVO1 PCout(10)            //����צ�ӵĿ���
#define SERVO2 PCout(11)            //����צ�ӵĶ��
#define SERVO3 PCout(12)
#define SERVO4 PDout(2)
#define SERVO5 PBout(5)
#define SERVO6 PBout(8)


extern uint16 ServoPwmDutySet[];//������
extern uint16 ServoPwmDuty[];

extern bool ServoPwmDutyHaveChange;//����仯��־λ

void ServoSetPluseAndTime_Motor_Control(uint8 id, uint16 p, uint16 time);//ʱ�����
void ServoPwmDutyCompare_Motor_Control(void);//����仯�Ƚϼ��ٶȿ���
void Init_Motor_Control(void);//��ʼ��PWM
void InitTimer3_Motor_Control(void);
void Calculate_TargetPosition_Motor_Control(float pose[6]);
void Goto_TargetPose_Motor_Control(float pose[3]);
void _Pluse2Angle(float angle[6]);//����ת���ɽǶ�
void Ctrl_Joint_Motor_Control(U8 id, U8 dir); //���ƶ��
void Ctrl_Gripper_Motor_Control(U8 dir); //���Ƽ�צ
void Ctrl_Zeroing_Motor_Control(void);   //���ƻ�е�ۻ���
void Grap_Preparation_Motor_Control(void); //ץȡ׼��


#endif

