//
// Created by lenovo on 2023/7/5.
//

#ifndef _VISIONCOMMUNICATION_H
#define _VISIONCOMMUNICATION_H

#include "include.h"


void Init_Vision_Comm(void);        //�Ӿ�����ͨ�ų�ʼ��
void MsgHandle_Vision_Comm(void);  //�Ӿ���Ϣ����

void Send_TargetPosition_Vision_Comm(float pos[6]);  //���Ӿ���������

void Send_Angle_Vision_Comm(float joint[6]);  //���ͻ�е�۽Ƕ���Ϣ
void TaskRun_Vision_Comm(void);

void UART1SendDataPacket(uint8 dat[], uint8 count);


#endif 
