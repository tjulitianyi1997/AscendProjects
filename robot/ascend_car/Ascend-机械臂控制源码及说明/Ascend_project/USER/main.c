
#include "include.h"



int main(void) {
    
    SystemInit();              //ϵͳʱ�ӳ�ʼ��Ϊ72M  SYSCLK_FREQ_72MHz
    InitDelay(72);             //��ʱ��ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

    Init_Vision_Comm();        //�Ӿ�ͨѶģ���ʼ��
	Init_Motor_Control();      //�������-PWM��ʼ��
    Init_Timer2();             //���ڲ���100us�Ķ�ʱ�ж�

    while (1) {
        TaskRun_Vision_Comm();  
    }
}



