//
// Created by lenovo on 2023/7/11.
//

#include "include.h"


u32 gSystemTickCount = 0;

uint16 BatteryVoltage;
static u8  fac_us=0;  //us��ʱ������
static u16 fac_ms=0;  //ms��ʱ������

//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void InitDelay(u8 SYSCLK){
    
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
    fac_us=SYSCLK/8;
    fac_ms=(u16)fac_us*1000;
}


//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864
void DelayMs(u16 nms){
    
    u32 temp;
    SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
    SysTick->VAL =0x00;   //��ռ�����
    SysTick->CTRL=0x01 ;  //��ʼ����
    do{
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
    SysTick->CTRL=0x00;  //�رռ�����
    SysTick->VAL =0X00;  //��ռ�����
}


//��ʱnus
//nusΪҪ��ʱ��us��.
void DelayUs(u32 nus){
    
    u32 temp;
    SysTick->LOAD=nus*fac_us; //ʱ�����
    SysTick->VAL=0x00;   //��ռ�����
    SysTick->CTRL=0x01 ; //��ʼ����
    do{
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1 << 16)));//�ȴ�ʱ�䵽��
    SysTick->CTRL = 0x00; //�رռ�����
    SysTick->VAL = 0X00;  //��ռ�����
}


void Init_Timer2(void){

    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
    TIM_TimeBaseStructure.TIM_Period = (10 - 1);         //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = (720 - 1);     //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;         //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);      //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
            TIM2,  //TIM2
            TIM_IT_Update  |  //TIM �ж�Դ
            TIM_IT_Trigger,   //TIM �����ж�Դ
            ENABLE //ʹ��
    );
    TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;     //TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}


//��ʱ��2�ж�  100us
void TIM2_IRQHandler(void){   //TIM2�ж�
    
    static uint32 time = 0;
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //���ָ����TIM�жϷ������:TIM �ж�Դ
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);     //���TIMx���жϴ�����λ:TIM �ж�Դ
        if(++time >= 10){     //100us�ӳ�10������1ms gSystemTickCount�Լ�һ��   
            time = 0;
            gSystemTickCount++;//ʱ���ʱ        
        }
    }
}


bool IsTimeStep20msDone(void){     //����20ms�ж�
    
    static uint32 time = 1;
    static uint32 times = 0;
    if(gSystemTickCount > time){   //time�Լ�10������gSystemTickCount 1ms�Լ�һ�Σ�����times 10ms�Լ�һ��
        time += 10;
        times++;
        if (times % 2 == 0){
           return TRUE;
        }
    }
    return FALSE;	
}


bool IsTimeStep1000msDone(void){   //����1000ms�ж�
    
    static uint32 time = 1;
    static uint32 times = 0;
    if(gSystemTickCount > time){  //time�Լ�10������gSystemTickCount 1ms�Լ�һ�Σ�����times 10ms�Լ�һ��
        time += 10;
        times++;
        if (times % 100 == 0){    
           return TRUE;
        }
    }
    return FALSE;	
}


