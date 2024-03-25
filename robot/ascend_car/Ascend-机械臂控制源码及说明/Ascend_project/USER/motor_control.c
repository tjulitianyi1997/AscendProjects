#include "include.h"


uint16 ServoPwmDuty[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};     //PWM������
uint16 ServoPwmDutySet[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};  //PWM������
float ServoPwmDutyInc[8];  //Ϊ���ٶȿ��ƣ���PWM�������仯ʱ��ÿ2.5ms��20ms������PWM����

bool ServoPwmDutyHaveChange = FALSE; //�����б仯��־λ

uint16 ServoTime = 2000; //����ӵ�ǰ�Ƕ��˶���ָ���Ƕȵ�ʱ�䣬Ҳ���ǿ����ٶ�


static void _Angle2Pluse(float angle[4], uint16 pulse[6]){  //���Ƕ�ת��������ֵ	
    
    uint8 i;
    for (i = 0; i < 4; ++i) {
        if(i == 1){

            angle[i] = -angle[i];
        }
        if(angle[i] >= 270 && angle[i] <= 360){
        
            angle[i] =  angle[i] - 360;
        }
        else if(angle[i] >= -360 && angle[i] <= -270){
        
            angle[i] = 360 + angle[i];
        }
        else if(angle[i] < -360){
            
            angle[i] += 360;
        }
        else if(angle[i] > 360){

            angle[i] -= 360;
        }
        else if(angle[i] > 630 && angle[i] < 720)
        {
            angle[i] = -(90-(angle[i] - 630));
        }
        else if(angle[i] > -720 && angle[i] < -630)
        {
            angle[i]= 90+(angle[i] + 630);
        }
        pulse[i] = ((angle[i]*1500) / 135) + 1500;        
    }
}

void _Pluse2Angle(float angle[4]){ //����ת���ɽǶ�	
   
    uint8 i;
//	angle[4] = ((ServoPwmDuty[6-i])*135.0)/1500-135;
//	angle[5] = ((ServoPwmDuty[6-i])*135.0)/1500-135;
    
    for (i = 0; i < 4; ++i){
        angle[i] = ((ServoPwmDuty[6-i])*135.0)/1500-135;
        if(i == 1)
        {
            angle[i] = -angle[i];
        }
    }
}

void ServoSetPluseAndTime_Motor_Control(uint8 id, uint16 p, uint16 time){
    
    if(id != 4)
    {
        if (id > 7 || p < 500 || p > 2500){
            return;
        }
    }
    if (time < 20){
        time = 20;
    }    
    if (time > 30000){
        time = 30000; 
    }				
    ServoPwmDutySet[id] = p;
    ServoTime = time;
    ServoPwmDutyHaveChange = TRUE;
}

void ServoPwmDutyCompare_Motor_Control(void){  //����仯�Ƚϼ��ٶȿ���
    
    uint8 i;		
    static uint16 ServoPwmDutyIncTimes;  //��Ҫ�����Ĵ���
    static bool ServoRunning = FALSE;    //���������ָ���ٶ��˶���ָ���������Ӧ��λ��
    
    if (ServoPwmDutyHaveChange){         //ֹͣ�˶������������仯ʱ�Ž��м���  
        ServoPwmDutyHaveChange = FALSE;
        ServoPwmDutyIncTimes = ServoTime/20; //��ÿ20ms����һ�κ���ʱ�ô˾�
        
        for (i = 0; i < 8; i++){
            if (ServoPwmDutySet[i] > ServoPwmDuty[i]){
                ServoPwmDutyInc[i] = ServoPwmDutySet[i] - ServoPwmDuty[i];
                ServoPwmDutyInc[i] = -ServoPwmDutyInc[i];
            } else{
                ServoPwmDutyInc[i] = ServoPwmDuty[i] - ServoPwmDutySet[i];
            }
            ServoPwmDutyInc[i] /= ServoPwmDutyIncTimes;  //ÿ�ε�������������������������Ե����Ĵ���
        }
        ServoRunning = TRUE; //�����ʼ����
    }
    
    if (ServoRunning){			
        ServoPwmDutyIncTimes--;
        for (i = 0; i < 8; i++){
            if (ServoPwmDutyIncTimes == 0){ //���һ�ε�����ֱ�ӽ��趨ֵ������ǰֵ           	  
                ServoPwmDuty[i] = ServoPwmDutySet[i];
                ServoRunning = FALSE;       //�����趨λ�ã����ֹͣ�˶�
            } else{
                ServoPwmDuty[i] = ServoPwmDutySet[i] +
                                  (signed short int) (ServoPwmDutyInc[i] * ServoPwmDutyIncTimes);							
            }
        }
    }
}


void InitTimer3_Motor_Control(void){
    
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC->APB1ENR |= 1 << 1; //TIM3ʱ��ʹ��
    TIM3->ARR = 10000 - 1;  //�趨�������Զ���װֵ,�պ�1ms
    TIM3->PSC = 72 - 1;     //Ԥ��Ƶ��72,�õ�1Mhz�ļ���ʱ��
    TIM3->DIER |= 1 << 0;   //��������ж�
//	TIM3->DIER|=1<<6;       //�������ж�	   
    TIM3->CR1 |= 0x01;      //ʹ�ܶ�ʱ��3

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

}


void Init_Motor_Control(void){

    GPIO_InitTypeDef GPIO_InitStructure;

    InitTimer3_Motor_Control();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}


//��PWM����ת�����Զ�װ�ؼĴ�����ֵ
void Timer3_ARRValue_Motor_Control(uint16 pwm){

    TIM3->ARR = pwm + 1;
}


//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void){

    static uint16 i = 1;
    if (TIM3->SR & 0x0001){ //����ж�  
        switch (i){
            case 1:  //���ڻ�е�۹ؽ���1~6���Դ˴�����
// 				SERVO0 = 1;	//PWM���ƽŸߵ�ƽ
                //����ʱ��0��ֵ������Pwm0Duty�����������жϣ��´��жϻ������һ��case���
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[0]);
                break;
            case 2:
// 				SERVO0 = 0;	   //PWM���ƽŵ͵�ƽ
                //�˼�������ֵ�������жϱ�ʾ��һ����ԪҪ��������Ŀ�ʼ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[0]);
                break;
            case 3:
                SERVO1 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[1]);
                break;
            case 4:
                SERVO1 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[1]);
                break;
            case 5:
                SERVO2 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[2]);
                break;
            case 6:
                SERVO2 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[2]);
                break;
            case 7:
                SERVO3 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[3]);
                break;
            case 8:
                SERVO3 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[3]);
                break;
            case 9:
                SERVO4 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[4]);
                break;
            case 10:
                SERVO4 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[4]);
                break;
            case 11:
                SERVO5 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[5]);
                break;
            case 12:
                SERVO5 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[5]);
                break;
            case 13:
                SERVO6 = 1;
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[6]);
                break;
            case 14:
                SERVO6 = 0;    //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[6]);
                break;
            case 15:           //���Ե��߹ؽ�
// 				SERVO7 = 1;	
                Timer3_ARRValue_Motor_Control(ServoPwmDuty[7]);
                break;
            case 16:
// 				SERVO7 = 0;	   //PWM���ƽŵ͵�ƽ
                Timer3_ARRValue_Motor_Control(2500 - ServoPwmDuty[7]);
                i = 0;
                break;
        }
        i++;
    }
    TIM3->SR &= ~(1 << 0); //����жϱ�־λ
}


void Goto_TargetPose_Motor_Control(float pose[6]){ //λ��ת��
    
    uint16 pulse[4];
    float angle[4] = {0};
    float Pose[6] = {0};
	uint8 i;
    Pose[0] = pose[0];
    Pose[1] = pose[1];
    Pose[2] = pose[2];
    Pose[3] = pose[3];
    Pose[4] = pose[4];
    Pose[5] = pose[5];
    AlgoInverseKinematics(Pose,angle);  //���
    _Angle2Pluse(angle, pulse); 
//    for(i = 6;i > 2;i--)
//    {
//        if(pulse[i] > 2600 || pulse[i] < 350)
//        {
//            return;
//        }
//    }
    for (i = 6; i > 2; i--) {
        if(i == 4)
        {
            pulse[6-i] -= 85;
        }
        ServoSetPluseAndTime_Motor_Control(i, pulse[6-i],2000);
    }
}

void Calculate_TargetPosition_Motor_Control(float pose[6]){  //����λ����Ϣ
    
    float Angle[4];
    float T04[16] = {0};
    _Pluse2Angle(Angle); 
    AlgoPositiveSolution(Angle,T04,pose);
}

void Ctrl_Joint_Motor_Control(U8 id, U8 dir){  //��е�۹ؽڿ���
	
    int pwm_idx = 0;
	if (id < 1 || id > 6){
		return;
	}

	pwm_idx = 7 - id;
    if(id != 3)
    {
        if (dir == 0x01){
            if(ServoPwmDutySet[pwm_idx] < 500){  //��λ��С500����ֵ
                ServoPwmDutySet[pwm_idx] = 500;
                return;
            }
            ServoSetPluseAndTime_Motor_Control( pwm_idx, ServoPwmDutySet[pwm_idx] - 20, 20);
        } else if(dir == 0x00){        
            if(ServoPwmDutySet[pwm_idx] > 2500){ //��λ���2500����ֵ
                ServoPwmDutySet[pwm_idx] = 2500;
                return;
            }
            ServoSetPluseAndTime_Motor_Control( pwm_idx, ServoPwmDutySet[pwm_idx] + 20, 20);
        }		
    }
    if(id == 3)
    {
        if (dir == 0x01){
            if(ServoPwmDutySet[pwm_idx] < 350){  //��λ��С350����ֵ
                ServoPwmDutySet[pwm_idx] = 350;
                return;
            }
            ServoSetPluseAndTime_Motor_Control( pwm_idx, ServoPwmDutySet[pwm_idx] - 20, 20);
        } else if(dir == 0x00){
            if(ServoPwmDutySet[pwm_idx] > 2600){ //��λ���3000����ֵ
                ServoPwmDutySet[pwm_idx] = 2600;
                return;
            }
            ServoSetPluseAndTime_Motor_Control( pwm_idx, ServoPwmDutySet[pwm_idx] + 20, 20);
        }		
    }
}

void Ctrl_Gripper_Motor_Control(U8 dir){ //��е�ۼ�צ����
    
    if (dir == 0x01){        
		ServoSetPluseAndTime_Motor_Control( 1, 600,1500);        
	} else if(dir == 0x00){
		ServoSetPluseAndTime_Motor_Control( 1, 1500, 1500);
    }
}

void Ctrl_Zeroing_Motor_Control(void){   //���ƻ�е�ۻ���
    
    ServoSetPluseAndTime_Motor_Control( 1, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 2, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 3, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 4, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 5, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 6, 1500, 1500);
}

void Grap_Preparation_Motor_Control(void){ //ץȡ׼��
    
    ServoSetPluseAndTime_Motor_Control( 1, 600, 1500);
    ServoSetPluseAndTime_Motor_Control( 2, 1500, 1500);
    ServoSetPluseAndTime_Motor_Control( 3, 2060, 1500);
    ServoSetPluseAndTime_Motor_Control( 4, 2667, 1500);
    ServoSetPluseAndTime_Motor_Control( 5, 2212, 1500);
    ServoSetPluseAndTime_Motor_Control( 6, 1500, 1500);
}

