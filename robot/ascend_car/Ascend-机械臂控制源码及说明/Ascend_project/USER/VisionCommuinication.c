//
// Created by lenovo on 2023/7/5.
//
#include "include.h"

#define LED        PBout(9)
#define LED_ON 0

#define CMD_GET_TARGET_POSE            0x00    //�����ȡĩ��λλ��
#define CMD_GET_ALL_JOINT              0x01    //�����ȡ�ؽڽǶ�
#define CMD_SET_TARGET_POSE            0x02    //����Ŀ������λ��
#define CMD_CONTROL_ROBOT              0x03    //������ƻ�е��
#define CMD_CONTROL_GRIPPER            0x04    //������Ƽ�צ
#define CMD_CONTROL_ZEROING            0x05    //���ƻ�е�ۻ���
#define CMD_GRAB_PREPARATION           0x06    //ץȡ׼��
 

static bool fUartRxComplete = FALSE;
static uint8 UartRxBuffer[260];
uint8 Uart1RxBuffer[260];

uint8 frameIndexSumSum[256];


static void Init_LED(void) {
    
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PA�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  //LED0-->PC.2 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}


void Init_Vision_Comm(void){  //�Ӿ�����ͨ�ų�ʼ������USART1
    
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    //NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = 9600;  //һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�����ж�

    USART_Cmd(USART1, ENABLE); //ʹ�ܴ���


    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
		
    Init_LED();
    LED = LED_ON;		
}

void Uart1SendData(BYTE dat){ //ͨ����������
    
    while ((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������
    USART1->DR = (u8) dat;
    while ((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������
}

void UART1SendDataPacket(uint8 dat[], uint8 count){ //�������ݰ�  
    
    uint32 i;
    for (i = 0; i < count; i++) {
        while ((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������
        USART1->DR = dat[i];
        while ((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������
    }
}

void USART1_IRQHandler(void){ //�жϴ�����
    
    uint8 i;
    uint8 rxBuf;
    static uint8 startCodeSum = 0;
    static bool fFrameStart = FALSE;
    static uint8 messageLength = 0;
    static uint8 messageLengthSum = 1;
																			
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) {
        return;
    }

    rxBuf = USART_ReceiveData(USART1); //��ȡ���յ�������
    if (!fFrameStart) {
        if (rxBuf == 0x55){ //��ȡ֡ͷ0x55
            startCodeSum++;
            if (startCodeSum == 2){
                startCodeSum = 0;
                fFrameStart = TRUE;
                messageLength = 1;				
            }
        } else {
            fFrameStart = FALSE;
            messageLength = 0;
            startCodeSum = 0;
        }
    }
    
    if (fFrameStart){       
        Uart1RxBuffer[messageLength] = rxBuf;
        if (messageLength == 2) {
            messageLengthSum = Uart1RxBuffer[messageLength];
            if (messageLengthSum < 1){                 
                messageLengthSum = 1;
                fFrameStart = FALSE;
            }
        }
        messageLength++;

        if (messageLength == messageLengthSum + 3){  //��ȡ���      		  
            if (fUartRxComplete == FALSE) {			
                for (i = 0; i < messageLength; i++){                   
                    UartRxBuffer[i] = Uart1RxBuffer[i];   //���յ����ݴ�������
                    fUartRxComplete = TRUE;  
                }               
            }
            fFrameStart = FALSE;
        }
    }
}

void Send_TargetPosition_Vision_Comm(float pos[6]){   //����λ����Ϣ    
    
    U8 buf[30];
	uint8 i;
	uint16 checksum = 0;  //����У��
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = 27;   //���ݳ���
    buf[3] = CMD_GET_TARGET_POSE;  //��������

    for (i = 0; i < 6; i++){
        unsigned int tmp = *((unsigned int *)(&pos[i]));
        buf[i*4+4] = GET_U32_BYTE(tmp, 0);  //��λ����Ϣ�����ֽڲ��
        buf[i*4+5] = GET_U32_BYTE(tmp, 1); 
        buf[i*4+6] = GET_U32_BYTE(tmp, 2); 
        buf[i*4+7] = GET_U32_BYTE(tmp, 3);
    }
    for (i = 0; i < 26; i++){
        checksum += buf[2 + i];
    }

    buf[28] = GET_U16_BYTE(checksum, 0);
    buf[29] = GET_U16_BYTE(checksum, 1);

    UART1SendDataPacket(buf, buf[2] + 3); //���÷������ݰ������������ݷ���ͨ�����ڷ���

}

void Send_Angle_Vision_Comm(float joint[6]){ //���ͽǶ���Ϣ   
    
    U8 buf[30];
	uint8 i;
	uint16 checksum = 0; //����У��
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = 27; //���ݳ���
    buf[3] = CMD_GET_ALL_JOINT; //��������
    
    for (i = 0; i < 6; i++){
        unsigned int tmp = *((unsigned int *)(&joint[i]));
        buf[i*4+4] = GET_U32_BYTE(tmp, 0);   //���ؽڽǶ���Ϣ�����ֽڲ��
        buf[i*4+5] = GET_U32_BYTE(tmp, 1);
        buf[i*4+6] = GET_U32_BYTE(tmp, 2);
        buf[i*4+7] = GET_U32_BYTE(tmp, 3);
    }

    for (i = 0; i < 26; i++){
        checksum += buf[2 + i];
    }

    buf[28] = GET_U16_BYTE(checksum, 0);
    buf[29] = GET_U16_BYTE(checksum, 1);

    UART1SendDataPacket(buf, buf[2] + 3); //���÷������ݰ������������ݷ���ͨ�����ڷ���
}

void MsgHandle_Vision_Comm(void){  //�Ӿ���Ϣ����

    uint8 cmd;
    float Angle[6];
    float pose[6];
    if (!fUartRxComplete){
        return;
    }
    
    cmd = UartRxBuffer[3];  //��������
    switch (cmd){
        case CMD_GET_TARGET_POSE: //�����ȡĩ��λ��
        {
                     
            Calculate_TargetPosition_Motor_Control(pose);
            Send_TargetPosition_Vision_Comm(pose);
            break;
        }
        
        case CMD_GET_ALL_JOINT:  //�����ȡ��е�۽Ƕ�
        {    
            
            _Pluse2Angle(Angle);            //����ת���ɽǶ�
            Send_Angle_Vision_Comm(Angle);  //���ջ�е�۹ؽڽǶ�
            break;
        }      

        case CMD_SET_TARGET_POSE:  //����Ŀ������λ��
        {
            uint32 tmp = 0;
            
            tmp = SET_U32_BYTE(tmp, 0, UartRxBuffer[4]);
            tmp = SET_U32_BYTE(tmp, 1, UartRxBuffer[5]);
            tmp = SET_U32_BYTE(tmp, 2, UartRxBuffer[6]);
            tmp = SET_U32_BYTE(tmp, 3, UartRxBuffer[7]);
            pose[0] = *((float *)(&tmp));
            
            tmp = 0;
            tmp = SET_U32_BYTE(tmp, 0, UartRxBuffer[8]);
            tmp = SET_U32_BYTE(tmp, 1, UartRxBuffer[9]);
            tmp = SET_U32_BYTE(tmp, 2, UartRxBuffer[10]);
            tmp = SET_U32_BYTE(tmp, 3, UartRxBuffer[11]);
            pose[1] = *((float *)(&tmp));
            
            tmp = 0;
            tmp = SET_U32_BYTE(tmp, 0, UartRxBuffer[12]);
            tmp = SET_U32_BYTE(tmp, 1, UartRxBuffer[13]);
            tmp = SET_U32_BYTE(tmp, 2, UartRxBuffer[14]);
            tmp = SET_U32_BYTE(tmp, 3, UartRxBuffer[15]);
            pose[2] = *((float *)(&tmp));
            
//            pose[3] = -150.0000F;
//            pose[4] = 20.0000F;
//            pose[5] = 90.0000F;
            pose[3] =  -178.65F;
            pose[4] =  0.0000F;
            pose[5] =  90.0000F;
            Goto_TargetPose_Motor_Control(pose);//�������
            break;
        } 

        case CMD_CONTROL_ROBOT:   //������ƶ��
        {
            Ctrl_Joint_Motor_Control(UartRxBuffer[5], UartRxBuffer[4]);
            break;
        }

        case CMD_CONTROL_GRIPPER: //������Ƽ�צ
        {
                
            Ctrl_Gripper_Motor_Control(UartRxBuffer[4]);
            break;
        }       
        
        case CMD_CONTROL_ZEROING: //���ƻ�е�ۻ���
        { 
             
            Ctrl_Zeroing_Motor_Control();
            break;
        }
        
        case CMD_GRAB_PREPARATION: //ץȡ׼��
        {
             
            Grap_Preparation_Motor_Control();
            break;
        }
        default:
            break;
    }
    fUartRxComplete = FALSE;
}


void TaskRun_Vision_Comm(void){
    
    if (IsTimeStep20msDone()){
        ServoPwmDutyCompare_Motor_Control();  //�����ٶȿ��ƺ���
    }

    if (IsTimeStep1000msDone()){
        LED = ~LED; //LED������
    }

    MsgHandle_Vision_Comm();  //��Ϣ����
}

