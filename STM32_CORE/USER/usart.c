#include "sys.h"
#include "usart.h"
#define EN_USART1_RX
//////////////////////////////////////////////////////////////////////////////////	 
//����1��ʼ��		   
//�޸�����:2010/5/27
//�汾��V1.3
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
////////////////////////////////////////////////////////////////////////////////// 	  


//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/ 

void uart_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

#ifdef EN_USART1_RX   //���ʹ���˽���	
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

#ifdef EN_USART1_RX   //���ʹ���˽���	
   //Usart1 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
#endif
  
   //USART ��ʼ������
   
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   
#ifdef EN_USART1_RX   //���ʹ���˽���	
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
#endif

    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}

#ifdef EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 USART_RX_BUF[64];     //���ջ���,���64���ֽ�.
//����״̬
//bit7��������ɱ�־
//bit6�����յ�0x0d
//bit5~0�����յ�����Ч�ֽ���Ŀ
u8 USART_RX_STA=0;       //����״̬���

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8	gc;
//	GPIO_SetBits(GPIOC,GPIO_Pin_6);
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{	
		gc=USART_ReceiveData(USART1);
    recvchar(gc);
  }
//	u8 Res;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//		{
//		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
//		
//		if((USART_RX_STA&0x80)==0)//����δ���
//			{
//			if(USART_RX_STA&0x40)//���յ���0x0d
//				{
//				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA|=0x80;	//��������� 
//				}
//			else //��û�յ�0X0D
//				{	
//				if(Res==0x0d)USART_RX_STA|=0x40;
//				else
//					{
//					USART_RX_BUF[USART_RX_STA&0X3F]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>63)USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//					}		 
//				}
//			}   		 
//     } 
} 

#endif


void PrintChar(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		UsartSend(*p);
		p++;
	}
}


void UsartSend(u16 ch)
{
	USART1->DR = (ch&(u16)0x01FF);   
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
   
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	 USART1->DR = (DataToSend & (uint16_t)0x01FF);
//	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & ((uint16_t)0x0080)));
}





/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
				,int16_t alt,int16_t tempr,int16_t press)
*��������:		����λ�����;�����������̬����
���������
		int16_t yaw ���������ĺ���Ƕȡ���λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
		int16_t pitch ����õ��ĸ����Ƕȣ���λ 0.1�ȡ�-900 - 900 ��Ӧ -90.0 -> 90.0 ��
		int16_t roll  �����õ��ĺ���Ƕȣ���λ0.1�ȡ� -1800 -> 1800 ��Ӧ -180.0  ->  180.0��
		int16_t alt   ��ѹ�߶ȡ� ��λ0.1�ס�  ��Χһ�����ͱ���
		int16_t tempr �¶� �� ��λ0.1���϶�   ��Χ��ֱ����ĵ�·�岻����������
		int16_t press ��ѹѹ������λ10Pa  һ������ѹǿ��101300pa ����Ѿ�����һ�����͵ķ�Χ����Ҫ����10�ٷ�����λ��
		int16_t IMUpersec  ��̬�������ʡ�����IMUpersecÿ�롣
���������û��	
*******************************************************************************/
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+2);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

