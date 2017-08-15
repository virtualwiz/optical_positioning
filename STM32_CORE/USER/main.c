#define ABS(x)  ((x)>0?(x):-(x)) 
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "stm32_dsp.h"
#include "table_fft.h"
#include "math.h"

#define SONAR_X 0
#define SONAR_Y 1

#define NPT 256

#define LOWPASS_WINDOW_LENGTH_1 20
#define LOWPASS_WINDOW_LENGTH_2 20

unsigned int value_buf_1[LOWPASS_WINDOW_LENGTH_1]; 
unsigned int value_buf_2[LOWPASS_WINDOW_LENGTH_2];

unsigned int Filter_1(unsigned int Data_In) 
{
	static char i = 0;
	char count; 
	unsigned int sum=0; 
	value_buf_1[i++] = Data_In; 
	if ( i == LOWPASS_WINDOW_LENGTH_1 )   
		i = 0;
	for ( count=0 ; count < LOWPASS_WINDOW_LENGTH_1 ; count++) 
		sum += value_buf_1[count];
	return (unsigned int)(sum/LOWPASS_WINDOW_LENGTH_1); 
}

unsigned int Filter_2(unsigned int Data_In) 
{
	static char i = 0;
	char count; 
	unsigned int sum=0; 
	value_buf_2[i++] = Data_In; 
	if ( i == LOWPASS_WINDOW_LENGTH_2 )   
		i = 0;
	for ( count=0 ; count < LOWPASS_WINDOW_LENGTH_2 ; count++) 
		sum += value_buf_2[count];
	return (unsigned int)(sum/LOWPASS_WINDOW_LENGTH_2); 
}

long lBufInArray[NPT];
long lBufOutArray[NPT];
unsigned long lBufMagArray[NPT / 2];

typedef struct
{
	char Area;
	float Coord_X;
	float Coord_Y;
	unsigned char DemodMode;
	unsigned char Channel;
	char DtmfResult;
} Display_InitTypeDef;

extern u8 TIM2CH1_CAPTURE_STA;
extern u16 TIM2CH1_CAPTURE_VAL;
extern u8 TIM3CH1_CAPTURE_STA;
extern u16 TIM3CH1_CAPTURE_VAL;

char CmdFromYuDalao[9] = "?????????";
char isCmd = 0;
void recvchar(u8 gc)     //´¦Àí´®¿ÚÁ÷Ê½Êý¾Ý
{	
	static int ptr = 0;
	if(gc == '$')
	{
		isCmd = 1;
	}
	else if(isCmd)
	{
		CmdFromYuDalao[ptr] = gc;
		++ptr;
		if(ptr == 9)
		{
			ptr = 0;
			isCmd = 0;
		}
	}
}

void IO_Configure()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

	//TRIG

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//ECHO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM2_Config(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //Ê¹ÄÜTIM2Ê±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ

	TIM_TimeBaseStructure.TIM_Period = arr;						//Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//Ô¤·ÖÆµÆ÷
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//¸ù¾ÝTIM_TimeBaseInitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;				 //CC1S=01 	Ñ¡ÔñÊäÈë¶Ë IC1Ó³Éäµ½TI1ÉÏ
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //ÉÏÉýÑØ²¶»ñ
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //Ó³Éäµ½TI1ÉÏ
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			  //TIM2ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //ÏÈÕ¼ÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //´ÓÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);							  //¸ù¾ÝNVIC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷

	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE); //ÔÊÐí¸üÐÂÖÐ¶Ï ,ÔÊÐíCC1IE²¶»ñÖÐ¶Ï
	//TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC3, ENABLE); //ÔÊÐí¸üÐÂÖÐ¶Ï ,ÔÊÐíCC1IE²¶»ñÖÐ¶Ï

	TIM_Cmd(TIM2, ENABLE); //Ê¹ÄÜ¶¨Ê±Æ÷2
}

void TIM3_Config(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //Ê¹ÄÜTIM3Ê±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ

	TIM_TimeBaseStructure.TIM_Period = arr;						//Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//Ô¤·ÖÆµÆ÷
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);				//¸ù¾ÝTIM_TimeBaseInitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»

	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;				 //CC1S=01 	Ñ¡ÔñÊäÈë¶Ë IC1Ó³Éäµ½TI1ÉÏ
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //ÉÏÉýÑØ²¶»ñ
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //Ó³Éäµ½TI1ÉÏ
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;			  //TIM3ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //ÏÈÕ¼ÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //´ÓÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);							  //¸ù¾ÝNVIC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷

	TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC1, ENABLE); //ÔÊÐí¸üÐÂÖÐ¶Ï ,ÔÊÐíCC1IE²¶»ñÖÐ¶Ï
	//TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC3, ENABLE); //ÔÊÐí¸üÐÂÖÐ¶Ï ,ÔÊÐíCC1IE²¶»ñÖÐ¶Ï

	TIM_Cmd(TIM3, ENABLE); //Ê¹ÄÜ¶¨Ê±Æ÷2
}

void TIM_Capture_Restart()
{
	TIM2CH1_CAPTURE_STA = 0;
	TIM3CH1_CAPTURE_STA = 0;
}
void Sonic_Ping(unsigned char sensor)
{
	if (sensor)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_10);
		delay_us(15);
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		delay_us(15);
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	}
}

char USART_Transmit_Buffer[30];
void Display_Update()
{
	int cx;
		for(cx=0;cx<30;cx++)
		{
			UART1_Put_Char(USART_Transmit_Buffer[cx]);
		}
}

void ADC_Config()
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;
}
unsigned int DBG_ADC_VALUE;
unsigned int ADC_GetSingleVoltage()
{
    unsigned int ResultVolt = 0;
    unsigned char i;
    for (i = 0; i < 64; i++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
            ;
        ResultVolt += (unsigned int)ADC_GetConversionValue(ADC1);
    }
    ResultVolt = ResultVolt >> 5;
    //ResultVolt = (ResultVolt * 3300) >> 10;
    DBG_ADC_VALUE = ResultVolt;
    return ResultVolt;
}

void PrepareFftData()
{
    unsigned int ptr;
    for (ptr = 0; ptr < NPT; ptr++)
    {
        lBufInArray[ptr] = (long)ADC_GetSingleVoltage();
    }
}

void GetPowerMag()
{
    signed short lX, lY;
    float X, Y, Mag;
    unsigned short i;
    for (i = 0; i < NPT / 2; i++)
    {
        lX = (lBufOutArray[i] << 16) >> 16;
        lY = (lBufOutArray[i] >> 16);
        X = NPT * ((float)lX) / 32768;
        Y = NPT * ((float)lY) / 32768;
        Mag = sqrt(X * X + Y * Y) / NPT;
        if (i == 0)
            lBufMagArray[i] = (unsigned long)(Mag * 32768);
        else
            lBufMagArray[i] = (unsigned long)(Mag * 65536);
    }
}

unsigned int peak;
int peak_ptr;

int Dtmf[10] = {50, 55, 60, 65, 70, 75, 80, 85, 90, 95};
int DtmfDiff[10];
int DtmfDiffMin;

int main(void)
{
	int i;
  int freq_ptr;
	Display_InitTypeDef Display_InitStructure;
	delay_init();									//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	IO_Configure();
	TIM2_Config(0xFFFF, 10);
	TIM3_Config(0xFFFF, 10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	uart_init(115200);
	ADC_Config();
	while (1)
	{
		PrepareFftData();
		cr4_fft_256_stm32(lBufOutArray, lBufInArray, NPT);
		GetPowerMag();

		peak = 0;

		for (freq_ptr = 45; freq_ptr <= 115; freq_ptr++)
		{
				if (lBufMagArray[freq_ptr] > peak)
				{
						peak = lBufMagArray[freq_ptr];
						peak_ptr = freq_ptr;
				}
		}

//		for (i = 0; i <= 9; i++)
//		{
//				if (ABS(peak_ptr - Dtmf[i]) <= 5)
//				{
//						peak = i;
//						break;
//				}
//		}
		
		for (i = 0; i <= 9; i++)
		{
			DtmfDiff[i] = peak_ptr - Dtmf[i];
		}
		DtmfDiffMin = 999;
		for (i = 0; i <= 9; i++)
		{
			if(ABS(DtmfDiff[i])<DtmfDiffMin)
			{
				DtmfDiffMin = DtmfDiff[i];
				peak = i;
			}
		}		
		
		
		TIM_Capture_Restart();
		delay_ms(8);
		Sonic_Ping(SONAR_X);
		delay_ms(8);
		Sonic_Ping(SONAR_Y);
		delay_ms(8);
		
		Display_InitStructure.Coord_X = 2.7283e-3 * Filter_1(TIM2CH1_CAPTURE_VAL);
		Display_InitStructure.Coord_Y = 2.7283e-3 * Filter_2(TIM3CH1_CAPTURE_VAL);
		if(Display_InitStructure.Coord_X == 0 || Display_InitStructure.Coord_Y == 0)
		{
				TIM2_Config(0xFFFF, 10);
				TIM3_Config(0xFFFF, 10);
		}
		Display_InitStructure.Coord_X=Display_InitStructure.Coord_X-40+2;
		Display_InitStructure.Coord_Y=40-Display_InitStructure.Coord_Y-2;
		if(Display_InitStructure.Coord_X<20&&Display_InitStructure.Coord_X>-20&&Display_InitStructure.Coord_Y<20&&Display_InitStructure.Coord_Y>-20)
			Display_InitStructure.Area='A';
		else
		{
       if(Display_InitStructure.Coord_Y>0&&Display_InitStructure.Coord_X>0)
       {
         if( Display_InitStructure.Coord_Y>Display_InitStructure.Coord_X)
         {
            Display_InitStructure.Area='B';
         }
         else
         {
           Display_InitStructure.Area='C';
         }
        
       }
       else if(Display_InitStructure.Coord_Y>0&&Display_InitStructure.Coord_X<0)
       {
          if(ABS(Display_InitStructure.Coord_Y)>(ABS(Display_InitStructure.Coord_X)))
         {
          Display_InitStructure.Area='B';
         }
         else
         {
          Display_InitStructure.Area='E';
         }
       }
       else if(Display_InitStructure.Coord_Y<0&&Display_InitStructure.Coord_X<0)
       {
         if( (ABS(Display_InitStructure.Coord_Y))>(ABS(Display_InitStructure.Coord_X)))
         {
          Display_InitStructure.Area='D';
         }
         else
         {
          Display_InitStructure.Area='E';
         }
       }
       else
       {
          if( (ABS(Display_InitStructure.Coord_Y))>(ABS(Display_InitStructure.Coord_X)))
         {
          Display_InitStructure.Area='D';
         }
         else
         {
          Display_InitStructure.Area='C';
         }
			 }
		}
		
		
if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))
{
		sprintf(USART_Transmit_Buffer,"DS48(70,113,'%c',7,0);\r\n",CmdFromYuDalao[0]);
		Display_Update();
		
		sprintf(USART_Transmit_Buffer,"DS48(177,113,'%c%c%c.%c',7,0);\r\n",CmdFromYuDalao[1],CmdFromYuDalao[2],CmdFromYuDalao[3],CmdFromYuDalao[4]);
		Display_Update();
		
		sprintf(USART_Transmit_Buffer,"DS48(328,113,'%c%c%c.%c',7,0);\r\n",CmdFromYuDalao[5],CmdFromYuDalao[6],CmdFromYuDalao[7],CmdFromYuDalao[8]);
		Display_Update();
}
else
{
		sprintf(USART_Transmit_Buffer,"DS48(70,113,'%c',7,0);\r\n",Display_InitStructure.Area);
		Display_Update();
		
		sprintf(USART_Transmit_Buffer,"DS48(177,113,'%.1f ',7,0);\r\n",Display_InitStructure.Coord_X>100? 99.9:Display_InitStructure.Coord_X);
		Display_Update();
		
		sprintf(USART_Transmit_Buffer,"DS48(328,113,'%.1f ',7,0);\r\n",Display_InitStructure.Coord_Y>100? 99.9:Display_InitStructure.Coord_Y);
		Display_Update();
}

		delay_ms(10);
		sprintf(USART_Transmit_Buffer,"DS48(70,235,'%d',7,0);\r\n",peak<10?peak:0);
		Display_Update();
		
	}
}
