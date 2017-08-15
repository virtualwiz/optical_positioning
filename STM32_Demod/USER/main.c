#define ABS(x)  ((x)>0?(x):-(x)) 
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "lcd.h"
#include "stm32_dsp.h"
#include "table_fft.h"
#include "math.h"
#define PI2 6.28
#define NPT 256
#define Fs 44800
#define EXPR 200 * sin(PI2 *i * 5400 / Fs) + 300 * sin(PI2 *i * 15000 / Fs)
#define STR(a) #a
#define _STR(a) STR(a)

long lBufInArray[NPT];
long lBufOutArray[NPT];
unsigned long lBufMagArray[NPT / 2];

void Timer_Init(u16 arr,u16 psc)
{    
	RCC->APB2ENR|= 1<<2;			//GPIOAÊ±ÖÓÊ¹ÄÜ
	RCC->APB1ENR |=1<<1;      //TIM3Ê±ÖÓÊ¹ÄÜ     
	GPIOA->CRL&=0X0FFFFFFF;//PA7Êä³ö
	GPIOA->CRL|=0XB0000000;//¸´ÓÃ¹¦ÄÜÊä³ö PWMÄ£Ê½ 
		
	
	TIM3->ARR=arr;  //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ 
	TIM3->PSC=psc;  //Ô¤·ÖÆµÆ÷
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2Ä£Ê½£¨¸ßµçÆ½ÎªÕ¼¿Õ±È£©  
	TIM3->CCMR1|=1<<11; //CH2Ô¤×°ÔØÊ¹ÄÜ    
	TIM3->CCER|=1<<4;   //OC2 Êä³öÊ¹ÄÜ    
	TIM3->CR1=0x8000;   //ARPEÊ¹ÄÜ 
	TIM3->CR1|=0x01;    //¿ª¶¨Ê±Æ÷3    
} 

void Timer_Config(int freq,int duty)
{
			TIM3->ARR = 200000 / freq;
			TIM3->CCR2 = (2000 / freq) * duty;
}

void GPIO_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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
int peak_ptr_old;

int main(void)
{
    int i;
    int freq_ptr;
    delay_init();                                   //��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(9600);
    ADC_Config();
    GPIO_Config();
    //LCD_Init();
		Timer_Init(1000,719);

    while (1)
    {
        PrepareFftData();
        cr4_fft_256_stm32(lBufOutArray, lBufInArray, NPT);
        GetPowerMag();
//        for (i = 0; i <= 127; i++)
//        {
//            LCD_Fill(i * 2, 0, i * 2 + 1, 320, WHITE);
//            LCD_DrawLine(i * 2, 280 - lBufMagArray[i] / 4,
//                         i * 2 + 1, 280 - lBufMagArray[i + 1] / 4);
//        }
        peak = 0;

        for (freq_ptr = 25; freq_ptr <= 106; freq_ptr++)
        {
            if (lBufMagArray[freq_ptr] > peak)
            {
                peak = lBufMagArray[freq_ptr];
                peak_ptr = freq_ptr;
            }
        }
				if(ABS(peak_ptr - peak_ptr_old)>=2)
					TIM3->CNT = 0;
				peak_ptr_old = peak_ptr;
				if(peak>=700)
					Timer_Config(18*peak_ptr,50);
				else 
					Timer_Config(18*peak_ptr,0);
    }
}
