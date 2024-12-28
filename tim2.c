#include "tim2.h"

void Tim2_Init(void)
{
    TIM_TimeBaseInitTypeDef timer_init;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    timer_init.TIM_Period = 65535; 
    timer_init.TIM_Prescaler = 72 - 1; // Adjusted prescaler for 1us resolution
    timer_init.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM2,&timer_init);
    TIM_Cmd(TIM2,ENABLE); // Enable Timer2
}

void Delay_1ms(void)
{
    TIM_SetCounter(TIM2,0); // Set initial value for Timer2
    while(TIM_GetCounter(TIM2)<1000); // Adjusted delay count for 1ms delay
}

void Delay_ms(unsigned int time)
{
    while(time--){
        Delay_1ms();
    }
}
