#ifndef __TIM2_H__
#define __TIM2_H__

#ifdef __cplusplus
 extern "C" {
#endif
	#include "stm32f10x.h"                  // Device header
	#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
	#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
	#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
	
	
	void Tim2_Init(void);
	void Delay_ms(unsigned int time);
	void Delay_1ms(void);


#ifdef __cplusplus
}
#endif

#endif
