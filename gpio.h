#ifndef __GPIO__
#define __GPIO__

#include "stm32f10x.h" 

#define LED_PORT     GPIOC
#define LED_START    GPIO_Pin_13 
#define LED_END      GPIO_Pin_14

#define BUTTON_PORT  GPIOB
#define BUTTON_START GPIO_Pin_6
#define BUTTON_END   GPIO_Pin_5 

void GPIO_init(void );
void Set_Pin(uint16_t pin);
void Reset_Pin(uint16_t pin);
uint16_t Read_Pin(uint16_t pin);

#endif
