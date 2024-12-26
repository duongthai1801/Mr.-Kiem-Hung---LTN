#include "stm32f10x.h"

void GPIO_init(void)
{
    // Enable clock for GPIOC (LED port)
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    
    // Configure LED pins (PC13, PC14) as output push-pull, max speed 50MHz
    GPIOC->CRH &= ~(0xFF << ((13-8)*4));  // Clear bits for PC13
    GPIOC->CRH |= (0x03 << ((13-8)*4));   // Output mode 50MHz
    GPIOC->CRH &= ~(0xFF << ((14-8)*4));  // Clear bits for PC14
    GPIOC->CRH |= (0x03 << ((14-8)*4));   // Output mode 50MHz
    
    // Enable clock for GPIOB (Button port)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    
    // Configure button pins (PB12, PB13) as input pull-up
    GPIOB->CRH &= ~(0xFF << ((12-8)*4));  // Clear bits for PB12
    GPIOB->CRH |= (0x08 << ((12-8)*4));   // Input mode with pull-up
    GPIOB->CRH &= ~(0xFF << ((13-8)*4));  // Clear bits for PB13
    GPIOB->CRH |= (0x08 << ((13-8)*4));   // Input mode with pull-up
    
    // Enable pull-up for buttons
    GPIOB->ODR |= (1<<12) | (1<<13);
    
    // Initialize LEDs to OFF state
    GPIOC->BSRR = (1<<13) | (1<<14);
}

void Set_Pin(uint16_t pin)
{
    GPIOC->BRR = pin;    // Reset bit = LED ON
}

void Reset_Pin(uint16_t pin)
{
    GPIOC->BSRR = pin;   // Set bit = LED OFF
}

uint16_t Read_Pin(uint16_t pin)
{
    return GPIOB->IDR & pin;  // Read input data register
}
