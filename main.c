#include "stm32f10x.h"
#include "uart.h"
#include "tim2.h"
#include "i2c_lcd.h"
#include "gpio.h"
#include "mma845.h"

void System_Init(void);

int main(void) {
    System_Init();
    USART1_Send_String("System initialized\r\n");
    
    // Initialize I2C with error checking
    I2C1_Init();
    USART1_Send_String("I2C1 initialized\r\n");
    
    // Initialize MMA8451 with error checking
    if(MMA8451_Init() != 1) {
        USART1_Send_String("MMA8451 initialization failed\r\n");
        while(1);
    }
    USART1_Send_String("MMA8451 initialized\r\n");
    
    int16_t x, y, z;
    float ax, ay, az;   
    while(1) 
		{
        // Read accelerometer data
        MMA8451_Read_Accel(&x, &y, &z);
        
        // Convert to g values
        ax = MMA8451_Convert_Accel(x);
        ay = MMA8451_Convert_Accel(y);
        az = MMA8451_Convert_Accel(z);
        
        
        // Send to UART
        USART1_Send_String("X: ");
        USART1_Send_Number((int)(ax * 1000));
        USART1_Send_String(" Y: ");
        USART1_Send_Number((int)(ay * 1000));
        USART1_Send_String(" Z: ");
        USART1_Send_Number((int)(az * 1000));
        USART1_Send_String("\r\n");
        
        Delay_ms(100);  // Update every 100ms
    }
}

void System_Init(void) {
    GPIO_init();
    Tim2_Init();
    USART1_Init(9600);
    I2C2_LCD_Init();
    LCD_Init();
    LCD_Clear();
    LCD_Send_String("Starting...");
    Delay_ms(1000);
}