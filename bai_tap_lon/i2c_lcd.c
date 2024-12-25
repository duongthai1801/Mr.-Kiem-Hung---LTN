#include "i2c_lcd.h"

#define I2C_TIMEOUT 10000
uint8_t lcd_address = 0; // Global variable for LCD address

uint8_t I2C2_Scan_Address(void) {
    uint8_t address;
    uint8_t found = 0;
    
    USART1_Send_String("Scanning I2C2 LCD addresses...\r\n");
    
    // PCF8574 possible addresses (0x20-0x27, 0x38-0x3F)
    for(address = 0x20; address <= 0x3F; address++) {
        // Skip addresses between 0x28-0x37
        if(address == 0x28) address = 0x38;
        
        uint32_t timeout = I2C_TIMEOUT;
        
        // Start condition
        I2C2_Send_Start();
        
        // Send address (already in 8-bit format for write)
        I2C2->DR = (address << 1);
        
        // Wait for ADDR or AF (Address Fail)
        while(!(I2C2->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) {
            if(--timeout == 0) break;
        }
        
        if(I2C2->SR1 & I2C_SR1_ADDR) {
            // Device responded
            (void)I2C2->SR2; // Clear ADDR flag
            USART1_Send_String("LCD found at address: 0x");
            USART1_Send_Number(address);
            USART1_Send_String("\r\n");
            found = address;
            break;
        }
        
        // Stop and clear flags
        I2C2_Send_Stop();
        I2C2->SR1 &= ~I2C_SR1_AF;
        Delay_ms(5);
    }
    
    if(!found) {
        USART1_Send_String("LCD not found, using default address 0x4E\r\n");
        return 0x27; // 0x4E >> 1 (7-bit address)
    }
    
    return found;
}

void I2C2_LCD_Init(void)
{
    // Enable clocks for I2C2 and GPIOB
    RCC->APB1ENR |= (1 << 22); // RCC_APB1ENR_I2C2EN
    RCC->APB2ENR |= (1 << 3);  // RCC_APB2ENR_IOPBEN

    // Configure PB10 (SCL) and PB11 (SDA) as alternate function open-drain
    GPIOB->CRH &= ~((0xF << 8) | (0xF << 12)); // Clear MODE10, CNF10, MODE11, CNF11
    GPIOB->CRH |= ((0xB << 8) | (0xB << 12));  // Set MODE10, CNF10, MODE11, CNF11 to 0b1011 (50MHz, AF_OD)

    // Reset I2C2
    I2C2->CR1 |= (1 << 15); // I2C_CR1_SWRST
    I2C2->CR1 &= ~(1 << 15); // Clear I2C_CR1_SWRST

    // Configure I2C2
    I2C2->CR2 = (36 & 0x3F); // PCLK1 frequency in MHz (36 MHz)
    I2C2->CCR = 180; // 100kHz I2C clock
    I2C2->TRISE = 37; // Maximum rise time

    // Configure I2C2 control registers
    I2C2->OAR1 = (1 << 14); // 7-bit address mode, own address 0
    I2C2->CR1 = (1 << 0); // Enable I2C2
}

void I2C2_Send_Data(uint8_t data)
{
	I2C2->DR = data;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
}

void I2C2_Send_Address(uint8_t addr)
{
	I2C2->DR = addr;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	volatile uint32_t temp = I2C2->SR2;
}

void I2C2_Send_Start(void)
{
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
}

void I2C2_Send_Stop(void)
{
	I2C2->CR1 |= I2C_CR1_STOP;
}

void LCD_Write_byte(char data)
{
	I2C2_Send_Start();
	I2C2_Send_Address(lcd_address);
	I2C2_Send_Data(data);
	I2C2_Send_Stop();
}

void LCD_Init (void)
{
    // Scan for LCD address
    lcd_address = I2C2_Scan_Address();
    if(!lcd_address) {
        USART1_Send_String("LCD not found!\r\n");
        return;
    }
    
    lcd_address <<= 1; // Convert to 8-bit address
    
	/* Set 4-bits interface */
	LCD_Control_Write(0x33);		 
	Delay_ms(10);
	LCD_Control_Write(0x32);
	Delay_ms(50);
	/* Start to set LCD function */
	LCD_Control_Write(0x28);
		Delay_ms(50);	
	/* clear LCD */
	LCD_Control_Write(0x01);
		Delay_ms(50);
	/* wait 60ms */
	
	/* set entry mode */
	LCD_Control_Write(0x06);
		Delay_ms(50);;
	/* set display to on */	
	LCD_Control_Write(0x0C);
		Delay_ms(50);;
	/* move cursor to home and set data address to 0 */
	LCD_Control_Write(0x02);
		Delay_ms(50);
}

void LCD_Data_Write(char data)
{
	char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x0d;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0d;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	    LCD_Write_byte(data_t[i]);
	}

}

void LCD_Control_Write(char data)
{
    char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x04;  //en=1, rs=0
	data_t[1] = data_u;  //en=0, rs=0
	data_t[2] = data_l|0x04;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	    LCD_Write_byte(data_t[i]);
	}

}

void LCD_Send_String (char *str)
{
	while (*str) 
	{
		LCD_Data_Write (*str++);
	}
}

void LCD_Clear(void)
{
	LCD_Control_Write(0x01);
	Delay_ms(10);
}
