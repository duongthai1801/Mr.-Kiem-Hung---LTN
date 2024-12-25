#ifndef __I2C_LCD_H
#define __I2C_LCD_H

#include "stm32f10x.h"
#include "tim2.h"

//#define PCF8574A_Address      	0x4E
#define I2C_Chanel      		I2C2

void I2C2_LCD_Init(void);
void I2C2_Send_Data(uint8_t data);
void I2C2_Send_Address(uint8_t addr);
void I2C2_Send_Start(void);
void I2C2_Send_Stop(void);
void LCD_Write_byte(char data);
void LCD_Init (void);
void LCD_Data_Write(char data);
void LCD_Control_Write(char data);
void LCD_Send_String (char *str);
void LCD_Clear(void);

// Add scanner prototype
uint8_t I2C2_Scan_Address(void);
extern uint8_t lcd_address; // Add global address variable

#endif
