#ifndef __MMA845__
#define __MMA845__

#include "uart.h"
#include <stdint.h>
#include <math.h>

// MMA8451 Definitions
#define WHO_AM_I        0x0D
#define OUT_X_MSB       0x01
#define CTRL_REG1       0x2A
#define XYZ_DATA_CFG    0x0E

// Function prototypes with return types
void I2C1_Init(void);
uint8_t I2C1_Start(void);
void I2C1_Stop(void);
uint8_t I2C1_Write_Addr(uint8_t addr);
void I2C1_Write(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C1_Read(uint8_t addr, uint8_t reg);
uint8_t MMA8451_Init(void);
void MMA8451_Read_Accel(int16_t *x, int16_t *y, int16_t *z);
float MMA8451_Convert_Accel(int16_t accel);
uint8_t I2C1_Scan_Address(void);

#endif