#include "mma845.h"

#define I2C_TIMEOUT 10000

// Status tracking
static uint8_t i2c_error = 0;
static uint8_t mma8451_address = 0;  // Global variable to store device address

void I2C1_Init(void) {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // Configure GPIO first (PB6=SCL, PB7=SDA)
    GPIOB->CRL &= ~(0xFF << 24);  // Clear PB6,PB7
    GPIOB->CRL |= (0xDD << 24);   // Alternate Open-drain, Pull-up enabled
    
    // Set pins high initially
    GPIOB->ODR |= (1<<6) | (1<<7);
    
    // Reset I2C
    I2C1->CR1 |= I2C_CR1_SWRST;
    Delay_ms(10);
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    
    // Configure I2C
    I2C1->CR2 = 36;               // 36MHz PCLK1
    I2C1->CCR = 180;              // 100kHz in standard mode
    I2C1->TRISE = 37;             // Maximum rise time
    
    // Enable I2C with longer timeout
    I2C1->CR1 |= I2C_CR1_PE;
    Delay_ms(100);                // Wait for bus to stabilize
    
    USART1_Send_String("I2C1 Initialized\r\n");
}

uint8_t I2C1_Scan_Address(void) {
    uint8_t address;
    uint8_t found = 0;
    
    USART1_Send_String("Scanning for MMA8451...\r\n");
    
    // Only scan MMA8451 possible addresses
    for(address = 0x1C; address <= 0x1D; address++) {
        I2C1_Start();
        
        // Send write address
        I2C1->DR = (address << 1);
        uint32_t timeout = I2C_TIMEOUT;
        
        while(!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) {
            if(--timeout == 0) break;
        }
        
        if(I2C1->SR1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;
            USART1_Send_String("MMA8451 candidate at: 0x");
            USART1_Send_Number(address);
            USART1_Send_String("\r\n");
            found = address;
            break;
        }
        
        I2C1_Stop();
        I2C1->SR1 &= ~I2C_SR1_AF;
        Delay_ms(10);
    }
    
    if(!found) {
        USART1_Send_String("MMA8451 not found at expected addresses\r\n");
    }
    
    return found;
}

uint8_t I2C1_Start(void) {
    uint32_t timeout = I2C_TIMEOUT * 10; // Increase timeout
    
    // Clear error flags
    I2C1->SR1 = 0;
    I2C1->SR2 = 0;
    
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB)) {
        if(--timeout == 0) {
            // Recovery procedure
            I2C1->CR1 &= ~I2C_CR1_PE;  // Disable I2C
            Delay_ms(10);
            I2C1->CR1 |= I2C_CR1_PE;   // Re-enable I2C
            USART1_Send_String("Start Condition Timeout - Recovery attempted\r\n");
            return 1;
        }
    }
    return 0;
}

void I2C1_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

uint8_t I2C1_Write_Addr(uint8_t addr) {
    uint32_t timeout = I2C_TIMEOUT;
    
    I2C1->DR = addr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if(--timeout == 0) {
            USART1_Send_String("Address Write Timeout\r\n");
            return 1;
        }
        if(I2C1->SR1 & I2C_SR1_AF) {
            USART1_Send_String("No ACK received\r\n");
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 1;
        }
    }
    (void)I2C1->SR2;
    return 0;
}

uint8_t I2C1_Read(uint8_t addr, uint8_t reg) {
    uint8_t data;
    uint32_t timeout = I2C_TIMEOUT;
    
    if(I2C1_Start()) return 0;
    if(I2C1_Write_Addr(addr)) return 0;
    
    // Write register address
    I2C1->DR = reg;
    while(!(I2C1->SR1 & I2C_SR1_TXE)) {
        if(--timeout == 0) return 0;
    }
    
    if(I2C1_Start()) return 0;
    if(I2C1_Write_Addr(addr | 0x01)) return 0;
    
    // Disable ACK
    I2C1->CR1 &= ~I2C_CR1_ACK;
    
    // Wait for data
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_RXNE)) {
        if(--timeout == 0) return 0;
    }
    
    data = I2C1->DR;
    I2C1_Stop();
    
    return data;
}

void I2C1_Write(uint8_t addr, uint8_t reg, uint8_t data) {
    if(I2C1_Start()) return;
    if(I2C1_Write_Addr(addr)) return;
    
    I2C1->DR = reg;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    
    I2C1_Stop();
}

uint8_t MMA8451_Init(void) {
    uint8_t who_am_i;
    
    USART1_Send_String("Starting MMA8451 init\r\n");
    
    // Find device address
    mma8451_address = I2C1_Scan_Address() << 1;
    if(!mma8451_address) {
        USART1_Send_String("Device not found\r\n");
        return 0;
    }
    
    // Read WHO_AM_I
    who_am_i = I2C1_Read(mma8451_address, WHO_AM_I);
    USART1_Send_String("WHO_AM_I = 0x");
    USART1_Send_Number(who_am_i);
    USART1_Send_String("\r\n");
    
    // if(who_am_i != 0x1A) {
    //     USART1_Send_String("Wrong ID\r\n");
    //     return 0;
    // }
    
    // Configure device
    I2C1_Write(mma8451_address, CTRL_REG1, 0x00);    // Standby
    I2C1_Write(mma8451_address, XYZ_DATA_CFG, 0x00); // 2g range
    I2C1_Write(mma8451_address, CTRL_REG1, 0x01);    // Active mode
    
    return 1;
}

void MMA8451_Read_Accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    
    if(I2C1_Start()) return;
    if(I2C1_Write_Addr(mma8451_address)) return;
    
    I2C1->DR = OUT_X_MSB;
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    
    if(I2C1_Start()) return;
    if(I2C1_Write_Addr(mma8451_address | 0x01)) return;
    
    // Enable ACK for multiple bytes
    I2C1->CR1 |= I2C_CR1_ACK;
    
    // Read 6 bytes
    for(int i = 0; i < 6; i++) {
        if(i == 5) {
            I2C1->CR1 &= ~I2C_CR1_ACK; // Last byte, send NACK
        }
        while(!(I2C1->SR1 & I2C_SR1_RXNE));
        data[i] = I2C1->DR;
    }
    
    I2C1_Stop();
    
    // Combine MSB and LSB (14-bit values)
    *x = ((int16_t)((data[0] << 8) | data[1])) >> 2;
    *y = ((int16_t)((data[2] << 8) | data[3])) >> 2;
    *z = ((int16_t)((data[4] << 8) | data[5])) >> 2;
}

float MMA8451_Convert_Accel(int16_t accel) {
    // Convert to g (2g range = ±2g)
    return (float)accel / 4096.0f;
}