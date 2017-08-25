#ifndef __I2C_H
#define __I2C_H

void I2C_Initialization();
void I2C_InitFast();
void I2C_SendOneByte(uint8_t address, uint8_t data);
void I2C_SendTwoBytes(uint8_t address, uint8_t data1, uint8_t data2);
uint8_t I2C_ReadOneByte(uint8_t address, uint8_t reg);
uint8_t I2C_ReadNByte(uint8_t address, uint8_t reg, uint8_t * data, uint8_t length);


#endif