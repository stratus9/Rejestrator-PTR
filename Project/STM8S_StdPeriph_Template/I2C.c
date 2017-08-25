#include "stm8s.h"
#include "main.h"
#include "I2C.h"
#include <stdlib.h>

//==========================================================================================================
//                              I2C
//==========================================================================================================

void I2C_Initialization(){
  //I2C_DeInit();
  //I2C_Init(300000, 0xAA, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);
  I2C_InitFast();
  I2C_Cmd(ENABLE);
}

void I2C_InitFast(){
  /*------------------------- I2C FREQ Configuration ------------------------*/
  I2C->FREQR = 16;

  /*--------------------------- I2C CCR Configuration ------------------------*/
  I2C->CR1 &= (uint8_t)(~0x01); // Disable I2C to configure TRISER

  I2C->TRISER = 6; // Set Maximum Rise Time: 300ns max in Fast Mode


  /* Write CCR with new calculated value */
  I2C->CCRL = 8;
  I2C->CCRH = 0x80;

  I2C->CR1 |= 0x01; // Enable I2C 

  /* Configure I2C acknowledgement */
  //I2C_AcknowledgeConfig(I2C_ACK_CURR);
  I2C->CR2 |= I2C_CR2_ACK;
  I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);

  /*--------------------------- I2C OAR Configuration ------------------------*/
  I2C->OARL = 0xAA;
  I2C->OARH = 0x40;
}

void I2C_SendOneByte(uint8_t address, uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;    //I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address | 0x00;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;     //I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void I2C_SendTwoBytes(uint8_t address, uint8_t data1, uint8_t data2){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;    //I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address | 0x00;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data1;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data2;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;     //I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

uint8_t I2C_ReadOneByte(uint8_t address, uint8_t reg){
  uint8_t data;
  I2C_SendOneByte(address, reg);
  
  uint8_t res = 1;
    volatile uint32_t timeout;
  
    //Wait for the bus to be ready
    timeout = 0x0FFF;
    while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
    if (!timeout) {
      //Error
      res = 0;
      return res;
    }
    
    //Send the start condition
    I2C->CR2 |= I2C_CR2_START;  //I2C->CR2 |= I2C_CR2_START;
    timeout = 0x0FFF;
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    
    //Send the slave address
    //I2C_Send7bitAddress(address, I2C_DIRECTION_RX);
    I2C->DR = (uint8_t)(address | I2C_DIRECTION_RX);
    timeout = 0x0FFF;
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    else if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
        I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
        I2C->CR2 |= I2C_CR2_STOP;       //I2C->CR2 |= I2C_CR2_STOP;
        res = 0;
        goto stop;
    }
    
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    
  stop:
    //Send the stop condition
    I2C->CR2 |= I2C_CR2_STOP;
    
    if (res) {
      //Wait for the data to be received
      timeout = 0x0FFF;
      while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
      if (!timeout) {
        //Error
        res = 0;
      }
      else {
        //Get the data
        data = I2C->DR;
      }
    }
  return data;
}

uint8_t I2C_ReadNByte(uint8_t address, uint8_t reg, uint8_t * data, uint8_t length){
  I2C_SendOneByte(address, reg);
  I2C->CR2 |= I2C_CR2_ACK;
  I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);
  uint8_t res = 1;
    volatile uint32_t timeout;
    uint16_t i;
  
    //Wait for the bus to be ready
    timeout = 0x0FFF;
    while(timeout-- && I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
    if (!timeout) {
      //Error
      res = 0;
      return res;
    }
    
    //Send the start condition
    I2C->CR2 |= I2C_CR2_START;
    timeout = 0x0FFF;
    while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    
    //Send the slave address
    //I2C_Send7bitAddress(address, I2C_DIRECTION_RX);
    I2C->DR = (uint8_t)(address | I2C_DIRECTION_RX);
    timeout = 0x0FFF;
    while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    else if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
        I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
        I2C->CR2 |= I2C_CR2_STOP;
        res = 0;
        goto stop;
    }
    
    for (i=0; i<length; i++) {
      if (i == length - 1) {
        I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
        //Send the stop condition
        I2C->CR2 |= I2C_CR2_STOP;
      }

      //Wait for the data to be received
      timeout = 0x0FFF;
      while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
      if (!timeout) {
        //Error
        res = 0;
      }
      else {
        //Get the data
        *data = I2C->DR;
        data++;
      }
    }
    
   
  stop:
    //If something bad happened, send the stop condition
    if (!res)
      I2C->CR2 |= I2C_CR2_STOP;
    
    return res;
}