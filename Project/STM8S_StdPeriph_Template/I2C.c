#include "stm8s.h"
#include "main.h"
#include "I2C.h"
#include <stdlib.h>

//==========================================================================================================
//                              I2C
//==========================================================================================================
void I2C_Initialization(){
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

  I2C->CR1 |= I2C_CR1_PE;
}

uint8_t I2C_SendNByte(uint8_t address, uint8_t * data, uint8_t length){
  while((I2C->SR3 & I2C_SR3_BUSY)); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;    //I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  I2C->SR3;
  
  for (uint8_t i=0; i<length; i++){
    I2C->SR3;
    I2C->SR2;
    I2C->SR1;
    I2C->DR = *data;
    data++;
    while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  }
  
  I2C->CR2 |= I2C_CR2_STOP;     //I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  return 1;
}

uint8_t I2C_ReadNByte(uint8_t address, uint8_t reg, uint8_t * data, uint8_t length){
  I2C_SendNByte(address, &reg, 1);
  I2C->CR2 |= I2C_CR2_ACK;
  I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);
  uint8_t res = 1;
    volatile uint32_t timeout;
    uint16_t i;
  
    while((I2C->SR3 & I2C_SR3_BUSY)); //wait for not busy
    
    //Send the start condition
    I2C->CR2 |= I2C_CR2_START;
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
    
    //Send the slave address
    I2C->DR = (uint8_t)(address | 0x01);
    while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
    I2C->SR3;
    
    I2C->SR1;
    I2C->SR2;
    I2C->SR3;
    for (i=0; i<length; i++) {
      if (i == length - 1) {
        I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
        I2C->CR2 |= I2C_CR2_STOP; //Send the stop condition
      }

      //Wait for the data to be received
      while(!(I2C->SR1 & 0x40)){}
      I2C->SR1;
      I2C->SR2;
      I2C->SR3;
      *data = I2C->DR;
      data++;
    }
    
   
  stop:
    //If something bad happened, send the stop condition
    if (!res)
      I2C->CR2 |= I2C_CR2_STOP;
    
    return res;
}