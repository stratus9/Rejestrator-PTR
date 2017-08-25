/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include <stdlib.h>
#include "SPI.h"
#include "FLASH.h"
#include "Timers.h"

//==========================================================================================================
//                              FLASH
//==========================================================================================================

void FLASH_PowerUp(){
  FLASH_CS(1);
  SPI_RWByte(0xAB);
  FLASH_CS(0);
}

uint16_t FLASH_ReadID(){
	FLASH_CS(1);
	SPI_RWByte(0x90);
	SPI_RWByte(0x00);
	SPI_RWByte(0x00);
	SPI_RWByte(0x00);
	uint16_t tmp = SPI_RWByte(0) << 8;
	tmp |= SPI_RWByte(0);
	FLASH_CS(0);
	return tmp;
}

void FLASH_CS(uint8_t value) {
  if(value){
    GPIOA->DDR |= 0x08;   //Output
    GPIOA->CR1 |= 0x08;   //Push-Pull
    GPIOA->ODR &= ~0x08;
  }
  else{
     GPIOA->DDR |= 0x08;   //Output
    GPIOA->CR1 |= 0x08;   //Push-Pull
    GPIOA->ODR |= 0x08;
  }
}

void FLASH_1byteCommand(uint8_t value){
	FLASH_CS(1);
	SPI_RWByte(value);
	FLASH_CS(0);
}

void FLASH_WriteEnable(uint8_t value){
	if(value) FLASH_1byteCommand(0x06);
	else FLASH_1byteCommand(0x04);
}

void FLASH_arrayRead(uint32_t address, uint8_t * array, uint32_t length){
	FLASH_waitForReady();
	FLASH_CS(1);
	SPI_RWByte(0x03);	//READ
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	for (uint32_t i=0; i<length; i++){
		array[i] = SPI_RWByte(0);
	}
	FLASH_CS(0);
}

void FLASH_waitForReady(void){
	while(FLASH_status() & 0x01) Delay(1000);
}

uint8_t FLASH_status(void){
	FLASH_CS(1);
	SPI_RWByte(0x05);	//READ
	uint8_t stat = SPI_RWByte(0);
	FLASH_CS(0);
	return stat;
}

void FLASH_pageWrite(uint32_t page, uint8_t * array, uint16_t length){  //strona ma 256B
	uint32_t address = page<<8;
	FLASH_waitForReady();
	FLASH_WriteEnable(1);
	FLASH_CS(1);
	SPI_RWByte(0x02);	//Page program
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	for(uint16_t i = 0; i < length; i++){
		SPI_RWByte(*array++);
	}
	FLASH_CS(0);
}