#ifndef __FLASH_H
#define __FLASH_H

void FLASH_PowerUp();
uint16_t FLASH_ReadID();
void FLASH_CS(uint8_t value);
void FLASH_1byteCommand(uint8_t value);
void FLASH_WriteEnable(uint8_t value);
void FLASH_arrayRead(uint32_t address, uint8_t * array, uint32_t length);
void FLASH_waitForReady(void);
uint8_t FLASH_status(void);
void FLASH_pageWrite(uint32_t page, uint8_t * array, uint16_t length);


#endif