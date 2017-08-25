/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include <stdlib.h>
#include "I2C.h"
#include "ADXL.h"

//==========================================================================================================
//                              ADXL345
//==========================================================================================================
void ADXL_init(){
  I2C_SendTwoBytes(0x3A, 0x2D, 0x00);
  I2C_SendTwoBytes(0x3A, 0x2D, 0x10);
  I2C_SendTwoBytes(0x3A, 0x2D, 0x08);
  I2C_SendTwoBytes(0x3A, 0x31, 0x0B);   // +/- 16g Full ress
}

void ADXL_read(sensors_t * sensor){
  uint8_t buffer[7];
  I2C_ReadNByte(0x3A, 0x32, buffer, 6);
  sensor->accX = buffer[1]<<8 | buffer[0];
  sensor->accY = buffer[3]<<8 | buffer[2];
  sensor->accZ = buffer[5]<<8 | buffer[4];
}