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
  uint8_t data[2];
  data[0] = 0x2D; data[1] = 0x00;
  I2C_SendNByte(0x3A, data, 2);
  data[0] = 0x2D; data[1] = 0x10;
  I2C_SendNByte(0x3A, data, 2);
  data[0] = 0x2D; data[1] = 0x08;
  I2C_SendNByte(0x3A, data, 2);
  data[0] = 0x31; data[1] = 0x0B;
  I2C_SendNByte(0x3A, data, 2);   // +/- 16g Full ress
}

void ADXL_read(sensors_t * sensor){
  uint8_t buffer[7];
  I2C_ReadNByte(0x3A, 0x32, buffer, 6);
  sensor->accX = buffer[1]<<8 | buffer[0];
  sensor->accY = buffer[3]<<8 | buffer[2];
  sensor->accZ = buffer[5]<<8 | buffer[4];
  
  sensor->acc_sum_smooth = (int16_t)(0.9*sensor->acc_sum_smooth + 0.1*(abs(sensor->accX) + abs(sensor->accY) + abs(sensor->accZ))); 
}