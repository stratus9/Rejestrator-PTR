/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include <stdlib.h>
#include "I2C.h"
#include "BMP.h"
//#include <stdint.h>

//==========================================================================================================
//                              BMP280
//==========================================================================================================
void BMP_init(bmp_t * BMP) {
  uint8_t buffer[25];
  I2C_ReadNByte(0xEE, 0x88, buffer, 24);
  BMP->T1 = buffer[0] | (buffer[1] << 8);
  BMP->T2 = buffer[2] | (buffer[3] << 8);
  BMP->T3 = buffer[4] | (buffer[5] << 8);
  BMP->P1 = buffer[6] | (buffer[7] << 8);
  BMP->P2 = buffer[8] | (buffer[9] << 8);
  BMP->P3 = buffer[10] |(buffer[11] << 8);
  BMP->P4 = buffer[12] | (buffer[13] << 8);
  BMP->P5 = buffer[14] | (buffer[15] << 8);
  BMP->P6 = buffer[16] | (buffer[17] << 8);
  BMP->P7 = buffer[18] | (buffer[19] << 8);
  BMP->P8 = buffer[20] | (buffer[21] << 8);
  BMP->P9 = buffer[22] | (buffer[23] << 8);
  
  /*
  BMP->T1 = 27504;      //buffer[0] | (buffer[1] << 8);
  BMP->T2 = 26435;      //buffer[2] | (buffer[3] << 8);
  BMP->T3 = -1000;      //buffer[4] | (buffer[5] << 8);
  BMP->P1 = 36477;      //buffer[6] | (buffer[7] << 8);
  BMP->P2 = -10685;     //buffer[8] | (buffer[9] << 8);
  BMP->P3 = 3024;       //buffer[10] |(buffer[11] << 8);
  BMP->P4 = 2855;       //buffer[12] | (buffer[13] << 8);
  BMP->P5 = 140;        //buffer[14] | (buffer[15] << 8);
  BMP->P6 = -7;         //buffer[16] | (buffer[17] << 8);
  BMP->P7 = 15500;      //buffer[18] | (buffer[19] << 8);
  BMP->P8 = -14600;     //buffer[20] | (buffer[21] << 8);
  BMP->P9 = 6000;       //buffer[22] | (buffer[23] << 8);
  */
  
  uint8_t data[2];
  data[0] = 0xF4; data[1] = 0x3F;       //temp - 1x oversampling, press - 16x oversampling, Normal mode (powinn byc 8x)
  I2C_SendNByte(0xEE, data, 2);
}

void BMP_deinit(){
  uint8_t data[2];
  data[0] = 0xF4; data[1] = 0x00;       //Sleep mode
  I2C_SendNByte(0xEE, data, 2);
}

void BMP_read(bmp_t * BMP) {
  uint8_t buffer[7];
  int32_t UT;
  int32_t UP;
  
  I2C_ReadNByte(0xEE, 0xF7, buffer, 6);
  
  UP = buffer[0];
  UP = UP << 8 | buffer[1];
  UP = UP << 8 | buffer[2];
  UP >>= 4;
  
  UT = buffer[3];
  UT = UT << 8 | buffer[4];
  UT = UT << 8 | buffer[5];
  UT >>= 4;
  
  //--------- Temp -------------
  int32_t var1, var2, tfine;
  var1  = ((((UT>>3) - ((int32_t)BMP->T1 <<1))) * ((int32_t)BMP->T2)) >> 11;
  var2  = (((((UT>>4) - ((int32_t)BMP->T1)) * ((UT>>4) - ((int32_t)BMP->T1))) >> 12) * ((int32_t)BMP->T3)) >> 14;
  
  tfine = var1 + var2;
  BMP->temp = (tfine)>>9;
  
  //--------- Press -----------
  float fvar1=0.0, fvar2=0.0, p=0.0;
  fvar1 = (float)tfine/2.0 - 64000.0;
  fvar2 = fvar1 * fvar1*((double)BMP->P6)/32768.0;
  fvar2 = fvar2 + fvar1*((float)BMP->P5)*2.0;
  fvar2 = (fvar2/4.0) + ((float)BMP->P4)*65536.0;
  fvar1 = (((float)BMP->P3) * fvar1 * fvar1/524288.0 + ((float)BMP->P2) * fvar1)/524288.0;
  fvar1 = (1.0 + fvar1/32768.0)*((float)BMP->P1);
  p = 1048576.0-((float)UP);
  p = (p-(fvar2/4096.0))*6250.0 / fvar1;
  fvar1 = ((float)BMP->P9)*p*p/2147483648.0;
  fvar2 = p*((float)BMP->P8)/32768.0;
  p = p + (fvar1 + fvar2+((float)BMP->P7))/16.0;
  p *= 100.0;
  
  BMP->press_raw = (int32_t)(p);
  if((p > 5000000UL) && (labs(BMP->press_raw - (int32_t)p) < 20000UL)) BMP->press = (int32_t)(p*0.05 + BMP->press*0.95);
}

int32_t BMP_altitude(int32_t startPress, int32_t currPress){
   float x1, x2, x3, x4;
   //return (1.0-powf(currPress/startPress, 0.1902632365))*43538.0;
   x1 = (float)currPress/(float)startPress;
   x2 = -4863.0*x1*x1*x1;
   x3 =  16830.0*x1*x1;
   x4 = -27490.0*x1;
   return (int32_t)((x2 + x3 + x4 + 15520.0)*100.0);
}

void BMP_velo(bmp_t * bmp){
  int32_t velo = (bmp->altitude - bmp->old_altitude)*100;
  bmp->velocity = (int32_t)(9*bmp->velocity + 1*velo)/10;
}