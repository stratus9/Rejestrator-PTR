/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include "fonts.h"
#include <stdlib.h>
#include "I2C.h"
#include "Timers.h"
#include "OLED.h"


//==========================================================================================================
//                              OLED
//==========================================================================================================
void OLED_Init(){
  OLED_Reset();
  OLED_SendCommand(0xAE);//wy³¹cz panel OLED
  //OLED_SendCommand(0x00);//adres kolumny LOW
  //OLED_SendCommand(0x10);//adres kolumny HIGH
  //OLED_SendCommand(0x40);//adres startu linii 
  
  OLED_SendCommand(0x20);//tryb adresowania strony 
  OLED_SendCommand(0x02);
  
  OLED_SendCommand(0x81);//ustaw kontrast
  OLED_SendCommand(0xCF);
  
  OLED_SendCommand(0xA0);//ustaw remapowanie    //by³o A1
  OLED_SendCommand(0xC8);//kierunek skanowania //by³o C0
  
  OLED_SendCommand(0xA8);//ustaw multiplex ratio 
  OLED_SendCommand(31);//1/64
  
  OLED_SendCommand(0xA6);//wyœwietlanie bez inwersji 
  OLED_SendCommand(0xD3);//ustaw display offset 
  OLED_SendCommand(0x00);//bez offsetu
  OLED_SendCommand(0xD5);//ustaw divide ratio/czêstotliwoœæ oscylatora
  OLED_SendCommand(0x80);//100ramek/sec
  OLED_SendCommand(0xD9);//ustaw okres pre charge
  OLED_SendCommand(0xF1);//pre charge 15 cykli, discharge 1 cykl
  OLED_SendCommand(0xDA);//konfiguracja wyprowadzeñ sterownika
  OLED_SendCommand(0x12);
  OLED_SendCommand(0xDB);//ustawienie vcomh
  OLED_SendCommand(0x40);
  OLED_SendCommand(0x8D);//ustawienie Charge Pump
  OLED_SendCommand(0x14);
  OLED_SendCommand(0xA4);//"pod³¹czenie" zawartoœci RAM do panelu OLED
  OLED_SendCommand(0xA6);//wy³¹czenie inwersji wyœwietlania
  OLED_SendCommand(0xAF);//w³¹cza wyœwietlacz
  OLED_setContrast(1);
}

void OLED_Reset(){
  GPIOD->ODR &= ~0x04;   //Enable I2C Pull-down
  Delay(100000);
  GPIOD->ODR |= 0x04;   //Enable I2C Pull-up
  Delay(100000);
}

void OLED_SendCommand(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = 0x3C<<1;                            // Transmit address+W
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = 0x00;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_SendData(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = 0x3C<<1;                            // Transmit address+W
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = 0x40;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_RefreshRAM(OLED_t * OLED){
  uint8_t i, j;
  
  for (i = 0; i < 4; i ++) { 
    OLED_SendCommand(0xB0 + i);
    OLED_SetColStart();   
    for (j = 0; j < 96; j ++) {
      OLED_SendData(OLED->OLED_dispBuff[j][i]); 
    }
  }  
}

void OLED_Clear(OLED_t * OLED, uint8_t fill){ 
  uint8_t i, j;
  
  for (i = 0; i < 4; i ++) {
    for (j = 0; j < 96; j ++) {
      OLED->OLED_dispBuff[j][i] = fill;
    }
  }
  
  OLED_RefreshRAM(OLED);//zawartoœæ bufora do RAM obrazu
} 

void OLED_SetColStart(){ 
    OLED_SendCommand(0x00); //low
    OLED_SendCommand(0x12); //high
}


void OLED_DrawPoint(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t p){
  uint8_t chPos, chBx, chTemp = 0;
  
  if (x > 95 || y > 31) {
    return;
  }
  chPos = y / 8; 
  chBx = y % 8;
  chTemp = 1 << (chBx);
  
  if (p) {
    OLED->OLED_dispBuff[x][chPos] |= chTemp;
    
  } else {
    OLED->OLED_dispBuff[x][chPos] &= ~chTemp;
  }
}

//wyœwietlenie jednego znaku 
//argumenty:
//x,y - wspó³rzêdne na ekranie
//Chr - kod ASCII znaku 
//size - rozmiar 12, lub 16
//mode=1 znak wyœwietlany normalnie, mode=0 znak wyœwietlany w negatywie
//*******************************************************************************
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr){    
  uint8_t i, j;
  uint8_t chTemp, chYpos0 = y;
  
  //Chr = Chr - ' ' + 1;    
  Chr = OLED_charDecoder(Chr);
  for (i = 0; i < 12; i ++) {  
        chTemp = c_chFont1206_lite[Chr][i];

    for (j = 0; j < 8; j ++) {
      if (chTemp & 0x80) {
        OLED_DrawPoint(OLED, x, y, 1);
      } else {
        OLED_DrawPoint(OLED, x, y, 0);
      }
      chTemp <<= 1;
      y ++;
      
      if ((y - chYpos0) == 12) {
        y = chYpos0;
        x ++;
        break;
      }
    }  
  }
}

uint8_t OLED_charDecoder(uint8_t znak){
  if(znak == ' ') return 0;
  if((znak >= '0') && (znak <= '9')) return znak-41;
  if(znak == '!') return 1;
  if(znak == '%') return 2;
  if(znak == '+') return 3;
  if(znak == '-') return 4;
  if(znak == '.') return 5;
  if(znak == '/') return 6;
  if(znak == ':') return 17;
  if(znak == 'A') return 18;
  if(znak == 'H') return 19;
  if(znak == 'P') return 20;
  if(znak == 'V') return 21;
  if(znak == '^') return 22;
  if(znak == 'a') return 23;
  if(znak == 'm') return 24;
  if(znak == 'x') return 25;
  if(znak == 's') return 26;
  return 0;
}

//*********************************************************************************
//wyœwietlenie ³añcucha znaków - napisu 
//argumenty
//x,y - wspó³rzêdne na ekranie
//*txt - wskaŸnik na pocz¹tek bufora zwieraj¹cego ³añcuch znaków ASCII do wyœwietlenia
//size - wysokoœæ znaków 12, lub 16 pikseli 
//mode=1 znaki wyœwietlane normalnie, mode=0 znaki wyœwietlane w negatywie
//*********************************************************************************
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y,  uint8_t *txt){
  while (*txt != '\0') {    
    
    OLED_displayChar(OLED, x, y, *txt);
    x += 6;
    txt ++;
  }
}

void OLED_setContrast(uint8_t value){
  OLED_SendCommand(0x81);//ustaw kontrast
  OLED_SendCommand(value);
}

void OLED_int2string(uint8_t * string, uint32_t number){
  string[0] = (number>99999)?( number/100000 %10 + '0'):(' ');
  string[1] = (number>9999 )?((number/10000)%10 + '0'):(' ');
  string[2] = (number>999  )?((number/1000 )%10 + '0'):(' ');
  string[3] = (number/100  )%10 + '0';
  string[4] = '.';
  string[5] = (number/10   )%10 + '0';
  string[6] = '\0';
}

void OLED_paramTemplate(OLED_t * OLED){
  OLED_dispTxt(OLED, 0, 0,"Vmax:       m/s");
  OLED_dispTxt(OLED, 0,10,"Amax:       m/s^");
  OLED_dispTxt(OLED, 0,21,"Hmax:       m");
}

void OLED_dispInt(OLED_t * OLED, uint8_t x, uint8_t y, int32_t value){
  uint8_t bufor[7];
  OLED_int2string(bufor, value);
  
  OLED_dispTxt(OLED,x,y,bufor);
}


//inline void OLED_dispVelocity(OLED_t * OLED, uint32_t value){
//  OLED_dispInt(OLED, 32, 0, value);
//}

//inline void OLED_dispAcceleration(OLED_t * OLED, uint32_t value){
//  OLED_dispInt(OLED, 32, 10, value);
//}

//inline void OLED_dispAltitude(OLED_t * OLED, uint32_t value){
//  OLED_dispInt(OLED, 32, 21, value);
//}

/*
void OLED_drawLine(OLED_t * OLED, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode){
  uint8_t tmp;
  uint8_t x,y;
  uint8_t dx, dy;
  int8_t err;
  int8_t ystep;
  
  uint8_t swapxy = 0;
  
  if ( x1 > x2 ) dx = x1-x2; else dx = x2-x1;
  if ( y1 > y2 ) dy = y1-y2; else dy = y2-y1;
  
  if ( dy > dx ) 
  {
    swapxy = 1;
    tmp = dx; dx =dy; dy = tmp;
    tmp = x1; x1 =y1; y1 = tmp;
    tmp = x2; x2 =y2; y2 = tmp;
  }
  if ( x1 > x2 ) 
  {
    tmp = x1; x1 =x2; x2 = tmp;
    tmp = y1; y1 =y2; y2 = tmp;
  }
  err = dx >> 1;
  if ( y2 > y1 ) ystep = 1; else ystep = -1;
  y = y1;
  
  if ( x2 == 255 ) x2--;
  
  for( x = x1; x <= x2; x++ ){
    if ( swapxy == 0 ) 
      OLED_DrawPoint(OLED, x, y, mode); 
    else 
      OLED_DrawPoint(OLED, y, x, mode); 
    err -= (uint8_t)dy;
    if ( err < 0 ){
      y += (uint8_t)ystep;
      err += (uint8_t)dx;
    }
  }
}
*/

/*
void OLED_drawPlotTemplate(OLED_t * OLED, char type){
  //----  X axis  ---------
  OLED_drawLine(OLED,  0, 29, 90, 29, 1);
  OLED_drawLine(OLED, 90, 29, 88, 27, 1);
  OLED_drawLine(OLED, 90, 29, 88, 31, 1);
  OLED_displayChar(OLED, 91, 20, 't');
    
  //---- Y axis  ----------
  OLED_drawLine(OLED, 2, 6, 2, 31, 1);
  OLED_drawLine(OLED, 2, 6, 0,  8, 1);
  OLED_drawLine(OLED, 2, 6, 4,  8, 1);
  OLED_displayChar(OLED, 4, 0, type);
}
*/


//==========================================================================================================
//                              Dev
//==========================================================================================================
/*
void dev_CheckSensors(){
  OLED_Clear(&OLED_buffer, 0);
  
  if(I2C_ReadOneByte(0xEE, 0xD0) == 0x58) OLED_dispTxt(&OLED_buffer,0,0,"BMP  OK");
  else OLED_dispTxt(&OLED_buffer,0,0,"BMP  NOPE");
  
  if(I2C_ReadOneByte(0x3A, 0x00) == 0xE5) OLED_dispTxt(&OLED_buffer,0,12,"ADXL OK");
  else OLED_dispTxt(&OLED_buffer,0,12,"ADXL  NOPE");
  
  OLED_RefreshRAM(&OLED_buffer);
}
*/