#ifndef __OLED_H
#define __OLED_H

#define OLED_dispVelocity(OLED, value) OLED_dispInt(OLED, 32, 0, value)
#define OLED_dispAcceleration(OLED, value) OLED_dispInt(OLED, 32, 10, value)
#define OLED_dispAltitude(OLED, value) OLED_dispInt(OLED, 32, 21, value)

void OLED_Deinit();
void OLED_Init();
void OLED_Reset();
void OLED_SendCommand(uint8_t data);
void OLED_SendData(uint8_t data);
void OLED_RefreshRAM(OLED_t * OLED);
void OLED_Clear(OLED_t * OLED, uint8_t fill);
void OLED_SetColStart(void);
void OLED_DrawPoint(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t p);
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr);
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t *txt);
void OLED_setContrast(uint8_t value);
void OLED_int2string(uint8_t * string, int32_t number);
void OLED_paramTemplate(OLED_t * OLED);
void OLED_dispInt(OLED_t * OLED, uint8_t x, uint8_t y, int32_t value);

void OLED_drawLine(OLED_t * OLED, uint8_t x1, uint8_t y01, uint8_t x12, uint8_t y2, uint8_t mode);
void OLED_drawPlotTemplate(OLED_t * OLED, char type);
uint8_t OLED_charDecoder(uint8_t znak);

#endif