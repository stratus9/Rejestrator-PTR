#ifndef __MAIN_H
#define __MAIN_H


// --------------------- Struct typedef ---------------------------------------
typedef struct OLED_s{
  uint8_t OLED_dispBuff[96][4];
} OLED_t;

typedef struct dataset_s{
  uint8_t data[100];
  
} dataset_t;

typedef struct sensors_s{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  
}sensors_t;


/* Private function prototypes -----------------------------------------------*/
void Delay(uint32_t);
float Q_rsqrt( float number );

void USART_Initialization(void);
void UART1_Init_Fast();
void UART1_DeInit_Fast();
void UART1_ENABLE();
void USART_SendString(char *);
void USART_SendChar(char);

void GPIO_Initialization(void);
void GPIO_Init_Fast();

void Timer1_Init();
void Timer2_Init();
void Timer2_ISR();

void Beep_Initialization(void);
inline void Beep_Start(void);
inline void Beep_Stop(void);

void LED_BLUE(uint8_t value);
void LED_GREEN(uint8_t value);

void I2C_Initialization(void);
void I2C_SendOneByte(uint8_t address, uint8_t data);
void I2C_SendTwoBytes(uint8_t address, uint8_t data1, uint8_t data2);
uint8_t I2C_ReadOneByte(uint8_t address, uint8_t reg);
uint8_t I2C_ReadNByte(uint8_t address, uint8_t reg, uint8_t * data, uint8_t length);

void OLED_Init();
void OLED_Reset();
void OLED_SendCommand(uint8_t data);
void OLED_SendData(uint8_t data);
void OLED_RefreshRAM(OLED_t * OLED);
void OLED_Clear(OLED_t * OLED, uint8_t fill);
void OLED_SetColStart(void);
void OLED_DrawPoint(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t p);
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr, uint8_t size, uint8_t mode);
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t *txt, uint8_t size, uint8_t mode);
void OLED_setContrast(uint8_t value);
void OLED_int2string(char * string, uint32_t number);
void OLED_paramTemplate(OLED_t * OLED);
void OLED_dispInt(OLED_t * OLED, uint8_t x, uint8_t y, int32_t value);
inline void OLED_dispVelocity(OLED_t * OLED, uint32_t value);
inline void OLED_dispAcceleration(OLED_t * OLED, uint32_t value);
inline void OLED_dispAltitude(OLED_t * OLED, uint32_t value);

void OLED_drawLine(OLED_t * OLED, uint8_t x1, uint8_t y01, uint8_t x12, uint8_t y2, uint8_t mode);
void OLED_drawPlotTemplate(OLED_t * OLED, char type);
void OLED_drawPlotData(OLED_t * OLED, dataset_t * data);
void datasetPrepare(dataset_t * data);

void dev_CheckSensors();

void ADXL_init();
void ADXL_read(sensors_t * sensor);

#endif /* __MAIN_H */