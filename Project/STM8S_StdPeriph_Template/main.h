#ifndef __MAIN_H
#define __MAIN_H


// --------------------- Struct typedef ---------------------------------------
typedef struct OLED_s{
  uint8_t OLED_dispBuff[96][4];
} OLED_t;

typedef struct sensors_s{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  
  int16_t min_acc;
  int16_t max_acc;

  uint16_t tmp;
  
}sensors_t;

typedef struct bmp_s{
  uint16_t T1;
  int16_t  T2;
  int16_t  T3;
  
  uint16_t P1;
  int16_t  P2;
  int16_t  P3;
  int16_t  P4;
  int16_t  P5;
  int16_t  P6;
  int16_t  P7;
  int16_t  P8;
  int16_t  P9;
  
  int32_t UT;
  int32_t UP;
  
  //int32_t var1;
  //int32_t var2;
  int32_t tfine;
  int32_t T;
  
  //float fvar1;
  //float fvar2;
  //float p;
  
  int32_t press;
  float press_f;
  float min_pressure;
  float max_pressure;
  float diff_pressure;
  int32_t temp;
  float altitude;
  int32_t max_altitude;
  float velocity;
  
  //float x1, x2, x3, x4;
} bmp_t;

typedef union {
	uint8_t array[32];
	struct{
		float pressure;
                float pressure_diff;
                uint8_t state;
                int32_t altitude;
                int32_t temperature;
                uint16_t Vbat;
                int16_t accX;
                int16_t accY;
                int16_t accZ;
                int16_t velocity;
		};
} FLASH_dataStruct_t;

typedef struct{
	uint16_t pageNo;
	uint8_t position;
	union{
		FLASH_dataStruct_t FLASH_dataStruct[8];
		uint8_t data[256];
		};
}FLASH_pageStruct_t;

typedef struct {
  uint8_t devState;
  uint8_t flightState;
  uint8_t button;
} state_t;


/* Private function prototypes -----------------------------------------------*/
void StateMachine();
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

void SPI_Initialization();
inline void SPI_wait();
void SPI_sendByte(uint8_t value);
uint8_t SPI_ReadByte();
uint8_t SPI_RWByte(uint8_t value);
void SPI_ClearRXBuffer();
void FLASH_CS(uint8_t value);
uint16_t FLASH_ReadID();
void FLASH_PowerUp();
void FLASH_1byteCommand(uint8_t value);
void FLASH_WriteEnable(uint8_t value);
void FLASH_arrayRead(uint32_t address, uint8_t * array, uint32_t length);
void FLASH_waitForReady();
uint8_t FLASH_status();
void FLASH_pageWrite(uint32_t page, uint8_t * array, uint16_t length);

void Timer1_Init();
void Timer2_Init();
void Timer2_ISR();

void Beep_Initialization();
inline void Beep_Start();
inline void Beep_Stop();

void LED_BLUE(uint8_t value);
void LED_GREEN(uint8_t value);

void I2C_Initialization();
void I2C_InitFast();
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
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr);
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t *txt);
void OLED_setContrast(uint8_t value);
void OLED_int2string(char * string, uint32_t number);
void OLED_paramTemplate(OLED_t * OLED);
void OLED_dispInt(OLED_t * OLED, uint8_t x, uint8_t y, int32_t value);
inline void OLED_dispVelocity(OLED_t * OLED, uint32_t value);
inline void OLED_dispAcceleration(OLED_t * OLED, uint32_t value);
inline void OLED_dispAltitude(OLED_t * OLED, uint32_t value);

void OLED_drawLine(OLED_t * OLED, uint8_t x1, uint8_t y01, uint8_t x12, uint8_t y2, uint8_t mode);
void OLED_drawPlotTemplate(OLED_t * OLED, char type);
//void OLED_drawPlotData(OLED_t * OLED, dataset_t * data);
uint8_t OLED_charDecoder(char znak);
//void datasetPrepare(dataset_t * data);

void dev_CheckSensors();

void BMP_init(bmp_t * BMP);
void BMP_read(bmp_t * BMP);
float BMP_altitude(float startPress, float currPress);

void ADXL_init();
void ADXL_read(sensors_t * sensor);

void ButtonISR();
void ISR_init();

#endif /* __MAIN_H */