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
  float start_altitude;
  float real_altitude;
  float max_altitude;
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

extern volatile uint8_t beep;
extern volatile uint8_t beep_trigger;

typedef struct {
  uint8_t devState;
  uint8_t flightState;
  uint8_t button;
} state_t;


/* Private function prototypes -----------------------------------------------*/
void StateMachine();


/* Private function prototypes -----------------------------------------------*/
void GPIO_Initialization(void);
void GPIO_Init_Fast();

void LED_BLUE(uint8_t value);
void LED_GREEN(uint8_t value);

void dev_CheckSensors();


void ButtonISR();
void ISR_init();

#endif /* __MAIN_H */