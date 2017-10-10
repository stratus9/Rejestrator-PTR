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
  
  int16_t acc_sum_smooth;
  
  int16_t min_acc;
  int16_t max_acc;

  //uint16_t tmp;
  
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
  
  //int32_t UT;
  //int32_t UP;
  
  //int32_t var1;
  //int32_t var2;
  int32_t tfine;
  int32_t T;
  int32_t max_velocity;
  
  //float fvar1;
  //float fvar2;
  //float p;
  
  int32_t press;
  int32_t press_raw;
  int32_t min_pressure;
  int32_t max_pressure;
  int32_t diff_pressure;
  int32_t temp;
  int32_t altitude;
  int32_t old_altitude;
  int32_t start_altitude;
  int32_t real_altitude;
  int32_t max_altitude;
  int32_t velocity;
} bmp_t;


typedef union {
	uint8_t array[32];
	struct{
                uint8_t marker;         // OK
		int32_t pressure;       // OK
                int32_t pressure_raw;   // OK
                uint8_t state;          // OK
                int32_t altitude;       // OK
                int32_t temperature;    // OK
                uint16_t Vbat;          // ??? warunkowo OK
                int16_t accX;           // OK
                int16_t accY;           // OK
                int16_t accZ;           // OK
                int16_t velocity;       // OK
		};
} FLASH_dataStruct_t;   //24 B


typedef struct{
	uint16_t pageNo;
	uint8_t position;
	union{
		FLASH_dataStruct_t FLASH_dataStruct[8];
		uint8_t data[256];
		};
}FLASH_pageStruct_t;


typedef volatile struct {
  uint8_t devState;
  uint8_t flightState;
  uint8_t button;
  uint8_t I2C_inprogress;
} state_t;

extern volatile uint8_t beep;
extern volatile uint8_t beep_trigger;
extern state_t state_d;

/* Private function prototypes -----------------------------------------------*/
void StateMachine();
void sleep_start();
void sleep_exit();

/* Private function prototypes -----------------------------------------------*/
void GPIO_Initialization(void);
void GPIO_Init_Fast();

void LED_BLUE(uint8_t value);
void LED_GREEN(uint8_t value);

void dev_CheckSensors();


void ButtonISR();
void ISR_init();

void FLASH_saveData();
void OLED_dispMax();

#endif /* __MAIN_H */