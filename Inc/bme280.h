#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

extern I2C_HandleTypeDef hi2c1;



#define BME_ADDRESS 0xEC //0x76<<1

typedef struct
{
	int32_t temp;
	int32_t press;
	int32_t hum;
}bme_structTypedef;

typedef enum
{
	bme_id = 0xD0,
	bme_reset = 0xE0,
	bme_ctrlHum = 0xF2,
	bme_ctrlMeas = 0xF4,
	bme_config = 0xF5,
	bme_press = 0xF7,
	bme_temp = 0xFA,
	bme_hum = 0xFD
}bme_enumTypedef;


uint8_t bme280_init();
void bme280_read(bme_structTypedef *data);
