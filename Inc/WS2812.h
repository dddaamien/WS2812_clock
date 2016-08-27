
#ifndef WS2812_H_
#define WS2812_H_

#include "stm32f1xx_hal.h"
#include <stdint-gcc.h>



#define ws2812_port_set ((uint32_t*)&GPIOB->BSRR)	// Adresse port set
#define ws2812_port_clr	((uint32_t*)&GPIOB->BRR)	// Adresse port reset

#define ws2812_mask_set  GPIO_PIN_7		// Bitmask set pin
#define ws2812_mask_clr  GPIO_PIN_7		// Bitmask clear pin

struct structWs2812GRB
{
	uint8_t green;
	uint8_t red;
	uint8_t blue;
};

struct structWs2812HSV
{
	uint16_t hue;
	uint8_t saturation;
	uint8_t value;
};

void ws2812_sendarray(uint8_t *ledarray,int length);
void HSVtoRGB(struct structWs2812GRB *Srgb, struct structWs2812HSV *Shsv );

#endif /* WS2812_H_ */
