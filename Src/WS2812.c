#include "WS2812.h"

#define ws2812_DEL1 "	nop		\n\t"
#define ws2812_DEL2 "	b	.+2	\n\t"
#define ws2812_DEL4 ws2812_DEL2 ws2812_DEL2
#define ws2812_DEL8 ws2812_DEL4 ws2812_DEL4
#define ws2812_DEL16 ws2812_DEL8 ws2812_DEL8


void ws2812_sendarray(uint8_t *data,int datlen)
{
	uint32_t maskhi = ws2812_mask_set;
	uint32_t masklo = ws2812_mask_clr;
	volatile uint32_t *set = ws2812_port_set;
	volatile uint32_t *clr = ws2812_port_clr;
	uint32_t i;
	uint32_t curbyte;

	while (datlen)
	{
		datlen=datlen-1;
		curbyte=*data++;

	asm volatile(
			"		cpsid f						\n\t" //désactive les interruptions
			"		lsl %[dat],#24				\n\t"
			"		movs %[ctr],#8				\n\t"
			"ilop%=:							\n\t"
			"		str %[maskhi], [%[set]]		\n\t"
//			"		lsl %[dat], #1				\n\t"

			ws2812_DEL16 //350nS:H

			"		tst %[dat], #0x10000000		\n\t"//10000000
			"		bne one%=					\n\t"//bcs mi eq
			"		str %[masklo], [%[clr]]		\n\t"
			"one%=:								\n\t"

			ws2812_DEL16 //350nS:H

			//"		sub %[ctr], #1				\n\t"
			"		str %[masklo], [%[clr]]		\n\t"
			"		sub %[ctr], #1				\n\t"
			"		lsl %[dat], #1				\n\t"//
			"		teq %[ctr], #0				\n\t"//
			"		beq	end%=					\n\t"

			ws2812_DEL16 //700nS:L
			//ws2812_DEL4

			"		b 	ilop%=					\n\t"
			"end%=:								\n\t"
			"		cpsie f						\n\t" //active les interruptions
			:	[ctr] "+r" (i)
			:	[dat] "r" (curbyte), [set] "r" (set), [clr] "r" (clr), [masklo] "r" (masklo), [maskhi] "r" (maskhi)
			);
	}
}

void HSVtoRGB(struct structWs2812GRB *Srgb, struct structWs2812HSV *Shsv )
{
	uint8_t i;
	uint8_t f, p, q, t;

	if(  Shsv->saturation == 0 )
	{
		// achromatic (grey)
		Srgb->red = Srgb->green = Srgb->blue = Shsv->value;
		return;
	}

	f = Shsv->hue % 60;
	Shsv->hue /= 60;			// sector 0 to 5

	i =  Shsv->hue;
	//f =  Shsv->hue - i;			// factorial part of h
	p =  Shsv->value * ( 255 -  Shsv->saturation );
	q =  Shsv->value * ( 255 -  Shsv->saturation * f );
	t =  Shsv->value * ( 255 -  Shsv->saturation * ( 255 - f ) );

	switch( i ) {
		case 0:
			Srgb->red =  Shsv->value;
			Srgb->green = t;
			Srgb->blue = p;
			break;
		case 1:
			Srgb->red = q;
			Srgb->green =  Shsv->value;
			Srgb->blue = p;
			break;
		case 2:
			Srgb->red = p;
			Srgb->green =  Shsv->value;
			Srgb->blue = t;
			break;
		case 3:
			Srgb->red = p;
			Srgb->green = q;
			Srgb->blue =  Shsv->value;
			break;
		case 4:
			Srgb->red = t;
			Srgb->green = p;
			Srgb->blue =  Shsv->value;
			break;
		default:		// case 5:
			Srgb->red =  Shsv->value;
			Srgb->green = p;
			Srgb->blue = q;
			break;
	}

}
