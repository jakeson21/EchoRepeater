/*
 * Delay.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Fuguru
 */


#include "Delay.h"

void Delay_Init(fDelay_TypeDef* hdelay, int32_t length)
{
	hdelay->inc = 1;
	hdelay->max_length = DELAY_LENGTH;
	hdelay->length = length;
	for (uint32_t n = 0; n < DELAY_LENGTH; n++)
	{
		hdelay->buffer[n] = 0;
	}
}

float Delay_Step(fDelay_TypeDef* hdelay, float input)
{
	if (hdelay->length == 0) return input;

	float last = hdelay->buffer[hdelay->position];
	hdelay->buffer[hdelay->position] = input;
	hdelay->position += hdelay->inc;
	hdelay->position %= hdelay->length;
	return last;
}

float Delay_Peek(fDelay_TypeDef* hdelay)
{
	return hdelay->buffer[hdelay->position];
}

void Delay_AdjustLength(fDelay_TypeDef* hdelay, int32_t length)
{
	hdelay->length = length;
	hdelay->position %= hdelay->length;
}
