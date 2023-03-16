/*
 * TappedDelay.c
 *
 *  Created on: Nov 22, 2022
 *      Author: Fuguru
 */


#include "TappedDelay.h"

void TappedDelay_Init(fTappedDelay_TypeDef* hdelay, uint32_t d1, uint32_t d2, uint32_t d3)
{
	hdelay->delay1.length = d1;
	hdelay->delay1.position = 0;
	for (uint32_t n=0; n<hdelay->delay1.length; n++)
	{
		hdelay->delay1.buffer[n] = 0;
	}

	hdelay->delay2.length = d2;
	hdelay->delay2.position = 0;
	for (uint32_t n=0; n<hdelay->delay2.length; n++)
	{
		hdelay->delay2.buffer[n] = 0;
	}

	hdelay->delay3.length = d3;
	hdelay->delay3.position = 0;
	for (uint32_t n=0; n<hdelay->delay3.length; n++)
	{
		hdelay->delay3.buffer[n] = 0;
	}

	hdelay->g0 = 0.8;
	hdelay->g1 = 0.2;
	hdelay->g2 = 0.1;
	hdelay->g3 = 0.05;
}

float TappedDelay_Step(fTappedDelay_TypeDef* hdelay, float input)
{
	float output = input*hdelay->g0;
	float temp = 0.0;
	temp = Delay_Step(&hdelay->delay1, input);
	output += temp*hdelay->g1;
	temp = Delay_Step(&hdelay->delay2, temp);
	output += temp*hdelay->g2;
	temp = Delay_Step(&hdelay->delay3, temp);
	output += temp*hdelay->g3;
	return output;
}

