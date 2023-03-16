/*
 * FbComb.c
 *
 *  Created on: Nov 23, 2022
 *      Author: Fuguru
 */

#include "FbComb.h"

void FbComb_Init(fFbComb_TypeDef* hfilter, uint32_t length, float feedforward, float feedback)
{
	Delay_Init(&hfilter->delay1, length);
	hfilter->b0 = feedforward;
	hfilter->a = feedback;
}

float FbComb_Step(fFbComb_TypeDef* hfilter, float input)
{
	float yn_M = Delay_Peek(&hfilter->delay1);
	float vn = input - hfilter->a*yn_M;
	float yn = hfilter->b0*(vn);
	Delay_Step(&hfilter->delay1, vn);
	return yn;
}
