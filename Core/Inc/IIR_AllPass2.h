/*
 * IIR_AllPass2.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 */

#ifndef INC_IIR_ALLPASS2_H_
#define INC_IIR_ALLPASS2_H_

#include "stdint.h"

typedef struct
{
	float b[3];
	float a[3];
	float x[3];
	float y[3];
	float r;
	float w0;
} IIR_AP2;

void IIR_AP2_Init(IIR_AP2* hAPF2, float omega, float R);
void IIR_AP2_Compute(IIR_AP2* hAPF2, float* input, float* output, int32_t length);
float IIR_AP2_Step(IIR_AP2* hAPF2, float input);


#endif /* INC_IIR_HIGHPASS_H_ */
