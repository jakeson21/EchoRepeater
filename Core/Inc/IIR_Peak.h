/*
 * IIR_Peak.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 */

#ifndef INC_IIR_PEAK_H_
#define INC_IIR_PEAK_H_

#include "stdint.h"

typedef struct
{
	float b[3];
	float a[3];
	float x[3];
	float y[3];
} IIR_Peak;

void IIR_Peak_Init(IIR_Peak* hPeak, float omega, float bandwidth);
void IIR_Peak_Compute(IIR_Peak* hPeak, float* input, float* output, int32_t length);
float IIR_Peak_Step(IIR_Peak* hPeak, float input);



#endif /* INC_IIR_PEAK_H_ */
