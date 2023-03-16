/*
 * IIR_LowPass.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Fuguru
 */

#ifndef INC_IIR_LOWPASS_H_
#define INC_IIR_LOWPASS_H_

#include "stdint.h"

typedef struct
{
	float b[3];
	float a[3];
	float x[3];
	float y[3];
} IIR_LPF;

void IIR_LPF_Init(IIR_LPF* hlpf, float omega);
void IIR_LPF_Compute(IIR_LPF* hlpf, float* input, float* output, int32_t length);
float IIR_LPF_Step(IIR_LPF* hlpf, float input);


#endif /* INC_IIR_LOWPASS_H_ */
