/*
 * IIR_HighPass.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 */

#ifndef INC_IIR_HIGHPASS_H_
#define INC_IIR_HIGHPASS_H_

#include "stdint.h"

typedef struct
{
	float b[3];
	float a[3];
	float x[3];
	float y[3];
} IIR_HPF;

void IIR_HPF_Init(IIR_HPF* hHPF, float omega);
void IIR_HPF_Compute(IIR_HPF* hHPF, float* input, float* output, int32_t length);
float IIR_HPF_Step(IIR_HPF* hHPF, float input);


#endif /* INC_IIR_HIGHPASS_H_ */
